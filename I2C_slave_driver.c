/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "I2C_slave_driver.h"
#include "Config_RIIC1.h"
#include "retarget.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern volatile uint16_t g_riic1_rx_count;
extern volatile uint8_t g_riic1_mode_flag;
extern volatile uint8_t g_riic1_state;
extern unsigned short get_tick(void);
extern unsigned char R_PORT_GetGPIOLevel(unsigned short n, unsigned char Pin);
extern void r_Config_RIIC1_callback_transmitend(void);
extern void r_Config_RIIC1_callback_receiveend(void);
extern void r_Config_RIIC1_callback_receiveerror(MD_STATUS status);

/*_____ D E F I N I T I O N S ______________________________________________*/
static unsigned char g_i2c_slave_rx_buf[I2C_SLAVE_RX_BUFFER_LEN];
static unsigned char g_i2c_slave_tx_buf[I2C_SLAVE_TX_BUFFER_LEN];
static unsigned char g_i2c_slave_rsp_seed = 0x10U;

/* deferred log data (printed in main loop, not ISR) */
static volatile unsigned char g_slave_evt_rx_pending = 0U;
static volatile unsigned short g_slave_evt_rx_len = 0U;
static unsigned char g_slave_evt_rx_buf[I2C_SLAVE_RX_BUFFER_LEN];

static volatile unsigned char g_slave_evt_tx_ready_pending = 0U;
static volatile unsigned short g_slave_evt_tx_ready_len = 0U;
static unsigned char g_slave_evt_tx_ready_buf[I2C_SLAVE_TX_BUFFER_LEN];

static volatile unsigned char g_slave_evt_tx_done_pending = 0U;
static volatile unsigned char g_slave_evt_error_pending = 0U;
static volatile unsigned char g_slave_evt_error_code = 0U;

static volatile unsigned char g_slave_evt_stat_pending = 0U;
static volatile unsigned char g_slave_evt_last_error_flags = 0U;
static volatile unsigned short g_slave_evt_start_count = 0U;
static volatile unsigned short g_slave_evt_stop_count = 0U;
static volatile unsigned short g_slave_evt_nack_count = 0U;
static volatile unsigned short g_slave_evt_frame_count = 0U;
static volatile unsigned short g_slave_evt_ack_packet_count = 0U;
static volatile unsigned short g_slave_evt_tx_underrun_count = 0U;
static volatile unsigned short g_slave_evt_stretch_timeout_count = 0U;
static volatile unsigned short g_slave_evt_stretch_max_ms = 0U;
static volatile unsigned short g_slave_evt_recover_count = 0U;

static volatile unsigned char g_slave_reinit_pending = 0U;
static volatile unsigned char g_slave_reinit_reason = 0U;
static volatile unsigned char g_slave_reinit_error = 0U;

#if (I2C_SLAVE_BUS_MONITOR_ENABLE == 1U)
static unsigned char g_slave_scl_low_active = 0U;
static unsigned short g_slave_scl_low_start_tick = 0U;
static unsigned char g_slave_bus_low_active = 0U;
static unsigned short g_slave_bus_low_start_tick = 0U;
#endif

#define I2C_SLAVE_RECOVER_REASON_ERROR                (1U)
#if (I2C_SLAVE_BUS_MONITOR_ENABLE == 1U)
#define I2C_SLAVE_RECOVER_REASON_STRETCH              (2U)
#define I2C_SLAVE_RECOVER_REASON_BUS_LOW              (3U)
#endif

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
static MD_STATUS I2C_slave_start_receive(void);

static unsigned char I2C_slave_checksum(const unsigned char *buf, unsigned short len)
{
    unsigned short i;
    unsigned short sum = 0U;

    for (i = 0U; i < len; i++)
    {
        sum += buf[i];
    }

    return (unsigned char)(sum & 0xFFU);
}

static void I2C_slave_print_packet(const char *tag, const unsigned char *buf, unsigned short len)
{
    unsigned short i;

    tiny_printf("%s len=%d :", tag, len);
    for (i = 0U; i < len; i++)
    {
        tiny_printf(" %02X", buf[i]);
    }
    tiny_printf("\r\n");
}

#if (I2C_SLAVE_BUS_MONITOR_ENABLE == 1U)
static unsigned short I2C_slave_elapsed_ms(unsigned short start, unsigned short now)
{
    if (now >= start)
    {
        return (unsigned short)(now - start);
    }

    return (unsigned short)((60000U - start) + now);
}
#endif

static void I2C_slave_release_bus_hold(void)
{
    RIIC1.MR3.UINT32 &= (uint32_t)~_RIIC_WAIT;
    RIIC1.MR3.UINT32 &= (uint32_t)~_RIIC_RDRF_FLAG_SET_SCL_EIGHTH;
    RIIC1.MR3.UINT32 &= (uint32_t)~_RIIC_ACKBT_BIT_MODIFICATION_ENABLED;
    RIIC1.MR3.UINT32 &= (uint32_t)~_RIIC_NACK_TRANSMISSION;

    RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_NACK_DETECTED;
    RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_DETECTED;
    RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_DETECTED;
    RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_DETECTED;
    RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_ARBITRATION_LOST;
}

static void I2C_slave_mark_stats_pending(void)
{
    g_slave_evt_stat_pending = 1U;
}

static void I2C_slave_apply_timeout_int_policy(void)
{
#if (I2C_SLAVE_HW_TIMEOUT_INT_ENABLE == 1U)
    RIIC1.FER.UINT32 |= _RIIC_TIMEOUT_FUNCTION_ENABLED;
    RIIC1.IER.UINT32 |= _RIIC_TIMEOUT_INT_ENABLE;
#else
    RIIC1.FER.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_FUNCTION_ENABLED;
    RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_INT_ENABLE;
    RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_DETECTED;
#endif
}

static void I2C_slave_schedule_reinit(unsigned char reason, unsigned char err)
{
#if (I2C_SLAVE_ERROR_AUTO_REINIT_ENABLE == 1U)
    if (g_slave_reinit_pending == 0U)
    {
        g_slave_reinit_pending = 1U;
        g_slave_reinit_reason = reason;
        g_slave_reinit_error = err;
    }
#else
    (void)reason;
    (void)err;
#endif
}

#if (I2C_SLAVE_ERROR_AUTO_REINIT_ENABLE == 1U)
static MD_STATUS I2C_slave_reinit_peripheral(unsigned char reason, unsigned char err)
{
    MD_STATUS ret;
    unsigned char scl_level;
    unsigned char sda_level;

    DI();
    R_Config_RIIC1_Stop();
    RIIC1.CR1.UINT32 &= _RIIC_DISABLE;
    RIIC1.CR1.UINT32 |= _RIIC_RESET;
    RIIC1.CR1.UINT32 |= _RIIC_INTERNAL_RESET;
    RIIC1.CR1.UINT32 &= _RIIC_CLEAR_INTERNAL_RESET;
    R_Config_RIIC1_Create();
    R_Config_RIIC1_Start();
    I2C_slave_apply_timeout_int_policy();
    I2C_slave_release_bus_hold();
    ret = I2C_slave_start_receive();
    EI();

    g_slave_evt_recover_count++;
    g_slave_evt_error_code = err;
    g_slave_evt_error_pending = (ret == MD_OK) ? 0U : 1U;
    g_slave_evt_last_error_flags = (unsigned char)(RIIC1.SR2.UINT32 & 0xFFU);
    I2C_slave_mark_stats_pending();

    scl_level = R_PORT_GetGPIOLevel(I2C_SLAVE_MONITOR_SCL_PORT, I2C_SLAVE_MONITOR_SCL_PIN);
    sda_level = R_PORT_GetGPIOLevel(I2C_SLAVE_MONITOR_SDA_PORT, I2C_SLAVE_MONITOR_SDA_PIN);
    tiny_printf("[S][RECOVER] reason=%d err=0x%02X ret=0x%02X line=%d/%d\r\n", reason, err, ret, scl_level, sda_level);
    return ret;
}
#endif

static void I2C_slave_monitor_bus_hold(void)
{
#if (I2C_SLAVE_BUS_MONITOR_ENABLE == 1U)
    unsigned char transfer_active;
    unsigned char scl_low;
    unsigned char sda_low;
    unsigned short now;
    unsigned short elapsed;

    transfer_active = ((g_riic1_state == _RIIC_SLAVE_RECEIVES_DATA) ||
                       (g_riic1_state == _RIIC_SLAVE_SENDS_DATA) ||
                       (g_riic1_state == _RIIC_SLAVE_SENDS_END)) ? 1U : 0U;
    scl_low = (R_PORT_GetGPIOLevel(I2C_SLAVE_MONITOR_SCL_PORT, I2C_SLAVE_MONITOR_SCL_PIN) == 0U) ? 1U : 0U;
    sda_low = (R_PORT_GetGPIOLevel(I2C_SLAVE_MONITOR_SDA_PORT, I2C_SLAVE_MONITOR_SDA_PIN) == 0U) ? 1U : 0U;
    now = get_tick();

    if ((transfer_active != 0U) && (scl_low != 0U))
    {
        if (g_slave_scl_low_active == 0U)
        {
            g_slave_scl_low_active = 1U;
            g_slave_scl_low_start_tick = now;
        }
        else
        {
            elapsed = I2C_slave_elapsed_ms(g_slave_scl_low_start_tick, now);
            if (elapsed > g_slave_evt_stretch_max_ms)
            {
                g_slave_evt_stretch_max_ms = elapsed;
                I2C_slave_mark_stats_pending();
            }

            if ((elapsed >= I2C_SLAVE_STRETCH_LIMIT_MS) && (g_slave_reinit_pending == 0U))
            {
                g_slave_evt_stretch_timeout_count++;
                I2C_slave_mark_stats_pending();
                tiny_printf("[S][HOLD] stretch timeout %dms\r\n", elapsed);
                I2C_slave_schedule_reinit(I2C_SLAVE_RECOVER_REASON_STRETCH, 0xE1U);
            }
        }
    }
    else
    {
        g_slave_scl_low_active = 0U;
    }

    if ((scl_low != 0U) && (sda_low != 0U))
    {
        if (g_slave_bus_low_active == 0U)
        {
            g_slave_bus_low_active = 1U;
            g_slave_bus_low_start_tick = now;
        }
        else
        {
            elapsed = I2C_slave_elapsed_ms(g_slave_bus_low_start_tick, now);
            if ((elapsed >= I2C_SLAVE_BUS_LOW_RECOVER_MS) && (g_slave_reinit_pending == 0U))
            {
                I2C_slave_mark_stats_pending();
                tiny_printf("[S][HOLD] bus low timeout %dms (SCL=0,SDA=0)\r\n", elapsed);
                I2C_slave_schedule_reinit(I2C_SLAVE_RECOVER_REASON_BUS_LOW, 0xE2U);
            }
        }
    }
    else
    {
        g_slave_bus_low_active = 0U;
    }
#endif
}

static MD_STATUS I2C_slave_start_receive(void)
{
    I2C_slave_release_bus_hold();
    return R_Config_RIIC1_Slave_Receive(g_i2c_slave_rx_buf, I2C_SLAVE_RX_BUFFER_LEN);
}

static unsigned char I2C_slave_validate_write_packet(const unsigned char *buf, unsigned short len)
{
    unsigned short payload_len;
    unsigned short expected_len;
#if (I2C_SLAVE_RX_CHECKSUM_ENABLE == 1U)
    unsigned char expect_checksum;
#endif

    if (len < 5U)
    {
        return 0U;
    }

    if ((buf[0U] != I2C_SLAVE_PACKET_HEADER) || (buf[len - 1U] != I2C_SLAVE_PACKET_TAIL))
    {
        return 0U;
    }

    payload_len = buf[2U];
    expected_len = (unsigned short)(payload_len + 5U); /* hdr + reg + len + data + checksum + tail */
    if (expected_len != len)
    {
        return 0U;
    }

#if (I2C_SLAVE_RX_CHECKSUM_ENABLE == 1U)
    expect_checksum = I2C_slave_checksum(&buf[1U], (unsigned short)(payload_len + 2U)); /* reg + len + data */
    if (expect_checksum != buf[3U + payload_len])
    {
        return 0U;
    }
#endif

    return 1U;
}

static void I2C_slave_copy_to_log_rx(unsigned short len)
{
    unsigned short i;

    if (len > I2C_SLAVE_RX_BUFFER_LEN)
    {
        len = I2C_SLAVE_RX_BUFFER_LEN;
    }

    for (i = 0U; i < len; i++)
    {
        g_slave_evt_rx_buf[i] = g_i2c_slave_rx_buf[i];
    }

    g_slave_evt_rx_len = len;
    g_slave_evt_rx_pending = 1U;
}

static void I2C_slave_copy_to_log_tx_ready(unsigned short len)
{
    unsigned short i;

    if (len > I2C_SLAVE_TX_BUFFER_LEN)
    {
        len = I2C_SLAVE_TX_BUFFER_LEN;
    }

    for (i = 0U; i < len; i++)
    {
        g_slave_evt_tx_ready_buf[i] = g_i2c_slave_tx_buf[i];
    }

    g_slave_evt_tx_ready_len = len;
    g_slave_evt_tx_ready_pending = 1U;
}

static MD_STATUS I2C_slave_prepare_read_response(unsigned char read_reg, unsigned short data_len)
{
    MD_STATUS ret;
    unsigned short i;
    unsigned short tx_len;

    if ((data_len == 0U) || ((unsigned short)(data_len + 4U) > I2C_SLAVE_TX_BUFFER_LEN))
    {
        return MD_ARGERROR;
    }

    g_i2c_slave_tx_buf[0U] = read_reg;
    g_i2c_slave_tx_buf[1U] = (unsigned char)data_len;
    for (i = 0U; i < data_len; i++)
    {
        g_i2c_slave_tx_buf[2U + i] = (unsigned char)(g_i2c_slave_rsp_seed + i);
    }
    g_i2c_slave_rsp_seed++;

    g_i2c_slave_tx_buf[2U + data_len] = I2C_slave_checksum(g_i2c_slave_tx_buf, (unsigned short)(data_len + 2U));
    g_i2c_slave_tx_buf[3U + data_len] = I2C_SLAVE_PACKET_TAIL;
    tx_len = data_len + 4U;

    ret = R_Config_RIIC1_Slave_Send(g_i2c_slave_tx_buf, tx_len);
    if (ret == MD_OK)
    {
        RIIC1.MR3.UINT32 &= (uint32_t)~_RIIC_WAIT;   /* do not hold SCL indefinitely */
        I2C_slave_copy_to_log_tx_ready(tx_len);
    }

    return ret;
}

void IICA0_slave_Init(void)
{
    MD_STATUS ret;

    g_slave_evt_rx_pending = 0U;
    g_slave_evt_tx_ready_pending = 0U;
    g_slave_evt_tx_done_pending = 0U;
    g_slave_evt_error_pending = 0U;
    g_slave_evt_error_code = 0U;
    g_slave_evt_stat_pending = 0U;
    g_slave_evt_last_error_flags = 0U;
    g_slave_evt_start_count = 0U;
    g_slave_evt_stop_count = 0U;
    g_slave_evt_nack_count = 0U;
    g_slave_evt_frame_count = 0U;
    g_slave_evt_ack_packet_count = 0U;
    g_slave_evt_tx_underrun_count = 0U;
    g_slave_evt_stretch_timeout_count = 0U;
    g_slave_evt_stretch_max_ms = 0U;
    g_slave_evt_recover_count = 0U;
    g_slave_reinit_pending = 0U;
    g_slave_reinit_reason = 0U;
    g_slave_reinit_error = 0U;
#if (I2C_SLAVE_BUS_MONITOR_ENABLE == 1U)
    g_slave_scl_low_active = 0U;
    g_slave_bus_low_active = 0U;
#endif

    I2C_slave_apply_timeout_int_policy();
    I2C_slave_release_bus_hold();
    ret = I2C_slave_start_receive();
    if (ret != MD_OK)
    {
        g_slave_evt_error_code = (unsigned char)ret;
        g_slave_evt_error_pending = 1U;
    }
}

void IICA0_slave_Task(void)
{
    unsigned char rx_pending;
    unsigned short rx_len;
    unsigned char rx_buf[I2C_SLAVE_RX_BUFFER_LEN];

    unsigned char tx_ready_pending;
    unsigned short tx_ready_len;
    unsigned char tx_ready_buf[I2C_SLAVE_TX_BUFFER_LEN];

    unsigned char tx_done_pending;
    unsigned char error_pending;
    unsigned char error_code;
    unsigned char stat_pending;
    unsigned char last_error_flags;
    unsigned short start_count;
    unsigned short stop_count;
    unsigned short nack_count;
    unsigned short frame_count;
    unsigned short ack_packet_count;
    unsigned short tx_underrun_count;
    unsigned short stretch_timeout_count;
    unsigned short stretch_max_ms;
    unsigned short recover_count;
    unsigned char reinit_pending;
    unsigned char reinit_reason;
    unsigned char reinit_error;

    unsigned short i;

    DI();
    rx_pending = g_slave_evt_rx_pending;
    rx_len = g_slave_evt_rx_len;
    if (rx_pending != 0U)
    {
        for (i = 0U; i < rx_len; i++)
        {
            rx_buf[i] = g_slave_evt_rx_buf[i];
        }
        g_slave_evt_rx_pending = 0U;
    }

    tx_ready_pending = g_slave_evt_tx_ready_pending;
    tx_ready_len = g_slave_evt_tx_ready_len;
    if (tx_ready_pending != 0U)
    {
        for (i = 0U; i < tx_ready_len; i++)
        {
            tx_ready_buf[i] = g_slave_evt_tx_ready_buf[i];
        }
        g_slave_evt_tx_ready_pending = 0U;
    }

    tx_done_pending = g_slave_evt_tx_done_pending;
    g_slave_evt_tx_done_pending = 0U;

    error_pending = g_slave_evt_error_pending;
    error_code = g_slave_evt_error_code;
    g_slave_evt_error_pending = 0U;

    stat_pending = g_slave_evt_stat_pending;
    last_error_flags = g_slave_evt_last_error_flags;
    start_count = g_slave_evt_start_count;
    stop_count = g_slave_evt_stop_count;
    nack_count = g_slave_evt_nack_count;
    frame_count = g_slave_evt_frame_count;
    ack_packet_count = g_slave_evt_ack_packet_count;
    tx_underrun_count = g_slave_evt_tx_underrun_count;
    stretch_timeout_count = g_slave_evt_stretch_timeout_count;
    stretch_max_ms = g_slave_evt_stretch_max_ms;
    recover_count = g_slave_evt_recover_count;
    g_slave_evt_stat_pending = 0U;

    reinit_pending = g_slave_reinit_pending;
    reinit_reason = g_slave_reinit_reason;
    reinit_error = g_slave_reinit_error;
    g_slave_reinit_pending = 0U;
    EI();

    if (rx_pending != 0U)
    {
        unsigned char reg_addr = 0U;

        I2C_slave_print_packet("[S][RX]", rx_buf, rx_len);
        if (I2C_slave_validate_write_packet(rx_buf, rx_len) == 0U)
        {
            tiny_printf("[S] invalid write packet\r\n");
            I2C_slave_mark_stats_pending();
        }
        else
        {
            reg_addr = rx_buf[1U];
            g_slave_evt_ack_packet_count++;

            if ((reg_addr != I2C_SLAVE_CMD_WRITE_ONLY_REG) &&
                (reg_addr != I2C_SLAVE_CMD_WRITE_ONLY_REG_EXT) &&
                (reg_addr != I2C_SLAVE_CMD_WRITE_THEN_READ_REG) &&
                (reg_addr != I2C_SLAVE_CMD_WRITE_THEN_READ_REG_EXT))
            {
                tiny_printf("[S] unsupported reg 0x%02X\r\n", reg_addr);
                I2C_slave_mark_stats_pending();
            }
        }
    }

    if (tx_ready_pending != 0U)
    {
        I2C_slave_print_packet("[S][TX-READY]", tx_ready_buf, tx_ready_len);
    }

    if (tx_done_pending != 0U)
    {
        tiny_printf("[S] tx complete\r\n");
    }

    if (error_pending != 0U)
    {
        tiny_printf("[S] error:0x%02X\r\n", error_code);
    }

    if (stat_pending != 0U)
    {
        tiny_printf("[S][EVT] START=%d STOP=%d FRAME=%d ACK_PKT=%d NACK=%d UNDERRUN=%d STRETCH_TO=%d STRETCH_MAX=%dms ERR_FLG=0x%02X RECOVER=%d\r\n",
                    start_count,
                    stop_count,
                    frame_count,
                    ack_packet_count,
                    nack_count,
                    tx_underrun_count,
                    stretch_timeout_count,
                    stretch_max_ms,
                    last_error_flags,
                    recover_count);
    }

#if (I2C_SLAVE_ERROR_AUTO_REINIT_ENABLE == 1U)
    if (reinit_pending != 0U)
    {
        (void)I2C_slave_reinit_peripheral(reinit_reason, reinit_error);
    }
#else
    (void)reinit_pending;
    (void)reinit_reason;
    (void)reinit_error;
#endif

    I2C_slave_monitor_bus_hold();
}

void IICA0_slave_on_receive_end(void)
{
    unsigned short rx_len = g_riic1_rx_count;
    unsigned char reg_addr;
    MD_STATUS ret;

    I2C_slave_release_bus_hold();
    I2C_slave_copy_to_log_rx(rx_len);
    if (rx_len > 0U)
    {
        g_slave_evt_frame_count++;
    }

    if (I2C_slave_validate_write_packet(g_i2c_slave_rx_buf, rx_len) == 0U)
    {
        ret = I2C_slave_start_receive();
        if (ret != MD_OK)
        {
            g_slave_evt_error_code = (unsigned char)ret;
            g_slave_evt_error_pending = 1U;
            I2C_slave_schedule_reinit(I2C_SLAVE_RECOVER_REASON_ERROR, (unsigned char)ret);
        }
        return;
    }

    reg_addr = g_i2c_slave_rx_buf[1U];

    if (reg_addr == I2C_SLAVE_CMD_WRITE_THEN_READ_REG)
    {
        ret = I2C_slave_prepare_read_response(I2C_SLAVE_CMD_READBACK_REG, I2C_SLAVE_READ_DATA_LEN_SHORT);
    }
    else if (reg_addr == I2C_SLAVE_CMD_WRITE_THEN_READ_REG_EXT)
    {
        ret = I2C_slave_prepare_read_response(I2C_SLAVE_CMD_READBACK_REG_EXT, I2C_SLAVE_READ_DATA_LEN_LONG);
    }
    else
    {
        ret = I2C_slave_start_receive();
    }

    if (ret != MD_OK)
    {
        g_slave_evt_error_code = (unsigned char)ret;
        g_slave_evt_error_pending = 1U;
        I2C_slave_schedule_reinit(I2C_SLAVE_RECOVER_REASON_ERROR, (unsigned char)ret);
    }
}

void IICA0_slave_on_transmit_end(void)
{
    MD_STATUS ret;

    I2C_slave_release_bus_hold();
    g_slave_evt_tx_done_pending = 1U;
    ret = I2C_slave_start_receive();
    if (ret != MD_OK)
    {
        g_slave_evt_error_code = (unsigned char)ret;
        g_slave_evt_error_pending = 1U;
        I2C_slave_schedule_reinit(I2C_SLAVE_RECOVER_REASON_ERROR, (unsigned char)ret);
    }
}

void IICA0_slave_on_error(unsigned char err)
{
    MD_STATUS ret;

    I2C_slave_release_bus_hold();

    if ((err == MD_ERROR2) || (err == MD_ERROR3))
    {
        /* Timeout/NACK are treated as non-fatal and re-arm receive immediately. */
        g_slave_evt_last_error_flags = (unsigned char)(RIIC1.SR2.UINT32 & 0xFFU);
        g_slave_evt_error_pending = 0U;
        I2C_slave_mark_stats_pending();
        ret = I2C_slave_start_receive();
        if (ret != MD_OK)
        {
            g_slave_evt_error_code = (unsigned char)ret;
            g_slave_evt_error_pending = 1U;
            I2C_slave_schedule_reinit(I2C_SLAVE_RECOVER_REASON_ERROR, (unsigned char)ret);
        }
    }
    else
    {
        g_slave_evt_error_code = err;
        g_slave_evt_last_error_flags = (unsigned char)(RIIC1.SR2.UINT32 & 0xFFU);
        I2C_slave_mark_stats_pending();
#if (I2C_SLAVE_ERROR_AUTO_REINIT_ENABLE == 1U)
        g_slave_evt_error_pending = 1U;
        I2C_slave_schedule_reinit(I2C_SLAVE_RECOVER_REASON_ERROR, err);
#else
        ret = I2C_slave_start_receive();
        if (ret != MD_OK)
        {
            g_slave_evt_error_code = (unsigned char)ret;
            g_slave_evt_error_pending = 1U;
        }
        else
        {
            g_slave_evt_error_pending = 0U;
        }
#endif
    }
}

void IICA0_slave_on_start_detected(void)
{
    I2C_slave_release_bus_hold();
    g_slave_evt_start_count++;
}

void IICA0_slave_on_stop_detected(void)
{
    I2C_slave_release_bus_hold();
    g_slave_evt_stop_count++;
}

void IICA0_slave_on_nack_detected(void)
{
    I2C_slave_release_bus_hold();
    /* In slave-transmit flow, master NACK at final byte is normal end-of-read. */
    g_slave_evt_nack_count++;
}

void IICA0_slave_on_tx_underrun(void)
{
    g_slave_evt_tx_underrun_count++;
    I2C_slave_mark_stats_pending();
}

/*
    Custom RIIC1 error ISR body.
    This function is called from r_Config_RIIC1_error_interrupt()
    when I2C_RIIC1_CUSTOM_ERROR_ISR_ENABLE == 1.
*/
uint8_t I2C_Slave_RIIC1_ErrorISR_Custom(void)
{
#if (I2C_RIIC1_CUSTOM_ERROR_ISR_ENABLE == 1U)
    volatile uint8_t dummy;
    uint32_t pending;

    if ((RIIC1.IER.UINT32 & _RIIC_ARBITRATION_LOST_INT_ENABLE) && (RIIC1.SR2.UINT32 & _RIIC_ARBITRATION_LOST))
    {
        r_Config_RIIC1_callback_receiveerror(MD_ERROR1);
        return 1U;
    }
    else if ((RIIC1.IER.UINT32 & _RIIC_TIMEOUT_INT_ENABLE) && (RIIC1.SR2.UINT32 & _RIIC_TIMEOUT_DETECTED))
    {
        r_Config_RIIC1_callback_receiveerror(MD_ERROR2);
        return 1U;
    }
    else if ((RIIC1.IER.UINT32 & _RIIC_NACK_RECEPTION_INT_ENABLE) && (RIIC1.SR2.UINT32 & _RIIC_NACK_DETECTED))
    {
        if (_RIIC_SLAVE_TRANSMIT == g_riic1_mode_flag)
        {
            /* Master NACK at end of slave TX is normal. Finish only after STOP is observed. */
            dummy = RIIC1.DRR.UINT32;
            RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_NACK_DETECTED;
            IICA0_slave_on_nack_detected();

            if ((RIIC1.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED) != 0U)
            {
                RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_DETECTED;
                RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_INT_ENABLE;
                RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
                g_riic1_state = _RIIC_SLAVE_WAIT_START_CONDITION;
                IICA0_slave_on_stop_detected();
                r_Config_RIIC1_callback_transmitend();
            }
            else
            {
                RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
                RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_INT_ENABLE;
                g_riic1_state = _RIIC_SLAVE_SENDS_STOP;
            }
        }
        else
        {
            RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_NACK_DETECTED;
        }
        return 1U;
    }
    else if (_RIIC_SLAVE_RECEIVE == g_riic1_mode_flag)
    {
        if ((RIIC1.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED) == _RIIC_STOP_CONDITION_DETECTED)
        {
            /*
                STOP-start mode: avoid finalizing frame too early.
                First observe STOP -> move to RECEIVES_STOP, then finalize on next pass.
            */
            if (_RIIC_SLAVE_RECEIVES_STOP != g_riic1_state)
            {
                g_riic1_state = _RIIC_SLAVE_RECEIVES_STOP;
                return 1U;
            }

            RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_DETECTED;
            RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            g_riic1_state = _RIIC_SLAVE_WAIT_START_CONDITION;
            IICA0_slave_on_stop_detected();
            if (g_riic1_rx_count < 5U)
            {
                /* Abnormal: frame ended before minimum packet length. */
                g_slave_evt_error_code = 0xE3U;
                g_slave_evt_error_pending = 1U;
                I2C_slave_mark_stats_pending();
            }
            if (g_riic1_rx_count > 0U)
            {
                r_Config_RIIC1_callback_receiveend();

                /*
                    Stop-start mode can have next START already latched while finalizing write frame.
                    If we already prepared TX (WAIT_START), consume START immediately and enter SENDS_DATA.
                */
                if ((_RIIC_SLAVE_TRANSMIT == g_riic1_mode_flag) &&
                    (_RIIC_SLAVE_WAIT_START_CONDITION == g_riic1_state))
                {
                    if ((RIIC1.SR2.UINT32 & _RIIC_START_CONDITION_DETECTED) != 0U)
                    {
                        RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_DETECTED;
                    }
                    RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_INT_ENABLE;
                    RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
                    g_riic1_state = _RIIC_SLAVE_SENDS_DATA;
                    IICA0_slave_on_start_detected();
                    return 1U;
                }
            }
            return 1U;
        }
        else if (((RIIC1.SR2.UINT32 & _RIIC_START_CONDITION_DETECTED) != 0U) &&
                 (_RIIC_SLAVE_RECEIVES_DATA == g_riic1_state))
        {
            /*
                Repeated-start (Sr) handling for external master write-then-read:
                1) finalize current write frame immediately (no STOP required)
                2) if callback prepares slave TX, consume same START and switch to SENDS_DATA
            */
            if (g_riic1_rx_count > 0U)
            {
                g_riic1_state = _RIIC_SLAVE_WAIT_START_CONDITION;
                r_Config_RIIC1_callback_receiveend();

                if ((_RIIC_SLAVE_TRANSMIT == g_riic1_mode_flag) &&
                    (_RIIC_SLAVE_WAIT_START_CONDITION == g_riic1_state))
                {
                    if ((RIIC1.SR2.UINT32 & _RIIC_START_CONDITION_DETECTED) != 0U)
                    {
                        RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_DETECTED;
                    }
                    RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_INT_ENABLE;
                    RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
                    g_riic1_state = _RIIC_SLAVE_SENDS_DATA;
                    IICA0_slave_on_start_detected();
                    return 1U;
                }
            }

            /*
                Remain in RX path for unsupported reg/write-only cases.
                Do not force RECEIVE state if callback already switched to another mode.
            */
            if (_RIIC_SLAVE_RECEIVE == g_riic1_mode_flag)
            {
                RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_DETECTED;
                RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
                RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
                g_riic1_state = _RIIC_SLAVE_RECEIVES_DATA;
                IICA0_slave_on_start_detected();
            }
            return 1U;
        }
        else if (_RIIC_SLAVE_WAIT_START_CONDITION == g_riic1_state)
        {
            RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_DETECTED;
            RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            g_riic1_state = _RIIC_SLAVE_RECEIVES_DATA;
            IICA0_slave_on_start_detected();
            return 1U;
        }
    }
    else if (_RIIC_SLAVE_TRANSMIT == g_riic1_mode_flag)
    {
        if (_RIIC_SLAVE_SENDS_STOP == g_riic1_state)
        {
            RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_NACK_DETECTED;
            RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_DETECTED;
            RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            IICA0_slave_on_stop_detected();
            r_Config_RIIC1_callback_transmitend();
            return 1U;
        }
        else if (_RIIC_SLAVE_WAIT_START_CONDITION == g_riic1_state)
        {
            if ((RIIC1.SR2.UINT32 & _RIIC_START_CONDITION_DETECTED) != 0U)
            {
                RIIC1.SR2.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_DETECTED;
                RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_INT_ENABLE;
                RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
                g_riic1_state = _RIIC_SLAVE_SENDS_DATA;
                IICA0_slave_on_start_detected();
                return 1U;
            }

            /*
                Failsafe for missed START IRQ edge in stop-start mode:
                if HW already switched to transmit direction, don't keep waiting.
            */
            if ((RIIC1.CR2.UINT32 & _RIIC_TRANSMIT_MODE) != 0U)
            {
                RIIC1.IER.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_INT_ENABLE;
                RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
                g_riic1_state = _RIIC_SLAVE_SENDS_DATA;
                g_slave_evt_error_code = 0xE4U;
                g_slave_evt_error_pending = 1U;
                I2C_slave_mark_stats_pending();
                return 1U;
            }
        }
    }

    /* Defensive fallback: clear any sticky error-event flag to avoid ISR storm. */
    pending = RIIC1.SR2.UINT32 & (uint32_t)(_RIIC_NACK_DETECTED |
                                             _RIIC_STOP_CONDITION_DETECTED |
                                             _RIIC_START_CONDITION_DETECTED |
                                             _RIIC_TIMEOUT_DETECTED |
                                             _RIIC_ARBITRATION_LOST);
    if (pending != 0U)
    {
        RIIC1.SR2.UINT32 &= (uint32_t)~pending;
        I2C_slave_release_bus_hold();
        return 1U;
    }

    return 0U;
#else
    return 0U;
#endif
}
