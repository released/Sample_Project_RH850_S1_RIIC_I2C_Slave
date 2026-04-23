/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "I2C_master_driver.h"
#include "misc_config.h"
#include "custom_func.h"
#include "retarget.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_8bit flag_IIC_Master_CTL;
#define FLAG_IIC_Master_TRIG_I2C_SEND                (flag_IIC_Master_CTL.bit0)
#define FLAG_IIC_Master_TRIG_I2C_RCV                 (flag_IIC_Master_CTL.bit1)

/*_____ D E F I N I T I O N S ______________________________________________*/
#define I2C_MASTER_TX_DATA_MAX_LEN                   (127U)
#define I2C_MASTER_FRAME_MAX_LEN                     (I2C_MASTER_MAX_PAYLOAD + 5U)
#define I2C_MASTER_READ_FRAME_MIN_LEN                (4U)

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
extern volatile uint8_t g_riic0_mode_flag;
extern volatile uint8_t g_riic0_state;

static unsigned char I2C_packet_checksum(const unsigned char *buf, unsigned short len)
{
    unsigned short i;
    unsigned short sum = 0U;

    for (i = 0U; i < len; i++)
    {
        sum += buf[i];
    }

    return (unsigned char)(sum & 0xFFU);
}

static void I2C_print_packet(const char *tag, const unsigned char *buf, unsigned short len)
{
    unsigned short i;

    tiny_printf("%s len=%d :", tag, len);
    for (i = 0U; i < len; i++)
    {
        tiny_printf(" %02X", buf[i]);
    }
    tiny_printf("\r\n");
}

static unsigned short I2C_elapsed_ms(unsigned short start, unsigned short now)
{
    if (now >= start)
    {
        return (unsigned short)(now - start);
    }

    return (unsigned short)((I2C_MASTER_TICK_WRAP_MS - start) + now);
}

static MD_STATUS drv_IIC_Master_wait_send_done(void)
{
    unsigned short start_tick = get_tick();

    while (drv_get_IIC_Master_send_flag())
    {
        if ((RIIC0.SR2.UINT32 & _RIIC_NACK_DETECTED) != 0U)
        {
            drv_set_IIC_Master_send_flag(0U);
            drv_IIC_Master_callback_error(MD_ERROR3);
            return MD_ERROR3;
        }

        if (((RIIC0.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED) != 0U) &&
            (g_riic0_mode_flag == _RIIC_MASTER_TRANSMIT))
        {
            /* Fallback when callback is not reached in time. */
            RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_NACK_DETECTED;
            RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_DETECTED;
            RIIC0.IER.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC0.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            drv_set_IIC_Master_send_flag(0U);
            return MD_OK;
        }

        if (I2C_elapsed_ms(start_tick, get_tick()) >= I2C_MASTER_SEND_TIMEOUT_MS)
        {
            tiny_printf("[I2C master] send timeout\r\n");
            drv_set_IIC_Master_send_flag(0U);
            return MD_BUSY2;
        }
    }

    return MD_OK;
}

static MD_STATUS drv_IIC_Master_wait_receive_done(void)
{
    unsigned short start_tick = get_tick();

    while (drv_get_IIC_Master_receive_flag())
    {
        if ((RIIC0.SR2.UINT32 & _RIIC_NACK_DETECTED) != 0U)
        {
            drv_set_IIC_Master_receive_flag(0U);
            drv_IIC_Master_callback_error(MD_ERROR3);
            return MD_ERROR3;
        }

        if (((RIIC0.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED) != 0U) &&
            (g_riic0_mode_flag == _RIIC_MASTER_RECEIVE))
        {
            /* Fallback when callback is not reached in time. */
            RIIC0.MR3.UINT32 &= (uint32_t)~_RIIC_RDRF_FLAG_SET_SCL_EIGHTH;
            RIIC0.MR3.UINT32 &= (uint32_t)~_RIIC_ACKBT_BIT_MODIFICATION_ENABLED;
            RIIC0.MR3.UINT32 &= (uint32_t)~_RIIC_NACK_TRANSMISSION;
            RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_NACK_DETECTED;
            RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_DETECTED;
            RIIC0.IER.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC0.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            drv_set_IIC_Master_receive_flag(0U);
            return MD_OK;
        }

        if (I2C_elapsed_ms(start_tick, get_tick()) >= I2C_MASTER_RECV_TIMEOUT_MS)
        {
            tiny_printf("[I2C master] receive timeout\r\n");
            drv_set_IIC_Master_receive_flag(0U);
            return MD_BUSY2;
        }
    }

    return MD_OK;
}

void I2C_Master_Init(void)
{
    drv_set_IIC_Master_send_flag(0U);
    drv_set_IIC_Master_receive_flag(0U);
}

static MD_STATUS I2C_validate_read_frame(const unsigned char *rx_buf, unsigned short rx_len)
{
    unsigned short payload_len;
    unsigned short expected_len;
#if (I2C_MASTER_RX_CHECKSUM_ENABLE == 1U)
    unsigned char checksum;
#endif

    if ((rx_buf == NULL) || (rx_len < I2C_MASTER_READ_FRAME_MIN_LEN))
    {
        return MD_ARGERROR;
    }

    payload_len = rx_buf[1U];
    expected_len = (unsigned short)(payload_len + 4U);   /* reg + len + data + checksum + tail */
    if (expected_len != rx_len)
    {
        tiny_printf("[I2C master] rx len mismatch exp=%d got=%d\r\n", expected_len, rx_len);
        return MD_ERROR;
    }

    if (rx_buf[rx_len - 1U] != I2C_PACKET_TAIL)
    {
        tiny_printf("[I2C master] rx tail error\r\n");
        return MD_ERROR;
    }

#if (I2C_MASTER_RX_CHECKSUM_ENABLE == 1U)
    checksum = I2C_packet_checksum(rx_buf, (unsigned short)(payload_len + 2U)); /* reg + len + data */
    if (checksum != rx_buf[rx_len - 2U])
    {
        tiny_printf("[I2C master] rx checksum error exp=%02X got=%02X\r\n",
                    checksum,
                    rx_buf[rx_len - 2U]);
        return MD_ERROR;
    }
#endif

    return MD_OK;
}

MD_STATUS I2C_Master_Transfer(const I2C_MASTER_PACKET_TRANSFER_T *transfer)
{
    MD_STATUS ret;

    if (transfer == NULL)
    {
        return MD_ARGERROR;
    }

    if (transfer->is_write_then_read == 0U)
    {
        return I2C_Master_SendWriteOnlyPacket(transfer->device_addr,
                                              transfer->write_reg_addr,
                                              transfer->tx_payload,
                                              transfer->tx_payload_len);
    }

    if ((transfer->rx_buf == NULL) || (transfer->rx_len == 0U))
    {
        return MD_ARGERROR;
    }

    ret = I2C_Master_SendWriteOnlyPacket(transfer->device_addr,
                                         transfer->write_reg_addr,
                                         transfer->tx_payload,
                                         transfer->tx_payload_len);
    if (ret != MD_OK)
    {
        return ret;
    }

    if (transfer->read_delay_ms > 0U)
    {
        ostimer_dealyms(transfer->read_delay_ms);
    }

    drv_set_IIC_Master_receive_flag(1U);
    ret = drv_IIC_Master_read(transfer->device_addr, transfer->rx_buf, transfer->rx_len);
    if (ret != MD_OK)
    {
        drv_set_IIC_Master_receive_flag(0U);
        return ret;
    }

    ret = drv_IIC_Master_wait_receive_done();
    if (ret == MD_OK)
    {
        ret = I2C_validate_read_frame(transfer->rx_buf, transfer->rx_len);
    }

    if (ret == MD_OK)
    {
        I2C_print_packet("[M][RX]", transfer->rx_buf, transfer->rx_len);
    }

    return ret;
}

MD_STATUS drv_IIC_Master_read(unsigned char adr, unsigned char * const rx_buf, unsigned short rx_num)
{
    MD_STATUS ret;

    if ((rx_buf == NULL) || (rx_num == 0U))
    {
        return MD_ARGERROR;
    }

    ret = R_Config_RIIC0_Master_Receive(adr, rx_buf, rx_num);
    if (ret != MD_OK)
    {
        tiny_printf("[drv_IIC_Master_read error]0x%02X\r\n", ret);
    }

    return ret;
}

MD_STATUS drv_IIC_Master_write(unsigned char adr, unsigned char * const tx_buf, unsigned short tx_num)
{
    MD_STATUS ret;

    if ((tx_buf == NULL) || (tx_num == 0U))
    {
        return MD_ARGERROR;
    }

    ret = R_Config_RIIC0_Master_Send(adr, tx_buf, tx_num);
    if (ret != MD_OK)
    {
        tiny_printf("[drv_IIC_Master_write error]0x%02X\r\n", ret);
    }

    return ret;
}

void drv_IIC_Master_callback_error(unsigned char err)
{
    /* Clear sticky timeout/status flags to avoid repeated error interrupt callbacks. */
    RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_DETECTED;
    RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_ARBITRATION_LOST;
    RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_NACK_DETECTED;
    RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_STOP_CONDITION_DETECTED;
    RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_START_CONDITION_DETECTED;

    R_Config_RIIC0_StopCondition();
    tiny_printf("[I2C master error]0x%02X\r\n", err);
}

void drv_set_IIC_Master_receive_flag(bool flag)
{
    FLAG_IIC_Master_TRIG_I2C_RCV = flag;
}

bool drv_get_IIC_Master_receive_flag(void)
{
    return FLAG_IIC_Master_TRIG_I2C_RCV;
}

void drv_set_IIC_Master_send_flag(bool flag)
{
    FLAG_IIC_Master_TRIG_I2C_SEND = flag;
}

bool drv_get_IIC_Master_send_flag(void)
{
    return FLAG_IIC_Master_TRIG_I2C_SEND;
}

MD_STATUS IICA0_read(unsigned char device_addr, unsigned char reg_addr, unsigned char *rx_xfer_data, unsigned short rx_num)
{
    MD_STATUS ret;
    unsigned char reg = reg_addr;

    if ((rx_xfer_data == NULL) || (rx_num == 0U))
    {
        return MD_ARGERROR;
    }

    drv_set_IIC_Master_send_flag(1U);
    ret = drv_IIC_Master_write(device_addr, &reg, 1U);
    if (ret != MD_OK)
    {
        drv_set_IIC_Master_send_flag(0U);
        return ret;
    }

    ret = drv_IIC_Master_wait_send_done();
    if (ret != MD_OK)
    {
        return ret;
    }

    drv_set_IIC_Master_receive_flag(1U);
    ret = drv_IIC_Master_read(device_addr, rx_xfer_data, rx_num);
    if (ret != MD_OK)
    {
        drv_set_IIC_Master_receive_flag(0U);
        return ret;
    }

    return drv_IIC_Master_wait_receive_done();
}

MD_STATUS IICA0_write(unsigned char device_addr, unsigned char reg_addr, unsigned char *tx_xfer_data, unsigned short tx_num)
{
    MD_STATUS ret;
    unsigned short i;
    unsigned char buffer[128U];

    if (tx_num > I2C_MASTER_TX_DATA_MAX_LEN)
    {
        return MD_ARGERROR;
    }

    if ((tx_xfer_data == NULL) && (tx_num > 0U))
    {
        return MD_ARGERROR;
    }

    buffer[0U] = reg_addr;
    for (i = 0U; i < tx_num; i++)
    {
        buffer[i + 1U] = tx_xfer_data[i];
    }

    drv_set_IIC_Master_send_flag(1U);
    ret = drv_IIC_Master_write(device_addr, buffer, (unsigned short)(tx_num + 1U));
    if (ret != MD_OK)
    {
        drv_set_IIC_Master_send_flag(0U);
        return ret;
    }

    return drv_IIC_Master_wait_send_done();
}

MD_STATUS I2C_Master_SendWriteOnlyPacket(unsigned char device_addr,
                                         unsigned char reg_addr,
                                         unsigned char *payload,
                                         unsigned char payload_len)
{
    /*
        Packet format used by both Write-only and Write-then-Read(write phase):
        [0] Header  : 0x5A
        [1] Reg     : command/register
        [2] Len     : payload length
        [3..] Data  : payload bytes
        [N-2] Checksum : 1-byte sum of (Reg + Len + Data)
        [N-1] Tail  : 0xA5

        Example commands in this project:
        1) Write-only  (key '1') reg=0x30, len=4
           5A 30 04 d0 d1 d2 d3 cs A5
        2) Write-Read  (key '2') write reg=0x40, len=0
           5A 40 00 cs A5
        3) Write-only  (key '3') reg=0x50, len=8, last byte auto-increment
           5A 50 08 d8..d15 cs A5
        4) Write-Read  (key '4') write reg=0x60, len=0
           5A 60 00 cs A5
    */
    MD_STATUS ret;
    unsigned short frame_len;
    unsigned char frame[I2C_MASTER_FRAME_MAX_LEN];

    if (payload_len > I2C_MASTER_MAX_PAYLOAD)
    {
        return MD_ARGERROR;
    }

    if ((payload == NULL) && (payload_len > 0U))
    {
        return MD_ARGERROR;
    }

    frame[0U] = I2C_PACKET_HEADER;
    frame[1U] = reg_addr;
    frame[2U] = payload_len;

    if (payload_len > 0U)
    {
        memcpy(&frame[3U], payload, payload_len);
    }

    frame[3U + payload_len] = I2C_packet_checksum(&frame[1U], (unsigned short)(payload_len + 2U)); /* reg + len + data */
    frame[4U + payload_len] = I2C_PACKET_TAIL;
    frame_len = (unsigned short)(payload_len + 5U);

    drv_set_IIC_Master_send_flag(1U);
    ret = drv_IIC_Master_write(device_addr, frame, frame_len);
    if (ret != MD_OK)
    {
        drv_set_IIC_Master_send_flag(0U);
        return ret;
    }

    ret = drv_IIC_Master_wait_send_done();
    if (ret == MD_OK)
    {
        I2C_print_packet("[M][TX]", frame, frame_len);
    }

    return ret;
}

MD_STATUS I2C_Master_WriteThenReadPacket(unsigned char device_addr,
                                         unsigned char write_reg_addr,
                                         unsigned char *rx_buf,
                                         unsigned short rx_num)
{
    I2C_MASTER_PACKET_TRANSFER_T transfer;

    transfer.device_addr = device_addr;
    transfer.write_reg_addr = write_reg_addr;
    transfer.tx_payload = NULL;
    transfer.tx_payload_len = 0U;
    transfer.is_write_then_read = 1U;
    transfer.read_delay_ms = 1U;
    transfer.rx_buf = rx_buf;
    transfer.rx_len = rx_num;

    return I2C_Master_Transfer(&transfer);
}
