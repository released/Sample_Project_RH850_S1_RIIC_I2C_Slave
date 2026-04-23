/*_____ I N C L U D E S ____________________________________________________*/
// #include <stdio.h>
#include <string.h>
#include "r_smc_entry.h"

#include "misc_config.h"
#include "custom_func.h"
#include "I2C_master_driver.h"
#include "I2C_slave_driver.h"
#include "retarget.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

volatile struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_SPECIFIC           	    (flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 	        	(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                    		    (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


#define FLAG_PROJ_TRIG_1                                (flag_PROJ_CTL.bit8)
#define FLAG_PROJ_TRIG_2                                (flag_PROJ_CTL.bit9)
#define FLAG_PROJ_TRIG_3                                (flag_PROJ_CTL.bit10)
#define FLAG_PROJ_TRIG_4                                (flag_PROJ_CTL.bit11)
#define FLAG_PROJ_TRIG_5                                (flag_PROJ_CTL.bit12)
#define FLAG_PROJ_REVERSE13                             (flag_PROJ_CTL.bit13)
#define FLAG_PROJ_REVERSE14                             (flag_PROJ_CTL.bit14)
#define FLAG_PROJ_REVERSE15                             (flag_PROJ_CTL.bit15)

/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned short counter_tick = 0U;
volatile unsigned long ostmr_tick = 0U;

#define BTN_PRESSED_LONG                                (2500U)
#define I2C_TARGET_ADDR                                 (0x70U)   /* 7-bit I2C slave address */
#define I2C_CMD1_PAYLOAD_LEN                            (4U)
#define I2C_CMD3_PAYLOAD_LEN                            (8U)
#define I2C_CMD2_RX_FRAME_LEN                           (I2C_SLAVE_READ_DATA_LEN_SHORT + 4U)
#define I2C_CMD4_RX_FRAME_LEN                           (I2C_SLAVE_READ_DATA_LEN_LONG + 4U)
#define I2C_PERIOD_SEND_INTERVAL_MS                     (500U)
#define APP_TICK_WRAP_MS                                (60000U)

/*
    [Packet description]
    Common TX packet format:
    5A, reg, len, data_0 ... data_n, checksum, A5
    checksum = 1-byte sum of (reg + len + data_0 ... data_n)

    This sample uses 4 command registers:
    - reg 0x30 (key '1'): Write-only
      TX: 5A 30 04 data0 data1 data2 data3 cs A5

    - reg 0x40 / 0x41 (key '2'): Write-then-Read
      TX(write): 5A 40 00 cs A5
      RX(read) : 41 04 data0 data1 data2 data3 cs A5

    - reg 0x50 (key '3'): Write-only, 8-byte payload
      TX: 5A 50 08 data8 ... data15 cs A5
      note: data15 auto-increments by 1 each key press

    - reg 0x60 / 0x61 (key '4'): Write-then-Read, 16-byte payload response
      TX(write): 5A 60 00 cs A5
      RX(read) : 61 10 data0 ... data15 cs A5
*/

#pragma section privateData

const unsigned char dummy_3 = 0x5AU;

volatile unsigned char dummy_2 = 0xFFU;

volatile unsigned char dummy_1;

#pragma section default

volatile unsigned long g_u32_counter = 0U;

volatile UART_MANAGER_T UART0Manager = 
{
	.g_uart0rxbuf = 0U,                                         /* UART0 receive buffer */
	.g_uart0rxerr = 0U,                                         /* UART0 receive error status */
};

#if (APP_I2C_MASTER_ENABLED == 1U)
static volatile unsigned char g_i2c_cmd_trig1 = 0U;
static volatile unsigned char g_i2c_cmd_trig2 = 0U;
static volatile unsigned char g_i2c_cmd_trig3 = 0U;
static volatile unsigned char g_i2c_cmd_trig4 = 0U;
static unsigned char g_i2c_periodic_enabled = 0U;
static unsigned char g_i2c_periodic_cmd = 0U;          /* 0:none, 1..4: command id */
static unsigned short g_i2c_periodic_last_tick = 0U;
#endif

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
#if (APP_I2C_MASTER_ENABLED == 1U)
static unsigned short app_elapsed_ms(unsigned short start, unsigned short now)
{
    if (now >= start)
    {
        return (unsigned short)(now - start);
    }

    return (unsigned short)((APP_TICK_WRAP_MS - start) + now);
}

static void print_packet_local(const char *tag, const unsigned char *buf, unsigned short len)
{
    unsigned short i;

    tiny_printf("%s len=%d :", tag, len);
    for (i = 0U; i < len; i++)
    {
        tiny_printf(" %02X", buf[i]);
    }
    tiny_printf("\r\n");
}

static MD_STATUS app_i2c_transfer(I2C_MASTER_PACKET_TRANSFER_T *transfer,
                                  const char *fail_tag,
                                  const char *rx_tag)
{
    MD_STATUS ret;

    ret = I2C_Master_Transfer(transfer);
    if (ret != MD_OK)
    {
        tiny_printf("%s failed:0x%02X\r\n", fail_tag, ret);
        return ret;
    }

    if ((transfer->is_write_then_read != 0U) && (rx_tag != NULL))
    {
        print_packet_local(rx_tag, transfer->rx_buf, transfer->rx_len);
    }

    return MD_OK;
}

static void handle_i2c_cmd_1_write_only_0x30(void)
{
    unsigned char payload[I2C_CMD1_PAYLOAD_LEN] = {0x11U, 0x22U, 0x33U, 0x44U};
    static unsigned char counter = 0U;
    I2C_MASTER_PACKET_TRANSFER_T transfer;

    payload[I2C_CMD1_PAYLOAD_LEN - 1U] = counter++;

    transfer.device_addr = I2C_TARGET_ADDR;
    transfer.write_reg_addr = I2C_WRITE_ONLY_REG;
    transfer.tx_payload = payload;
    transfer.tx_payload_len = I2C_CMD1_PAYLOAD_LEN;
    transfer.is_write_then_read = 0U;
    transfer.read_delay_ms = 0U;
    transfer.rx_buf = NULL;
    transfer.rx_len = 0U;

    (void)app_i2c_transfer(&transfer, "[APP][CMD1]", NULL);
}

static void handle_i2c_cmd_2_write_then_read_0x40_0x41(void)
{
    unsigned char rx_frame[I2C_CMD2_RX_FRAME_LEN] = {0U};
    I2C_MASTER_PACKET_TRANSFER_T transfer;

    transfer.device_addr = I2C_TARGET_ADDR;
    transfer.write_reg_addr = I2C_WRITE_THEN_READ_REG;
    transfer.tx_payload = NULL;
    transfer.tx_payload_len = 0U;
    transfer.is_write_then_read = 1U;
    transfer.read_delay_ms = 1U;
    transfer.rx_buf = rx_frame;
    transfer.rx_len = I2C_CMD2_RX_FRAME_LEN;

    (void)app_i2c_transfer(&transfer, "[APP][CMD2]", "[APP][CMD2][RX]");
}

static void handle_i2c_cmd_3_write_only_0x50(void)
{
    unsigned char payload[I2C_CMD3_PAYLOAD_LEN] = {0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU, 0x00U};
    static unsigned char data_15_counter = 0U;
    I2C_MASTER_PACKET_TRANSFER_T transfer;

    payload[I2C_CMD3_PAYLOAD_LEN - 1U] = data_15_counter++;

    transfer.device_addr = I2C_TARGET_ADDR;
    transfer.write_reg_addr = I2C_WRITE_ONLY_REG_EXT;
    transfer.tx_payload = payload;
    transfer.tx_payload_len = I2C_CMD3_PAYLOAD_LEN;
    transfer.is_write_then_read = 0U;
    transfer.read_delay_ms = 0U;
    transfer.rx_buf = NULL;
    transfer.rx_len = 0U;

    (void)app_i2c_transfer(&transfer, "[APP][CMD3]", NULL);
}

static void handle_i2c_cmd_4_write_then_read_0x60_0x61(void)
{
    unsigned char rx_frame[I2C_CMD4_RX_FRAME_LEN] = {0U};
    I2C_MASTER_PACKET_TRANSFER_T transfer;

    transfer.device_addr = I2C_TARGET_ADDR;
    transfer.write_reg_addr = I2C_WRITE_THEN_READ_REG_EXT;
    transfer.tx_payload = NULL;
    transfer.tx_payload_len = 0U;
    transfer.is_write_then_read = 1U;
    transfer.read_delay_ms = 1U;
    transfer.rx_buf = rx_frame;
    transfer.rx_len = I2C_CMD4_RX_FRAME_LEN;

    (void)app_i2c_transfer(&transfer, "[APP][CMD4]", "[APP][CMD4][RX]");
}

static void i2c_send_by_cmd(unsigned char cmd_id)
{
    switch (cmd_id)
    {
        case 1U:
            handle_i2c_cmd_1_write_only_0x30();
            break;
        case 2U:
            handle_i2c_cmd_2_write_then_read_0x40_0x41();
            break;
        case 3U:
            handle_i2c_cmd_3_write_only_0x50();
            break;
        case 4U:
            handle_i2c_cmd_4_write_then_read_0x60_0x61();
            break;
        default:
            break;
    }
}

static void i2c_toggle_periodic_cmd(unsigned char cmd_id)
{
    if ((g_i2c_periodic_enabled != 0U) && (g_i2c_periodic_cmd == cmd_id))
    {
        g_i2c_periodic_enabled = 0U;
        g_i2c_periodic_cmd = 0U;
        tiny_printf("[APP] periodic CMD%d disabled\r\n", cmd_id);
        return;
    }

    g_i2c_periodic_enabled = 1U;
    g_i2c_periodic_cmd = cmd_id;
    g_i2c_periodic_last_tick = get_tick();

    tiny_printf("[APP] periodic CMD%d enabled (500ms)\r\n", cmd_id);
    i2c_send_by_cmd(cmd_id);                             /* send immediately once */
    g_i2c_periodic_last_tick = get_tick();
}

static void i2c_periodic_task(void)
{
    unsigned short now;

    if ((g_i2c_periodic_enabled == 0U) || (g_i2c_periodic_cmd == 0U))
    {
        return;
    }

    now = get_tick();
    if (app_elapsed_ms(g_i2c_periodic_last_tick, now) >= I2C_PERIOD_SEND_INTERVAL_MS)
    {
        i2c_send_by_cmd(g_i2c_periodic_cmd);
        g_i2c_periodic_last_tick = now;
    }
}
#endif



void ostmr_tick_counter(void)
{
	ostmr_tick++;
}

void ostmr_1ms_IRQ(void)
{
	ostmr_tick_counter();
}

void ostimer_dealyms(unsigned long ms)
{
    R_Config_OSTM0_Start();
    ostmr_tick = 0U;

    while(ostmr_tick < ms);

    R_Config_OSTM0_Stop();

}

unsigned short get_tick(void)
{
	return (counter_tick);
}

void set_tick(unsigned short t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000U)
    {
        set_tick(0U);
    }
}

void delay_ms(unsigned long ms)
{
    unsigned long tickstart = get_tick();
    unsigned long wait = ms;
	unsigned long tmp = 0U;
	
    while (1)
    {
		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
		{
			tmp = get_tick() - tickstart;
		}
		else // tickstart = 59000 , tick_counter = 2048
		{
			tmp = 60000U -  tickstart + get_tick();
		}		
		
		if (tmp > wait)
			break;
    }
}

unsigned char R_PORT_GetGPIOLevel(unsigned short n,unsigned char Pin)
{
    unsigned short PortLevel;

    switch(n)
    {
        case 0U:
            PortLevel = PORT.PPR0;
            break;
        case 8U:
            PortLevel = PORT.PPR8;
            break;
        case 9U:
            PortLevel = PORT.PPR9;
            break;
        case 10U:
            PortLevel = PORT.PPR10;
            break;
        case 11U:
            PortLevel = PORT.PPR11;
            break;
        case 0x2C8U:
            PortLevel = PORT.APPR0;
            break;
    }
    PortLevel &= 1U<<Pin;
    
    if(PortLevel == 0U)
    {
        return 0U;
    }
    else
    {
        return 1U;
    }
}
void tmr_1ms_IRQ(void)
{
    tick_counter();

    if ((get_tick() % 1000U) == 0U)
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 1U;
    }

    if ((get_tick() % 250U) == 0U)
    {
        FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 1U;
    }

    if ((get_tick() % 50U) == 0U)
    {

    }	

}

void LED_Toggle(void)
{
    static unsigned char flag_gpio = 0U;
		
    GPIO_TOGGLE(0,14);//PORT.PNOT0 |= 1u<<14;
	
	if (!flag_gpio)
	{
		flag_gpio = 1U;
        GPIO_HIGH(P8,5);//PORT.P8 |= 1u<<5;
	}
	else
	{
		flag_gpio = 0U;
		GPIO_LOW(P8,5);//PORT.P8 &= ~(1u<<5);
	}	
}

void loop(void)
{
	// static unsigned long LOG1 = 0U;

#if (APP_I2C_SLAVE_ENABLED == 1U)
    IICA0_slave_Task();
#endif

#if (APP_I2C_MASTER_ENABLED == 1U)
    if (g_i2c_cmd_trig1 != 0U)
    {
        g_i2c_cmd_trig1 = 0U;
        i2c_toggle_periodic_cmd(1U);
    }

    if (g_i2c_cmd_trig2 != 0U)
    {
        g_i2c_cmd_trig2 = 0U;
        i2c_toggle_periodic_cmd(2U);
    }

    if (g_i2c_cmd_trig3 != 0U)
    {
        g_i2c_cmd_trig3 = 0U;
        i2c_toggle_periodic_cmd(3U);
    }

    if (g_i2c_cmd_trig4 != 0U)
    {
        g_i2c_cmd_trig4 = 0U;
        i2c_toggle_periodic_cmd(4U);
    }

    i2c_periodic_task();
#endif

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0U;

        g_u32_counter++;
        LED_Toggle();   
        // tiny_printf("timer:%4d\r\n",LOG1++);
    }

    if (FLAG_PROJ_TIMER_PERIOD_SPECIFIC)
    {
        FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 0U;
    }
}

void UARTx_ErrorCheckProcess(unsigned char err)
{
    if (err)          /* Check reception error */
    {   
        /* Reception error */
        switch(err)
        {
            case _UART_PARITY_ERROR_FLAG:   /* Parity error */
                tiny_printf("uart rx:Parity Error Flag\r\n");
                break;
            case _UART_FRAMING_ERROR_FLAG:  /* Framing error */
                tiny_printf("uart rx:Framing Error Flag\r\n");
                break;
            case _UART_OVERRUN_ERROR_FLAG:  /* Overrun error */
                tiny_printf("uart rx:Overrun Error Flag\r\n");
                break;
            case _UART_BIT_ERROR_FLAG:      /* Bit error */
                tiny_printf("uart rx:Bit Error Flag\r\n");
                break;
        }
        UART0Manager.g_uart0rxerr = 0U;
    }
}

void UARTx_Process(unsigned char rxbuf)
{    
    if (rxbuf == 0x00U)
    {
        return;
    }

    if (rxbuf > 0x7FU)
    {
        tiny_printf("invalid command\r\n");
    }
    else
    {
        tiny_printf("press:%c(0x%02X)\r\n" , rxbuf,rxbuf);   // %c :  C99 libraries.
        switch(rxbuf)
        {
#if (APP_I2C_MASTER_ENABLED == 1U)
            case '1':
                g_i2c_cmd_trig1 = 1U;
                break;
            case '2':
                g_i2c_cmd_trig2 = 1U;
                break;
            case '3':
                g_i2c_cmd_trig3 = 1U;
                break;
            case '4':
                g_i2c_cmd_trig4 = 1U;
                break;
#else
            case '1':
            case '2':
            case '3':
            case '4':
                tiny_printf("I2C command is disabled in SLAVE_ONLY mode\r\n");
                break;
#endif
            case '5':
                FLAG_PROJ_TRIG_5 = 1U;
                break;

            case 'X':
            case 'x':
            case 'Z':
            case 'z':
                RH850_software_reset();
                break;

            default:       
                // exception
                break;                
        }
    }
}

void RH850_software_reset(void)
{
    unsigned long  reg32_value;

    reg32_value = 0x00000001UL;
    WPROTR.PROTCMD0 = _WRITE_PROTECT_COMMAND;
    RESCTL.SWRESA = reg32_value;
    RESCTL.SWRESA = (unsigned long) ~reg32_value;
    RESCTL.SWRESA = reg32_value;
    while (WPROTR.PROTS0 != reg32_value)
    {
        NOP();
    }
}

void RLIN3_UART_SendChar(unsigned char c)
{
    /*
        UTS : 0 - transmission is not in progress    
    */
    while (((RLN30.LST & _UART_TRANSMISSION_OPERATED) != 0U));    
    RLN30.LUTDR.UINT16 = c;
    // RLN30.LUTDR.UINT8[L] = (unsigned char) c;  
}

void SendChar(unsigned char ch)
{
    RLIN3_UART_SendChar(ch);
}

void hardware_init(void)
{
    EI();

#if (APP_I2C_MASTER_ENABLED == 1U)
    g_i2c_cmd_trig1 = 0U;
    g_i2c_cmd_trig2 = 0U;
    g_i2c_cmd_trig3 = 0U;
    g_i2c_cmd_trig4 = 0U;
    g_i2c_periodic_enabled = 0U;
    g_i2c_periodic_cmd = 0U;
    g_i2c_periodic_last_tick = 0U;
#endif

    R_Config_TAUJ0_0_Start();
    R_Config_OSTM0_Start();

    /*
        LED : 
            - LED18 > P0_14
            - LED17 > P8_5 
        UART : 
            - TX > P10_10
            - RX > P10_9    
    */
    R_Config_UART0_Receive((uint8_t *)&UART0Manager.g_uart0rxbuf, 1U);
    R_Config_UART0_Start();

    /*
        RIIC1 : I2C slave
        RIIC1SCL : P8_1
        RIIC1SDA : P8_0
    */
#if (APP_I2C_SLAVE_ENABLED == 1U)
    R_Config_RIIC1_Start();
    IICA0_slave_Init();
#endif

    /*
        RIIC0 : I2C master
        RIIC0SCL : P10_3
        RIIC0SDA : P10_2
    */
#if (APP_I2C_MASTER_ENABLED == 1U)
    R_Config_RIIC0_Start();
    I2C_Master_Init();
#endif

    tiny_printf("I2C target address : 0x%02X\r\n", I2C_TARGET_ADDR);
#if (APP_I2C_ROLE == APP_I2C_ROLE_MASTER_ONLY)
    tiny_printf("I2C app mode : MASTER_ONLY\r\n");
#elif (APP_I2C_ROLE == APP_I2C_ROLE_SLAVE_ONLY)
    tiny_printf("I2C app mode : SLAVE_ONLY\r\n");
#else
    tiny_printf("I2C app mode : DUAL\r\n");
#endif

#if (APP_I2C_MASTER_ENABLED == 1U)
    tiny_printf("press '1' : toggle periodic 500ms write-only      reg=0x30\r\n");
    tiny_printf("press '2' : toggle periodic 500ms write-then-read reg=0x40 -> read 0x41\r\n");
    tiny_printf("press '3' : toggle periodic 500ms write-only      reg=0x50\r\n");
    tiny_printf("press '4' : toggle periodic 500ms write-then-read reg=0x60 -> read 0x61\r\n");
#else
    tiny_printf("master command is disabled in this build\r\n");
#endif

#if (APP_I2C_SLAVE_ENABLED == 1U)
    tiny_printf("[SLAVE TEST] external master (Pico) can use:\r\n");
    tiny_printf("  slave rx checksum check : %s\r\n",
                (I2C_SLAVE_RX_CHECKSUM_ENABLE == 1U) ? "enable" : "disable");
    tiny_printf("  write-only      : 5A 30 04 d0 d1 d2 d3 cs A5  (or reg 0x50 len=8)\r\n");
    tiny_printf("  write-then-read : write 5A 40 00 cs A5,\r\n");
    tiny_printf("                    then read frame 8 bytes (len=0x04, data=4 bytes)\r\n");
    tiny_printf("                    response: 41 04 data0 data1 data2 data3 cs A5\r\n");
    tiny_printf("  write-then-read : write 5A 60 00 cs A5,\r\n");
    tiny_printf("                    then read frame 20 bytes (len=0x10, data=16 bytes)\r\n");
    tiny_printf("                    response: 61 10 data0 ... data15 cs A5\r\n");
#endif

    tiny_printf("\r\nhardware_init rdy\r\n");

}
