#ifndef __I2C_MASTER_DRIVER_H__
#define __I2C_MASTER_DRIVER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "r_smc_entry.h"
#include "r_cg_macrodriver.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
#define I2C_MASTER_TICK_WRAP_MS                       (60000U)
#define I2C_MASTER_SEND_TIMEOUT_MS                    (30U)
#define I2C_MASTER_RECV_TIMEOUT_MS                    (30U)
#define I2C_PACKET_HEADER                             (0x5AU)
#define I2C_PACKET_TAIL                               (0xA5U)
#define I2C_MASTER_MAX_PAYLOAD                        (32U)
#define I2C_MASTER_RX_CHECKSUM_ENABLE                 (0U)    /* 0: disable, 1: enable */
#define I2C_WRITE_ONLY_REG                            (0x30U)
#define I2C_WRITE_THEN_READ_REG                       (0x40U)
#define I2C_WRITE_THEN_READ_TARGET_REG                (0x41U)
#define I2C_WRITE_ONLY_REG_EXT                        (0x50U)
#define I2C_WRITE_THEN_READ_REG_EXT                   (0x60U)
#define I2C_WRITE_THEN_READ_TARGET_REG_EXT            (0x61U)

typedef struct _i2c_master_packet_transfer_t
{
    unsigned char device_addr;        /* 7-bit slave address */
    unsigned char write_reg_addr;
    unsigned char *tx_payload;
    unsigned char tx_payload_len;
    unsigned char is_write_then_read; /* 0: write only, 1: write then read */
    unsigned short read_delay_ms;     /* used only when is_write_then_read = 1 */
    unsigned char *rx_buf;
    unsigned short rx_len;
} I2C_MASTER_PACKET_TRANSFER_T;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Master_Init(void);
MD_STATUS I2C_Master_Transfer(const I2C_MASTER_PACKET_TRANSFER_T *transfer);
MD_STATUS I2C_Master_SendWriteOnlyPacket(unsigned char device_addr,
                                         unsigned char reg_addr,
                                         unsigned char *payload,
                                         unsigned char payload_len);
MD_STATUS I2C_Master_WriteThenReadPacket(unsigned char device_addr,
                                         unsigned char write_reg_addr,
                                         unsigned char *rx_buf,
                                         unsigned short rx_num);

MD_STATUS drv_IIC_Master_read(unsigned char adr, unsigned char * const rx_buf, unsigned short rx_num);
MD_STATUS drv_IIC_Master_write(unsigned char adr, unsigned char * const tx_buf, unsigned short tx_num);
MD_STATUS IICA0_read(unsigned char device_addr, unsigned char reg_addr, unsigned char* rx_xfer_data, unsigned short rx_num);
MD_STATUS IICA0_write(unsigned char device_addr, unsigned char reg_addr, unsigned char* tx_xfer_data, unsigned short tx_num);

void drv_IIC_Master_callback_error(unsigned char err);
void drv_set_IIC_Master_receive_flag(bool flag);
bool drv_get_IIC_Master_receive_flag(void);
void drv_set_IIC_Master_send_flag(bool flag);
bool drv_get_IIC_Master_send_flag(void);


#endif //__I2C_MASTER_DRIVER_H__
