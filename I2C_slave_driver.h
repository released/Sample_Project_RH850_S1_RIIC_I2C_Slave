#ifndef __I2C_SLAVE_DRIVER_H__
#define __I2C_SLAVE_DRIVER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "r_smc_entry.h"
#include "r_cg_macrodriver.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define I2C_SLAVE_RX_BUFFER_LEN                       (64U)
#define I2C_SLAVE_TX_BUFFER_LEN                       (32U)
#define I2C_SLAVE_READ_DATA_LEN_SHORT                 (4U)
#define I2C_SLAVE_READ_DATA_LEN_LONG                  (16U)
#define I2C_SLAVE_STRETCH_LIMIT_MS                    (1U)
#define I2C_SLAVE_BUS_LOW_RECOVER_MS                  (2U)
#define I2C_SLAVE_TX_UNDERRUN_DEFAULT                 (0xFFU)
#define I2C_SLAVE_BUS_MONITOR_ENABLE                  (0U)
#define I2C_SLAVE_ERROR_AUTO_REINIT_ENABLE            (0U)
#define I2C_SLAVE_MONITOR_SCL_PORT                    (8U)
#define I2C_SLAVE_MONITOR_SCL_PIN                     (1U)
#define I2C_SLAVE_MONITOR_SDA_PORT                    (8U)
#define I2C_SLAVE_MONITOR_SDA_PIN                     (0U)

#define I2C_SLAVE_PACKET_HEADER                       (0x5AU)
#define I2C_SLAVE_PACKET_TAIL                         (0xA5U)
#define I2C_SLAVE_CMD_WRITE_ONLY_REG                  (0x30U)
#define I2C_SLAVE_CMD_WRITE_THEN_READ_REG             (0x40U)
#define I2C_SLAVE_CMD_READBACK_REG                    (0x41U)
#define I2C_SLAVE_CMD_WRITE_ONLY_REG_EXT              (0x50U)
#define I2C_SLAVE_CMD_WRITE_THEN_READ_REG_EXT         (0x60U)
#define I2C_SLAVE_CMD_READBACK_REG_EXT                (0x61U)
#define I2C_SLAVE_RX_CHECKSUM_ENABLE                  (0U)    /* 0: disable, 1: enable */
#define I2C_RIIC1_CUSTOM_ERROR_ISR_ENABLE             (1U)
#define I2C_SLAVE_HW_TIMEOUT_INT_ENABLE               (0U)

/*_____ F U N C T I O N S __________________________________________________*/
void IICA0_slave_Init(void);
void IICA0_slave_Task(void);
void IICA0_slave_on_receive_end(void);
void IICA0_slave_on_transmit_end(void);
void IICA0_slave_on_error(unsigned char err);
void IICA0_slave_on_start_detected(void);
void IICA0_slave_on_stop_detected(void);
void IICA0_slave_on_nack_detected(void);
void IICA0_slave_on_tx_underrun(void);
uint8_t I2C_Slave_RIIC1_ErrorISR_Custom(void);

#endif /* __I2C_SLAVE_DRIVER_H__ */
