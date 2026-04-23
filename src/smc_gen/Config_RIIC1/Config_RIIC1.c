/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2018, 2024 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Config_RIIC1.c
* Component Version: 1.6.0
* Device(s)        : R7F701684
* Description      : This file implements device driver for Config_RIIC1.
***********************************************************************************************************************/
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_userdefine.h"
#include "Config_RIIC1.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
volatile uint8_t          g_riic1_mode_flag;               /* RIIC1 master transmit receive flag */
volatile uint8_t          g_riic1_state;                   /* RIIC1 state */
volatile uint8_t *        gp_riic1_tx_address;             /* RIIC1 transmit buffer address */
volatile uint16_t         g_riic1_tx_count;                /* RIIC1 transmit data number */
volatile uint8_t *        gp_riic1_rx_address;             /* RIIC1 receive buffer address */
volatile uint16_t         g_riic1_rx_count;                /* RIIC1 receive data number */
volatile uint16_t         g_riic1_rx_length;               /* RIIC1 receive data length */
volatile uint8_t          g_riic1_dummy_read_count;        /* RIIC1 count for dummy read */
extern volatile uint32_t  g_cg_sync_read;
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_Create
* Description  : This function initializes the RIIC Bus Interface.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_RIIC1_Create(void)
{
    uint32_t tmp_port;

    /* Disable RIIC1 interrupt (INTRIIC1TI) operation and clear request */
    INTC2.ICRIIC1TI.BIT.MKRIIC1TI = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1TI.BIT.RFRIIC1TI = _INT_REQUEST_NOT_OCCUR;
    /* Disable RIIC1 interrupt (INTRIIC1TEI) operation and clear request */
    INTC2.ICRIIC1TEI.BIT.MKRIIC1TEI = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1TEI.BIT.RFRIIC1TEI = _INT_REQUEST_NOT_OCCUR;
    /* Disable RIIC1 interrupt (INTRIIC1RI) operation and clear request */
    INTC2.ICRIIC1RI.BIT.MKRIIC1RI = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1RI.BIT.RFRIIC1RI = _INT_REQUEST_NOT_OCCUR;
    /* Disable RIIC1 interrupt (INTRIIC1EE) operation and clear request */
    INTC2.ICRIIC1EE.BIT.MKRIIC1EE = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1EE.BIT.RFRIIC1EE = _INT_REQUEST_NOT_OCCUR;
    /* Set RIIC1 interrupt (INTRIIC1TI) setting */
    INTC2.ICRIIC1TI.BIT.TBRIIC1TI = _INT_TABLE_VECTOR;
    INTC2.ICRIIC1TI.UINT16 &= _INT_PRIORITY_LOWEST;
    /* Set RIIC1 interrupt (INTRIIC1TEI) setting */
    INTC2.ICRIIC1TEI.BIT.TBRIIC1TEI = _INT_TABLE_VECTOR;
    INTC2.ICRIIC1TEI.UINT16 &= _INT_PRIORITY_LOWEST;
    /* Set RIIC1 interrupt (INTRIIC1RI) setting */
    INTC2.ICRIIC1RI.BIT.TBRIIC1RI = _INT_TABLE_VECTOR;
    INTC2.ICRIIC1RI.UINT16 &= _INT_PRIORITY_LOWEST;
    /* Set RIIC1 interrupt (INTRIIC1EE) setting */
    INTC2.ICRIIC1EE.BIT.TBRIIC1EE = _INT_TABLE_VECTOR;
    INTC2.ICRIIC1EE.UINT16 &= _INT_PRIORITY_LOWEST;
    /* Reset RIIC1 */
    RIIC1.CR1.UINT32 &= _RIIC_DISABLE;
    RIIC1.CR1.UINT32 |= _RIIC_RESET;
    RIIC1.CR1.UINT32 |= _RIIC_INTERNAL_RESET;
    /* Set RIIC1 setting */
    RIIC1.SAR0.UINT32 = _RIIC_ADDRESS_FORMAT_7BITS | _RIIC1_SLAVE_ADDRESS0;
    RIIC1.SER.UINT32 = _RIIC_DEVICEID_DETECT_DISABLE | _RIIC_GENERAL_CALL_ADDRESS_DISABLE | _RIIC_SLAVE_2_DISABLE | 
                       _RIIC_SLAVE_1_DISABLE | _RIIC_SLAVE_0_ENABLE;
    RIIC1.MR1.UINT32 = _RIIC_CLOCK_SELECTION_1;
    RIIC1.BRL.UINT32 = _RIIC_RIICBRL_DEFAULT_VALUE | _RIIC1_BITRATE_LOW_LEVEL_PERIOD;
    RIIC1.MR2.UINT32 = _RIIC_TIMEOUT_LOW_COUNT_ENABLE | _RIIC_TIMEOUT_HIGH_COUNT_ENABLE | 
                       _RIIC_TIMEOUT_DETECTION_MODE_LONG;
    RIIC1.MR3.UINT32 = _RIIC_DIGITAL_NF_STAGE_SINGLE;
    RIIC1.FER.UINT32 = _RIIC_SCL_SYNC_CIRCUIT_USED | _RIIC_NOISE_FILTER_USED | _RIIC_TRANSFER_SUSPENSION_ENABLED | 
                       _RIIC_NACK_ARBITRATION_ENABLE | _RIIC_SLAVE_ARBITRATION_ENABLE | _RIIC_TIMEOUT_FUNCTION_ENABLED;
    RIIC1.IER.UINT32 = _RIIC_TRANSMIT_DATA_EMPTY_INT_ENABLE | _RIIC_TRANSMIT_END_INT_ENABLE | 
                       _RIIC_RECEIVE_COMPLETE_INT_ENABLE | _RIIC_START_CONDITION_INT_ENABLE | 
                       _RIIC_STOP_CONDITION_INT_ENABLE | _RIIC_TIMEOUT_INT_ENABLE | 
                       _RIIC_ARBITRATION_LOST_INT_ENABLE | _RIIC_NACK_RECEPTION_INT_ENABLE;
    /* Cancel internal reset */
    RIIC1.CR1.UINT32 &= _RIIC_CLEAR_INTERNAL_RESET;
    /* Synchronization processing */
    g_cg_sync_read = RIIC1.MR1.UINT32;
    __syncp();

    /* Set RIIC1SCL pin */
    PORT.PBDC8 &= _PORT_CLEAR_BIT1;
    PORT.PM8 |= _PORT_SET_BIT1;
    PORT.PMC8 &= _PORT_CLEAR_BIT1;
    tmp_port = PORT.PODC8;
    PORT.PPCMD8 = _WRITE_PROTECT_COMMAND;
    PORT.PODC8 = (tmp_port | _PORT_SET_BIT1);
    PORT.PODC8 = (uint32_t) ~(tmp_port | _PORT_SET_BIT1);
    PORT.PODC8 = (tmp_port | _PORT_SET_BIT1);
    PORT.PBDC8 |= _PORT_SET_BIT1;
    PORT.PFC8 &= _PORT_CLEAR_BIT1;
    PORT.PFCE8 &= _PORT_CLEAR_BIT1;
    PORT.PFCAE8 |= _PORT_SET_BIT1;
    PORT.PMC8 |= _PORT_SET_BIT1;
    PORT.PM8 &= _PORT_CLEAR_BIT1;

    /* Set RIIC1SDA pin */
    PORT.PBDC8 &= _PORT_CLEAR_BIT0;
    PORT.PM8 |= _PORT_SET_BIT0;
    PORT.PMC8 &= _PORT_CLEAR_BIT0;
    tmp_port = PORT.PODC8;
    PORT.PPCMD8 = _WRITE_PROTECT_COMMAND;
    PORT.PODC8 = (tmp_port | _PORT_SET_BIT0);
    PORT.PODC8 = (uint32_t) ~(tmp_port | _PORT_SET_BIT0);
    PORT.PODC8 = (tmp_port | _PORT_SET_BIT0);
    PORT.PBDC8 |= _PORT_SET_BIT0;
    PORT.PFC8 &= _PORT_CLEAR_BIT0;
    PORT.PFCE8 &= _PORT_CLEAR_BIT0;
    PORT.PFCAE8 |= _PORT_SET_BIT0;
    PORT.PMC8 |= _PORT_SET_BIT0;
    PORT.PM8 &= _PORT_CLEAR_BIT0;

    R_Config_RIIC1_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_Start
* Description  : This function starts the RIIC1 Bus Interface.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_RIIC1_Start(void)
{
    /* Clear RIIC1 interrupt request and enable operation */
    INTC2.ICRIIC1TI.BIT.RFRIIC1TI = _INT_REQUEST_NOT_OCCUR;
    INTC2.ICRIIC1TEI.BIT.RFRIIC1TEI = _INT_REQUEST_NOT_OCCUR;
    INTC2.ICRIIC1RI.BIT.RFRIIC1RI = _INT_REQUEST_NOT_OCCUR;
    INTC2.ICRIIC1EE.BIT.RFRIIC1EE = _INT_REQUEST_NOT_OCCUR;
    INTC2.ICRIIC1TI.BIT.MKRIIC1TI = _INT_PROCESSING_ENABLED;
    INTC2.ICRIIC1TEI.BIT.MKRIIC1TEI = _INT_PROCESSING_ENABLED;
    INTC2.ICRIIC1RI.BIT.MKRIIC1RI = _INT_PROCESSING_ENABLED;
    INTC2.ICRIIC1EE.BIT.MKRIIC1EE = _INT_PROCESSING_ENABLED;
}

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_Stop
* Description  : This function stops the RIIC1 Bus Interface.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_RIIC1_Stop(void)
{
    /* Disable RIIC1 interrupt operation and clear request */
    INTC2.ICRIIC1TI.BIT.MKRIIC1TI = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1TEI.BIT.MKRIIC1TEI = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1RI.BIT.MKRIIC1RI = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1EE.BIT.MKRIIC1EE = _INT_PROCESSING_DISABLED;
    INTC2.ICRIIC1TI.BIT.RFRIIC1TI = _INT_REQUEST_NOT_OCCUR;
    INTC2.ICRIIC1TEI.BIT.RFRIIC1TEI = _INT_REQUEST_NOT_OCCUR;
    INTC2.ICRIIC1RI.BIT.RFRIIC1RI = _INT_REQUEST_NOT_OCCUR;
    INTC2.ICRIIC1EE.BIT.RFRIIC1EE = _INT_REQUEST_NOT_OCCUR;
    /* Synchronization processing */
    g_cg_sync_read = INTC2.ICRIIC1RI.UINT16;
    __syncp();
}

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_Slave_Send
* Description  : This function writes data to a slave device.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    transmit data length
* Return Value : status -
*                    MD_OK
***********************************************************************************************************************/
MD_STATUS R_Config_RIIC1_Slave_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    g_riic1_tx_count = tx_num;
    gp_riic1_tx_address = tx_buf;
    RIIC1.MR3.UINT32 |= (_RIIC_WAIT | _RIIC_RDRF_FLAG_SET_SCL_EIGHTH | _RIIC_ACKBT_BIT_MODIFICATION_ENABLED);
    RIIC1.MR3.UINT32 &= (uint32_t) ~_RIIC_NACK_TRANSMISSION;
    g_riic1_mode_flag = _RIIC_SLAVE_TRANSMIT;
    g_riic1_state = _RIIC_SLAVE_WAIT_START_CONDITION;

    return (status);
}

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_Slave_Receive
* Description  : This function reads data from a slave device.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    receive data length
* Return Value : status -
*                    MD_OK
***********************************************************************************************************************/
MD_STATUS R_Config_RIIC1_Slave_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    g_riic1_rx_length = rx_num;
    g_riic1_rx_count = 0U;
    gp_riic1_rx_address = rx_buf;
    RIIC1.MR3.UINT32 |= (_RIIC_WAIT | _RIIC_RDRF_FLAG_SET_SCL_EIGHTH | _RIIC_ACKBT_BIT_MODIFICATION_ENABLED);
    RIIC1.MR3.UINT32 &= (uint32_t) ~_RIIC_NACK_TRANSMISSION;
    g_riic1_dummy_read_count = 0U;
    g_riic1_mode_flag = _RIIC_SLAVE_RECEIVE;
    g_riic1_state = _RIIC_SLAVE_WAIT_START_CONDITION;

    return (status);
}

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_StartCondition
* Description  : This function generates RIIC1 start condition.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_RIIC1_StartCondition(void)
{
    /* Set start condition flag */
    RIIC1.CR2.UINT32 |= _RIIC_START_CONDITION_REQUEST;
}

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_StopCondition
* Description  : This function generates RIIC1 stop condition.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_RIIC1_StopCondition(void)
{
    /* Set stop condition flag */
    RIIC1.CR2.UINT32 |= _RIIC_STOP_CONDITION_REQUEST;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
