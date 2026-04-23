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
* File Name        : Config_RIIC1_user.c
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
#include "I2C_slave_driver.h"
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile uint8_t          g_riic1_mode_flag;               /* RIIC1 master transmit receive flag */
extern volatile uint8_t          g_riic1_state;                   /* RIIC1 master state */
extern volatile const uint8_t *  gp_riic1_tx_address;             /* RIIC1 transmit buffer address */
extern volatile uint16_t         g_riic1_tx_count;                /* RIIC1 transmit data number */
extern volatile uint8_t *        gp_riic1_rx_address;             /* RIIC1 receive buffer address */
extern volatile uint16_t         g_riic1_rx_count;                /* RIIC1 receive data number */
extern volatile uint16_t         g_riic1_rx_length;               /* RIIC1 receive data length */
extern volatile uint8_t          g_riic1_dummy_read_count;        /* RIIC1 count for dummy read */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_RIIC1_Create_UserInit
* Description  : This function adds user code after initializing RIIC1 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_RIIC1_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC1_callback_transmitend
* Description  : This function is RIIC1 sendend callback service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void r_Config_RIIC1_callback_transmitend(void)
{
    /* Start user code for r_Config_RIIC1_callback_transmitend. Do not edit comment generated here */
    IICA0_slave_on_transmit_end();
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC1_callback_receiveend
* Description  : This function is RIIC1 receiveend callback service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void r_Config_RIIC1_callback_receiveend(void)
{
    /* Start user code for r_Config_RIIC1_callback_receiveend. Do not edit comment generated here */
    IICA0_slave_on_receive_end();
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC1_callback_receiveerror
* Description  : This function is RIIC1 error callback service routine.
* Arguments    : status -
*                    error id
* Return Value : None
***********************************************************************************************************************/
void r_Config_RIIC1_callback_receiveerror(MD_STATUS status)
{
    /* Start user code for r_Config_RIIC1_callback_receiveerror. Do not edit comment generated here */
    IICA0_slave_on_error((unsigned char)status);
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC1_error_interrupt
* Description  : This function is RIIC1 error interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC1_error_interrupt(enable=false, channel=241, fpu=true, callt=false)
void r_Config_RIIC1_error_interrupt(void)
{
#if (I2C_RIIC1_CUSTOM_ERROR_ISR_ENABLE == 1U)
    /*
        NOTE:
        This project uses custom slave RIIC1 error ISR handler:
        I2C_Slave_RIIC1_ErrorISR_Custom().
        Keep this dispatch in r_Config_RIIC1_error_interrupt().
    */
    (void)I2C_Slave_RIIC1_ErrorISR_Custom();
#else    
    volatile uint8_t dummy;

    if ((RIIC1.IER.UINT32 & _RIIC_ARBITRATION_LOST_INT_ENABLE) && (RIIC1.SR2.UINT32 & _RIIC_ARBITRATION_LOST))
    {
        r_Config_RIIC1_callback_receiveerror(MD_ERROR1);
    }
    else if ((RIIC1.IER.UINT32 & _RIIC_TIMEOUT_INT_ENABLE) && (RIIC1.SR2.UINT32 & _RIIC_TIMEOUT_DETECTED))
    {
        r_Config_RIIC1_callback_receiveerror(MD_ERROR2);
    }
    else if ((RIIC1.IER.UINT32 & _RIIC_NACK_RECEPTION_INT_ENABLE) && (RIIC1.SR2.UINT32 & _RIIC_NACK_DETECTED))
    {
        if (_RIIC_SLAVE_TRANSMIT == g_riic1_mode_flag)
        {
            /* Dummy read to release SCL */
            dummy = RIIC1.DRR.UINT32;
            while ((RIIC1.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED) != _RIIC_STOP_CONDITION_DETECTED)
            {
                NOP();
            }
            RIIC1.SR2.UINT32 &= (uint32_t) ~_RIIC_NACK_DETECTED;
            RIIC1.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC1.IER.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            if((RIIC1.SR2.UINT32 & _RIIC_TRANSMIT_END) == _RIIC_TRANSMIT_END)
            {
                r_Config_RIIC1_callback_transmitend();
            }
            else
            {
                r_Config_RIIC1_callback_receiveerror(MD_ERROR3);
            }
        }
    }
    else if (_RIIC_SLAVE_RECEIVE == g_riic1_mode_flag)
    {
        if (_RIIC_SLAVE_RECEIVES_STOP == g_riic1_state)
        {
            RIIC1.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC1.IER.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            r_Config_RIIC1_callback_receiveend();
        }
        else if (_RIIC_SLAVE_WAIT_START_CONDITION == g_riic1_state)
        {
            RIIC1.SR2.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_DETECTED;
            RIIC1.IER.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
            g_riic1_state = _RIIC_SLAVE_RECEIVES_DATA;
        }
    }
    else if (_RIIC_SLAVE_TRANSMIT == g_riic1_mode_flag)
    {
        if (_RIIC_SLAVE_SENDS_STOP == g_riic1_state)
        {
            RIIC1.SR2.UINT32 &= (uint32_t) ~_RIIC_NACK_DETECTED;
            RIIC1.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC1.IER.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
            r_Config_RIIC1_callback_transmitend();
        }
        else if (_RIIC_SLAVE_WAIT_START_CONDITION == g_riic1_state)
        {
            RIIC1.SR2.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_DETECTED;
            RIIC1.IER.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_INT_ENABLE;
            RIIC1.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
            g_riic1_state = _RIIC_SLAVE_SENDS_DATA;
        }
    }
#endif    
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC1_transmit_interrupt
* Description  : This function is RIIC1 send interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC1_transmit_interrupt(enable=false, channel=240, fpu=true, callt=false)
void r_Config_RIIC1_transmit_interrupt(void)
{
    if (_RIIC_SLAVE_TRANSMIT == g_riic1_mode_flag)
    {
        if (_RIIC_SLAVE_SENDS_DATA == g_riic1_state)
        {
            RIIC1.DRT.UINT32 = *gp_riic1_tx_address;
            gp_riic1_tx_address++;
            g_riic1_tx_count--;

            if (0U == g_riic1_tx_count)
            {
                g_riic1_state = _RIIC_SLAVE_SENDS_END;
            }
        }
    }
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC1_transmitend_interrupt
* Description  : This function is RIIC1 send end interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC1_transmitend_interrupt(enable=false, channel=243, fpu=true, callt=false)
void r_Config_RIIC1_transmitend_interrupt(void)
{
    volatile uint8_t dummy;
    if (_RIIC_SLAVE_SENDS_END == g_riic1_state)
    {
        g_riic1_state = _RIIC_SLAVE_SENDS_STOP;
        /* Dummy read to release SCL */
        dummy = RIIC1.DRR.UINT32;
    }
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC1_receive_interrupt
* Description  : This function is RIIC1 receive interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC1_receive_interrupt(enable=false, channel=242, fpu=true, callt=false)
void r_Config_RIIC1_receive_interrupt(void)
{
    volatile uint8_t dummy;
    uint16_t length;

    if (_RIIC_SLAVE_RECEIVES_DATA == g_riic1_state)
    {
        if (g_riic1_dummy_read_count < 1U)
        {
            dummy = RIIC1.DRR.UINT32;
            g_riic1_dummy_read_count++;
            return;
        }

        RIIC1.MR3.UINT32 |= _RIIC_ACKBT_BIT_MODIFICATION_ENABLED;
        RIIC1.MR3.UINT32 &= (uint32_t) ~_RIIC_NACK_TRANSMISSION;
        *gp_riic1_rx_address = RIIC1.DRR.UINT32;
        gp_riic1_rx_address++;
        g_riic1_rx_count++;
        length = g_riic1_rx_length;

        if (RIIC1.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED)
        {
            /* check stop request */
            g_riic1_state = _RIIC_SLAVE_RECEIVES_STOP;
        }
        else if (g_riic1_rx_count == length)
        {
            g_riic1_state = _RIIC_SLAVE_RECEIVES_STOP;
        }
    }
    else if (_RIIC_SLAVE_SENDS_DATA == g_riic1_state)
    {
        dummy = RIIC1.DRR.UINT32;
    }
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

