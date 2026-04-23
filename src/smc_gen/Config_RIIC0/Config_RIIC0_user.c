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
* File Name        : Config_RIIC0_user.c
* Component Version: 1.6.1
* Device(s)        : R7F701684
* Description      : This file implements device driver for Config_RIIC0.
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
#include "Config_RIIC0.h"
/* Start user code for include. Do not edit comment generated here */
#include "I2C_master_driver.h"
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile uint8_t          g_riic0_mode_flag;               /* RIIC0 master transmit receive flag */
extern volatile uint8_t          g_riic0_state;                   /* RIIC0 master state */
extern volatile uint16_t         g_riic0_slave_address;           /* RIIC0 slave address */
extern volatile const uint8_t *  gp_riic0_tx_address;             /* RIIC0 transmit buffer address */
extern volatile uint16_t         g_riic0_tx_count;                /* RIIC0 transmit data number */
extern volatile uint8_t *        gp_riic0_rx_address;             /* RIIC0 receive buffer address */
extern volatile uint16_t         g_riic0_rx_count;                /* RIIC0 receive data number */
extern volatile uint16_t         g_riic0_rx_length;               /* RIIC0 receive data length */
extern volatile uint8_t          g_riic0_stop_generation;         /* RIIC0 stop condition generation flag */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_RIIC0_Create_UserInit
* Description  : This function adds user code after initializing RIIC0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_Config_RIIC0_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /*
        Disable RIIC0 timeout detection for application-level transfer timeout control.
        Smart Config enables TMOL/TMOH by default; with external-device pacing this can
        trigger MD_ERROR2(0x83) repeatedly in idle/high-level periods.
    */
    RIIC0.IER.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_INT_ENABLE;
    RIIC0.MR2.UINT32 &= (uint32_t)~(_RIIC_TIMEOUT_LOW_COUNT_ENABLE | _RIIC_TIMEOUT_HIGH_COUNT_ENABLE);
    RIIC0.FER.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_FUNCTION_ENABLED;
    RIIC0.SR2.UINT32 &= (uint32_t)~_RIIC_TIMEOUT_DETECTED;
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC0_callback_transmitend
* Description  : This function is RIIC0 sendend callback service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void r_Config_RIIC0_callback_transmitend(void)
{
    /* Start user code for r_Config_RIIC0_callback_transmitend. Do not edit comment generated here */
    drv_set_IIC_Master_send_flag(0U);
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC0_callback_receiveend
* Description  : This function is RIIC0 receiveend callback service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void r_Config_RIIC0_callback_receiveend(void)
{
    /* Start user code for r_Config_RIIC0_callback_receiveend. Do not edit comment generated here */
    drv_set_IIC_Master_receive_flag(0U);
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC0_callback_receiveerror
* Description  : This function is RIIC0 error callback service routine.
* Arguments    : status -
*                    error id
* Return Value : None
***********************************************************************************************************************/
void r_Config_RIIC0_callback_receiveerror(MD_STATUS status)
{
    /* Start user code for r_Config_RIIC0_callback_receiveerror. Do not edit comment generated here */
    drv_set_IIC_Master_send_flag(0U);
    drv_set_IIC_Master_receive_flag(0U);
    drv_IIC_Master_callback_error((unsigned char)status);
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC0_error_interrupt
* Description  : This function is RIIC0 error interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC0_error_interrupt(enable=false, channel=77, fpu=true, callt=false)
void r_Config_RIIC0_error_interrupt(void)
{
    volatile uint8_t dummy;

    if((RIIC0.IER.UINT32 & _RIIC_ARBITRATION_LOST_INT_ENABLE) && (RIIC0.SR2.UINT32 & _RIIC_ARBITRATION_LOST))
    {
        r_Config_RIIC0_callback_receiveerror(MD_ERROR1);
    }
    else if((RIIC0.IER.UINT32 & _RIIC_TIMEOUT_INT_ENABLE) && (RIIC0.SR2.UINT32 & _RIIC_TIMEOUT_DETECTED))
    {
        r_Config_RIIC0_callback_receiveerror(MD_ERROR2);
    }
    else if((RIIC0.IER.UINT32 & _RIIC_NACK_RECEPTION_INT_ENABLE) && (RIIC0.SR2.UINT32 & _RIIC_NACK_DETECTED)) 
    {
        if(_RIIC_MASTER_TRANSMIT == g_riic0_mode_flag)
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC0.CR2.UINT32 |= _RIIC_STOP_CONDITION_REQUEST;
            while (!(RIIC0.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED))
            {
                NOP();
            }
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_NACK_DETECTED;
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
        }
        else if (_RIIC_MASTER_RECEIVE == g_riic0_mode_flag)
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC0.CR2.UINT32 |= _RIIC_STOP_CONDITION_REQUEST;
            /* Dummy read the ICDRR register */
            dummy = RIIC0.DRR.UINT32;
            while ((RIIC0.SR2.UINT32 & _RIIC_STOP_CONDITION_DETECTED) != _RIIC_STOP_CONDITION_DETECTED)
            {
                NOP();
            }
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_NACK_DETECTED;
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
        }
        r_Config_RIIC0_callback_receiveerror(MD_ERROR3);
    }
    else if (_RIIC_MASTER_TRANSMIT == g_riic0_mode_flag)
    {
        if ((_RIIC_MASTER_SENDS_ADR_7_W == g_riic0_state) || (_RIIC_MASTER_SENDS_ADR_10A_W == g_riic0_state))
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_DETECTED;
            RIIC0.IER.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_INT_ENABLE;
            RIIC0.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
        }
        else if (_RIIC_MASTER_SENDS_STOP == g_riic0_state)
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_NACK_DETECTED;
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC0.IER.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC0.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;

            r_Config_RIIC0_callback_transmitend();
        }
    }
    else if (_RIIC_MASTER_RECEIVE == g_riic0_mode_flag)
    {
        if ((_RIIC_MASTER_SENDS_ADR_7_R == g_riic0_state) || (_RIIC_MASTER_SENDS_ADR_10A_W == g_riic0_state))
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_DETECTED;
            RIIC0.IER.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_INT_ENABLE;
            RIIC0.IER.UINT32 |= _RIIC_STOP_CONDITION_INT_ENABLE;
        }
        else if (_RIIC_MASTER_RECEIVES_RESTART == g_riic0_state)
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_DETECTED;
            RIIC0.IER.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_INT_ENABLE;
            g_riic0_state = _RIIC_MASTER_SENDS_ADR_10A_R;
        }
        else if (_RIIC_MASTER_RECEIVES_STOP == g_riic0_state)
        {
            RIIC0.MR3.UINT32 &= (uint32_t) ~_RIIC_RDRF_FLAG_SET_SCL_EIGHTH;
            RIIC0.MR3.UINT32 &= (uint32_t) ~_RIIC_ACKBT_BIT_MODIFICATION_ENABLED;
            RIIC0.MR3.UINT32 &= (uint32_t) ~_RIIC_NACK_TRANSMISSION;
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_NACK_DETECTED;
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC0.IER.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_INT_ENABLE;
            RIIC0.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;

            r_Config_RIIC0_callback_receiveend();
        }
    }
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC0_transmit_interrupt
* Description  : This function is RIIC0 send interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC0_transmit_interrupt(enable=false, channel=76, fpu=true, callt=false)
void r_Config_RIIC0_transmit_interrupt(void)
{
    if (_RIIC_MASTER_TRANSMIT == g_riic0_mode_flag)
    {
        if (_RIIC_MASTER_SENDS_ADR_7_W == g_riic0_state)
        {
            RIIC0.DRT.UINT32 = (uint8_t)(g_riic0_slave_address << _RIIC_ADDRESS_7BIT_SHIFT);
            g_riic0_state = _RIIC_MASTER_SENDS_DATA;
        }
        else if (_RIIC_MASTER_SENDS_ADR_10A_W == g_riic0_state)
        {
            RIIC0.DRT.UINT32 = (uint8_t)(((g_riic0_slave_address & _RIIC_ADDRESS_10BIT_UPPER) >> 
                               _RIIC_ADDRESS_10BIT_SHIFT) | _RIIC_ADDRESS_10BIT_UPPER_FORMAT);
            g_riic0_state = _RIIC_MASTER_SENDS_ADR_10B;
        }
        else if (_RIIC_MASTER_SENDS_ADR_10B == g_riic0_state)
        {
            RIIC0.DRT.UINT32 = (uint8_t)(g_riic0_slave_address & _RIIC_ADDRESS_10BIT_LOWER);
            g_riic0_state = _RIIC_MASTER_SENDS_DATA;
        }
        else if (_RIIC_MASTER_SENDS_DATA == g_riic0_state)
        {
            if (g_riic0_tx_count > 0U)
            {
                RIIC0.DRT.UINT32 = *gp_riic0_tx_address;
                gp_riic0_tx_address++;
                g_riic0_tx_count--;
            } 
            else
            {
                g_riic0_state = _RIIC_MASTER_SENDS_END;
            }
        }
    }
    else if (_RIIC_MASTER_RECEIVE == g_riic0_mode_flag)
    {
        if (_RIIC_MASTER_SENDS_ADR_7_R == g_riic0_state)
        {
            RIIC0.DRT.UINT32 = (uint8_t)((g_riic0_slave_address << _RIIC_ADDRESS_7BIT_SHIFT)  | 
                               _RIIC_ADDRESS_RECEIVE);
            g_riic0_state = _RIIC_MASTER_RECEIVES_START;
        }
        else if (_RIIC_MASTER_SENDS_ADR_10A_W == g_riic0_state)
        {
            RIIC0.DRT.UINT32 = (uint8_t)(((g_riic0_slave_address & _RIIC_ADDRESS_10BIT_UPPER) >> 
                               _RIIC_ADDRESS_10BIT_SHIFT) | _RIIC_ADDRESS_10BIT_UPPER_FORMAT);
            g_riic0_state = _RIIC_MASTER_SENDS_ADR_10B;
        }
        else if (_RIIC_MASTER_SENDS_ADR_10B == g_riic0_state)
        {
            RIIC0.DRT.UINT32 = (uint8_t)(g_riic0_slave_address & _RIIC_ADDRESS_10BIT_LOWER);
            g_riic0_state = _RIIC_MASTER_RECEIVES_RESTART;
        }
        else if (_RIIC_MASTER_SENDS_ADR_10A_R == g_riic0_state)
        {
            RIIC0.DRT.UINT32 = (uint8_t)(((g_riic0_slave_address & _RIIC_ADDRESS_10BIT_UPPER) >> 
                               _RIIC_ADDRESS_10BIT_SHIFT) | _RIIC_ADDRESS_10BIT_UPPER_FORMAT | 
                               _RIIC_ADDRESS_RECEIVE);
            g_riic0_state = _RIIC_MASTER_RECEIVES_START;
        }
    }
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC0_transmitend_interrupt
* Description  : This function is RIIC0 send end interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC0_transmitend_interrupt(enable=false, channel=79, fpu=true, callt=false)
void r_Config_RIIC0_transmitend_interrupt(void)
{
    if (_RIIC_MASTER_SENDS_END == g_riic0_state)
    {
        if (1U == g_riic0_stop_generation)
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
            RIIC0.CR2.UINT32 |= _RIIC_STOP_CONDITION_REQUEST;

            g_riic0_state = _RIIC_MASTER_SENDS_STOP;
        }
        else
        {
            RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_TRANSMIT_END;
            r_Config_RIIC0_callback_transmitend();
        }
    }
    else if (_RIIC_MASTER_RECEIVES_RESTART == g_riic0_state)
    {
        RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_START_CONDITION_DETECTED;
        RIIC0.IER.UINT32 |= _RIIC_START_CONDITION_INT_ENABLE;
        /* Set restart condition flag */
        RIIC0.CR2.UINT32 |= _RIIC_RESTART_CONDITION_REQUEST;
        RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_TRANSMIT_END;
    }
}

/***********************************************************************************************************************
* Function Name: r_Config_RIIC0_receive_interrupt
* Description  : This function is RIIC0 receive interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma interrupt r_Config_RIIC0_receive_interrupt(enable=false, channel=78, fpu=true, callt=false)
void r_Config_RIIC0_receive_interrupt(void)
{
    volatile uint8_t dummy;
    uint16_t temp;

    if (_RIIC_MASTER_RECEIVES_START == g_riic0_state)
    {
        if ((2U == g_riic0_rx_length) || (1U == g_riic0_rx_length))
        {
            RIIC0.MR3.UINT32 |= _RIIC_WAIT;
        }

        if (1U == g_riic0_rx_length)
        {
            RIIC0.MR3.UINT32 |= _RIIC_RDRF_FLAG_SET_SCL_EIGHTH;
            RIIC0.MR3.UINT32 |= _RIIC_ACKBT_BIT_MODIFICATION_ENABLED;
            RIIC0.MR3.UINT32 |= _RIIC_NACK_TRANSMISSION;
        }

        /* Dummy read to release SCL */
        dummy = RIIC0.DRR.UINT32;

        if (1U == g_riic0_rx_length)
        {
            g_riic0_state = _RIIC_MASTER_RECEIVES_STOPPING;
        }
        else
        {
            g_riic0_state = _RIIC_MASTER_RECEIVES_DATA;
        }
    }
    else if (_RIIC_MASTER_RECEIVES_DATA == g_riic0_state)
    {
        temp = g_riic0_rx_length;
        if (g_riic0_rx_count < temp)
        {
            if ((temp - 3U) == g_riic0_rx_count)
            {
                RIIC0.MR3.UINT32 |= _RIIC_WAIT;
                *gp_riic0_rx_address = RIIC0.DRR.UINT32;
                gp_riic0_rx_address++;
                g_riic0_rx_count++;
            }
            else if ((temp - 2U) == g_riic0_rx_count)
            {
                RIIC0.MR3.UINT32 |= _RIIC_RDRF_FLAG_SET_SCL_EIGHTH;
                RIIC0.MR3.UINT32 |= _RIIC_ACKBT_BIT_MODIFICATION_ENABLED;
                RIIC0.MR3.UINT32 |= _RIIC_NACK_TRANSMISSION;
                *gp_riic0_rx_address = RIIC0.DRR.UINT32;
                gp_riic0_rx_address++;
                g_riic0_rx_count++;
                g_riic0_state = _RIIC_MASTER_RECEIVES_STOPPING;
            }
            else
            {
                *gp_riic0_rx_address = RIIC0.DRR.UINT32;
                gp_riic0_rx_address++;
                g_riic0_rx_count++;
            }
        }
    }
    else if (_RIIC_MASTER_RECEIVES_STOPPING == g_riic0_state)
    {
        RIIC0.SR2.UINT32 &= (uint32_t) ~_RIIC_STOP_CONDITION_DETECTED;
        RIIC0.CR2.UINT32 |= _RIIC_STOP_CONDITION_REQUEST;
        *gp_riic0_rx_address = RIIC0.DRR.UINT32;
        gp_riic0_rx_address++;
        g_riic0_rx_count++;
        RIIC0.MR3.UINT32 &= (uint32_t) ~_RIIC_WAIT;
        g_riic0_state = _RIIC_MASTER_RECEIVES_STOP;
    }
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

