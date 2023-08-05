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
* Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name        : Config_CMT1_user.c
* Component Version: 2.3.0
* Device(s)        : R5F51305AxFM
* Description      : This file implements device driver for Config_CMT1.
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
#include "Config_CMT1.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
#define MAC_timeout_level 600
#define LED_MCC (PORTH.PODR.BIT.B0)
#define LED_GEN (PORTH.PODR.BIT.B3)
#define LED_STT (PORT2.PODR.BIT.B7)
extern uint16_t MAC_registers[130];
extern uint8_t GenStart;
extern uint8_t GenIsConnected;
extern uint32_t GenRunTime; // second
extern uint32_t MAC_timeout;
extern uint8_t error_check;
uint32_t wait_time=0;
uint32_t freq_ustbl_time_count=0;
uint32_t gen_reset_time_count=0;
uint32_t MAC_runtime=0;
extern uint32_t Timer1Relay5;
extern uint32_t Timer2Relay5;
bool gen_running_timeout=0;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_Config_CMT1_Create_UserInit
* Description  : This function adds user code after initializing the CMT1 channel
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_Config_CMT1_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_Config_CMT1_cmi1_interrupt
* Description  : This function is CMI1 interrupt service routine
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

#if FAST_INTERRUPT_VECTOR == VECT_CMT1_CMI1
#pragma interrupt r_Config_CMT1_cmi1_interrupt(vect=VECT(CMT1,CMI1),fint)
#else
#pragma interrupt r_Config_CMT1_cmi1_interrupt(vect=VECT(CMT1,CMI1))
#endif
static void r_Config_CMT1_cmi1_interrupt(void)
{
    /* Start user code for r_Config_CMT1_cmi1_interrupt. Do not edit comment generated here */
	if(wait_time<0xFFFFFFFF) wait_time++; //use in waittime()
	else wait_time =0;
	// freq unstable check counter
	if(freq_ustbl_time_count<180)
	{
		freq_ustbl_time_count++;
	}
	// Count time for MAC timeout connection with MCC - LED_MCC
	if(MAC_timeout< MAC_timeout_level)
	{
		MAC_timeout++;
		LED_MCC=0;
	}
	else //no connection to MCC
	{
		LED_MCC ^=1;
	}
	//LED_GEN
	if(GenIsConnected) LED_GEN=0;
	else LED_GEN ^=1;
	//LED_STT
	if(error_check) LED_STT ^=1;
	else LED_STT=0;
	// Count runtime for Generator
	if(GenStart==1)
	{
		GenRunTime++;
	}
	else
	{
		GenRunTime=0;
	}
	if(GenRunTime> MAC_registers[0x5B]*3600)
	{
		gen_running_timeout=1;
		GenRunTime=0;
	}
	MAC_registers[0x3D] = (uint16_t)(GenRunTime/60);

	// Auto reset error for generator
	if(MAC_registers[0x4C]!=0) // Gen has error
	{
		if(gen_reset_time_count<MAC_registers[0x50]) gen_reset_time_count++; // count time in second
		else // time count >= auto reset timeout
		{
			gen_reset_time_count=0;
			if((MAC_registers[0x53]>0)&&(MAC_registers[0x70]<MAC_registers[0x55])) //Check condition 0x53 auto reset mode, 0x70: reset count, 0x55 max reset
			{
				MAC_registers[0x70]++; //increase reset count
				MAC_registers[0x4C]=0;
			}
		}
	}
	//MAC run time in mins
	if(MAC_runtime<0xFFFFFFFF) MAC_runtime++;
	else MAC_runtime=0;
	MAC_registers[0x71] = (uint16_t)((MAC_runtime/60)>>16);
	MAC_registers[0x72] = (uint16_t)((MAC_runtime/60));

	//Relay 5 timer
	if(Timer1Relay5!=0) Timer1Relay5--;
	if(Timer2Relay5!=0) Timer2Relay5--;

    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */

/* End user code. Do not edit comment generated here */
