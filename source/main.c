/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_smartdma.h"
#include "fsl_smartdma.h"
#include "fsl_power.h"
#include "fsl_inputmux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PINFUNC_CAMERA		15	/* Pin function for Camera engine to be configured by ARM CPU*/

/*******************************************************************************
 * Variables
 ******************************************************************************/
smartdma_camera_param_t para;
volatile uint8_t g_samrtdma_stack[32];

volatile uint32_t DataReadyFlag;
volatile uint16_t  g_camera_buffer[320*240];
uint32_t *pdata;     //frame buffer address
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void Lcd_Init(void);			//LCD initialization
extern void Ov7670_Init(uint32_t instance);			//camera module initialization
extern void Lcd_Refresh(uint16_t *pData);

/*******************************************************************************
 * Code
 ******************************************************************************/
void SmartDMA_camera_callback(void *param){
	DataReadyFlag = 1; // tell ARM data is ready from Camera engine
};

/**
 * brief	Initialize camera pin
 * param 	None
 * return	None
 */
void camera_pin_Init(void){
    /* Connect trigger sources to camera engine */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX->CAMERA_ENGINE_INPUTMUX[0]	 =  13; // set p0_13 as VSYNC input function pin, every edge will be responded
    INPUTMUX->CAMERA_ENGINE_INPUTMUX[1]	 =  14; // set p0_14 as HSYNC input function pin, every edge will be responded
    INPUTMUX->CAMERA_ENGINE_INPUTMUX[2]	 =  15; // set p0_15 as pixel input function pin, every edge will be responded
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
    // configure camera interface pins
    IOCON->PIO[0][0]  = PINFUNC_CAMERA | 1<<8|1<<10|2<<4| 1<<6;	//set p0_0 D0 on the camera port
    IOCON->PIO[0][1]  = PINFUNC_CAMERA | 1<<8|2<<4| 1<<6;				//set p0_1 D1 on the camera port
    IOCON->PIO[0][2]  = PINFUNC_CAMERA | 1<<8|2<<4| 1<<6;				//set p0_2 D2 on the camera port
    IOCON->PIO[0][3]  = PINFUNC_CAMERA | 1<<8|1<<6;							//set p0_3 D3 on the camera port
    IOCON->PIO[0][4]  = PINFUNC_CAMERA | 1<<8|1<<6;							//set p0_4 D4 on the camera port
    IOCON->PIO[0][5]  = PINFUNC_CAMERA | 1<<8|2<<4|1<<6;				//set p0_5 D5 on the camera port
    IOCON->PIO[0][6]  = PINFUNC_CAMERA | 1<<8;									//set p0_6 D6 on the camera port
    IOCON->PIO[0][7]  = PINFUNC_CAMERA | 1<<8;									//set p0_7 D7 on the camera port
    IOCON->PIO[0][18] = PINFUNC_CAMERA | 1<<8| 1<<10;						//P0_18 will toggle when camera engine receive every VSYNC dege
    IOCON->PIO[0][14] = PINFUNC_CAMERA | 1<<8| 1<<10;						//P0_14 will toggle when camera engine receive every VSYNC dege
}
/*!
 * @brief Main function
 */
int main(void)
{
	int32_t comp_flag=0;

    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
#if !defined(DONT_ENABLE_FLASH_PREFETCH)
    /* enable flash prefetch for better performance */
    SYSCON->FMCCR |= SYSCON_FMCCR_PREFEN_MASK;
#endif

    /* attach 12 MHz clock to FLEXCOMM4 (I2C master) */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);
	GPIO_PinWrite(GPIO, 0, 17, 0);	//toggle P1_11

    camera_pin_Init();
	Lcd_Init();			//LCD initialization
	Ov7670_Init(0); //outside camera module initialization

	SMARTDMA_InitWithoutFirmware();
	SMARTDMA_InstallFirmware(SMARTDMA_CAMERA_MEM_ADDR,s_smartdmaCameraFirmware,
			SMARTDMA_CAMERA_FIRMWARE_SIZE);
	SMARTDMA_InstallCallback(SmartDMA_camera_callback, NULL);

	NVIC_EnableIRQ(Reserved46_IRQn);
	NVIC_SetPriority(Reserved46_IRQn, 3);

	para.smartdma_stack = (void *)g_samrtdma_stack;//offset 0
	para.p_buffer	 =  (uint32_t *)g_camera_buffer;//offset 2
	SMARTDMA_Boot(kEZH_Camera_320240_Whole_Buf, &para, 0x2);
	DataReadyFlag = 0;
	PRINTF("camera engine for LPC55S69 demo\r\n");

    while (1)
    {
		while(DataReadyFlag == 0);		  // Waiting for respond from EZH, the waiting time is about 7ms
		GPIO_PinWrite(GPIO, 1, 11, 0);	//toggle P1_11
		Lcd_Refresh(g_camera_buffer); 	//the refresh time is about 25ms
		DataReadyFlag = 0;              // clear the flag
		GPIO_PinWrite(GPIO, 1, 11, 1);  //toggle P1_11
    }
}
