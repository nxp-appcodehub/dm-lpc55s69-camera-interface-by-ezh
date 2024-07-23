/*
 * Copyright 2019-2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_smartdma.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.lpc_smartdma"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef void (*smartdma_func_t)(void);

#define SMARTDMA_HANDSHAKE_EVENT  0U
#define SMARTDMA_HANDSHAKE_ENABLE 1U
#define SMARTDMA_MASK_RESP        2U
#define SMARTDMA_ENABLE_AHBBUF    3U
#define SMARTDMA_ENABLE_GPISYNCH  4U

#define SMARTDMA SMARTDMA0

/*******************************************************************************
 * Variables
 ******************************************************************************/
static smartdma_func_t *s_smartdmaApiTable;
static smartdma_callback_t s_smartdmaCallback;
static void *s_smartdmaCallbackParam;


/* ----------------------------------------------------------------------------
   -- SMARTDMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMARTDMA_Peripheral_Access_Layer SMARTDMA Peripheral Access Layer
 * @{
 */

/** SMARTDMA - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[32];
  __IO uint32_t BOOTADR;                           /**< Boot Address, offset: 0x20 */
  __IO uint32_t CTRL;                              /**< Control, offset: 0x24 */
  __I  uint32_t PC;                                /**< Program Counter, offset: 0x28 */
  __I  uint32_t SP;                                /**< Stack Pointer, offset: 0x2C */
  __IO uint32_t BREAK_ADDR;                        /**< Breakpoint Address, offset: 0x30 */
  __IO uint32_t BREAK_VECT;                        /**< Breakpoint Vector, offset: 0x34 */
  __IO uint32_t EMER_VECT;                         /**< Emergency Vector, offset: 0x38 */
  __IO uint32_t EMER_SEL;                          /**< Emergency Select, offset: 0x3C */
  __IO uint32_t ARM2EZH;                           /**< ARM to EZH Interrupt Control, offset: 0x40 */
  __IO uint32_t EZH2ARM;                           /**< EZH to ARM Trigger, offset: 0x44 */
  __IO uint32_t PENDTRAP;                          /**< Pending Trap Control, offset: 0x48 */
} SMARTDMA_Type;

/* ----------------------------------------------------------------------------
   -- SMARTDMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMARTDMA_Register_Masks SMARTDMA Register Masks
 * @{
 */

/*! @name BOOTADR - Boot Address */
/*! @{ */

#define SMARTDMA_BOOTADR_ADDR_MASK               (0xFFFFFFFCU)
#define SMARTDMA_BOOTADR_ADDR_SHIFT              (2U)
/*! ADDR - 32-bit boot address, the boot address should be 4-byte aligned. */
#define SMARTDMA_BOOTADR_ADDR(x)                 (((uint32_t)(((uint32_t)(x)) << SMARTDMA_BOOTADR_ADDR_SHIFT)) & SMARTDMA_BOOTADR_ADDR_MASK)
/*! @} */

/*! @name CTRL - Control */
/*! @{ */

#define SMARTDMA_CTRL_START_MASK                 (0x1U)
#define SMARTDMA_CTRL_START_SHIFT                (0U)
/*! START - Start Bit Ignition */
#define SMARTDMA_CTRL_START(x)                   (((uint32_t)(((uint32_t)(x)) << SMARTDMA_CTRL_START_SHIFT)) & SMARTDMA_CTRL_START_MASK)

#define SMARTDMA_CTRL_EXF_MASK                   (0x2U)
#define SMARTDMA_CTRL_EXF_SHIFT                  (1U)
/*! EXF - External Flag */
#define SMARTDMA_CTRL_EXF(x)                     (((uint32_t)(((uint32_t)(x)) << SMARTDMA_CTRL_EXF_SHIFT)) & SMARTDMA_CTRL_EXF_MASK)

#define SMARTDMA_CTRL_ERRDIS_MASK                (0x4U)
#define SMARTDMA_CTRL_ERRDIS_SHIFT               (2U)
/*! ERRDIS - Error Disable */
#define SMARTDMA_CTRL_ERRDIS(x)                  (((uint32_t)(((uint32_t)(x)) << SMARTDMA_CTRL_ERRDIS_SHIFT)) & SMARTDMA_CTRL_ERRDIS_MASK)

#define SMARTDMA_CTRL_BUFEN_MASK                 (0x8U)
#define SMARTDMA_CTRL_BUFEN_SHIFT                (3U)
/*! BUFEN - Buffer Enable */
#define SMARTDMA_CTRL_BUFEN(x)                   (((uint32_t)(((uint32_t)(x)) << SMARTDMA_CTRL_BUFEN_SHIFT)) & SMARTDMA_CTRL_BUFEN_MASK)

#define SMARTDMA_CTRL_SYNCEN_MASK                (0x10U)
#define SMARTDMA_CTRL_SYNCEN_SHIFT               (4U)
/*! SYNCEN - Sync Enable */
#define SMARTDMA_CTRL_SYNCEN(x)                  (((uint32_t)(((uint32_t)(x)) << SMARTDMA_CTRL_SYNCEN_SHIFT)) & SMARTDMA_CTRL_SYNCEN_MASK)

#define SMARTDMA_CTRL_WKEY_MASK                  (0xFFFF0000U)
#define SMARTDMA_CTRL_WKEY_SHIFT                 (16U)
/*! WKEY - Write Key */
#define SMARTDMA_CTRL_WKEY(x)                    (((uint32_t)(((uint32_t)(x)) << SMARTDMA_CTRL_WKEY_SHIFT)) & SMARTDMA_CTRL_WKEY_MASK)
/*! @} */

/*! @name PC - Program Counter */
/*! @{ */

#define SMARTDMA_PC_PC_MASK                      (0xFFFFFFFFU)
#define SMARTDMA_PC_PC_SHIFT                     (0U)
/*! PC - Program Counter */
#define SMARTDMA_PC_PC(x)                        (((uint32_t)(((uint32_t)(x)) << SMARTDMA_PC_PC_SHIFT)) & SMARTDMA_PC_PC_MASK)
/*! @} */

/*! @name SP - Stack Pointer */
/*! @{ */

#define SMARTDMA_SP_SP_MASK                      (0xFFFFFFFFU)
#define SMARTDMA_SP_SP_SHIFT                     (0U)
/*! SP - Stack Pointer */
#define SMARTDMA_SP_SP(x)                        (((uint32_t)(((uint32_t)(x)) << SMARTDMA_SP_SP_SHIFT)) & SMARTDMA_SP_SP_MASK)
/*! @} */

/*! @name BREAK_ADDR - Breakpoint Address */
/*! @{ */

#define SMARTDMA_BREAK_ADDR_ADDR_MASK            (0xFFFFFFFCU)
#define SMARTDMA_BREAK_ADDR_ADDR_SHIFT           (2U)
/*! ADDR - 32-bit address to swap to EZHB_BREAK_VECT location */
#define SMARTDMA_BREAK_ADDR_ADDR(x)              (((uint32_t)(((uint32_t)(x)) << SMARTDMA_BREAK_ADDR_ADDR_SHIFT)) & SMARTDMA_BREAK_ADDR_ADDR_MASK)
/*! @} */

/*! @name BREAK_VECT - Breakpoint Vector */
/*! @{ */

#define SMARTDMA_BREAK_VECT_VEC_MASK             (0xFFFFFFFCU)
#define SMARTDMA_BREAK_VECT_VEC_SHIFT            (2U)
/*! VEC - Vector address of user debug routine. */
#define SMARTDMA_BREAK_VECT_VEC(x)               (((uint32_t)(((uint32_t)(x)) << SMARTDMA_BREAK_VECT_VEC_SHIFT)) & SMARTDMA_BREAK_VECT_VEC_MASK)
/*! @} */

/*! @name EMER_VECT - Emergency Vector */
/*! @{ */

#define SMARTDMA_EMER_VECT_VEC_MASK              (0xFFFFFFFCU)
#define SMARTDMA_EMER_VECT_VEC_SHIFT             (2U)
/*! VEC - Vector address of emergency code routine */
#define SMARTDMA_EMER_VECT_VEC(x)                (((uint32_t)(((uint32_t)(x)) << SMARTDMA_EMER_VECT_VEC_SHIFT)) & SMARTDMA_EMER_VECT_VEC_MASK)
/*! @} */

/*! @name EMER_SEL - Emergency Select */
/*! @{ */

#define SMARTDMA_EMER_SEL_EN_MASK                (0x100U)
#define SMARTDMA_EMER_SEL_EN_SHIFT               (8U)
/*! EN - Emergency code routine */
#define SMARTDMA_EMER_SEL_EN(x)                  (((uint32_t)(((uint32_t)(x)) << SMARTDMA_EMER_SEL_EN_SHIFT)) & SMARTDMA_EMER_SEL_EN_MASK)

#define SMARTDMA_EMER_SEL_RQ_MASK                (0x200U)
#define SMARTDMA_EMER_SEL_RQ_SHIFT               (9U)
/*! RQ - Software emergency request */
#define SMARTDMA_EMER_SEL_RQ(x)                  (((uint32_t)(((uint32_t)(x)) << SMARTDMA_EMER_SEL_RQ_SHIFT)) & SMARTDMA_EMER_SEL_RQ_MASK)
/*! @} */

/*! @name ARM2EZH - ARM to EZH Interrupt Control */
/*! @{ */

#define SMARTDMA_ARM2EZH_IE_MASK                 (0x3U)
#define SMARTDMA_ARM2EZH_IE_SHIFT                (0U)
/*! IE - Interrupt Enable */
#define SMARTDMA_ARM2EZH_IE(x)                   (((uint32_t)(((uint32_t)(x)) << SMARTDMA_ARM2EZH_IE_SHIFT)) & SMARTDMA_ARM2EZH_IE_MASK)

#define SMARTDMA_ARM2EZH_GP_MASK                 (0xFFFFFFFCU)
#define SMARTDMA_ARM2EZH_GP_SHIFT                (2U)
/*! GP - General purpose register bits */
#define SMARTDMA_ARM2EZH_GP(x)                   (((uint32_t)(((uint32_t)(x)) << SMARTDMA_ARM2EZH_GP_SHIFT)) & SMARTDMA_ARM2EZH_GP_MASK)
/*! @} */

/*! @name EZH2ARM - EZH to ARM Trigger */
/*! @{ */

#define SMARTDMA_EZH2ARM_GP_MASK                 (0xFFFFFFFFU)
#define SMARTDMA_EZH2ARM_GP_SHIFT                (0U)
/*! GP - General purpose register bits Writing to EZH2ARM triggers the ARM interrupt when ARM2EZH [1:0] == 2h */
#define SMARTDMA_EZH2ARM_GP(x)                   (((uint32_t)(((uint32_t)(x)) << SMARTDMA_EZH2ARM_GP_SHIFT)) & SMARTDMA_EZH2ARM_GP_MASK)
/*! @} */

/*! @name PENDTRAP - Pending Trap Control */
/*! @{ */

#define SMARTDMA_PENDTRAP_STATUS_MASK            (0xFFU)
#define SMARTDMA_PENDTRAP_STATUS_SHIFT           (0U)
/*! STATUS - Status Flag or Pending Trap Request */
#define SMARTDMA_PENDTRAP_STATUS(x)              (((uint32_t)(((uint32_t)(x)) << SMARTDMA_PENDTRAP_STATUS_SHIFT)) & SMARTDMA_PENDTRAP_STATUS_MASK)

#define SMARTDMA_PENDTRAP_POL_MASK               (0xFF00U)
#define SMARTDMA_PENDTRAP_POL_SHIFT              (8U)
/*! POL - Polarity */
#define SMARTDMA_PENDTRAP_POL(x)                 (((uint32_t)(((uint32_t)(x)) << SMARTDMA_PENDTRAP_POL_SHIFT)) & SMARTDMA_PENDTRAP_POL_MASK)

#define SMARTDMA_PENDTRAP_EN_MASK                (0xFF0000U)
#define SMARTDMA_PENDTRAP_EN_SHIFT               (16U)
/*! EN - Enable Pending Trap */
#define SMARTDMA_PENDTRAP_EN(x)                  (((uint32_t)(((uint32_t)(x)) << SMARTDMA_PENDTRAP_EN_SHIFT)) & SMARTDMA_PENDTRAP_EN_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group SMARTDMA_Register_Masks */

  /** Peripheral SMARTDMA0 base address */
  #define SMARTDMA0_BASE                           (0x4001D000)
  /** Peripheral SMARTDMA0 base pointer */
  #define SMARTDMA0                                ((SMARTDMA_Type *)SMARTDMA0_BASE)
  /** Array initializer of SMARTDMA peripheral base addresses */
  #define SMARTDMA_BASE_ADDRS                      { SMARTDMA0_BASE }
  /** Array initializer of SMARTDMA peripheral base pointers */
  #define SMARTDMA_BASE_PTRS                       { SMARTDMA0 }

/*!
 * @}
 */ /* end of group SMARTDMA_Peripheral_Access_Layer */


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Codes
 ******************************************************************************/
/*!
 * brief Initialize the SMARTDMA.
 *
 * param apiMemAddr The address firmware will be copied to.
 * param firmware The firmware to use.
 * param firmwareSizeByte Size of firmware.
 */
void SMARTDMA_Init(uint32_t apiMemAddr, const void *firmware, uint32_t firmwareSizeByte)
{
    SMARTDMA_InitWithoutFirmware();
    SMARTDMA_InstallFirmware(apiMemAddr, firmware, firmwareSizeByte);
}

/*!
 * brief Initialize the SMARTDMA.
 *
 * This function is similar with SMARTDMA_Init, the difference is this function
 * does not install the firmware, the firmware could be installed using
 * SMARTDMA_InstallFirmware.
 */
void SMARTDMA_InitWithoutFirmware(void)
{
    /* Clear Smart DMA RAM */
    RESET_PeripheralReset(kEZHB_RST_SHIFT_RSTn);
    CLOCK_EnableClock(kCLOCK_Ezhb);
//    SMARTDMA_cfgHandshake(true,false);
//    SMARTDMA->PENDTRAP = (0xFF << 16);//enable pending trap
}

/*!
 * @brief Install the firmware.
 *
 * param apiMemAddr The address firmware will be copied to.
 * param firmware The firmware to use.
 * param firmwareSizeByte Size of firmware.
 */
void SMARTDMA_InstallFirmware(uint32_t apiMemAddr, const void *firmware, uint32_t firmwareSizeByte)
{
    (void)memcpy((void *)(uint8_t *)apiMemAddr, firmware, firmwareSizeByte);
    SMARTDMA->CTRL     = (0xC0DE0000U | (1U << SMARTDMA_ENABLE_GPISYNCH));
    s_smartdmaApiTable = (smartdma_func_t *)apiMemAddr;
}

/*!
 * brief Install the complete callback function..
 *
 * param callback The callback called when smartdma program finished.
 */
void SMARTDMA_InstallCallback(smartdma_callback_t callback, void *param)
{
    s_smartdmaCallback      = callback;
    s_smartdmaCallbackParam = param;
}

/*!
 * brief Boot the SMARTDMA to run program.
 *
 * param apiIndex Index of the API to call.
 * param pParam Pointer to the parameter.
 * param mask Value set to SMARTDMA_ARM2SMARTDMA[0:1].
 */
void SMARTDMA_Boot(uint32_t apiIndex, void *pParam, uint8_t mask)
{
    SMARTDMA->ARM2EZH = (uint32_t)(uint8_t *)pParam | (uint32_t)mask;
    SMARTDMA->BOOTADR = (uint32_t)(s_smartdmaApiTable[apiIndex]);
    SMARTDMA->CTRL    = 0xC0DE0011U | (0U << SMARTDMA_MASK_RESP) | (0U << SMARTDMA_ENABLE_AHBBUF); /* BOOT */
};

/*!
 * brief Deinitialize the SMARTDMA.
 */
void SMARTDMA_Deinit(void)
{
    SMARTDMA->CTRL = 0xC0DE0000U;
    CLOCK_DisableClock(kCLOCK_Ezhb);

}

/*!
 * brief Reset the SMARTDMA.
 */
void SMARTDMA_Reset(void)
{
    RESET_PeripheralReset(kEZHB_RST_SHIFT_RSTn);
    SMARTDMA->CTRL = (0xC0DE0000U | (1U << SMARTDMA_ENABLE_GPISYNCH));
}

/*!
 * brief SMARTDMA IRQ.
 */
void SMARTDMA_HandleIRQ(void)
{
    if (NULL != s_smartdmaCallback)
    {
        s_smartdmaCallback(s_smartdmaCallbackParam);
    }
}

/*!
 * brief SMARTDMA Set EX flag.
 */
void SMARTDMA_SetExternalFlag(uint8_t flag)
{
    volatile uint32_t smartdma_ctrl = (SMARTDMA->CTRL & 0x0000FFFF);
    if (flag == 0) {
    	smartdma_ctrl &= ~(1 << 1);
    } else {
    	smartdma_ctrl |= (1 << 1);
    }
    SMARTDMA->CTRL = (0xC0DE0000 | smartdma_ctrl);
}

void SMARTDMA_cfgHandshake(bool _enable_handshake, bool _enable_event){
	int enable_handshake;
	int enable_event;
	enable_handshake = (_enable_handshake) ? 1 : 0;
	enable_event     = (_enable_event)     ? 1 : 0;
	SMARTDMA->ARM2EZH |= (enable_handshake << SMARTDMA_HANDSHAKE_ENABLE) + (enable_event<<SMARTDMA_HANDSHAKE_EVENT);
}

/*!
 * brief SMARTDMA Access RAM.
 */
void SMARTDMA_AccessShareRAM(uint8_t flag){
	SMARTDMA_SetExternalFlag(flag);
}

/*!
 * brief SMARTDMA pendtrap function.
 */
void  SMARTDMA_Pendtrap(uint8_t flag){
    if (flag == 0) {
    	SMARTDMA->PENDTRAP &= ~(0xff);
    } else {
    	SMARTDMA->PENDTRAP |= 0xff;
    }
}
