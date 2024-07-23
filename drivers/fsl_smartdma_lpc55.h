/*
 * Copyright 2019-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef FSL_SMARTDMA_LPC55_H_
#define FSL_SMARTDMA_LPC55_H_

#include "fsl_common.h"

/*!
 * @addtogroup smartdma_LPC55 LPC55 SMARTDMA Firmware
 * @ingroup smartdma
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The firmware used for display. */
extern const uint8_t s_smartdmaDisplayFirmware[];

/*! @brief The s_smartdmaDisplayFirmware firmware memory address. */
#define SMARTDMA_DISPLAY_MEM_ADDR 0x04000000U

/*! @brief Size of s_smartdmaDisplayFirmware */
#define SMARTDMA_DISPLAY_FIRMWARE_SIZE (s_smartdmaDisplayFirmwareSize)

/*! @brief Size of s_smartdmaDisplayFirmware */
extern const uint32_t s_smartdmaDisplayFirmwareSize;

/*! @brief The firmware used for camera. */
extern const uint8_t s_smartdmaCameraFirmware[];

/*! @brief The s_smartdmaCameraFirmware firmware memory address. */
#define SMARTDMA_CAMERA_MEM_ADDR 0x04000000U

/*! @brief Size of s_smartdmacameraFirmware */
#define SMARTDMA_CAMERA_FIRMWARE_SIZE (s_smartdmaCameraFirmwareSize)

/*! @brief Size of s_smartdmacameraFirmware */
extern const uint32_t s_smartdmaCameraFirmwareSize;


/*! @brief The firmware used for keyscan. */
extern const uint8_t s_smartdmaKeyscanFirmware[];

/*! @brief The s_smartdmaKeyscanFirmware firmware memory address. */
#define SMARTDMA_KEYSCAN_MEM_ADDR 0x04000000U

/*! @brief Size of s_smartdmacameraFirmware */
#define SMARTDMA_KEYSCAN_FIRMWARE_SIZE (s_smartdmaKeyscanFirmwareSize)

/*! @brief Size of s_smartdmacameraFirmware */
extern const uint32_t s_smartdmaKeyscanFirmwareSize;


/*! @brief The firmware used for QEI. */
extern const uint8_t s_smartdmaQEIFirmware[];

/*! @brief The s_smartdmaQEIFirmware firmware memory address. */
#define SMARTDMA_QEI_MEM_ADDR 0x04000000U

/*! @brief Size of s_smartdmaQEIFirmware */
#define SMARTDMA_QEI_FIRMWARE_SIZE (s_smartdmaQEIFirmwareSize)

/*! @brief Size of s_smartdmacameraFirmware */
extern const uint32_t s_smartdmaQEIFirmwareSize;

/*!
 * @brief The API index when using s_smartdmaDisplayFirmware.
 */
enum _smartdma_display_api
{
    kSMARTDMA_FlexIO_DMA_Endian_Swap = 0U,
    kSMARTDMA_FlexIO_DMA_Reverse32,
    kSMARTDMA_FlexIO_DMA,
    kSMARTDMA_FlexIO_DMA_Reverse,              /*!< Send data to FlexIO with reverse order. */
    kSMARTDMA_RGB565To888,                     /*!< Convert RGB565 to RGB888 and save to output memory, use parameter
                                                  smartdma_rgb565_rgb888_param_t. */
    kSMARTDMA_FlexIO_DMA_RGB565To888,          /*!< Convert RGB565 to RGB888 and send to FlexIO, use parameter
                                                  smartdma_flexio_mculcd_param_t. */
    kSMARTDMA_FlexIO_DMA_ARGB2RGB,             /*!< Convert ARGB to RGB and send to FlexIO, use parameter
                                                  smartdma_flexio_mculcd_param_t. */
    kSMARTDMA_FlexIO_DMA_ARGB2RGB_Endian_Swap, /*!< Convert ARGB to RGB, then swap endian, and send to FlexIO, use
                                                 parameter smartdma_flexio_mculcd_param_t. */
    kSMARTDMA_FlexIO_DMA_ARGB2RGB_Endian_Swap_Reverse, /*!< Convert ARGB to RGB, then swap endian and reverse, and send
                                                         to FlexIO, use parameter smartdma_flexio_mculcd_param_t. */
};

/*!
 * @brief Parameter for FlexIO MCULCD
 */
typedef struct _smartdma_flexio_mculcd_param
{
    uint32_t *p_buffer;
    uint32_t buffersize;
    uint32_t *smartdma_stack;
} smartdma_flexio_mculcd_param_t;

/*!
 * @brief Parameter for RGB565To888
 */
typedef struct _smartdma_rgb565_rgb888_param
{
    uint32_t *inBuf;
    uint32_t *outBuf;
    uint32_t buffersize;
    uint32_t *smartdma_stack;
} smartdma_rgb565_rgb888_param_t;

/*!
 * @brief The API index when using s_smartdmaCameraFirmware
 */
enum _smartdma_camera_api
{
    kEZH_Camera_320240_Whole_Buf = 0,
    kEZH_Camera_320240_Div16_Buf = 1,
	kEZH_Camera_480320_Whole_Buf = 2,
	kEZH_Camera_320240_GrayScale_Div4_Buf = 3,
	kEZH_Camera_320240_GrayScale_Div16_Buf = 4,
	kEZH_Camera_384_384_Div16_Buf = 5,
	kEZH_Camera_320480_Whole_Buf = 6,
};

/*!
 * @brief Parameter for camera
 */
typedef struct _smartdma_camera_param
{
		uint32_t *smartdma_stack;   /* user must allocate min 64bytes */
		uint32_t *p_buffer;         /* user must allocate size for pixel data */
		uint32_t *p_stripe_index;         /* user must allocate size for pixel data */
		uint32_t *p_buffer_ping_pong;         /* user must allocate size for pixel data */
} smartdma_camera_param_t;

/*!
 * @brief The API index when using s_smartdmaKeyscanFirmware
 */
enum _smartdma_keyscan_api
{
    /*!using Smartdma to control GPIO . */
    kSMARTDMA_Keyscan_4x4 = 0U,

};

/*!
 * @brief Parameter for keyscan 4x4
 */
typedef struct _smartdma_keyscan_4x4_param
{
    uint32_t *smartdma_stack; /*!< Stack used by SMARTDMA, shall be at least 64 bytes. */
    uint32_t *p_gpio_reg;       /*!< Buffer to provide GPIO register for COL1, COL2, COL3, COL4, ROW1,ROW2,ROW3,ROW4. */
    uint32_t *p_keyvalue;       /*!< Buffer to store key value. */
    uint32_t *p_keycan_interval;       /*!< delay how many system clock cycles. */
} smartdma_keyscan_4x4_param_t;


/*!
 * @brief The API index when using s_smartdmaQEIFirmware
 */
enum _smartdma_qei_api
{
    /*!using Smartdma to qei decoder . */
    kSMARTDMA_qei = 0U,

};

typedef struct _smartdma_QSPI_Para{
  uint32_t   *coprocessor_stack;//offset 0
	uint32_t *debug_buffer;			//offset 1
	uint32_t *command_ID; 			//offset 2
	uint32_t *address_start;		//offset 3
	uint32_t *bytes_lenth;			//offset 4
	uint32_t *temp_rw;				//offset 5
	uint8_t  *read_buffer;			//offset 6
	uint8_t  *write_buffer;			//offset 7
	uint32_t *operation_ret;		//offset 8
} smartdma_QSPI_Para;

/*******************************************************************************
 * APIs
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#if defined(__cplusplus)
}
#endif

/* @} */

#endif /* FSL_SMARTDMA_LPC55_H_ */
