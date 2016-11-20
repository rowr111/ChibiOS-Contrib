/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    i2s_lld.h
 * @brief   KINETIS I2S subsystem low level driver header.
 *
 * @addtogroup I2S
 * @{
 */

#ifndef _I2S_LLD_H_
#define _I2S_LLD_H_

#include "mk22f12_extensions.h"
#include "mcg_lld.h"

#if (HAL_USE_I2S == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    KINETIS configuration options
 * @{
 */
/**
 * @brief   I2SD1 driver enable switch.
 * @details If set to @p TRUE the support for I2S1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(KINETIS_I2S_USE_I2S1) || defined(__DOXYGEN__)
#define KINETIS_I2S_USE_I2S1                  FALSE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef void (*sai_callback_t)(void *parameter);

/*! @brief Define the SAI bus type */
typedef enum _sai_protocol
{
    kSaiBusI2SLeft = 0x0u, /*!< Uses I2S left aligned format. @internal gui name="Left aligned" */
    kSaiBusI2SRight = 0x1u,/*!< Uses I2S right aligned format. @internal gui name="Right aligned" */
    kSaiBusI2SType = 0x2u, /*!< Uses I2S format. @internal gui name="I2S format" */
    kSaiBusPCMA = 0x3u,    /*!< Uses I2S PCM A format. @internal gui name="PCM A format" */
    kSaiBusPCMB = 0x4u,    /*!< Uses I2S PCM B format. @internal gui name="PCM B format" */
    kSaiBusAC97 = 0x5u     /*!< Uses I2S AC97 format. @internal gui name="AC97 format" */
 } sai_protocol_t;

/*! @brief Master or slave mode */
typedef enum _sai_master_slave
{
    kSaiMaster = 0x0u,/*!< Master mode */
    kSaiSlave = 0x1u/*!< Slave mode */
} sai_master_slave_t;

typedef enum _sai_mono_stereo
{
    kSaiMono = 0x0u, /*!< 1 Channel in frame. @internal gui name="Mono" */
    kSaiStereo = 0x1u /*!< 2 Channels in frame. @internal gui name="Stereo" */
} sai_mono_stereo_t;

/*! @brief Synchronous or asynchronous mode */
typedef enum _sai_sync_mode
{
    kSaiModeAsync = 0x0u,/*!< Asynchronous mode @internal gui name="Asynchronous" */
    kSaiModeSync = 0x1u,/*!< Synchronous mode (with receiver or transmit) @internal gui name="Synchronous" */
    kSaiModeSyncWithOtherTx = 0x2u,/*!< Synchronous with another SAI transmit @internal gui name="Synchronous with another transmit" */
    kSaiModeSyncWithOtherRx = 0x3u/*!< Synchronous with another SAI receiver @internal gui name="Synchronous with another receive" */
} sai_sync_mode_t;

/*! @brief Mater clock source */
typedef enum _sai_mclk_source
{
    kSaiMclkSourceSysclk = 0x0u,/*!< Master clock from the system clock @internal gui name="System clock" */
    kSaiMclkSourceSelect1 = 0x1u,/*!< Master clock from source 1 @internal gui name="Input clock 1" */
    kSaiMclkSourceSelect2 = 0x2u,/*!< Master clock from source 2 @internal gui name="Input clock 2" */
    kSaiMclkSourceSelect3 = 0x3u/*!< Master clock from source 3 @internal gui name="Input clock 3" */
} sai_mclk_source_t;

/*! @brief Bit clock source */
typedef enum _sai_bclk_source
{
    kSaiBclkSourceBusclk = 0x0u,/*!< Bit clock using bus clock. @internal gui name="Bus clock" */
    kSaiBclkSourceMclkDiv = 0x1u,/*!< Bit clock using master clock divider. @internal gui name="Master clock" */
    kSaiBclkSourceOtherSai0 = 0x2u,/*!< Bit clock from other SAI device. @internal gui name="From SAI0" */
    kSaiBclkSourceOtherSai1 = 0x3u/*!< Bit clock from other SAI device. @internal gui name="From SAI1" */
} sai_bclk_source_t;

/*! @brief The SAI state flag. */
typedef enum _sai_interrupt_request
{
    kSaiIntrequestWordStart = 0x1000u,/*!< Word start flag, means the first word in a frame detected */
    kSaiIntrequestSyncError = 0x800u,/*!< Sync error flag, means the sync error is detected */
    kSaiIntrequestFIFOWarning = 0x200u,/*!< FIFO warning flag, means the FIFO is empty */
    kSaiIntrequestFIFOError = 0x400u,/*!< FIFO error flag */
    kSaiIntrequestFIFORequest = 0x100u,/*!< FIFO request, means reached watermark */
    kSaiIntrequestAll = 0x1F00 /* All interrupt source */
} sai_interrupt_request_t;

/*! @brief The DMA request sources */
typedef enum _sai_dma_request
{
    kSaiDmaReqFIFOWarning = 0x2u,/*!< FIFO warning caused by the DMA request */
    kSaiDmaReqFIFORequest = 0x1u,/*!< FIFO request caused by the DMA request */
    kSaiDmaReqAll = 0x3u /* All DMA request source */
} sai_dma_request_t;

/*! @brief The SAI state flag */
typedef enum _sai_state_flag
{
    kSaiStateFlagWordStart = 0x100000u,/*!< Word start flag, means the first word in a frame detected. */
    kSaiStateFlagSyncError = 0x80000u,/*!< Sync error flag, means the sync error is detected */
    kSaiStateFlagFIFOError = 0x40000u,/*!< FIFO error flag */
    kSaiStateFlagFIFORequest = 0x10000u, /*!< FIFO request flag. */
    kSaiStateFlagFIFOWarning = 0x20000u, /*!< FIFO warning flag. */
    kSaiStateFlagSoftReset = 0x1000000u, /*!< Software reset flag */
    kSaiStateFlagAll = 0x11F0000u /*!< All flags. */
} sai_state_flag_t;

/*! @brief The reset type */
typedef enum _sai_reset_type
{
    kSaiResetTypeSoftware = 0x1000000u,/*!< Software reset, reset the logic state */
    kSaiResetTypeFIFO = 0x2000000u,/*!< FIFO reset, reset the FIFO read and write pointer */
    kSaiResetAll = 0x3000000u /*!< All reset. */
} sai_reset_type_t;

/*
 * @brief The SAI running mode
 * The mode includes normal mode, debug mode, and stop mode.
 */
typedef enum _sai_running_mode
{
    kSaiRunModeDebug = 0x0,/*!< In debug mode */ 
    kSaiRunModeStop = 0x1/*!< In stop mode */
} sai_run_mode_t;

#if FSL_FEATURE_SAI_HAS_FIFO_PACKING

/*
 * @brief The SAI packing mode
 * The mode includes 8 bit and 16 bit packing.
 */
typedef enum _sai_fifo_packing
{
    kSaiFifoPackingDisabled = 0x0, /*!< Packing disabled. */
    kSaiFifoPacking8bit = 0x2,/*!< 8 bit packing enabled. */
    kSaiFifoPacking16bit = 0x3 /*!< 16bit packing enabled. */
} sai_fifo_packing_t;
#endif

/*! @brief SAI clock configuration structure. */
typedef struct SaiClockSetting
{
    sai_mclk_source_t mclk_src; /*!< Master clock source. */
    sai_bclk_source_t bclk_src; /*!< Bit clock source. */
    uint32_t mclk_src_freq; /*!< Master clock source frequency. */
    uint32_t mclk; /*!< Master clock frequency. */
    uint32_t bclk; /*!< Bit clock frequency. */
    uint32_t bclk_src_freq; /* Bit clock source frequency. */
} sai_clock_setting_t;


typedef enum _sai_status
{
    kStatus_SAI_Success = 0U,
    kStatus_SAI_Fail = 1U,
    kStatus_SAI_DeviceBusy = 2U
} sai_status_t;

/*! @brief Defines the PCM data format
 *  @internal gui name="Audio data configuration" id="saiDataCfg"
 */
typedef struct SaiAudioDataFormat
{
    uint32_t sample_rate;/*!< Sample rate of the PCM file. @internal gui name="Sample rate" id="SampleRate" default="48000" */
    uint32_t mclk;/*!< Master clock frequency. @internal gui name="Master clock frequency" id="CfgMclk" */
    uint8_t  bits;/*!< Number of bits in a word. @internal gui name="Bits" id="Bits" default="32" */
    sai_mono_stereo_t  mono_stereo;/*!< Number of words in a frame. @internal gui name="Mode" id="Words" default="1" */
} sai_data_format_t;

/*! @brief SAI internal state 
* Users should allocate and transfer memory to the PD during the initialization function.
* Note: During the SAI execution, users should not free the state. Otherwise, the driver malfunctions.
*/
typedef struct sai_state
{
  sai_data_format_t format;
  uint8_t * address; // current address pointer, changes from original buffer offset
  uint32_t len;  // length of buffer, in samples
  uint32_t count;  // current count, in samples
  sai_callback_t  callback;
  void * callback_param;
  sai_sync_mode_t sync_mode;
  uint32_t fifo_channel;
#if (FSL_FEATURE_SAI_FIFO_COUNT > 1)
  uint32_t watermark;
#endif
  sai_master_slave_t master_slave;
  sai_protocol_t protocol;
#if defined FSL_FEATURE_EDMA_MODULE_CHANNEL
  //    edma_chn_state_t edma_chn;
  //    edma_software_tcd_t tcd[2];
#else
  uint8_t chn;
#endif
  //    semaphore_t sem;
  bool use_dma;
  uint32_t dma_source;
} sai_state_t;

/*! @brief The description structure for the SAI transmit/receive module.
 *  @internal gui name="Basic configuration" id="saiCfg"
 */
typedef struct SaiUserConfig
{
    sai_mclk_source_t   mclk_source;/*!< Master clock source. @internal gui name="MCLK source" id="CfgMclkSource" */
    uint8_t             channel;/*!< Which FIFO is used to transfer. @internal gui name="Channel" id="Channel" */
    sai_sync_mode_t     sync_mode;/*!< Synchronous or asynchronous. @internal gui name="Mode" id="Mode" */
    sai_protocol_t           protocol;/*!< I2S left, I2S right, or I2S type. @internal gui name="Protocol" id="BusType" default="2" */
    sai_master_slave_t  slave_master;/*!< Master or slave. @internal gui name="Master / Slave mode" id="MasterSlave" */
    sai_bclk_source_t   bclk_source;/*!< Bit clock from master clock or other modules. @internal gui name="Bit clock source" id="BclkSource" default="1" */
#if FSL_FEATURE_SAI_FIFO_COUNT > 1
    uint32_t    watermark;/*!< Time when to send an interrupt or the DMA request. @internal gui name="Watermark" id="Watermark" */
#endif
    uint32_t    dma_source; /*!< The DMA request source. @internal gui name="DMA request value" id="DmaRequest" */
}  sai_user_config_t;



/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*!
* @name Master clock configuration
* @{
*/

/*!
 * @brief Sets the master clock source.
 *
 * The source of the clock is different in each SoC.
 * This function sets the clock source for the SAI master clock source.
 * The master clock produces the bit clock for the data transfer.
 * @param base Register base address of SAI module.
 * @param source Mater clock source
 */
static inline void SAI_HAL_SetMclkSrc(I2S_TypeDef * base, sai_mclk_source_t source)
{
    I2S_BWR_MCR_MICS(base,source);
}

/*!
 * @brief Gets the master clock source.
 *
 * The source of the clock is different in each SoCs.
 * This function gets the clock source for the SAI master clock source.
 * The master clock produces the bit clock for the data transfer.
 * @param base Register base address of SAI module.
 * @return Mater clock source
 */
static inline uint32_t SAI_HAL_GetMclkSrc(I2S_TypeDef * base)
{
    return I2S_BRD_MCR_MICS(base);
}

/*!
 * @brief Enables or disables the MCLK internal.
 * 
 * This function enables or disables the internal MCLK.
 * @param base Register base address of SAI module.
 * @param enable True means enable, false means disable.
 */
static inline void SAI_HAL_SetMclkDividerCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_MCR_MOE(base,enable);
}

/*! @}*/

/*!
* @name Bit clock configuration
* @{
*/

/*!
 * @brief Sets the bit clock source of transmit. It is generated by the master clock, bus clock, and other devices.
 *
 * The function sets the source of the bit clock. The bit clock can be produced by the master
 * clock and from the bus clock or other SAI transmit/receive. Transmit and receive in the SAI module use the same bit 
 * clock either from transmit or receive. 
 * @param base Register base address of SAI module.
 * @param source Bit clock source.
 */
static inline void SAI_HAL_TxSetBclkSrc(I2S_TypeDef * base, sai_bclk_source_t source)
{
    I2S_BWR_TCR2_MSEL(base,source);
}

/*!
 * @brief Sets bit clock source of the receive. It is generated by the master clock, bus clock, and other devices.
 *
 * The function sets the source of the bit clock. The bit clock can be produced by the master
 * clock, and from the bus clock or other SAI transmit/receive. Transmit and receive in the SAI module use the same bit 
 * clock either from transmit or receive.
 * @param base Register base address of SAI module.
 * @param source Bit clock source.
 */
static inline void SAI_HAL_RxSetBclkSrc(I2S_TypeDef * base, sai_bclk_source_t source)
{
    I2S_BWR_RCR2_MSEL(base,source);
}

/*!
 * @brief Gets the bit clock source of transmit. It is generated by the master clock, bus clock, and other devices.
 *
 * The function gets the source of the bit clock. The bit clock can be produced by the master
 * clock and from the bus clock or other SAI transmit/receive. Transmit and receive in the SAI module use the same bit 
 * clock either from transmit or receive.
 * @param base Register base address of SAI module.
 * @return Bit clock source.
 */
static inline uint32_t SAI_HAL_TxGetBclkSrc(I2S_TypeDef * base)
{
    return I2S_BRD_TCR2_MSEL(base);
}

/*!
 * @brief Gets bit clock source of the receive. It is generated by the master clock, bus clock and other devices.
 *
 * The function gets the source of the bit clock. The bit clock can be produced by the master
 * clock, and from the bus clock or other SAI transmit/receive. Transmit and receive in the SAI module use the same bit 
 * clock either from transmit or receive.
 * @param base Register base address of SAI module.
 * @return Bit clock source.
 */
static inline uint32_t SAI_HAL_RxGetBclkSrc(I2S_TypeDef * base)
{
    return I2S_BRD_RCR2_MSEL(base);
}

/*!
 * @brief Sets the transmit bit clock divider value.
 *
 * bclk = mclk / divider. At the same time, bclk = sample_rate * channel * bits. This means
 * how much time is needed to transfer one bit.
 * Note that the function is called while the bit clock source is the master clock.
 * @param base Register base address of SAI module.
 * @param divider The divide number of bit clock.
 */
static inline void SAI_HAL_TxSetBclkDiv(I2S_TypeDef * base, uint32_t divider)
{
    I2S_BWR_TCR2_DIV(base,divider/2 -1);
}

/*!
 * @brief Sets the receive bit clock divider value.
 *
 * bclk = mclk / divider. At the same time, bclk = sample_rate * channel * bits. This means
 * how much time is needed to transfer one bit.
 * Note that the function is called while the bit clock source is the master clock.
 * @param base Register base address of SAI module.
 * @param divider The divide number of bit clock.
 */
static inline void SAI_HAL_RxSetBclkDiv(I2S_TypeDef * base, uint32_t divider)
{
    I2S_BWR_RCR2_DIV(base,divider/2 -1);
}

/*!
 * @brief Enables or disables the transmit bit clock input bit.
 * 
 * @param base Register base address of SAI module.
 * @param enable True means enable, false means disable.
 */
static inline void SAI_HAL_TxSetBclkInputCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_TCR2_BCI(base,enable);
}

/*!
 * @brief Enables or disables the receive bit clock input bit.
 * 
 * @param base Register base address of SAI module.
 * @param enable True means enable, false means disable.
 */
static inline void SAI_HAL_RxSetBclkInputCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_RCR2_BCI(base,enable);
}
/*!
 * @brief Sets the transmit bit clock swap.
 *
 * This field swaps the bit clock used by the transmitter. When the transmitter is configured in 
 * asynchronous mode and this bit is set, the transmitter is clocked by the receiver bit clock. 
 * This allows the transmitter and receiver to share the same bit clock, but the transmitter 
 * continues to use the transmit frame sync (SAI_TX_SYNC).
 * When the transmitter is configured in synchronous mode, the transmitter BCS field and receiver
 * BCS field must be set to the same value. When both are set, the transmitter and receiver are both
 * clocked by the transmitter bit clock (SAI_TX_BCLK) but use the receiver frame sync (SAI_RX_SYNC).
 * @param base Register base address of SAI module.
 * @param enable True means swap bit clock; false means no swap.
 */
static inline void SAI_HAL_TxSetSwapBclkCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_TCR2_BCS(base,enable);
}

/*!
 * @brief Sets the receive bit clock swap.
 *
 * This field swaps the bit clock used by the receiver. When the receiver is configured in 
 * asynchronous mode and this bit is set, the receiver is clocked by the transmitter bit clock
 * (SAI_TX_BCLK). This allows the transmitter and receiver to share the same bit clock, but the 
 * receiver continues to use the receiver frame sync (SAI_RX_SYNC). 
 * When the receiver is configured in synchronous mode, the transmitter BCS field and receiver BCS 
 * field must be set to the same value. When both are set, the transmitter and receiver are both 
 * clocked by the receiver bit clock (SAI_RX_BCLK) but use the transmitter frame sync (SAI_TX_SYNC).
 * @param base Register base address of SAI module.
 * @param enable True means swap bit clock; false means no swap.
 */
static inline void SAI_HAL_RxSetSwapBclkCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_RCR2_BCS(base, enable);
}
/*! @} */

#if (FSL_FEATURE_SAI_FIFO_COUNT > 1)
/*!
* @name watermark settings
* @{
*/

/*!
 * @brief Sets the transmit watermark value.
 *
 * While the value in the FIFO is less or equal to the watermark , it generates an interrupt 
 * request or a DMA request. The watermark value cannot be greater than the depth of FIFO.
 * @param base Register base address of SAI module.
 * @param watermark Watermark value of a FIFO.
 */
static inline void SAI_HAL_TxSetWatermark(I2S_TypeDef * base, uint32_t watermark)
{
    I2S_BWR_TCR1_TFW(base, watermark);
}

/*!
 * @brief Sets the transmit watermark value.
 *
 * While the value in the FIFO is more or equal to the watermark , it generates an interrupt 
 * request or a DMA request. The watermark value cannot be greater than the depth of FIFO.
 * @param base Register base address of SAI module.
 * @param watermark Watermark value of a FIFO.
 */
static inline void SAI_HAL_RxSetWatermark(I2S_TypeDef * base, uint32_t watermark)
{
    I2S_BWR_RCR1_RFW(base, watermark);
}

/*!
 * @brief Gets the transmit watermark value.
 *
 * @param base Register base address of SAI module.
 * @return The transmit watermark value.
 */
static inline uint32_t SAI_HAL_TxGetWatermark(I2S_TypeDef * base)
{
    return I2S_BRD_TCR1_TFW(base);
}

/*!
 * @brief Gets the receive watermark value.
 *
 * @param base Register base address of SAI module.
 * @return The transmit watermark value.
 */
static inline uint32_t SAI_HAL_RxGetWatermark(I2S_TypeDef * base)
{
    return I2S_BRD_RCR1_RFW(base);
}

#endif

/*! @}*/

/*!
 * @brief Gets the TDR register address.
 *
 * This function determines the destination/source address of the DMA transfer.
 * @param base Register base address of SAI module.
 * @param fifo_channel FIFO channel selected.
 * @return TDR register or RDR register address
 */
static inline uint32_t SAI_HAL_TxGetFifoAddr(I2S_TypeDef * base, uint32_t fifo_channel)
{
    return (uint32_t)(&I2S_TDR_REG(base, fifo_channel));
}

/*!
 * @brief Gets the RDR register address.
 *
 * This function determines the destination/source address of the DMA transfer.
 * @param base Register base address of SAI module.
 * @param fifo_channel FIFO channel selected.
 * @return TDR register or RDR register address
 */
static inline uint32_t SAI_HAL_RxGetFifoAddr(I2S_TypeDef * base, uint32_t fifo_channel)
{
    return (uint32_t)(&I2S_RDR_REG(base, fifo_channel));
}

/*!
 * @brief Enables the SAI transmit module.
 *
 * Enables the transmit. This function enables both the bit clock and the transfer channel.
 * @param base Register base address of SAI module.
 */
static inline void SAI_HAL_TxEnable(I2S_TypeDef * base)
{
    I2S_BWR_TCSR_BCE(base,true);
    I2S_BWR_TCSR_TE(base,true);
}

/*!
 * @brief Enables the SAI receive module.
 *
 * Enables the receive. This function enables both the bit clock and the receive channel.
 * @param base Register base address of SAI module.
 */
static inline void SAI_HAL_RxEnable(I2S_TypeDef * base)
{
    I2S_BWR_RCSR_BCE(base,true);    
    I2S_BWR_RCSR_RE(base,true);
}

/*!
 * @brief Disables the transmit module.
 *
 * Disables the transmit. This function disables both the bit clock and the transfer channel.
 * @param base Register base address of SAI module.
 */
static inline void SAI_HAL_TxDisable(I2S_TypeDef * base)
{
    I2S_BWR_TCSR_TE(base,false);
    I2S_BWR_TCSR_BCE(base,false);
}

/*!
 * @brief Disables the receive module.
 *
 * Disables the receive. This function disables both the bit clock and the receive channel.
 * @param base Register base address of SAI module.
 */
static inline void SAI_HAL_RxDisable(I2S_TypeDef * base)
{
    I2S_BWR_RCSR_RE(base,false);
    I2S_BWR_RCSR_BCE(base,false);
}

/*!
 * @brief Sets the transmit FIFO channel.
 *
 * A SAI base includes a transmit and an receive. Each has several channels according to 
 * different platforms. A channel means a path for the audio data input/output.
 * @param base Register base address of SAI module.
 * @param fifo_channel FIFO channel number.
 */
static inline void SAI_HAL_TxSetDataChn(I2S_TypeDef * base, uint8_t fifo_channel)
{
    I2S_BWR_TCR3_TCE(base, 1u << fifo_channel);
}

/*!
 * @brief Sets the receive FIFO channel.
 *
 * A SAI base includes a transmit and a receive. Each has several channels according to 
 * different platforms. A channel means a path for the audio data input/output.
 * @param base Register base address of SAI module.
 * @param fifo_channel FIFO channel number.
 */
static inline void SAI_HAL_RxSetDataChn(I2S_TypeDef * base, uint8_t fifo_channel)
{
    I2S_BWR_RCR3_RCE(base, 1u << fifo_channel);
}

/*!
 * @brief Gets the state of the flags in the TCSR.
 * @param base Register base address of SAI module.
 * @param flag_mask State flag type, it can be FIFO error, FIFO warning and so on.
 * @return True if detect word start otherwise false.
 */
static inline uint32_t SAI_HAL_TxGetStateFlag(I2S_TypeDef * base, uint32_t flag_mask)
{
    return (I2S_RD_TCSR(base) & flag_mask);
}

/*!
 * @brief Gets the state of the flags in the RCSR.
 * @param base Register base address of SAI module.
 * @param flag_mask State flag type, it can be FIFO error, FIFO warning and so on.
 * @return True if detect word start otherwise false.
 */
static inline uint32_t SAI_HAL_RxGetStateFlag(I2S_TypeDef * base, uint32_t flag_mask)
{
    return (I2S_RD_RCSR(base) & flag_mask);
}

/*!
 * @brief Receives the data from the FIFO.
 * @param base Register base address of SAI module.
 * @param rx_channel receive FIFO channel.
 * @param data Pointer to the address to be written in.
 * @return Received data. 
 */
static inline uint32_t SAI_HAL_ReceiveData(I2S_TypeDef * base, uint32_t rx_channel)
{
  osalDbgAssert((rx_channel < FSL_FEATURE_SAI_CHANNEL_COUNT), "rx channel too large");   
    return I2S_RD_RDR(base, rx_channel);
}

/*!
 * @brief Transmits data to the FIFO.
 * @param base Register base address of SAI module.
 * @param tx_channel transmit FIFO channel.
 * @param data Data value which needs to be written into FIFO.
 */
static inline void SAI_HAL_SendData(I2S_TypeDef * base, uint32_t tx_channel, uint32_t data)
{
  osalDbgAssert((tx_channel < FSL_FEATURE_SAI_CHANNEL_COUNT), "tx channel too large");  
    I2S_WR_TDR(base,tx_channel,data);
}

#if FSL_FEATURE_SAI_HAS_ON_DEMAND_MODE
/*!
 * @brief Transmits on-demand mode setting.
 *
 * When set, the frame sync is generated internally. A frame sync is only generated when the 
 * FIFO warning flag is clear.
 * @param base Register base address of SAI module.
 * @param enable True means on demand mode enable, false means disable.
 */
static inline void SAI_HAL_TxSetOndemandCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_TCR4_ONDEM(base, enable);
}

/*!
 * @brief Receives on-demand mode setting.
 *
 * When set, the frame sync is generated internally. A frame sync is only generated when the 
 * FIFO warning flag is clear.
 * @param base Register base address of SAI module.
 * @param enable True means on demand mode enable, false means disable.
 */
static inline void SAI_HAL_RxSetOndemandCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_RCR4_ONDEM(base, enable);
}
#endif

#if FSL_FEATURE_SAI_HAS_FIFO_FUNCTION_AFTER_ERROR
/*!
 * @brief Transmits the FIFO continues on error.
 *
 * Configures when the SAI continues transmitting after a FIFO error has been detected.
 * @param base Register base address of SAI module.
 * @param enable True means on demand mode enable, false means disable.
 */
static inline void SAI_HAL_TxSetFIFOErrorContinueCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_TCR4_FCONT(base, enable);
}

/*!
 * @brief Receives the FIFO continues on error.
 *
 * Configures when the SAI continues transmitting after a FIFO error has been detected.
 * @param base Register base address of SAI module.
 * @param enable True means on demand mode enable, false means disable.
 */
static inline void SAI_HAL_RxSetFIFOErrorContinueCmd(I2S_TypeDef * base, bool enable)
{
    I2S_BWR_RCR4_FCONT(base, enable);
}
#endif

#if FSL_FEATURE_SAI_HAS_FIFO_PACKING
/*!
 * @brief Transmits the FIFO packing mode setting.
 *
 * Enables packing 8-bit data or 16-bit data into each 32-bit FIFO word. If the word size is 
 * greater than 8-bit or 16-bit, only the first 8-bit or 16-bits are loaded from the FIFO. 
 * The first word in each frame always starts with a new 32-bit FIFO word and the first bit shifted
 * must be configured within the first packed word. When FIFO packing is enabled, the FIFO write
 * pointer only increments when the full 32-bit FIFO word has been written by software.
 * @param base Register base address of SAI module.
 * @param mode FIFO packing mode.
 */
static inline void SAI_HAL_TxSetFIFOPackingMode(I2S_TypeDef * base, sai_fifo_packing_t mode)
{
    I2S_BWR_TCR4_FPACK(base,mode);
}

/*!
 * @brief Receives the FIFO packing mode setting.
 *
 * Enables packing 8-bit data or 16-bit data into each 32-bit FIFO word. If the word size is 
 * greater than 8-bit or 16-bit, only the first 8-bit or 16-bits are loaded from the FIFO. 
 * The first word in each frame always starts with a new 32-bit FIFO word and the first bit shifted
 * must be configured within the first packed word. When FIFO packing is enabled, the FIFO write
 * pointer only increments when the full 32-bit FIFO word has been written by software.
 * @param base Register base address of SAI module.
 * @param mode FIFO packing mode.
 */
static inline void SAI_HAL_RxSetFIFOPackingMode(I2S_TypeDef * base, sai_fifo_packing_t mode)
{
    I2S_BWR_RCR4_FPACK(base,mode);
}
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a structure representing an I2S driver.
 */
typedef struct I2SDriver I2SDriver;

/**
 * @brief   I2S notification callback type.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 * @param[in] offset    offset in buffers of the data to read/write
 * @param[in] n         number of samples to read/write
 */
typedef void (*i2scallback_t)(I2SDriver *i2sp, size_t offset, size_t n);

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief   Transmission buffer pointer.
   * @note    Can be @p NULL if TX is not required.
   */
  const void                *tx_buffer;
  /**
   * @brief   Receive buffer pointer.
   * @note    Can be @p NULL if RX is not required.
   */
  void                      *rx_buffer;
  /**
   * @brief   TX and RX buffers size as number of samples.
   */
  size_t                    size;
  /**
   * @brief   Callback function called during streaming.
   */
  i2scallback_t             end_cb;
  /* End of the mandatory fields.*/

  sai_state_t               sai_tx_state;  // state for the TX channel
  sai_state_t               sai_rx_state;  // state for the RX channel
  sai_user_config_t         sai_tx_userconfig;  // config for the TX bank
  sai_user_config_t         sai_rx_userconfig;  // config for the RX bank
} I2SConfig;

/**
 * @brief   Structure representing an I2S driver.
 */
struct I2SDriver {
  /**
   * @brief   Driver state.
   */
  i2sstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  I2S_TypeDef               *I2S; // pointer to I2S hardware bank

  I2SConfig           *config;
  /* End of the mandatory fields.*/
};


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (KINETIS_I2S_USE_I2S1 == TRUE) && !defined(__DOXYGEN__)
extern I2SDriver I2SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void i2s_lld_init(void);
  void i2s_lld_start(I2SDriver *i2sp);
  void i2s_lld_stop(I2SDriver *i2sp);
  void i2s_lld_start_exchange(I2SDriver *i2sp);
  void i2s_lld_stop_exchange(I2SDriver *i2sp);
  void i2s_lld_start_rx(I2SDriver *i2sp);
  void i2s_lld_stop_rx(I2SDriver *i2sp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2S == TRUE */

#endif /* _I2S_LLD_H_ */

/** @} */
