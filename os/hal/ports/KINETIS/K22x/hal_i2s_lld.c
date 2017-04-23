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
 * @file    i2s_lld.c
 * @brief   KINETIS I2S subsystem low level driver source.
 *
 * @addtogroup I2S
 * @{
 */
#include "hal.h"
#include <string.h>

uint32_t rx_int_count = 0;

#if HAL_USE_I2S || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief I2S2 driver identifier.*/
#if KINETIS_I2S_USE_I2S1 || defined(__DOXYGEN__)
I2SDriver I2SD1;    // this declares the I2SD1 structure, but it's not initialized
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*!
* @name Module control
* @{
*/

/*!
 * @brief  Initializes the SAI transmit.
 *
 * The initialization resets the SAI module by setting the SR bit of TCSR register.
 * Note that the function writes 0 to every control registers.
 * @param base Register base address of SAI module.
 */
static void SAI_HAL_TxInit(I2S_TypeDef * base);

/*!
 * @brief  Initializes the SAI receive.
 *
 * The initialization resets the SAI module by setting the SR bit of RCSR register.
 * Note that the function writes 0 to every control registers.
 * @param base Register base address of SAI module.
 */
static void SAI_HAL_RxInit(I2S_TypeDef * base);

/*!
 * @brief Sets transmit protocol relevant settings.
 *
 * The bus mode indicates which protocol SAI uses. It can be I2S left, right, and so on. Each protocol
 * has a different configuration on bit clock and frame sync.
 * @param base Register base address of SAI module.
 * @param protocol The protocol selection. It can be I2S left aligned, I2S right aligned, and so on.
 */
static void SAI_HAL_TxSetProtocol(I2S_TypeDef * base, sai_protocol_t protocol);

/*!
 * @brief Sets receive protocol relevant settings.
 *
 * The bus mode indicates which protocol SAI uses. It can be I2S left, right, and so on. Each protocol
 * has a different configuration on bit clock and frame sync.
 * @param base Register base address of SAI module.
 * @param protocol The protocol selection. It can be I2S left aligned, I2S right aligned, and so on.
 */
static void SAI_HAL_RxSetProtocol(I2S_TypeDef * base, sai_protocol_t protocol);

/*!
 * @brief Sets master or slave mode.
 *
 * The function determines master or slave mode. Master mode  provides its
 * own clock and slave mode  uses an external clock.
 * @param base Register base address of SAI module.
 * @param master_slave_mode Mater or slave mode.
 */
static void SAI_HAL_TxSetMasterSlave(I2S_TypeDef * base, sai_master_slave_t master_slave_mode);

/*!
 * @brief Sets master or slave mode.
 *
 * The function determines master or slave mode. Master mode provides its
 * own clock and slave mode  uses external clock.
 * @param base Register base address of SAI module.
 * @param master_slave_mode Mater or slave mode.
 */
static void SAI_HAL_RxSetMasterSlave(I2S_TypeDef * base, sai_master_slave_t master_slave_mode);

/*! @}*/

/*!
* @name Overall Clock configuration
* @{
*/

/*!
 * @brief Sets up the clock for the SAI transmit.
 *
 * This function can sets the clock settings according to the configuration structure.
 * In this configuration setting structure, users can set clock source, clock source frequency,
 * and frequency of the master clock and bit clock.
 * If the bit clock source is the master clock, the master clock frequency should equal to the bit clock source
 * frequency. If the bit clock source is not the master clock, then settings of the master clock have no
 * effect on the setting.
 * @param base Register base address of SAI module.
 * @param clk_config Pointer to SAI clock configuration structure.
 */
static void SAI_HAL_TxClockSetup(I2S_TypeDef * base, sai_clock_setting_t *clk_config);

/*!
 * @brief Sets up the clock for the SAI receive.
 *
 * This function sets the clock settings according to the configuration structure.
 * In this configuration setting structure, users can set clock source, clock source frequency,
 * and frequency of the master clock and bit clock.
 * If the bit clock source is the master clock, the master clock frequency should equal to the bit clock source
 * frequency. If the bit clock source is not the master clock, then settings of the master clock have no
 * effect on the setting.
 * @param base Register base address of SAI module.
 * @param clk_config Pointer to SAI clock configuration structure.
 */
static void SAI_HAL_RxClockSetup(I2S_TypeDef * base, sai_clock_setting_t *clk_config);

/*! @}*/

/*!
* @name Mono or stereo configuration
* @{
*/

/*!
 * @brief Sets the transmit audio channel number. The channel can be mono or stereo. 
 * 
 * @param base Register base address of SAI module.
 * @param mono_stereo Mono or stereo mode.
 */
static void SAI_HAL_TxSetMonoStereo(I2S_TypeDef * base, sai_mono_stereo_t mono_stereo);

/*!
 * @brief Sets the receive audio channel number. The channel can be mono or stereo. 
 * 
 * @param base Register base address of SAI module.
 * @param mono_stereo Mono or stereo mode.
 */
static void SAI_HAL_RxSetMonoStereo(I2S_TypeDef * base, sai_mono_stereo_t mono_stereo);

/*! @} */

/*!
* @name Word configurations
* @{
*/

/*!
 * @brief Sets the transmit word width. 
 * 
 * This interface is for I2S and PCM series protocol. It sets the width of the first word and any other word 
 * in the same manner. At the same time, for I2S series protocol, it sets the frame sync width equal to the 
 * word width.
 * @param base Register base address of SAI module.
 * @param protocol Protocol used for transmit now.
 * @param bits transmit word width.
 */
static void SAI_HAL_TxSetWordWidth(I2S_TypeDef * base, sai_protocol_t protocol, uint32_t bits);

/*!
 * @brief Sets the receive word width. 
 * 
 * This interface is for I2S and PCM series protocol. It sets the width of the first word and any other word 
 * in the same manner. At the same time, for I2S series protocol, it sets the frame sync width equal to the 
 * word width.
 * @param base Register base address of SAI module.
 * @param protocol Protocol used for receive now.
 * @param bits receive word width.
 */
static void SAI_HAL_RxSetWordWidth(I2S_TypeDef * base, sai_protocol_t protocol, uint32_t bits);

/*!@}*/

/*!
 * @brief SAI transmit sync mode setting. 
 *
 * The mode can be asynchronous mode, synchronous, or synchronous with another SAI device.
 * When configured for a synchronous mode of operation, the receiver must be configured for the 
 * asynchronous operation.
 * @param base Register base address of SAI module.
 * @param sync_mode Synchronous mode or Asynchronous mode.
 */
static void SAI_HAL_TxSetSyncMode(I2S_TypeDef * base, sai_sync_mode_t sync_mode);

/*!
 * @brief SAI receive sync mode setting. 
 *
 * The mode can be asynchronous mode, synchronous, or synchronous with another SAI device.
 * When configured for a synchronous mode of operation, the receiver must be configured for the 
 * asynchronous operation.
 * @param base Register base address of SAI module.
 * @param sync_mode Synchronous mode or Asynchronous mode.
 */
static void SAI_HAL_RxSetSyncMode(I2S_TypeDef * base, sai_sync_mode_t sync_mode);

#if (FSL_FEATURE_SAI_FIFO_COUNT > 1)
/*!
 * @brief Gets the transmit FIFO read and write pointer.
 *
 * It is used to determine whether the FIFO is full or empty and know how much space there is for FIFO.
 * If read_ptr == write_ptr, the FIFO is empty. While the bit of the read_ptr and the write_ptr are
 * equal except for the MSB, the FIFO is full.
 * @param base Register base address of SAI module.
 * @param fifo_channel FIFO channel selected.
 * @param r_ptr Pointer to get transmit FIFO read pointer.
 * @param w_ptr Pointer to get transmit FIFO write pointer.
 */
static void SAI_HAL_TxGetFifoWRPointer(I2S_TypeDef * base,  uint32_t fifo_channel, 
       uint32_t * r_ptr, uint32_t * w_ptr);

/*!
 * @brief Gets the receive FIFO read and write pointer.
 *
 * It is used to determine whether the FIFO is full or empty and know how much space there is for FIFO.
 * If read_ptr == write_ptr, the FIFO is empty. While the bit of the read_ptr and the write_ptr are
 * equal except for the MSB, the FIFO is full.
 * @param base Register base address of SAI module.
 * @param fifo_channel FIFO channel selected.
 * @param r_ptr Pointer to get receive FIFO read pointer.
 * @param w_ptr Pointer to get receive FIFO write pointer.
 */
static void SAI_HAL_RxGetFifoWRPointer(I2S_TypeDef * base,  uint32_t fifo_channel,
        uint32_t * r_ptr, uint32_t * w_ptr);
#endif


#if FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER
/*!
 * @brief Sets the divider of the master clock.
 *
 * Using the divider to get the master clock frequency wanted from the source. 
 * mclk = clk_source * fract/divide. The input is the master clock frequency needed and the source clock frequency.
 * The master clock is decided by the sample rate and the multi-clock number.
 * Notice that mclk should be less than src_clk, or it hangs because the hardware refuses to write in this situation.
 * @param base Register base address of SAI module.
 * @param mclk Master clock frequency needed.
 * @param src_clk The source clock frequency.
 */
static void SAI_HAL_SetMclkDiv(I2S_TypeDef * base, uint32_t mclk, uint32_t src_clk);
#endif

/*!
 * @brief Enables the transmit interrupt from different interrupt sources.
 *
 * The interrupt source can be : Word start flag, Sync error flag, FIFO error flag, FIFO warning flag, and FIFO request flag.
 * This function sets which flag causes an interrupt request. 
 * @param base Register base address of SAI module.
 * @param source SAI interrupt request source.
 * @param enable Enable or disable.
 */
static void SAI_HAL_TxSetIntCmd(I2S_TypeDef * base, uint32_t source, bool enable);

/*!
 * @brief Enables the receive interrupt from different interrupt sources.
 *
 * The interrupt source can be : Word start flag, Sync error flag, FIFO error flag, FIFO warning flag, and FIFO request flag.
 * This function sets which flag causes an interrupt request. 
 * @param base Register base address of SAI module.
 * @param source SAI interrupt request source.
 * @param enable Enable or disable.
 */
static void SAI_HAL_RxSetIntCmd(I2S_TypeDef * base, uint32_t source, bool enable);

/*!
 * @brief Enables the transmit DMA request from different sources.
 *
 * The DMA sources can be: FIFO warning and FIFO request.
 * This function enables the DMA request from different DMA request sources.
 * @param base Register base address of SAI module.
 * @param source SAI DMA request source.
 * @param enable Enable or disable.
 */
static void SAI_HAL_TxSetDmaCmd(I2S_TypeDef * base, uint32_t source, bool enable);

/*!
 * @brief Enables the receive DMA request from different sources.
 *
 * The DMA sources can be: FIFO warning and FIFO request.
 * This function enables the DMA request from different DMA request sources.
 * @param base Register base address of SAI module.
 * @param source SAI DMA request source.
 * @param enable Enable or disable.
 */
static void SAI_HAL_RxSetDmaCmd(I2S_TypeDef * base, uint32_t source, bool enable);

/*!
 * @brief Clears the transmit state flags.
 *
 * The function clears the flags manually. It can clear word start, FIFO warning, FIFO error, and 
 * FIFO request flag.
 * @param base Register base address of SAI module.
 * @param flag_mask SAI state flag type. The flag can be word start, sync error, FIFO error/warning.
 */
static void SAI_HAL_TxClearStateFlag(I2S_TypeDef * base, uint32_t flag_mask);

/*!
 * @brief Clears the receive state flags.
 *
 * The function is used to clear the flags manually. It can clear word start, FIFO warning, FIFO error, and
 * FIFO request flag.
 * @param base Register base address of SAI module.
 * @param flag_mask SAI state flag type. The flag can be word start, sync error, FIFO error/warning.
 */
static void SAI_HAL_RxClearStateFlag(I2S_TypeDef * base, uint32_t flag_mask);

/*!
 * @brief Resets the transmit module.
 *
 * There are two kinds of reset: software reset and FIFO reset.
 * Software reset: resets all transmitter internal logic, including the bit clock generation, 
 * status flags, and FIFO pointers. It does not reset the configuration registers.
 * FIFO reset: synchronizes the FIFO write pointer to the same value as the FIFO read pointer. 
 * This empties the FIFO contents and is to be used after the Transmit FIFO Error Flag is set,
 * and before the FIFO is re-initialized and the Error Flag is cleared.
 * @param base Register base address of SAI module.
 * @param reset_mask SAI reset mask.
 */
static void SAI_HAL_TxSetReset(I2S_TypeDef * base, uint32_t reset_mask);

/*!
 * @brief Resets the receive module.
 *
 * There are two kinds of reset: software reset and FIFO reset.
 * Software reset: resets all transmitter internal logic, including the bit clock generation, 
 * status flags and FIFO pointers. It does not reset the configuration registers.
 * FIFO reset: synchronizes the FIFO write pointer to the same value as the FIFO read pointer. 
 * This empties the FIFO contents and is to be used after the Transmit FIFO Error Flag is set,
 * and before the FIFO is re-initialized and the Error Flag is cleared.
 * @param base Register base address of SAI module.
 * @param reset_mask SAI reset mask.
 */
static void SAI_HAL_RxSetReset(I2S_TypeDef * base, uint32_t reset_mask);

/*!
 * @brief Sets the running mode of the transmit. There is a debug mode, stop mode, and a normal mode.
 *
 * This function can set the working mode of the SAI base. Stop mode is always 
 * used in low power cases, and the debug mode disables the  SAI after the current 
 * transmit/receive is completed.
 * @param base Register base address of SAI module.
 * @param run_mode SAI running mode.
 * @param enable Enable or disable a mode.
 */
static void SAI_HAL_TxSetRunModeCmd(I2S_TypeDef * base, sai_run_mode_t run_mode, bool enable);

/*!
 * @brief Sets the running mode of the receive. There is a debug mode, stop mode, and a normal mode.
 *
 * This function can set the working mode of the SAI base. Stop mode is always 
 * used in low power cases, and the debug mode disables the  SAI after the current 
 * transmit/receive is completed.
 * @param base Register base address of SAI module.
 * @param run_mode SAI running mode.
 * @param enable Enable or disable a mode.
 */
static void SAI_HAL_RxSetRunModeCmd(I2S_TypeDef * base, sai_run_mode_t run_mode, bool enable);

/*!
 * @brief Uses blocking to receive data.
 * @param base The SAI base.
 * @param rx_channel receive FIFO channel.
 * @param rxBuff receive data buffer.
 * @param size receive data size.
 */
static void SAI_HAL_ReceiveDataBlocking(I2S_TypeDef * base, uint32_t rx_channel,
    uint8_t * rxBuff, uint32_t size);

/*!
 * @brief Uses blocking to send data.
 * @param base The SAI base.
 * @param tx_channel transmit FIFO channel.
 * @param txBuff transmit data buffer.
 * @param size transmit data size.
 */
static void SAI_HAL_SendDataBlocking(I2S_TypeDef * base, uint32_t tx_channel, 
    uint8_t * txBuff, uint32_t size);

static void serve_rx_interrupt(I2SDriver *i2sp) {
  I2S_TypeDef * reg_base = i2sp->I2S;
  uint8_t i = 0;
  uint32_t len = i2sp->config->sai_rx_state.len;
  uint32_t data;

  /* Judge if FIFO error */
  if(SAI_HAL_RxGetStateFlag(reg_base, kSaiStateFlagFIFOError)) {
    // well, if an error I guess we do nothing but clear the flag for now
    SAI_HAL_RxClearStateFlag(reg_base, kSaiStateFlagFIFOError);    
  }
  /* Interrupt used to transfer data. */
  if((SAI_HAL_RxGetStateFlag(reg_base, kSaiStateFlagFIFORequest)) &&
     (!i2sp->config->sai_rx_state.use_dma)) {
    uint8_t space = i2sp->config->sai_rx_state.watermark;
    /*Judge if the data need to transmit is less than space */
    if(space > (len - i2sp->config->sai_rx_state.count)) {
      space = len - i2sp->config->sai_rx_state.count;
    }
    /* Read data from FIFO to the buffer */
    for( i = 0; i < space; i++ ) {
      ((int32_t *)i2sp->config->rx_buffer)[i2sp->config->sai_rx_state.count++] = (int32_t) reg_base->RDR[0];
    }
    
    /* Determine how full we are, and what type of callback has to happen */
    if (i2sp->config->sai_rx_state.count >= len ) {
      if( i2sp->config->end_cb ) {
	(*i2sp->config->end_cb)(i2sp, 0, len); // just deal with full buffers for now
	// why? FIFO has some space, should give ~10k cycles to do the data copy and return.
	// no need to double-buffer at this performance level
      }
      // reset buffer to init params
      i2sp->config->sai_rx_state.address = i2sp->config->rx_buffer;  // reset the buffer address to initial state
      i2sp->config->sai_rx_state.count = 0;
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if KINETIS_I2S_USE_I2S1
OSAL_IRQ_HANDLER(KINETIS_I2S0_RX_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  osalSysLockFromISR(); 
  rx_int_count++;
  serve_rx_interrupt(&I2SD1);
  osalSysUnlockFromISR();  
  OSAL_IRQ_EPILOGUE();
}
#endif

#if KINETIS_I2S_USE_I2S1
OSAL_IRQ_HANDLER(KINETIS_I2S0_TX_VECTOR) {
  OSAL_IRQ_PROLOGUE();
  // placeholder
  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2S driver initialization.
 *
 * @notapi
 */
void i2s_lld_init(void) {

#if KINETIS_I2S_USE_I2S1
  i2sObjectInit(&I2SD1);
#endif
}


/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxInit
 * Description   : Initialize the sai Tx register, just set the register vaule to zero.
 *This function just clear the register value of sai.
 *END**************************************************************************/
static void SAI_HAL_TxInit(I2S_TypeDef * base)
{
    /* Software reset and FIFO reset */
    I2S_BWR_TCSR_SR(base, 1);
    I2S_BWR_TCSR_FR(base, 1);
    /* Clear all registers */
    I2S_WR_TCSR(base, 0);
#if (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    I2S_WR_TCR1(base, 0);
#endif
    I2S_WR_TCR2(base, 0);
    I2S_WR_TCR3(base, 0);
    I2S_WR_TCR4(base, 0);
    I2S_WR_TCR5(base, 0);
    I2S_WR_TMR(base,0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxInit
 * Description   : Initialize the sai Rx register, just set the register vaule to zero.
 *This function just clear the register value of sai.
 *END**************************************************************************/
static void SAI_HAL_RxInit(I2S_TypeDef * base)
{
    /* Software reset and FIFO reset */
    I2S_BWR_RCSR_SR(base, 1);
    I2S_BWR_RCSR_FR(base, 1);
    /* Clear all registers */
    I2S_WR_RCSR(base, 0);
#if (FSL_FEATURE_SAI_FIFO_COUNT > 1)
    I2S_WR_RCR1(base, 0);
#endif
    I2S_WR_RCR2(base, 0);
    I2S_WR_RCR3(base, 0);
    I2S_WR_RCR4(base, 0);
    I2S_WR_RCR5(base, 0);
    I2S_WR_RMR(base,0);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetProtocol
 * Description   : According to the protocol type to set the registers for tx.
 *The protocol can be I2S left, I2S right, I2S and so on.
 *END**************************************************************************/
void SAI_HAL_TxSetProtocol(I2S_TypeDef * base,sai_protocol_t protocol)
{
    switch (protocol)
    {
        case kSaiBusI2SLeft:
            I2S_BWR_TCR2_BCP(base,1u);/* Bit clock polarity */
            I2S_BWR_TCR4_MF(base,1u);/* MSB transmitted fisrt */
            I2S_BWR_TCR4_FSE(base,0u);/*Frame sync not early */
            I2S_BWR_TCR4_FSP(base,0u);/* Frame sync polarity, left channel is high */
            I2S_BWR_TCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_TCR3_WDFL(base, 0u); /* The first word set the start flag */
            break;

        case kSaiBusI2SRight:
            I2S_BWR_TCR2_BCP(base,1u);/* Bit clock polarity */
            I2S_BWR_TCR4_MF(base,1u);/* MSB transmitted firsrt */
            I2S_BWR_TCR4_FSE(base,0u);/*Frame sync not early */
            I2S_BWR_TCR4_FSP(base,0u);/* Frame sync polarity, left chennel is high */
            I2S_BWR_TCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_TCR3_WDFL(base, 0u); /* The first word set the start flag */
            break;

        case kSaiBusI2SType:
            I2S_BWR_TCR2_BCP(base,1u);/*Bit clock polarity */
            I2S_BWR_TCR4_MF(base,1u);/*MSB transmitted firsrt */
            I2S_BWR_TCR4_FSE(base,1u);/* Frame sync one bit early */
            I2S_BWR_TCR4_FSP(base,1u);/* Frame sync polarity, left channel is low */
            I2S_BWR_TCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_TCR3_WDFL(base, 0u); /* The first word set the start flag */
            break;

        case kSaiBusPCMA:
            I2S_BWR_TCR2_BCP(base,0u); /* Bit clock active low */
            I2S_BWR_TCR4_MF(base, 1u); /* MSB transmitted first */
            I2S_BWR_TCR4_SYWD(base, 0u); /* Only one bit clock in a frame sync */
            I2S_BWR_TCR4_FSE(base,1u);/* Frame sync one bit early */
            I2S_BWR_TCR4_FSP(base,0u);/* Frame sync polarity, left chennel is high */                
            I2S_BWR_TCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_TCR3_WDFL(base, 0u); /* The first word set the start flag */
            break;
            
        case kSaiBusPCMB:
            I2S_BWR_TCR2_BCP(base,0u); /* Bit clock active high */
            I2S_BWR_TCR4_MF(base, 1u); /* MSB transmitted first */
            I2S_BWR_TCR4_FSE(base,0u);/* Frame sync not early */
            I2S_BWR_TCR4_SYWD(base, 0u); /* Only one bit clock in a frame sync */
            I2S_BWR_TCR4_FSP(base,0u);/* Frame sync polarity, left chennel is high */
            I2S_BWR_TCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_TCR3_WDFL(base, 0u); /* The first word set the start flag */
            break;
            
        case kSaiBusAC97:
            I2S_BWR_TCR2_BCP(base,1u); /* Bit clock active high */
            I2S_BWR_TCR4_MF(base,1u); /* MSB transmitted first */
            I2S_BWR_TCR4_FSE(base,1u);/* Frame sync one bit early */
            I2S_BWR_TCR4_FRSZ(base,12u); /* There are 13 words in a frame in AC'97 */
            I2S_BWR_TCR4_SYWD(base,15u); /* Length of frame sync, 16 bit transmitted in first word */
            I2S_BWR_TCR5_W0W(base,15u); /* The first word have 16 bits */
            I2S_BWR_TCR5_WNW(base,19u); /* Other word is 20 bits */
            I2S_BWR_TCR3_WDFL(base, 0u); /* The first word set the start flag */
            break;
            
        default:
            break;
        }
}  

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetProtocol
 * Description   : According to the protocol type to set the registers for rx.
 *The protocol can be I2S left, I2S right, I2S and so on.
 *END**************************************************************************/
void SAI_HAL_RxSetProtocol(I2S_TypeDef * base,sai_protocol_t protocol)
{
    switch (protocol)
    {
        case kSaiBusI2SLeft:
            I2S_BWR_RCR2_BCP(base,1u);/* Bit clock polarity */
            I2S_BWR_RCR4_MF(base,1u);/* MSB transmitted fisrt */
            I2S_BWR_RCR4_FSE(base,0u);/*Frame sync one bit early */
            I2S_BWR_RCR4_FSP(base,0u);/* Frame sync polarity, left channel is high */
            I2S_BWR_RCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_RCR3_WDFL(base, 0u); /* The first word set the start flag */
            break;

        case kSaiBusI2SRight:
            I2S_BWR_RCR2_BCP(base,1u);/* Bit clock polarity */
            I2S_BWR_RCR4_MF(base,1u);/* MSB transmitted fisrt */
            I2S_BWR_RCR4_FSE(base,0u);/*Frame sync one bit early */
            I2S_BWR_RCR4_FSP(base,0u);/* Frame sync polarity, left chennel is high */
            I2S_BWR_RCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_RCR3_WDFL(base, 0u);/* The first word set the start flag */
            break;

        case kSaiBusI2SType:
            I2S_BWR_RCR2_BCP(base,1u);/*Bit clock polarity */
            I2S_BWR_RCR4_MF(base,1u);/*MSB transmitted fisrt */
            I2S_BWR_RCR4_FSE(base,1u);/* Frame sync one bit early */
            I2S_BWR_RCR4_FSP(base,1u);/* Frame sync polarity, left channel is low */
            I2S_BWR_RCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_RCR3_WDFL(base, 0u);/* The first word set the start flag */
            break;

        case kSaiBusPCMA:
            I2S_BWR_RCR2_BCP(base,0u); /* Bit clock active high */
            I2S_BWR_RCR4_MF(base, 1u); /* MSB transmitted first */
            I2S_BWR_RCR4_SYWD(base, 0u); /* Only one bit clock in a frame sync */
            I2S_BWR_RCR4_FSE(base,1u);/* Frame sync one bit early */
            I2S_BWR_RCR4_FSP(base,0u);/* Frame sync polarity, left chennel is high */                
            I2S_BWR_RCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */
            I2S_BWR_RCR3_WDFL(base, 0u);/* The first word set the start flag */
            break;
            
        case kSaiBusPCMB:
            I2S_BWR_RCR2_BCP(base,0u); /* Bit clock active high */
            I2S_BWR_RCR4_MF(base, 1u); /* MSB transmitted first */
            I2S_BWR_RCR4_FSE(base,0u);/* Frame sync not early */
            I2S_BWR_RCR4_SYWD(base, 0u); /* Only one bit clock in a frame sync */
            I2S_BWR_RCR4_FSP(base,0u);/* Frame sync polarity, left chennel is high */                
            I2S_BWR_RCR4_FRSZ(base,1u);/* I2S uses 2 word in a frame */                
            break;
            
        case kSaiBusAC97:
            I2S_BWR_RCR2_BCP(base,1u); /* Bit clock active high */
            I2S_BWR_RCR4_MF(base,1u); /* MSB transmitted first */
            I2S_BWR_RCR4_FSE(base,1u);/* Frame sync one bit early */
            I2S_BWR_RCR4_FRSZ(base,12u); /* There are 13 words in a frame in AC'97 */
            I2S_BWR_RCR4_SYWD(base,15u); /* Length of frame sync, 16 bit transmitted in first word */
            I2S_BWR_RCR5_W0W(base,15u); /* The first word have 16 bits */
            I2S_BWR_RCR5_WNW(base,19u); /* Other word is 20 bits */
            I2S_BWR_RCR3_WDFL(base, 0u);/* The first word set the start flag */
            break;

        default:
            break;
    }
}

#if FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER
/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_SetMclkDiv
 * Description   : Set the divider from the clock source to get the master clock.
 *The function would compute the divider number and set the number to the registers.
 *END**************************************************************************/
void SAI_HAL_SetMclkDiv(I2S_TypeDef * base, uint32_t mclk, uint32_t src_clk)
{
    uint32_t freq = src_clk;
    uint16_t fract, divide;
    uint32_t remaind = 0;
    uint32_t current_remainder = 0xffffffff;
    uint16_t current_fract = 0;
    uint16_t current_divide = 0;
    uint32_t mul_freq = 0;
    uint32_t max_fract = 256;
    /*In order to prevent overflow */
    freq /= 100;
    mclk/= 100;
    max_fract = mclk * 4096/freq + 1;
    if(max_fract > 256)
    {
        max_fract = 256;
    }
    /* Looking for the closet frequency */
    for (fract = 1; fract < max_fract; fract ++)
    {
        mul_freq = freq * fract;
        remaind = mul_freq % mclk;
        divide = mul_freq/mclk;
        /* Find the exactly frequency */
        if (remaind == 0)
        {
            current_fract = fract;
            current_divide = mul_freq/mclk;
            break;
        }
        /* closer to next one */
        if (remaind > mclk/2)
        {
            remaind = mclk - remaind;
            divide += 1;
        }
        /* Update the closest div and fract */
        if (remaind < current_remainder)
        {
            current_fract = fract;
            current_divide = divide;
            current_remainder = remaind;
        }
    }
    /* Clear the FRACT and DIV bit  */
    I2S_WR_MDR(base, 0U);
    /* Waiting for change updated */
    while(I2S_BRD_MCR_DUF(base))
    {}
    I2S_BWR_MDR_DIVIDE(base, current_divide -1);
    /* Waiting for the divider updated */
    while(I2S_BRD_MCR_DUF(base))
    {}
    I2S_BWR_MDR_FRACT(base, current_fract - 1);
    /* Waiting for the divider updated */
    while(I2S_BRD_MCR_DUF(base))
    {}
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxClockSetup
 * Description   : Set all clock parameters into register for tx.
 *The function would compute the divider number and set the number to the registers.
 *END**************************************************************************/
void SAI_HAL_TxClockSetup(I2S_TypeDef * base, sai_clock_setting_t * clk_config)
{
    /* First configure bit clock */
    uint32_t bclk_div = clk_config->bclk_src_freq/clk_config->bclk;
    I2S_BWR_TCR2_MSEL(base, clk_config->bclk_src);
    I2S_BWR_TCR2_DIV(base, (bclk_div/2-1));
    /* If bit clock source is mclk, compute mclk divider */
    if (clk_config->bclk_src == kSaiBclkSourceMclkDiv)
    {
        /* Enable MCLK */
        I2S_BWR_MCR_MOE(base, 1u);
        /* Configure MCLK source */
        I2S_BWR_MCR_MICS(base, clk_config->mclk_src);
#if FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER
        /* Configure MCLK divider */
        SAI_HAL_SetMclkDiv(base, clk_config->mclk, clk_config->mclk_src_freq);
#endif
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxClockSetup
 * Description   : Set all clock parameters into register for rx.
 *The function would compute the divider number and set the number to the registers.
 *END**************************************************************************/
void SAI_HAL_RxClockSetup(I2S_TypeDef * base, sai_clock_setting_t * clk_config)
{
    /* First configure bit clock */
    uint32_t bclk_div = clk_config->bclk_src_freq/clk_config->bclk;
    I2S_BWR_RCR2_MSEL(base, clk_config->bclk_src);
    I2S_BWR_RCR2_DIV(base, (bclk_div/2-1));
    /* If bit clock source is mclk, compute mclk divider */
    if (clk_config->bclk_src == kSaiBclkSourceMclkDiv)
    {
        /* Enable MCLK */
        I2S_BWR_MCR_MOE(base, 1u);
        /* Configure MCLK source */
        I2S_BWR_MCR_MICS(base, clk_config->mclk_src);
#if FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER
        /* Configure MCLK divider */
        SAI_HAL_SetMclkDiv(base, clk_config->mclk, clk_config->mclk_src_freq);
#endif
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetWordWidth
 * Description   : Set the tx word width. It can be 8bits, 16bits, 24bits and 32bits.
 *The function will set word width for different protocol.
 *END**************************************************************************/
void SAI_HAL_TxSetWordWidth(I2S_TypeDef * base, sai_protocol_t protocol, uint32_t bits)
{
    if ((protocol == kSaiBusI2SLeft) ||(protocol == kSaiBusI2SRight) ||(protocol == kSaiBusI2SType))
    {
        I2S_BWR_TCR4_SYWD(base, bits -1);
    }
    I2S_BWR_TCR5_W0W(base, bits -1);
    I2S_BWR_TCR5_WNW(base, bits -1);
    I2S_BWR_TCR5_FBT(base, bits -1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetWordWidth
 * Description   : Set the rx word width. It can be 8bits, 16bits, 24bits and 32bits.
 *The function will set word width for different protocol.
 *END**************************************************************************/
void SAI_HAL_RxSetWordWidth(I2S_TypeDef * base, sai_protocol_t protocol, uint32_t bits)
{
    if ((protocol == kSaiBusI2SLeft) ||(protocol == kSaiBusI2SRight) ||(protocol == kSaiBusI2SType))
    {
        I2S_BWR_RCR4_SYWD(base, bits -1);
    }
    I2S_BWR_RCR5_W0W(base, bits -1);
    I2S_BWR_RCR5_WNW(base, bits -1);
    I2S_BWR_RCR5_FBT(base, bits -1);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetMonoStereo
 * Description   : Set the mono or stereo mode for tx.
 *
 *END**************************************************************************/
void SAI_HAL_TxSetMonoStereo(I2S_TypeDef * base, sai_mono_stereo_t mono_stereo)
{
    if (mono_stereo == kSaiMono)
    {
        I2S_WR_TMR(base, 2u);
    }
    else
    {
        I2S_WR_TMR(base, 0u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetMonoStereo
 * Description   : Set the mono or stereo mode for rx.
 *
 *END**************************************************************************/
void SAI_HAL_RxSetMonoStereo(I2S_TypeDef * base, sai_mono_stereo_t mono_stereo)
{
    if (mono_stereo == kSaiMono)
    {
        I2S_WR_RMR(base, 2u);
    }
    else
    {
        I2S_WR_RMR(base, 0u);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetMasterSlave
 * Description   : Set the tx master or slave mode.
 *The slave or master mode only would affect the clock direction relevant registers.
 *END**************************************************************************/
void SAI_HAL_TxSetMasterSlave(I2S_TypeDef * base, sai_master_slave_t master_slave_mode)
{
    if (master_slave_mode == kSaiMaster)
    {
        I2S_BWR_TCR2_BCD(base,1);/* Bit clock generated internal */
        I2S_BWR_TCR4_FSD(base,1);/* Frame sync generated internal */
        I2S_BWR_MCR_MOE(base,1);/* Master clock generated internal */
    }
    else
    {
        I2S_BWR_TCR2_BCD(base,0);/* Bit clock generated external */
        I2S_BWR_TCR4_FSD(base,0);/* Frame sync generated external */
        I2S_BWR_MCR_MOE(base,0);/* Master clock generated external */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetMasterSlave
 * Description   : Set the rx master or slave mode.
 *The slave or master mode only would affect the clock direction relevant registers.
 *END**************************************************************************/
void SAI_HAL_RxSetMasterSlave(I2S_TypeDef * base, sai_master_slave_t master_slave_mode)
{
    if (master_slave_mode == kSaiMaster)
    {
        I2S_BWR_RCR2_BCD(base,1);/* Bit clock generated internal */
        I2S_BWR_RCR4_FSD(base,1);/* Frame sync generated internal */
        I2S_BWR_MCR_MOE(base,1);/* Master clock generated internal */
    }
    else
    {
        I2S_BWR_RCR2_BCD(base,0);/* Bit clock generated external */
        I2S_BWR_RCR4_FSD(base,0);/* Frame sync generated external */
        I2S_BWR_MCR_MOE(base,0);/* Master clock generated external */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetSyncMode
 * Description   : Set the tx sync mode.
 *Theer are four kinds of sync mode, async, sync, sync with other sai tx, sync with other sai rx.
 *END**************************************************************************/
void SAI_HAL_TxSetSyncMode(I2S_TypeDef * base, sai_sync_mode_t sync_mode)
{
    switch (sync_mode)
    {
        case kSaiModeAsync:
            I2S_BWR_TCR2_SYNC(base,0);
            break;
        case kSaiModeSync:
            I2S_BWR_TCR2_SYNC(base,1);
            I2S_BWR_RCR2_SYNC(base,0);/* Receiver must be async mode */
            break;
        case kSaiModeSyncWithOtherTx:
            I2S_BWR_TCR2_SYNC(base,2);
            break;
        case kSaiModeSyncWithOtherRx:
            I2S_BWR_TCR2_SYNC(base,3);
            break;
        default:
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetSyncMode
 * Description   : Set the rx sync mode.
 *Theer are four kinds of sync mode, async, sync, sync with other sai tx, sync with other sai rx.
 *END**************************************************************************/
void SAI_HAL_RxSetSyncMode(I2S_TypeDef * base,sai_sync_mode_t sync_mode)
{
    switch (sync_mode)
    {
        case kSaiModeAsync:
            I2S_BWR_RCR2_SYNC(base,0);
            break;
        case kSaiModeSync:
            I2S_BWR_RCR2_SYNC(base,1);
            I2S_BWR_TCR2_SYNC(base,0);/* Receiver must be async mode */
            break;
        case kSaiModeSyncWithOtherTx:
            I2S_BWR_RCR2_SYNC(base,3);
            break;
        case kSaiModeSyncWithOtherRx:
            I2S_BWR_RCR2_SYNC(base,2);
            break;
        default:
            break;
    }    
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetIntCmd
 * Description   : Enable the interrupt request source for tx.
 *The source can be word start, sync error, FIFO empty, FIFO error and FIFO request.
 *END**************************************************************************/
void SAI_HAL_TxSetIntCmd(I2S_TypeDef * base, uint32_t source, bool enable)
{
    uint32_t  val = I2S_RD_TCSR(base);
    if (enable == true)
    {
        I2S_WR_TCSR(base, val |source);
    }
    else
    {
        I2S_WR_TCSR(base, (val & (~source)));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetIntCmd
 * Description   : Enable the interrupt request source for rx.
 *The source can be word start, sync error, FIFO empty, FIFO error and FIFO request.
 *END**************************************************************************/
void SAI_HAL_RxSetIntCmd(I2S_TypeDef * base, uint32_t source, bool enable)
{
    uint32_t  val = I2S_RD_RCSR(base);
    if (enable == true)
    {
        I2S_WR_RCSR(base, val |source);
    }
    else
    {
        I2S_WR_RCSR(base, (val & (~source)));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetDmaCmd
 * Description   : Enable the dma request source for tx.
 *The source can be FIFO empty or FIFO request.
 *END**************************************************************************/
void SAI_HAL_TxSetDmaCmd(I2S_TypeDef * base, uint32_t source, bool enable)
{
    uint32_t  val = I2S_RD_TCSR(base);
    if (enable == true)
    {
        I2S_WR_TCSR(base, val |source);
    }
    else
    {
        I2S_WR_TCSR(base, (val & (~source)));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetDmaCmd
 * Description   : Enable the dma request source for rx.
 *The source can be FIFO empty or FIFO request.
 *END**************************************************************************/
void SAI_HAL_RxSetDmaCmd(I2S_TypeDef * base, uint32_t source, bool enable)
{
    uint32_t  val = I2S_RD_RCSR(base);
    if (enable == true)
    {
        I2S_WR_RCSR(base, val |source);
    }
    else
    {
        I2S_WR_RCSR(base, (val & (~source)));
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxClearStateFlag
 * Description   : Clear the state flag of tx registers.
 *The state flag incudes word start flag, sync error flag and fifo error flag.
 *END**************************************************************************/
void SAI_HAL_TxClearStateFlag(I2S_TypeDef * base, uint32_t flag_mask)
{
    uint32_t val = I2S_RD_TCSR(base);
    /* FIFO request cannot clear */
    if (flag_mask & kSaiStateFlagFIFORequest)
    {
        flag_mask &= (uint32_t)(~kSaiStateFlagFIFORequest);
    }
    /* FIFO warning cannot clear */
    if (flag_mask & kSaiStateFlagFIFOWarning)
    {
        flag_mask &= (uint32_t)(~kSaiStateFlagFIFOWarning);
    }
    /* Check if need to clear software reset. */
    if (flag_mask & kSaiStateFlagSoftReset)
    {
        val &= (uint32_t)(~kSaiStateFlagSoftReset);
        flag_mask &= (uint32_t)(~kSaiStateFlagSoftReset);
    }
    /* Clear other flags. */
    val  |= flag_mask;
    I2S_WR_TCSR(base, val);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxClearStateFlag
 * Description   : Clear the state flag of rx registers.
 *The state flag incudes word start flag, sync error flag and fifo error flag.
 *END**************************************************************************/
void SAI_HAL_RxClearStateFlag(I2S_TypeDef * base, uint32_t flag_mask)
{
    uint32_t val = I2S_RD_RCSR(base);
    /* FIFO request cannot clear */
    if (flag_mask & kSaiStateFlagFIFORequest)
    {
        flag_mask &= (uint32_t)(~kSaiStateFlagFIFORequest);
    }
    /* FIFO warning cannot clear */
    if (flag_mask & kSaiStateFlagFIFOWarning)
    {
        flag_mask &= (uint32_t)(~kSaiStateFlagFIFOWarning);
    }
    /* Check if need to clear software reset. */
    if (flag_mask & kSaiStateFlagSoftReset)
    {
        val &= (uint32_t)(~kSaiStateFlagSoftReset);
        flag_mask &= (uint32_t)(~kSaiStateFlagSoftReset);
    }
    /* Clear other flags. */
    val  |= flag_mask;
    I2S_WR_RCSR(base, val);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetReset
 * Description   : Reset tx according to reset mode.
 *The reset mode can be software reset and FIFO reset. 
 *END**************************************************************************/
void SAI_HAL_TxSetReset(I2S_TypeDef * base, uint32_t  reset_mask)
{
    uint32_t val = I2S_RD_TCSR(base);
    I2S_WR_TCSR(base, val |reset_mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetReset
 * Description   : Reset rx according to reset mode.
 *The reset mode can be software reset and FIFO reset. 
 *END**************************************************************************/
void SAI_HAL_RxSetReset(I2S_TypeDef * base, uint32_t reset_mask)
{
    uint32_t val = I2S_RD_RCSR(base);
    I2S_WR_RCSR(base, val |reset_mask);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxSetRunModeCmd
 * Description   : Set the work mode for tx.
 *The work mode have stop mode, debug mode and normal mode. 
 *END**************************************************************************/
void SAI_HAL_TxSetRunModeCmd(I2S_TypeDef * base, sai_run_mode_t run_mode, bool enable)
{
    switch (run_mode)
    {
        case kSaiRunModeStop:
            I2S_BWR_TCSR_STOPE(base, enable);/* Stop mode */
            break;
        case kSaiRunModeDebug:
            I2S_BWR_TCSR_DBGE(base, enable);/* Debug mode */
            break;
        default:
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxSetRunModeCmd
 * Description   : Set the work mode for rx.
 *The work mode have stop mode, debug mode and normal mode. 
 *END**************************************************************************/
void SAI_HAL_RxSetRunModeCmd(I2S_TypeDef * base,sai_run_mode_t run_mode,bool enable)
{
    switch (run_mode)
    {
        case kSaiRunModeStop:
            I2S_BWR_RCSR_STOPE(base, enable);/* Stop mode */
            break;
        case kSaiRunModeDebug:
            I2S_BWR_RCSR_DBGE(base, enable);/* Debug mode */
            break;
        default:
            break;
    }
}

#if FSL_FEATURE_SAI_FIFO_COUNT > 1
/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_TxGetFifoWRPointer
 * Description   : Get tx fifo read and write pointer.
 *
 *END**************************************************************************/
void SAI_HAL_TxGetFifoWRPointer(I2S_TypeDef * base,  uint32_t fifo_channel, 
       uint32_t * r_ptr, uint32_t * w_ptr)
{
    uint32_t val = I2S_RD_TFR(base, fifo_channel);
    *r_ptr = (val >> I2S_TFR_RFP_SHIFT) & I2S_TFR_RFP_MASK;
    *w_ptr = (val >> I2S_TFR_WFP_SHIFT) & I2S_TFR_WFP_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_RxGetFifoWRPointer
 * Description   : Get rx fifo read and write pointer.
 *
 *END**************************************************************************/
void SAI_HAL_RxGetFifoWRPointer(I2S_TypeDef * base,  uint32_t fifo_channel, 
       uint32_t * r_ptr, uint32_t * w_ptr)
{
    uint32_t val = I2S_RD_RFR(base, fifo_channel);
    *r_ptr = (val >> I2S_RFR_RFP_SHIFT) & I2S_RFR_RFP_MASK;
    *w_ptr = (val >> I2S_RFR_WFP_SHIFT) & I2S_RFR_WFP_MASK;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_ReceiveDataBlocking
 * Description   : Receive data in blocking way.
 *The sending would wait until there is vaild data in FIFO for reading.
 *END**************************************************************************/
void SAI_HAL_ReceiveDataBlocking(I2S_TypeDef * base,uint32_t rx_channel,
    uint8_t *rxBuff, uint32_t size)
{
  osalDbgAssert((rx_channel < FSL_FEATURE_SAI_CHANNEL_COUNT),"invalid rx count");
    uint32_t bytes = (I2S_BRD_RCR5_WNW(base) + 1)/8;
    uint32_t i =0, j = 0, data = 0;
    /* Wait while fifo is empty */
    for (i = 0; i < (size/bytes); i ++)
    {
        while(!I2S_BRD_RCSR_FWF(base))
        {}
        data = I2S_RD_RDR(base,rx_channel);
        for (j = 0; j < bytes; j ++)
        {
            *rxBuff = (data >> (8U * j)) & 0xFF;
            rxBuff ++;
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_HAL_SendDataBlocking
 * Description   : Send data in blocking way.
 *The sending would wait until there is space for writing.
 *END**************************************************************************/
void SAI_HAL_SendDataBlocking(I2S_TypeDef * base,uint32_t tx_channel,
    uint8_t * txBuff, uint32_t size)
{
  osalDbgAssert((tx_channel < FSL_FEATURE_SAI_CHANNEL_COUNT),"invalid tx count");
    uint32_t bytes = (I2S_BRD_TCR5_WNW(base) + 1)/8;
    uint32_t i =0, j = 0, data = 0, temp = 0;
    /* Wait while fifo is empty */
    for (i = 0; i < (size/bytes); i ++)
    {
        while(!I2S_BRD_TCSR_FWF(base))
        {}
        for (j = 0; j < bytes; j ++)
        {
            temp = (uint32_t)(*txBuff);
            data |= (temp << (8U * j));
            txBuff ++;
        }
        I2S_WR_TDR(base, tx_channel, data);
        data = 0;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_TxConfigDataFormat
 * Description   :Configure audio format information of tx.
 * The audio format information includes the sample rate, data length and so on.
 *END**************************************************************************/
sai_status_t SAI_DRV_TxConfigDataFormat(I2SDriver *i2sp, sai_data_format_t *format)
{
  I2S_TypeDef *reg_base = i2sp->I2S;
  memcpy(&i2sp->config->sai_tx_state.format, format, sizeof(sai_data_format_t));
  if(i2sp->config->sai_tx_state.master_slave == kSaiMaster)
    {
        uint32_t bclk = format->sample_rate * format->bits * 2;
        uint8_t divider;
        if(SAI_HAL_TxGetBclkSrc(reg_base) == 0)
        {
            divider = (CLOCK_SYS_GetBusClockFreq())/bclk;
        }
        else
        {
            divider = format->mclk/bclk;
        }
#if FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER
        uint32_t frequency = 0;
        /* Get the clock source frequency */
        uint32_t mclk_sel = SAI_HAL_GetMclkSrc(reg_base);
        frequency = CLOCK_SYS_GetSaiFreq((clock_sai_src_t)mclk_sel);
        /* Configure master clock */
        SAI_HAL_SetMclkDiv(reg_base, format->mclk, frequency);
#endif
        /* Master clock and bit clock setting */
        SAI_HAL_TxSetBclkDiv(reg_base, divider);
    }
    SAI_HAL_TxSetWordWidth(reg_base, i2sp->config->sai_tx_state.protocol, format->bits);
    /* The channel number configuration */
    SAI_HAL_TxSetMonoStereo(reg_base, format->mono_stereo);

    return kStatus_SAI_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_DRV_RxConfigDataFormat
 * Description   :Configure audio format information of rx.
 * The audio format information includes the sample rate, data length and so on.
 *END**************************************************************************/
sai_status_t SAI_DRV_RxConfigDataFormat(I2SDriver *i2sp, sai_data_format_t *format)
{
    I2S_TypeDef *reg_base = i2sp->I2S;

    memcpy(&i2sp->config->sai_rx_state.format, format, sizeof(sai_data_format_t));
    if(i2sp->config->sai_rx_state.master_slave == kSaiMaster)
    {
        uint32_t bclk = format->sample_rate * format->bits * 2;
        uint8_t divider;
        if(SAI_HAL_RxGetBclkSrc(reg_base) == 0)
        {
            divider = (CLOCK_SYS_GetBusClockFreq())/bclk;
        }
        else
        {
            divider = format->mclk/bclk;
        }
#if FSL_FEATURE_SAI_HAS_MCLKDIV_REGISTER
        uint32_t frequency = 0;
        /* Get the clock source frequency */
        uint32_t mclk_sel = SAI_HAL_GetMclkSrc(reg_base);
        frequency = CLOCK_SYS_GetSaiFreq((clock_sai_src_t)mclk_sel);
        /* Configure master clock */
        SAI_HAL_SetMclkDiv(reg_base, format->mclk, frequency);
#endif
        /* Master clock and bit clock setting */
        SAI_HAL_RxSetBclkDiv(reg_base, divider);
    }
    SAI_HAL_RxSetWordWidth(reg_base, i2sp->config->sai_rx_state.protocol, format->bits);
    /* The channel number configuration */
    SAI_HAL_RxSetMonoStereo(reg_base, format->mono_stereo);
    return kStatus_SAI_Success;
}

/**
 * @brief   Configures and activates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_start(I2SDriver *i2sp) {
  // by this point, i2sp has a config record bound to it from the i2s HAL API.

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (i2sp->state == I2S_STOP) {

#if KINETIS_I2S_USE_I2S1
    // bind our register set based upon the intent expressed by which I2SD* bank the driver was called from.
    // a bit circular but ok.
    if (&I2SD1 == i2sp) {
      I2SD1.I2S = I2S0;
    }

    // enable clocks for I2S0
    SIM->SCGC6 |= SIM_SCGC6_I2S;

    if( i2sp->config->tx_buffer != NULL ) {
      // setup PHY attributes
      SAI_HAL_TxInit(i2sp->I2S);  
      /* Mclk source select */
      if (i2sp->config->sai_tx_userconfig.slave_master == kSaiMaster) {
        SAI_HAL_SetMclkSrc(i2sp->I2S, i2sp->config->sai_tx_userconfig.mclk_source);
        SAI_HAL_TxSetBclkSrc(i2sp->I2S, i2sp->config->sai_tx_userconfig.bclk_source);
      }
      SAI_HAL_TxSetSyncMode(i2sp->I2S, i2sp->config->sai_tx_userconfig.sync_mode);
      SAI_HAL_TxSetMasterSlave(i2sp->I2S, i2sp->config->sai_tx_userconfig.slave_master);
      SAI_HAL_TxSetProtocol(i2sp->I2S, i2sp->config->sai_tx_userconfig.protocol);
      SAI_HAL_TxSetDataChn(i2sp->I2S, i2sp->config->sai_tx_userconfig.channel);
      SAI_HAL_TxSetWatermark(i2sp->I2S, i2sp->config->sai_tx_userconfig.watermark);

      /* Fill the state structure */
      i2sp->config->sai_tx_state.sync_mode = i2sp->config->sai_tx_userconfig.sync_mode;
      i2sp->config->sai_tx_state.fifo_channel = i2sp->config->sai_tx_userconfig.channel;
      i2sp->config->sai_tx_state.dma_source = i2sp->config->sai_tx_userconfig.dma_source;
      i2sp->config->sai_tx_state.watermark = i2sp->config->sai_tx_userconfig.watermark;
      i2sp->config->sai_tx_state.master_slave = i2sp->config->sai_tx_userconfig.slave_master;

      //    OSA_SemaCreate(&state->sem, 0);
      nvicEnableVector(I2S0_Tx_IRQn, KINETIS_I2S_TX_PRIORITY);

      // this is a bit circular because we're using our default format spec to set
      // the format now -- but more generically, the data format can change dynamically
      SAI_DRV_TxConfigDataFormat(i2sp, &i2sp->config->sai_tx_state.format);
    }

    if( i2sp->config->rx_buffer != NULL ) {

      SAI_HAL_RxInit(i2sp->I2S);
      /* Mclk source select */
      if (i2sp->config->sai_rx_userconfig.slave_master == kSaiMaster) {
        SAI_HAL_SetMclkSrc(i2sp->I2S, i2sp->config->sai_rx_userconfig.mclk_source);
        SAI_HAL_RxSetBclkSrc(i2sp->I2S, i2sp->config->sai_rx_userconfig.bclk_source);
      }
      SAI_HAL_RxSetSyncMode(i2sp->I2S, i2sp->config->sai_rx_userconfig.sync_mode);
      SAI_HAL_RxSetMasterSlave(i2sp->I2S, i2sp->config->sai_rx_userconfig.slave_master);
      SAI_HAL_RxSetProtocol(i2sp->I2S, i2sp->config->sai_rx_userconfig.protocol);
      SAI_HAL_RxSetDataChn(i2sp->I2S, i2sp->config->sai_rx_userconfig.channel);
      SAI_HAL_RxSetWatermark(i2sp->I2S, i2sp->config->sai_rx_userconfig.watermark);

      /* Fill the state structure */
      i2sp->config->sai_rx_state.sync_mode = i2sp->config->sai_rx_userconfig.sync_mode;
      i2sp->config->sai_rx_state.fifo_channel = i2sp->config->sai_rx_userconfig.channel;
      i2sp->config->sai_rx_state.dma_source = i2sp->config->sai_rx_userconfig.dma_source;
      i2sp->config->sai_rx_state.watermark = i2sp->config->sai_rx_userconfig.watermark;
      i2sp->config->sai_rx_state.master_slave = i2sp->config->sai_rx_userconfig.slave_master;
      //    OSA_SemaCreate(&state->sem, 0);
      nvicEnableVector(I2S0_Rx_IRQn, KINETIS_I2S_RX_PRIORITY);

      SAI_DRV_RxConfigDataFormat(i2sp, &i2sp->config->sai_rx_state.format);

      i2sp->I2S->RMR = I2S_RMR_RWM(0x2);  // mask out unused stereo channel
    }
#endif
  }
}

/**
 * @brief   Deactivates the I2S peripheral.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_stop(I2SDriver *i2sp) {
    I2S_TypeDef * reg_base = i2sp->I2S;

  /* If in ready state then disables the SPI clock.*/
  if (i2sp->state == I2S_READY) {
#if KINETIS_I2S_USE_I2S1
    if (&I2SD1 == i2sp) {

    SAI_HAL_TxSetDmaCmd(reg_base, kSaiDmaReqFIFORequest,false);
    SAI_HAL_TxSetIntCmd(reg_base, kSaiIntrequestFIFOError,false);

    SAI_HAL_TxSetIntCmd(reg_base,kSaiIntrequestFIFORequest,false);
    SAI_HAL_TxSetIntCmd(reg_base, kSaiIntrequestFIFOError, false);

    SAI_HAL_TxDisable(reg_base);
    SAI_HAL_TxSetReset(reg_base, kSaiResetTypeSoftware);
    SAI_HAL_TxClearStateFlag(reg_base, kSaiStateFlagSoftReset);
    /* Release dma channel */
    if (i2sp->config->sai_tx_state.use_dma)
    {
#if defined FSL_FEATURE_EDMA_MODULE_CHANNEL
      // FIXME - DMA channels
      //        EDMA_DRV_StopChannel(&i2sp->config->sai_tx_state.edma_chn);
      //        EDMA_DRV_ReleaseChannel(&i2sp->config->sai_tx_state.edma_chn);
#else
        DMA_DRV_FreeChannel(&i2sp->config->sai_tx_state.chn);
#endif
    }
    /* Destory sem */
    //OSA_SemaDestroy(&sai_state_ids[instance][0]->sem);
    //    i2sp->config->sai_tx_state = NULL;  // durr....

    SAI_HAL_RxSetDmaCmd(reg_base, kSaiDmaReqFIFORequest,false);
    SAI_HAL_RxSetIntCmd(reg_base, kSaiIntrequestFIFOError,false);    

    SAI_HAL_RxSetIntCmd(reg_base,kSaiIntrequestFIFORequest,false);
    SAI_HAL_RxSetIntCmd(reg_base, kSaiIntrequestFIFOError,false);

    SAI_HAL_RxDisable(reg_base);
    SAI_HAL_RxSetReset(reg_base, kSaiResetTypeSoftware);
    SAI_HAL_RxClearStateFlag(reg_base, kSaiStateFlagSoftReset);
    /* Release dma channel */
    if (i2sp->config->sai_rx_state.use_dma)
    {
#if defined FSL_FEATURE_EDMA_MODULE_CHANNEL
      // FIXME - DMA channels
      //        EDMA_DRV_ReleaseChannel(&i2sp->config->sai_rx_state.edma_chn);
#else
        DMA_DRV_FreeChannel(&i2sp->config->sai_rx_state.chn);
#endif
    }
    /* Destory sem */
    //OSA_SemaDestroy(&sai_state_ids[instance][1]->sem);

    //sai_state_ids[instance][1] = NULL;  // durrr....

    SIM->SCGC6 &= ~SIM_SCGC6_I2S; // disable clocks

    }
#endif
  }
}

/**
 * @brief   Starts a I2S data receive only.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_start_rx(I2SDriver *i2sp) {

  (void)i2sp;
  I2S_TypeDef * reg_base = i2sp->I2S;

  // hardware int handler already setup in driver
  // enable the interrupts to drain the FIFO
  SAI_HAL_RxSetIntCmd(reg_base, kSaiIntrequestAll, false);  // clear all interrupts except the one we want
  SAI_HAL_RxSetIntCmd(reg_base, kSaiIntrequestFIFORequest, true); // fires when watermark hit
  SAI_HAL_RxSetIntCmd(reg_base, kSaiIntrequestFIFOError, true); // ignore errors for now

  // then start the system running and I think we're done!
  if(i2sp->config->sai_rx_state.sync_mode == kSaiModeSync) {
        SAI_HAL_RxEnable(reg_base);
	//        SAI_HAL_TxEnable(reg_base);  // just in case this is causing us trouble?
  } else {
        SAI_HAL_RxEnable(reg_base);
  };
}

/**
 * @brief   Stops a I2S data receive only.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_stop_rx(I2SDriver *i2sp) {

  (void)i2sp;
}


/**
 * @brief   Starts a I2S data exchange.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_start_exchange(I2SDriver *i2sp) {

  (void)i2sp;
}

/**
 * @brief   Stops the ongoing data exchange.
 * @details The ongoing data exchange, if any, is stopped, if the driver
 *          was not active the function does nothing.
 *
 * @param[in] i2sp      pointer to the @p I2SDriver object
 *
 * @notapi
 */
void i2s_lld_stop_exchange(I2SDriver *i2sp) {

  (void)i2sp;
}

#endif /* HAL_USE_I2S */

/** @} */
