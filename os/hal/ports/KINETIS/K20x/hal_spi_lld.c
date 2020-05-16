/*
    ChibiOS - Copyright (C) 2014-2015 Fabio Utzig

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
 * @file    KINETIS/spi_lld.c
 * @brief   KINETIS SPI subsystem low level driver source.
 *
 * @addtogroup SPI
 * @{
 */

#include "hal.h"
#include "orderedmem.h"

#if HAL_USE_SPI || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if !defined(KINETIS_SPI0_RX_DMA_IRQ_PRIORITY)
#define KINETIS_SPI0_RX_DMA_IRQ_PRIORITY    8
#endif

#if !defined(KINETIS_SPI0_RX_DMAMUX_CHANNEL)
#define KINETIS_SPI0_RX_DMAMUX_CHANNEL      0
#endif

#if !defined(KINETIS_SPI0_RX_DMA_CHANNEL)
#define KINETIS_SPI0_RX_DMA_CHANNEL         0
#endif

#if !defined(KINETIS_SPI0_TX_DMAMUX_CHANNEL)
#define KINETIS_SPI0_TX_DMAMUX_CHANNEL      1
#endif

#if !defined(KINETIS_SPI0_TX_DMA_CHANNEL)
#define KINETIS_SPI0_TX_DMA_CHANNEL         1
#endif

#if !defined(KINETIS_SPI1_RX_DMA_IRQ_PRIORITY)
#define KINETIS_SPI1_RX_DMA_IRQ_PRIORITY    8
#endif

#if !defined(KINETIS_SPI1_RX_DMAMUX_CHANNEL)
#define KINETIS_SPI1_RX_DMAMUX_CHANNEL      0
#endif

#if !defined(KINETIS_SPI1_RX_DMA_CHANNEL)
#define KINETIS_SPI1_RX_DMA_CHANNEL         0
#endif

#if !defined(KINETIS_SPI1_TX_DMAMUX_CHANNEL)
#define KINETIS_SPI1_TX_DMAMUX_CHANNEL      1
#endif

#if !defined(KINETIS_SPI1_TX_DMA_CHANNEL)
#define KINETIS_SPI1_TX_DMA_CHANNEL         1
#endif

#if KINETIS_SPI_USE_SPI0
#define DMAMUX_SPI_RX_SOURCE    14
#define DMAMUX_SPI_TX_SOURCE    15
#endif

#if 0
#if KINETIS_SPI_USE_SPI1
#define DMAMUX_SPI_RX_SOURCE    16
#define DMAMUX_SPI_TX_SOURCE    16
#endif
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief SPI0 driver identifier.*/
#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)
SPIDriver SPID1;
#endif

/** @brief SPI1 driver identifier.*/
#if KINETIS_SPI_USE_SPI1 || defined(__DOXYGEN__)
SPIDriver SPID2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* Use a dummy byte as the source/destination when a buffer is not provided */
/* Note: The MMC driver relies on 0xFF being sent for dummy bytes. */
static volatile uint16_t dmaRxDummy;
static uint16_t dmaTxDummy = 0xFFFF;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static void spi_start_xfer(SPIDriver *spip, bool polling)
{
  /*
   * Enable the DSPI peripheral in master mode.
   * Clear the TX and RX FIFOs.
   * */
  writel( &spip->spi->MCR, SPIx_MCR_MSTR | SPIx_MCR_CLR_TXF | SPIx_MCR_CLR_RXF | SPIx_MCR_HALT );

#if KINETIS_SPI_USE_SPI0
  uint32_t pushr;
  if( spip->count == 0 )
    return;  // abort if we were called with null data

  writel( &spip->spi->MCR, readl(&spip->spi->MCR) & ~SPIx_MCR_HALT); // clear the halt bit 

  
  writel( &spip->spi->SR, readl( &spip->spi->SR) | SPIx_SR_TCF); // clear the TCF so it can flip
  writel( &spip->spi->RSER, readl( &spip->spi->RSER ) | SPIx_RSER_TCF_RE ); // enable int on frame complete, to initiate chaining

  if( spip->txbuf != NULL )
    pushr = SPI_PUSHR_CTCNT_MASK | spip->txbuf[0]; // clear TCR with push
  else
    pushr = SPI_PUSHR_CTCNT_MASK | 0xFF;
  writel( &spip->spi->PUSHR, pushr ); // kicks off a send of one data, ISR fires when done
#endif

}

static void spi_stop_xfer(SPIDriver *spip)
{
  writel( &spip->spi->RSER, 0); // disable interrupts
  /* Halt the DSPI peripheral */
  writel( &spip->spi->MCR, SPIx_MCR_MSTR | SPIx_MCR_HALT );

  /* Clear all the flags which are currently set. */
  spip->spi->SR |= spip->spi->SR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)

uint32_t num_spi0_int = 0;

OSAL_IRQ_HANDLER(KINETIS_SPI0_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();

  num_spi0_int++;
  
  if( SPID1.rxbuf != NULL ) {
    SPID1.rxbuf[ (SPID1.spi->TCR >> 16) - 1 ] = SPID1.spi->POPR;
  }
  
  if( SPID1.count > (SPID1.spi->TCR >> 16) ) {
    // clear the TCF
    SPID1.spi->SR |= SPIx_SR_TCF;
    // kick off another transfer, and we'll be back here again...
    //    SPID1.spi->PUSHR = SPID1.txbuf[(SPID1.spi->TCR >> 16)] | SPIx_PUSHR_PCS(1) | SPI_PUSHR_CONT_MASK;
    if( SPID1.txbuf != NULL )
      SPID1.spi->PUSHR = SPID1.txbuf[(SPID1.spi->TCR >> 16)];
    else
      SPID1.spi->PUSHR = 0xFF;
  } else {
    SPID1.spi->RSER &= ~SPIx_RSER_TCF_RE; // disable interrupt on frame complete, we're done.
    // palSetPad(IOPORT4, 4); // de-assert CS line  // now under manual control
    spi_stop_xfer(&SPID1);
    _spi_isr_code(&SPID1);
  }

  OSAL_IRQ_EPILOGUE();
}

#endif

#if KINETIS_SPI_USE_SPI1 || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(KINETIS_SPI1_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();

  if( SPID2.rxbuf != NULL ) {
    SPID2.rxbuf[ (SPID2.spi->TCR >> 16) - 1 ] = SPID2.spi->POPR;
  }

  /* Clear bit 0 in Interrupt Request Register (INT) by writing 0 to CINT */
  //DMA->CINT = KINETIS_SPI1_RX_DMA_CHANNEL; // no DMA, instead check and refill
  if( SPID2.count > (SPID2.spi->TCR >> 16) ) {
    // clear the TCF
    SPID2.spi->SR |= SPIx_SR_TCF;
    // kick off another transfer, and we'll be back here again...
    //    SPID2.spi->PUSHR = SPID2.txbuf[(SPID2.spi->TCR >> 16)] | SPIx_PUSHR_PCS(1) | SPI_PUSHR_CONT_MASK;
    SPID2.spi->PUSHR = SPID2.txbuf[(SPID2.spi->TCR >> 16)];
  } else {
    SPID2.spi->RSER &= ~SPIx_RSER_TCF_RE; // disable interrupt on frame complete, we're done.
    // palSetPad(IOPORT4, 4); // de-assert CS line  // now under manual control
    spi_stop_xfer(&SPID2);
    _spi_isr_code(&SPID2);
  }

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SPI driver initialization.
 *
 * @notapi
 */
void spi_lld_init(void) {
#if KINETIS_SPI_USE_SPI0
  spiObjectInit(&SPID1);
#endif
#if KINETIS_SPI_USE_SPI1
  spiObjectInit(&SPID2);
#endif
}

/**
 * @brief   Configures and activates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_start(SPIDriver *spip) {

  /* If in stopped state then enables the SPI and DMA clocks.*/
  if (spip->state == SPI_STOP) {

#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {

      /* Enable the clock for SPI0 */
      SIM->SCGC6 |= SIM_SCGC6_SPI0;

      SPID1.spi = SPI0;

      if (spip->config->tar0) {
        spip->spi->CTAR[0] = spip->config->tar0;
      } else {
        spip->spi->CTAR[0] = KINETIS_SPI_TAR0_DEFAULT;
      }
    }

    nvicDisableVector(DMA0_IRQn); // disable DMA, as this SPI lacks bidirectional DMA
    nvicEnableVector(SPI0_IRQn, KINETIS_SPI_SPI0_IRQ_PRIORITY);
    /* Extract the frame size from the TAR */
    uint16_t frame_size = ((spip->spi->CTAR[0] >> SPIx_CTARn_FMSZ_SHIFT) &
        SPIx_CTARn_FMSZ_MASK) + 1;

    /* DMA word size is 2 for a 16 bit frame size */
    spip->word_size = frame_size > 8 ? 2 : 1;
    
#endif

#if KINETIS_SPI_USE_SPI1
    if (&SPID2 == spip) {

      /* Enable the clock for SPI0 */
      SIM->SCGC6 |= SIM_SCGC6_SPI1;

      SPID2.spi = SPI1;

      if (spip->config->tar0) {
        spip->spi->CTAR[0] = spip->config->tar0;
      } else {
        spip->spi->CTAR[0] = KINETIS_SPI_TAR0_DEFAULT;
      }
    }
#endif

  }
}

/**
 * @brief   Deactivates the SPI peripheral.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_stop(SPIDriver *spip) {

  /* If in ready state then disables the SPI clock.*/
  if (spip->state == SPI_READY) {

#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {
      /* SPI halt.*/
      spip->spi->MCR |= SPIx_MCR_HALT; // this allows configs to be safely modified
    }
    spip->state = SPI_STOP;
    
#endif
    
  }
  
  
}

/**
 * @brief   Asserts the slave select signal and prepares for transfers.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_select(SPIDriver *spip) {

  palClearPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Deasserts the slave select signal.
 * @details The previously selected peripheral is unselected.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 *
 * @notapi
 */
void spi_lld_unselect(SPIDriver *spip) {

  palSetPad(spip->config->ssport, spip->config->sspad);
}

/**
 * @brief   Ignores data on the SPI bus.
 * @details This asynchronous function starts the transmission of a series of
 *          idle words on the SPI bus and ignores the received data.
 * @post    At the end of the operation the configured callback is invoked.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be ignored
 *
 * @notapi
 */
void spi_lld_ignore(SPIDriver *spip, size_t n) {

  spip->count = n;
  spip->rxbuf = NULL;
  spip->txbuf = NULL;

  spi_start_xfer(spip, false);
}

// clocked off of bus clock @ 50MHz
// BR(2) -> /5, PBR() -> /2, final speed is 50/10 = 5MHz
// changed PBR(1) -> /3, so speed is now 50% slower...
#define KINETIS_SPI_TAR_BUSCLK_XZ(n)		\
    SPIx_CTARn_FMSZ((n) - 1) | \
    SPIx_CTARn_DBR | \
    SPIx_CTARn_PBR(1) | \
    SPIx_CTARn_BR(2) |	  \
    SPIx_CTARn_CSSCK(0) | \
    SPIx_CTARn_ASC(0) | \
    SPIx_CTARn_DT(0)

//    SPIx_CTARn_CPOL |				\
//    SPIx_CTARn_CPHA |				\

void spiRuntSetup(SPIDriver *spip) {
  
  nvicDisableVector(DMA1_IRQn); // disable DMA, as this SPI lacks bidirectional DMA
  nvicEnableVector(SPI1_IRQn, KINETIS_SPI_SPI1_IRQ_PRIORITY); // use interrupts instead
  
  spip->spi->MCR = SPIx_MCR_MSTR | SPIx_MCR_CLR_TXF | \
    SPIx_MCR_CLR_RXF | SPIx_MCR_DIS_RXF | SPIx_MCR_DIS_TXF | SPIx_MCR_CLR_TXF;
  spip->spi->CTAR[0] = KINETIS_SPI_TAR_BUSCLK_XZ(8);  // 8-bit frame size
  
  //  spip->spi->MCR |= SPIx_MCR_PCSIS(1); // set to active low for CS0
  palSetPad(IOPORT4, 4); // clear the CS line
}

static void noopt4(SPIDriver *spip) {
  uint32_t pushr;
  
  //  pushr = SPI_PUSHR_CONT_MASK | SPI_PUSHR_CTCNT_MASK | SPIx_PUSHR_PCS(1) | spip->txbuf[0]; // clear TCR with push
  pushr = SPI_PUSHR_CTCNT_MASK | spip->txbuf[0]; // clear TCR with push
  spip->spi->PUSHR = pushr; // kicks off a send of one data, ISR fires when done
}

void spi_start_xfer_runt(SPIDriver *spip) {

  if( spip->count == 0 )
    return;  // abort if we were called with null data

  if( spip->txbuf == NULL )
    spip->txbuf = spip->rxbuf;  // dummy send rx buf data

  spip->spi->MCR |= SPIx_MCR_DIS_RXF | SPIx_MCR_CLR_RXF | SPIx_MCR_DIS_TXF | SPIx_MCR_CLR_TXF; // disable fifos
  spip->spi->MCR &= ~SPIx_MCR_HALT; // clear the halt bit, chibios sets it every time the transfer is done
  // palClearPad(IOPORT4, 4); // assert CS line  // this is now on "manual" control
  
  spip->spi->SR |= SPIx_SR_TCF; // clear the TCF so it can flip
  spip->spi->RSER |= SPIx_RSER_TCF_RE; // enable interrupt on frame complete, to initiate chaining
  
  noopt4(spip); // make sure this command comes last in case things are pushed around by the optimizer
  
  return;
}

/**
 * @brief   Exchanges data on the SPI bus.
 * @details This asynchronous function starts a simultaneous transmit/receive
 *          operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to be exchanged
 * @param[in] txbuf     the pointer to the transmit buffer
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_exchange(SPIDriver *spip, size_t n,
                      const void *txbuf, void *rxbuf) {

  spip->count = n;
  spip->rxbuf = rxbuf;
  spip->txbuf = txbuf;

#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)
  if( spip == &SPID1 ) 
    spi_start_xfer(spip, false);
  else
    spi_start_xfer_runt(spip);
#else
    spi_start_xfer_runt(spip);
#endif
}

/**
 * @brief   Sends data over the SPI bus.
 * @details This asynchronous function starts a transmit operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to send
 * @param[in] txbuf     the pointer to the transmit buffer
 *
 * @notapi
 */
void spi_lld_send(SPIDriver *spip, size_t n, const void *txbuf) {

  spip->count = n;
  spip->rxbuf = NULL;
  spip->txbuf = (void *)txbuf;

#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)
  if( spip == &SPID1 ) 
    spi_start_xfer(spip, false);
  else
    spi_start_xfer_runt(spip);
#else
    spi_start_xfer_runt(spip);
#endif
}

/**
 * @brief   Receives data from the SPI bus.
 * @details This asynchronous function starts a receive operation.
 * @post    At the end of the operation the configured callback is invoked.
 * @note    The buffers are organized as uint8_t arrays for data sizes below or
 *          equal to 8 bits else it is organized as uint16_t arrays.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] n         number of words to receive
 * @param[out] rxbuf    the pointer to the receive buffer
 *
 * @notapi
 */
void spi_lld_receive(SPIDriver *spip, size_t n, void *rxbuf) {

  spip->count = n;
  spip->rxbuf = rxbuf;
  spip->txbuf = NULL;

#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)
  if( spip == &SPID1 ) 
    spi_start_xfer(spip, false);
  else
    spi_start_xfer_runt(spip);
#else
    spi_start_xfer_runt(spip);
#endif
}

/**
 * @brief   Exchanges one frame using a polled wait.
 * @details This synchronous function exchanges one frame using a polled
 *          synchronization method. This function is useful when exchanging
 *          small amount of data on high speed channels, usually in this
 *          situation is much more efficient just wait for completion using
 *          polling than suspending the thread waiting for an interrupt.
 *
 * @param[in] spip      pointer to the @p SPIDriver object
 * @param[in] frame     the data frame to send over the SPI bus
 * @return              The received data frame from the SPI bus.
 */
uint16_t spi_lld_polled_exchange(SPIDriver *spip, uint16_t frame) {

  spi_start_xfer(spip, true);

  spip->spi->PUSHR = SPIx_PUSHR_TXDATA(frame);

  while ((spip->spi->SR & SPIx_SR_RFDF) == 0)
    ;

  frame = spip->spi->POPR;

  spi_stop_xfer(spip);

  return frame;
}

#endif /* HAL_USE_SPI */

/** @} */
