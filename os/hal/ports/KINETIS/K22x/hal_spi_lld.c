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
  spip->spi->MCR = SPIx_MCR_MSTR | SPIx_MCR_CLR_TXF | SPIx_MCR_CLR_RXF;

  /* If we are not polling then enable DMA */
  if (!polling) {

#if KINETIS_SPI_USE_SPI0
    /* Enable receive dma and transmit dma */
    spip->spi->RSER = SPIx_RSER_RFDF_DIRS | SPIx_RSER_RFDF_RE |
        SPIx_RSER_TFFF_RE | SPIx_RSER_TFFF_DIRS;

    /* Configure RX DMA */
    if (spip->rxbuf) {
      DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].DADDR = (uint32_t)spip->rxbuf;
      DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].DOFF = spip->word_size;
    } else {
      DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].DADDR = (uint32_t)&dmaRxDummy;
      DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].DOFF = 0;
    }
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].BITER_ELINKNO = spip->count;
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].CITER_ELINKNO = spip->count;

    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].CSR |= 0x8000; // turn on bandwidth control
    
    /* Enable Request Register (ERQ) for RX by writing 0 to SERQ */
    DMA->SERQ = KINETIS_SPI0_RX_DMA_CHANNEL;

    /* Configure TX DMA */
    if (spip->txbuf) {
      DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].SADDR =  (uint32_t)spip->txbuf;
      DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].SOFF = spip->word_size;
    } else {
      DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].SADDR =  (uint32_t)&dmaTxDummy;
      DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].SOFF = 0;
    }
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].BITER_ELINKNO = spip->count;
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].CITER_ELINKNO = spip->count;

    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].CSR |= 0x8000; // turn on bandwidth control

    /* Enable Request Register (ERQ) for TX by writing 1 to SERQ */
    DMA->SERQ = KINETIS_SPI0_TX_DMA_CHANNEL;
  }
#endif

#if 0
#if KINETIS_SPI_USE_SPI1
    /* Enable receive dma and transmit dma */
    spip->spi->RSER = SPIx_RSER_RFDF_DIRS | SPIx_RSER_RFDF_RE |
        SPIx_RSER_TFFF_RE | SPIx_RSER_TFFF_DIRS;

    /* Configure RX DMA */
    if (spip->rxbuf) {
      DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].DADDR = (uint32_t)spip->rxbuf;
      DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].DOFF = spip->word_size;
    } else {
      DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].DADDR = (uint32_t)&dmaRxDummy;
      DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].DOFF = 0;
    }
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].BITER_ELINKNO = spip->count;
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].CITER_ELINKNO = spip->count;

    /* Enable Request Register (ERQ) for RX by writing 0 to SERQ */
    DMA->SERQ = KINETIS_SPI1_RX_DMA_CHANNEL;

    /* Configure TX DMA */
    if (spip->txbuf) {
      DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].SADDR =  (uint32_t)spip->txbuf;
      DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].SOFF = spip->word_size;
    } else {
      DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].SADDR =  (uint32_t)&dmaTxDummy;
      DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].SOFF = 0;
    }
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].BITER_ELINKNO = spip->count;
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].CITER_ELINKNO = spip->count;

    /* Enable Request Register (ERQ) for TX by writing 1 to SERQ */
    DMA->SERQ = KINETIS_SPI1_TX_DMA_CHANNEL;
  }
#endif
#endif

}

static void spi_stop_xfer(SPIDriver *spip)
{
  /* Halt the DSPI peripheral */
  spip->spi->MCR = SPIx_MCR_MSTR | SPIx_MCR_HALT;

  /* Clear all the flags which are currently set. */
  spip->spi->SR |= spip->spi->SR;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if KINETIS_SPI_USE_SPI0 || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(KINETIS_DMA0_IRQ_VECTOR) {
  OSAL_IRQ_PROLOGUE();

  /* Clear bit 0 in Interrupt Request Register (INT) by writing 0 to CINT */
  DMA->CINT = KINETIS_SPI0_RX_DMA_CHANNEL;

  spi_stop_xfer(&SPID1);

  _spi_isr_code(&SPID1);

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

  // setup drive strengths
  PORTD_PCR0 = 0x203; // pull up enabled, fast slew  (CS0)
  PORTD_PCR1 = 0x203; // pull up enabled, fast slew (clk)
  PORTD_PCR2 = 0x200; // fast slew (mosi)
  PORTD_PCR3 = 0x200; // fast slew (miso)

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

    nvicEnableVector(DMA0_IRQn, KINETIS_SPI0_RX_DMA_IRQ_PRIORITY);

    SIM->SCGC6 |= SIM_SCGC6_DMAMUX;
    SIM->SCGC7 |= SIM_SCGC7_DMA;

    /* Clear DMA error flags */
    DMA->ERR = 0x0F;

#if KINETIS_SPI_USE_SPI0
    /* Rx, select SPI Rx FIFO */
    DMAMUX->CHCFG[KINETIS_SPI0_RX_DMAMUX_CHANNEL] = DMAMUX_CHCFGn_ENBL |
        DMAMUX_CHCFGn_SOURCE(DMAMUX_SPI_RX_SOURCE);

    /* Tx, select SPI Tx FIFO */
    DMAMUX->CHCFG[KINETIS_SPI0_TX_DMAMUX_CHANNEL] = DMAMUX_CHCFGn_ENBL |
        DMAMUX_CHCFGn_SOURCE(DMAMUX_SPI_TX_SOURCE);

    /* Extract the frame size from the TAR */
    uint16_t frame_size = ((spip->spi->CTAR[0] >> SPIx_CTARn_FMSZ_SHIFT) &
        SPIx_CTARn_FMSZ_MASK) + 1;

    /* DMA transfer size is 16 bits for a frame size > 8 bits */
    uint16_t dma_size = frame_size > 8 ? 1 : 0;

    /* DMA word size is 2 for a 16 bit frame size */
    spip->word_size = frame_size > 8 ? 2 : 1;

    /* configure DMA RX fixed values */
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].SADDR = (uint32_t)&SPI0->POPR;
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].SOFF = 0;
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].SLAST = 0;
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].DLASTSGA = 0;
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(dma_size) |
        DMA_ATTR_DSIZE(dma_size);
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].NBYTES_MLNO = spip->word_size;
    DMA->TCD[KINETIS_SPI0_RX_DMA_CHANNEL].CSR = DMA_CSR_DREQ_MASK |
        DMA_CSR_INTMAJOR_MASK;

    /* configure DMA TX fixed values */
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].SLAST = 0;
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].DADDR = (uint32_t)&SPI0->PUSHR;
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].DOFF = 0;
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].DLASTSGA = 0;
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(dma_size) |
        DMA_ATTR_DSIZE(dma_size);
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].NBYTES_MLNO = spip->word_size;
    DMA->TCD[KINETIS_SPI0_TX_DMA_CHANNEL].CSR = DMA_CSR_DREQ_MASK;
#endif
    
#if 0
#if KINETIS_SPI_USE_SPI1
    /* Rx, select SPI Rx FIFO */
    DMAMUX->CHCFG[KINETIS_SPI1_RX_DMAMUX_CHANNEL] = DMAMUX_CHCFGn_ENBL |
        DMAMUX_CHCFGn_SOURCE(DMAMUX_SPI_RX_SOURCE);

    /* Tx, select SPI Tx FIFO */
    DMAMUX->CHCFG[KINETIS_SPI1_TX_DMAMUX_CHANNEL] = DMAMUX_CHCFGn_ENBL |
        DMAMUX_CHCFGn_SOURCE(DMAMUX_SPI_TX_SOURCE);

    /* Extract the frame size from the TAR */
    uint16_t frame_size = ((spip->spi->CTAR[0] >> SPIx_CTARn_FMSZ_SHIFT) &
        SPIx_CTARn_FMSZ_MASK) + 1;

    /* DMA transfer size is 16 bits for a frame size > 8 bits */
    uint16_t dma_size = frame_size > 8 ? 1 : 0;

    /* DMA word size is 2 for a 16 bit frame size */
    spip->word_size = frame_size > 8 ? 2 : 1;

    /* configure DMA RX fixed values */
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].SADDR = (uint32_t)&SPI1->POPR;
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].SOFF = 0;
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].SLAST = 0;
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].DLASTSGA = 0;
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(dma_size) |
        DMA_ATTR_DSIZE(dma_size);
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].NBYTES_MLNO = spip->word_size;
    DMA->TCD[KINETIS_SPI1_RX_DMA_CHANNEL].CSR = DMA_CSR_DREQ_MASK |
        DMA_CSR_INTMAJOR_MASK;

    /* configure DMA TX fixed values */
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].SLAST = 0;
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].DADDR = (uint32_t)&SPI1->PUSHR;
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].DOFF = 0;
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].DLASTSGA = 0;
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(dma_size) |
        DMA_ATTR_DSIZE(dma_size);
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].NBYTES_MLNO = spip->word_size;
    DMA->TCD[KINETIS_SPI1_TX_DMA_CHANNEL].CSR = DMA_CSR_DREQ_MASK;
#endif
#else
#if KINETIS_SPI_USE_SPI1
    
#endif
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

#if 0 // this code breaks error handling and recovery, because it turns off DMA and prevents other stuff from working
    
    nvicDisableVector(DMA0_IRQn);
    nvicDisableVector(SPI1_IRQn);

    SIM->SCGC7 &= ~SIM_SCGC7_DMA;
    SIM->SCGC6 &= ~SIM_SCGC6_DMAMUX;

#if KINETIS_SPI_USE_SPI0
    if (&SPID1 == spip) {
      /* SPI halt.*/
      spip->spi->MCR |= SPIx_MCR_HALT;
    }

    /* Disable the clock for SPI0 */
    SIM->SCGC6 &= ~SIM_SCGC6_SPI0;
#endif

#if KINETIS_SPI_USE_SPI1
    if (&SPID2 == spip) {
      /* SPI halt.*/
      spip->spi->MCR |= SPIx_MCR_HALT;
    }

    /* Disable the clock for SPI1 */
    SIM->SCGC6 &= ~SIM_SCGC6_SPI1;
#endif
    
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
    SPIx_CTARn_CPOL | \
    SPIx_CTARn_CPHA | \
    SPIx_CTARn_DBR | \
    SPIx_CTARn_PBR(1) | \
    SPIx_CTARn_BR(2) |	  \
    SPIx_CTARn_CSSCK(0) | \
    SPIx_CTARn_ASC(0) | \
    SPIx_CTARn_DT(0)

void spiRuntSetup(SPIDriver *spip) {
  nvicDisableVector(DMA1_IRQn); // disable DMA, as this SPI lacks bidirectional DMA
  nvicEnableVector(SPI1_IRQn, KINETIS_SPI_SPI1_IRQ_PRIORITY); // use interrupts instead
  
  spip->spi->MCR = SPIx_MCR_MSTR | SPIx_MCR_CLR_TXF | \
    SPIx_MCR_CLR_RXF | SPIx_MCR_DIS_RXF | SPIx_MCR_DIS_TXF | SPIx_MCR_CLR_TXF;
  spip->spi->CTAR[0] = KINETIS_SPI_TAR_BUSCLK_XZ(8);  // 8-bit frame size
  
  //  spip->spi->MCR |= SPIx_MCR_PCSIS(1); // set to active low for CS0
  palSetPad(IOPORT4, 4); // clear the CS line
}

// because SPID2 on this chip doesn't have a proper FIFO/DMA system, do something
// much stupider to get things going
#if 0
void spi_start_xfer_runt(SPIDriver *spip) {
  uint32_t i = 0;
  uint32_t pushr;
  uint32_t count;
  uint8_t *data;
  uint8_t *rxdata;

  count = spip->count;
  data = (uint8_t *) spip->txbuf;
  rxdata = (uint8_t *) spip->rxbuf;
  if( data == NULL )
    data = rxdata; // for Rx-only, send dummy bytes, eg the rx buffer
  
  // poll loop to send data
  while( i < count ) {
    while ( (spip->spi->SR & SPI_SR_TFFF_MASK) && (i < count) ) {
      if( i == 0 ) // mark first with clear transfer count
	pushr = SPI_PUSHR_CONT_MASK | SPI_PUSHR_CTCNT_MASK | data[i];
      else
	pushr = SPI_PUSHR_CONT_MASK | data[i];
      
      spip->spi->PUSHR = pushr;
      i++;
      
      //      while( !(spip->spi->SR & SPI_SR_TCF_MASK) )
      //	; // wait for transfer to complete, because TFFF lies
      while( (spip->spi->TCR >> 16) != i )
	; // wait for transfer to complete

      if( rxdata != NULL )
	rxdata[i-1] = (uint8_t) spip->spi->POPR;
    }
  }

  // note that checking TCF isn't good enough -- the CPU runs fast enough
  // that TCR doesn't update for a few cycles after TCF and you'll get
  // an out of date TCR that's short the last transfer
  // so we just wait until the TCR reflects the actual amount received
  
  while( (spip->spi->TCR >> 16) != count )
    ; // wait for transfer to complete
  
  //  return (spip->spi->TCR >> 16);
  
  //  _spi_isr_code(&SPID2); // fill in the epilogue manually
  SPID2.state = SPI_READY;
  //osalSysLock(); // already in "S" state
  osalThreadResumeS(&(SPID2.thread), MSG_OK);
  //osalSysUnlock();
  
  return;
}
#else
void spi_start_xfer_runt(SPIDriver *spip) {
  uint32_t pushr;

  if( spip->count == 0 )
    return;  // abort if we were called with null data

  if( spip->txbuf == NULL )
    spip->txbuf = spip->rxbuf;  // dummy send rx buf data

  spip->spi->MCR |= SPIx_MCR_DIS_RXF | SPIx_MCR_CLR_RXF | SPIx_MCR_DIS_TXF | SPIx_MCR_CLR_TXF; // disable fifos
  spip->spi->MCR &= ~SPIx_MCR_HALT; // clear the halt bit, chibios sets it every time the transfer is done
  // palClearPad(IOPORT4, 4); // assert CS line  // this is now on "manual" control
  
  spip->spi->SR |= SPIx_SR_TCF; // clear the TCF so it can flip
  spip->spi->RSER |= SPIx_RSER_TCF_RE; // enable interrupt on frame complete, to initiate chaining
  
  
  //  pushr = SPI_PUSHR_CONT_MASK | SPI_PUSHR_CTCNT_MASK | SPIx_PUSHR_PCS(1) | spip->txbuf[0]; // clear TCR with push
  pushr = SPI_PUSHR_CTCNT_MASK | spip->txbuf[0]; // clear TCR with push
  spip->spi->PUSHR = pushr; // kicks off a send of one data, ISR fires when done
  
  return;
}
#endif

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

  if( spip == &SPID1 ) 
    spi_start_xfer(spip, false);
  else
    spi_start_xfer_runt(spip);
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

  if( spip == &SPID1 ) 
    spi_start_xfer(spip, false);
  else
    spi_start_xfer_runt(spip);
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

  if( spip == &SPID1 ) 
    spi_start_xfer(spip, false);
  else
    spi_start_xfer_runt(spip);
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
