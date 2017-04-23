#if !defined(MK22F51212_EXTENSIONS_H_)  /* Check if memory map has not been already included */
#define MK22F51212_EXTENSIONS_H_

/**
 * @brief Macro to access a single bit of a 32-bit peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_ACCESS32(Reg,Bit) (*((uint32_t volatile*)(0x42000000u + (32u*((uintptr_t)(Reg) - (uintptr_t)0x40000000u)) + (4u*((uintptr_t)(Bit))))))

/**
 * @brief Macro to access a single bit of a 16-bit peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_ACCESS16(Reg,Bit) (*((uint16_t volatile*)(0x42000000u + (32u*((uintptr_t)(Reg) - (uintptr_t)0x40000000u)) + (4u*((uintptr_t)(Bit))))))

/**
 * @brief Macro to access a single bit of an 8-bit peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_ACCESS8(Reg,Bit) (*((uint8_t volatile*)(0x42000000u + (32u*((uintptr_t)(Reg) - (uintptr_t)0x40000000u)) + (4u*((uintptr_t)(Bit))))))

#define SIM_SOPT1_RAMSIZE_SHIFT                  12
#define SIM_SOPT1_OSC32KOUT_SHIFT                16
#define SIM_SOPT1_OSC32KSEL_SHIFT                18
#define SIM_SOPT1_USBVSTBY_SHIFT                 29
#define SIM_SOPT1_USBSSTBY_SHIFT                 30
#define SIM_SOPT1_USBREGEN_SHIFT                 31
#define SIM_SOPT1CFG_URWE_SHIFT                  24
#define SIM_SOPT1CFG_UVSWE_SHIFT                 25
#define SIM_SOPT1CFG_USSWE_SHIFT                 26
#define SIM_SOPT2_RTCCLKOUTSEL_SHIFT             4
#define SIM_SOPT2_CLKOUTSEL_SHIFT                5
#define SIM_SOPT2_TRACECLKSEL_SHIFT              12
#define SIM_SOPT2_PLLFLLSEL_SHIFT                16
#define SIM_SOPT2_USBSRC_SHIFT                   18
#define SIM_SOPT2_LPUARTSRC_SHIFT                26
#define SIM_SOPT4_FTM0FLT0_SHIFT                 0
#define SIM_SOPT4_FTM0FLT1_SHIFT                 1
#define SIM_SOPT4_FTM1FLT0_SHIFT                 4
#define SIM_SOPT4_FTM2FLT0_SHIFT                 8
#define SIM_SOPT4_FTM3FLT0_SHIFT                 12
#define SIM_SOPT4_FTM1CH0SRC_SHIFT               18
#define SIM_SOPT4_FTM2CH0SRC_SHIFT               20
#define SIM_SOPT4_FTM2CH1SRC_SHIFT               22
#define SIM_SOPT4_FTM0CLKSEL_SHIFT               24
#define SIM_SOPT4_FTM1CLKSEL_SHIFT               25
#define SIM_SOPT4_FTM2CLKSEL_SHIFT               26
#define SIM_SOPT4_FTM3CLKSEL_SHIFT               27
#define SIM_SOPT4_FTM0TRG0SRC_SHIFT              28
#define SIM_SOPT4_FTM0TRG1SRC_SHIFT              29
#define SIM_SOPT4_FTM3TRG0SRC_SHIFT              30
#define SIM_SOPT4_FTM3TRG1SRC_SHIFT              31
#define SIM_SOPT5_UART0TXSRC_SHIFT               0
#define SIM_SOPT5_UART0RXSRC_SHIFT               2
#define SIM_SOPT5_UART1TXSRC_SHIFT               4
#define SIM_SOPT5_UART1RXSRC_SHIFT               6
#define SIM_SOPT5_LPUART0RXSRC_SHIFT             18
#define SIM_SOPT7_ADC0TRGSEL_SHIFT               0
#define SIM_SOPT7_ADC0PRETRGSEL_SHIFT            4
#define SIM_SOPT7_ADC0ALTTRGEN_SHIFT             7
#define SIM_SOPT7_ADC1TRGSEL_SHIFT               8
#define SIM_SOPT7_ADC1PRETRGSEL_SHIFT            12
#define SIM_SOPT7_ADC1ALTTRGEN_SHIFT             15
#define SIM_SOPT8_FTM0SYNCBIT_SHIFT              0
#define SIM_SOPT8_FTM1SYNCBIT_SHIFT              1
#define SIM_SOPT8_FTM2SYNCBIT_SHIFT              2
#define SIM_SOPT8_FTM3SYNCBIT_SHIFT              3
#define SIM_SOPT8_FTM0OCH0SRC_SHIFT              16
#define SIM_SOPT8_FTM0OCH1SRC_SHIFT              17
#define SIM_SOPT8_FTM0OCH2SRC_SHIFT              18
#define SIM_SOPT8_FTM0OCH3SRC_SHIFT              19
#define SIM_SOPT8_FTM0OCH4SRC_SHIFT              20
#define SIM_SOPT8_FTM0OCH5SRC_SHIFT              21
#define SIM_SOPT8_FTM0OCH6SRC_SHIFT              22
#define SIM_SOPT8_FTM0OCH7SRC_SHIFT              23
#define SIM_SOPT8_FTM3OCH0SRC_SHIFT              24
#define SIM_SOPT8_FTM3OCH1SRC_SHIFT              25
#define SIM_SOPT8_FTM3OCH2SRC_SHIFT              26
#define SIM_SOPT8_FTM3OCH3SRC_SHIFT              27
#define SIM_SOPT8_FTM3OCH4SRC_SHIFT              28
#define SIM_SOPT8_FTM3OCH5SRC_SHIFT              29
#define SIM_SOPT8_FTM3OCH6SRC_SHIFT              30
#define SIM_SOPT8_FTM3OCH7SRC_SHIFT              31
#define SIM_SDID_PINID_SHIFT                     0
#define SIM_SDID_FAMID_SHIFT                     4
#define SIM_SDID_DIEID_SHIFT                     7
#define SIM_SDID_REVID_SHIFT                     12
#define SIM_SDID_SERIESID_SHIFT                  20
#define SIM_SDID_SUBFAMID_SHIFT                  24
#define SIM_SDID_FAMILYID_SHIFT                  28
#define SIM_SCGC4_EWM_SHIFT                      1
#define SIM_SCGC4_I2C0_SHIFT                     6
#define SIM_SCGC4_I2C1_SHIFT                     7
#define SIM_SCGC4_UART0_SHIFT                    10
#define SIM_SCGC4_UART1_SHIFT                    11
#define SIM_SCGC4_UART2_SHIFT                    12
#define SIM_SCGC4_USBOTG_SHIFT                   18
#define SIM_SCGC4_CMP_SHIFT                      19
#define SIM_SCGC4_VREF_SHIFT                     20
#define SIM_SCGC5_LPTMR_SHIFT                    0
#define SIM_SCGC5_PORTA_SHIFT                    9
#define SIM_SCGC5_PORTB_SHIFT                    10
#define SIM_SCGC5_PORTC_SHIFT                    11
#define SIM_SCGC5_PORTD_SHIFT                    12
#define SIM_SCGC5_PORTE_SHIFT                    13
#define SIM_SCGC6_FTF_SHIFT                      0
#define SIM_SCGC6_DMAMUX_SHIFT                   1
#define SIM_SCGC6_FTM3_SHIFT                     6
#define SIM_SCGC6_ADC1_SHIFT                     7
#define SIM_SCGC6_DAC1_SHIFT                     8
#define SIM_SCGC6_RNGA_SHIFT                     9
#define SIM_SCGC6_LPUART0_SHIFT                  10
#define SIM_SCGC6_SPI0_SHIFT                     12
#define SIM_SCGC6_SPI1_SHIFT                     13
#define SIM_SCGC6_I2S_SHIFT                      15
#define SIM_SCGC6_CRC_SHIFT                      18
#define SIM_SCGC6_PDB_SHIFT                      22
#define SIM_SCGC6_PIT_SHIFT                      23
#define SIM_SCGC6_FTM0_SHIFT                     24
#define SIM_SCGC6_FTM1_SHIFT                     25
#define SIM_SCGC6_FTM2_SHIFT                     26
#define SIM_SCGC6_ADC0_SHIFT                     27
#define SIM_SCGC6_RTC_SHIFT                      29
#define SIM_SCGC6_DAC0_SHIFT                     31
#define SIM_SCGC7_FLEXBUS_SHIFT                  0
#define SIM_SCGC7_DMA_SHIFT                      1
#define SIM_CLKDIV1_OUTDIV4_SHIFT                16
#define SIM_CLKDIV1_OUTDIV2_SHIFT                24
#define SIM_CLKDIV1_OUTDIV1_SHIFT                28
#define SIM_CLKDIV2_USBFRAC_SHIFT                0
#define SIM_CLKDIV2_USBDIV_SHIFT                 1
#define SIM_FCFG1_FLASHDIS_SHIFT                 0
#define SIM_FCFG1_FLASHDOZE_SHIFT                1
#define SIM_FCFG1_PFSIZE_SHIFT                   24
#define SIM_FCFG2_MAXADDR1_SHIFT                 16
#define SIM_FCFG2_MAXADDR0_SHIFT                 24
#define SIM_UIDH_UID_SHIFT                       0
#define SIM_UIDMH_UID_SHIFT                      0
#define SIM_UIDML_UID_SHIFT                      0
#define SIM_UIDL_UID_SHIFT                       0

/*
 * MK22F51212 SIM
 *
 * System Integration Module
 *
 * Registers defined in this header file:
 * - SIM_SOPT1 - System Options Register 1
 * - SIM_SOPT1CFG - SOPT1 Configuration Register
 * - SIM_SOPT2 - System Options Register 2
 * - SIM_SOPT4 - System Options Register 4
 * - SIM_SOPT5 - System Options Register 5
 * - SIM_SOPT7 - System Options Register 7
 * - SIM_SOPT8 - System Options Register 8
 * - SIM_SDID - System Device Identification Register
 * - SIM_SCGC4 - System Clock Gating Control Register 4
 * - SIM_SCGC5 - System Clock Gating Control Register 5
 * - SIM_SCGC6 - System Clock Gating Control Register 6
 * - SIM_SCGC7 - System Clock Gating Control Register 7
 * - SIM_CLKDIV1 - System Clock Divider Register 1
 * - SIM_CLKDIV2 - System Clock Divider Register 2
 * - SIM_FCFG1 - Flash Configuration Register 1
 * - SIM_FCFG2 - Flash Configuration Register 2
 * - SIM_UIDH - Unique Identification Register High
 * - SIM_UIDMH - Unique Identification Register Mid-High
 * - SIM_UIDML - Unique Identification Register Mid Low
 * - SIM_UIDL - Unique Identification Register Low
 */

#define SIM_INSTANCE_COUNT (1U) /*!< Number of instances of the SIM module. */
#define SIM_IDX (0U) /*!< Instance number for SIM. */

/*******************************************************************************
 * SIM_SOPT1 - System Options Register 1
 ******************************************************************************/

/*!
 * @brief SIM_SOPT1 - System Options Register 1 (RW)
 *
 * Reset value: 0x80000000U
 *
 * The SOPT1 register is only reset on POR or LVD.
 */
/*!
 * @name Constants and macros for entire SIM_SOPT1 register
 */
/*@{*/
#define SIM_RD_SOPT1(base)       (SIM_SOPT1_REG(base))
#define SIM_WR_SOPT1(base, value) (SIM_SOPT1_REG(base) = (value))
#define SIM_RMW_SOPT1(base, mask, value) (SIM_WR_SOPT1(base, (SIM_RD_SOPT1(base) & ~(mask)) | (value)))
#define SIM_SET_SOPT1(base, value) (SIM_WR_SOPT1(base, SIM_RD_SOPT1(base) |  (value)))
#define SIM_CLR_SOPT1(base, value) (SIM_WR_SOPT1(base, SIM_RD_SOPT1(base) & ~(value)))
#define SIM_TOG_SOPT1(base, value) (SIM_WR_SOPT1(base, SIM_RD_SOPT1(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SOPT1 bitfields
 */

/*!
 * @name Register SIM_SOPT1, field RAMSIZE[15:12] (RO)
 *
 * This field specifies the amount of system RAM available on the device.
 *
 * Values:
 * - 0b0001 - 8 KB
 * - 0b0011 - 16 KB
 * - 0b0100 - 24 KB
 * - 0b0101 - 32 KB
 * - 0b0110 - 48 KB
 * - 0b0111 - 64 KB
 * - 0b1000 - 96 KB
 * - 0b1001 - 128 KB
 * - 0b1011 - 256 KB
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1_RAMSIZE field. */
#define SIM_RD_SOPT1_RAMSIZE(base) ((SIM_SOPT1_REG(base) & SIM_SOPT1_RAMSIZE_MASK) >> SIM_SOPT1_RAMSIZE_SHIFT)
#define SIM_BRD_SOPT1_RAMSIZE(base) (SIM_RD_SOPT1_RAMSIZE(base))
/*@}*/

/*!
 * @name Register SIM_SOPT1, field OSC32KOUT[17:16] (RW)
 *
 * Outputs the ERCLK32K on the selected pin in all modes of operation (including
 * LLS/VLLS and System Reset), overriding the existing pin mux configuration for
 * that pin. This field is reset only on POR/LVD.
 *
 * Values:
 * - 0b00 - ERCLK32K is not output.
 * - 0b01 - ERCLK32K is output on PTE0.
 * - 0b10 - ERCLK32K is output on PTE26.
 * - 0b11 - Reserved.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1_OSC32KOUT field. */
#define SIM_RD_SOPT1_OSC32KOUT(base) ((SIM_SOPT1_REG(base) & SIM_SOPT1_OSC32KOUT_MASK) >> SIM_SOPT1_OSC32KOUT_SHIFT)
#define SIM_BRD_SOPT1_OSC32KOUT(base) (SIM_RD_SOPT1_OSC32KOUT(base))

/*! @brief Set the OSC32KOUT field to a new value. */
#define SIM_WR_SOPT1_OSC32KOUT(base, value) (SIM_RMW_SOPT1(base, SIM_SOPT1_OSC32KOUT_MASK, SIM_SOPT1_OSC32KOUT(value)))
#define SIM_BWR_SOPT1_OSC32KOUT(base, value) (SIM_WR_SOPT1_OSC32KOUT(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT1, field OSC32KSEL[19:18] (RW)
 *
 * Selects the 32 kHz clock source (ERCLK32K) for LPTMR. This field is reset
 * only on POR/LVD.
 *
 * Values:
 * - 0b00 - System oscillator (OSC32KCLK)
 * - 0b01 - Reserved
 * - 0b10 - RTC 32.768kHz oscillator
 * - 0b11 - LPO 1 kHz
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1_OSC32KSEL field. */
#define SIM_RD_SOPT1_OSC32KSEL(base) ((SIM_SOPT1_REG(base) & SIM_SOPT1_OSC32KSEL_MASK) >> SIM_SOPT1_OSC32KSEL_SHIFT)
#define SIM_BRD_SOPT1_OSC32KSEL(base) (SIM_RD_SOPT1_OSC32KSEL(base))

/*! @brief Set the OSC32KSEL field to a new value. */
#define SIM_WR_SOPT1_OSC32KSEL(base, value) (SIM_RMW_SOPT1(base, SIM_SOPT1_OSC32KSEL_MASK, SIM_SOPT1_OSC32KSEL(value)))
#define SIM_BWR_SOPT1_OSC32KSEL(base, value) (SIM_WR_SOPT1_OSC32KSEL(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT1, field USBVSTBY[29] (RW)
 *
 * Controls whether the USB voltage regulator is placed in standby mode during
 * VLPR and VLPW modes.
 *
 * Values:
 * - 0b0 - USB voltage regulator not in standby during VLPR and VLPW modes.
 * - 0b1 - USB voltage regulator in standby during VLPR and VLPW modes.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1_USBVSTBY field. */
#define SIM_RD_SOPT1_USBVSTBY(base) ((SIM_SOPT1_REG(base) & SIM_SOPT1_USBVSTBY_MASK) >> SIM_SOPT1_USBVSTBY_SHIFT)
#define SIM_BRD_SOPT1_USBVSTBY(base) (BITBAND_ACCESS32(&SIM_SOPT1_REG(base), SIM_SOPT1_USBVSTBY_SHIFT))

/*! @brief Set the USBVSTBY field to a new value. */
#define SIM_WR_SOPT1_USBVSTBY(base, value) (SIM_RMW_SOPT1(base, SIM_SOPT1_USBVSTBY_MASK, SIM_SOPT1_USBVSTBY(value)))
#define SIM_BWR_SOPT1_USBVSTBY(base, value) (BITBAND_ACCESS32(&SIM_SOPT1_REG(base), SIM_SOPT1_USBVSTBY_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT1, field USBSSTBY[30] (RW)
 *
 * Controls whether the USB voltage regulator is placed in standby mode during
 * Stop, VLPS, LLS and VLLS modes.
 *
 * Values:
 * - 0b0 - USB voltage regulator not in standby during Stop, VLPS, LLS and VLLS
 *     modes.
 * - 0b1 - USB voltage regulator in standby during Stop, VLPS, LLS and VLLS
 *     modes.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1_USBSSTBY field. */
#define SIM_RD_SOPT1_USBSSTBY(base) ((SIM_SOPT1_REG(base) & SIM_SOPT1_USBSSTBY_MASK) >> SIM_SOPT1_USBSSTBY_SHIFT)
#define SIM_BRD_SOPT1_USBSSTBY(base) (BITBAND_ACCESS32(&SIM_SOPT1_REG(base), SIM_SOPT1_USBSSTBY_SHIFT))

/*! @brief Set the USBSSTBY field to a new value. */
#define SIM_WR_SOPT1_USBSSTBY(base, value) (SIM_RMW_SOPT1(base, SIM_SOPT1_USBSSTBY_MASK, SIM_SOPT1_USBSSTBY(value)))
#define SIM_BWR_SOPT1_USBSSTBY(base, value) (BITBAND_ACCESS32(&SIM_SOPT1_REG(base), SIM_SOPT1_USBSSTBY_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT1, field USBREGEN[31] (RW)
 *
 * Controls whether the USB voltage regulator is enabled.
 *
 * Values:
 * - 0b0 - USB voltage regulator is disabled.
 * - 0b1 - USB voltage regulator is enabled.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1_USBREGEN field. */
#define SIM_RD_SOPT1_USBREGEN(base) ((SIM_SOPT1_REG(base) & SIM_SOPT1_USBREGEN_MASK) >> SIM_SOPT1_USBREGEN_SHIFT)
#define SIM_BRD_SOPT1_USBREGEN(base) (BITBAND_ACCESS32(&SIM_SOPT1_REG(base), SIM_SOPT1_USBREGEN_SHIFT))

/*! @brief Set the USBREGEN field to a new value. */
#define SIM_WR_SOPT1_USBREGEN(base, value) (SIM_RMW_SOPT1(base, SIM_SOPT1_USBREGEN_MASK, SIM_SOPT1_USBREGEN(value)))
#define SIM_BWR_SOPT1_USBREGEN(base, value) (BITBAND_ACCESS32(&SIM_SOPT1_REG(base), SIM_SOPT1_USBREGEN_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SOPT1CFG - SOPT1 Configuration Register
 ******************************************************************************/

/*!
 * @brief SIM_SOPT1CFG - SOPT1 Configuration Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * The SOPT1CFG register is reset on System Reset not VLLS.
 */
/*!
 * @name Constants and macros for entire SIM_SOPT1CFG register
 */
/*@{*/
#define SIM_RD_SOPT1CFG(base)    (SIM_SOPT1CFG_REG(base))
#define SIM_WR_SOPT1CFG(base, value) (SIM_SOPT1CFG_REG(base) = (value))
#define SIM_RMW_SOPT1CFG(base, mask, value) (SIM_WR_SOPT1CFG(base, (SIM_RD_SOPT1CFG(base) & ~(mask)) | (value)))
#define SIM_SET_SOPT1CFG(base, value) (SIM_WR_SOPT1CFG(base, SIM_RD_SOPT1CFG(base) |  (value)))
#define SIM_CLR_SOPT1CFG(base, value) (SIM_WR_SOPT1CFG(base, SIM_RD_SOPT1CFG(base) & ~(value)))
#define SIM_TOG_SOPT1CFG(base, value) (SIM_WR_SOPT1CFG(base, SIM_RD_SOPT1CFG(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SOPT1CFG bitfields
 */

/*!
 * @name Register SIM_SOPT1CFG, field URWE[24] (RW)
 *
 * Writing one to the URWE bit allows the SOPT1 USBREGEN bit to be written. This
 * register bit clears after a write to USBREGEN.
 *
 * Values:
 * - 0b0 - SOPT1 USBREGEN cannot be written.
 * - 0b1 - SOPT1 USBREGEN can be written.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1CFG_URWE field. */
#define SIM_RD_SOPT1CFG_URWE(base) ((SIM_SOPT1CFG_REG(base) & SIM_SOPT1CFG_URWE_MASK) >> SIM_SOPT1CFG_URWE_SHIFT)
#define SIM_BRD_SOPT1CFG_URWE(base) (BITBAND_ACCESS32(&SIM_SOPT1CFG_REG(base), SIM_SOPT1CFG_URWE_SHIFT))

/*! @brief Set the URWE field to a new value. */
#define SIM_WR_SOPT1CFG_URWE(base, value) (SIM_RMW_SOPT1CFG(base, SIM_SOPT1CFG_URWE_MASK, SIM_SOPT1CFG_URWE(value)))
#define SIM_BWR_SOPT1CFG_URWE(base, value) (BITBAND_ACCESS32(&SIM_SOPT1CFG_REG(base), SIM_SOPT1CFG_URWE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT1CFG, field UVSWE[25] (RW)
 *
 * Writing one to the UVSWE bit allows the SOPT1 USBVSTBY bit to be written.
 * This register bit clears after a write to USBVSTBY.
 *
 * Values:
 * - 0b0 - SOPT1 USBVSTBY cannot be written.
 * - 0b1 - SOPT1 USBVSTBY can be written.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1CFG_UVSWE field. */
#define SIM_RD_SOPT1CFG_UVSWE(base) ((SIM_SOPT1CFG_REG(base) & SIM_SOPT1CFG_UVSWE_MASK) >> SIM_SOPT1CFG_UVSWE_SHIFT)
#define SIM_BRD_SOPT1CFG_UVSWE(base) (BITBAND_ACCESS32(&SIM_SOPT1CFG_REG(base), SIM_SOPT1CFG_UVSWE_SHIFT))

/*! @brief Set the UVSWE field to a new value. */
#define SIM_WR_SOPT1CFG_UVSWE(base, value) (SIM_RMW_SOPT1CFG(base, SIM_SOPT1CFG_UVSWE_MASK, SIM_SOPT1CFG_UVSWE(value)))
#define SIM_BWR_SOPT1CFG_UVSWE(base, value) (BITBAND_ACCESS32(&SIM_SOPT1CFG_REG(base), SIM_SOPT1CFG_UVSWE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT1CFG, field USSWE[26] (RW)
 *
 * Writing one to the USSWE bit allows the SOPT1 USBSSTBY bit to be written.
 * This register bit clears after a write to USBSSTBY.
 *
 * Values:
 * - 0b0 - SOPT1 USBSSTBY cannot be written.
 * - 0b1 - SOPT1 USBSSTBY can be written.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT1CFG_USSWE field. */
#define SIM_RD_SOPT1CFG_USSWE(base) ((SIM_SOPT1CFG_REG(base) & SIM_SOPT1CFG_USSWE_MASK) >> SIM_SOPT1CFG_USSWE_SHIFT)
#define SIM_BRD_SOPT1CFG_USSWE(base) (BITBAND_ACCESS32(&SIM_SOPT1CFG_REG(base), SIM_SOPT1CFG_USSWE_SHIFT))

/*! @brief Set the USSWE field to a new value. */
#define SIM_WR_SOPT1CFG_USSWE(base, value) (SIM_RMW_SOPT1CFG(base, SIM_SOPT1CFG_USSWE_MASK, SIM_SOPT1CFG_USSWE(value)))
#define SIM_BWR_SOPT1CFG_USSWE(base, value) (BITBAND_ACCESS32(&SIM_SOPT1CFG_REG(base), SIM_SOPT1CFG_USSWE_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SOPT2 - System Options Register 2
 ******************************************************************************/

/*!
 * @brief SIM_SOPT2 - System Options Register 2 (RW)
 *
 * Reset value: 0x00001000U
 *
 * SOPT2 contains the controls for selecting many of the module clock source
 * options on this device. See the Clock Distribution chapter for more information
 * including clocking diagrams and definitions of device clocks.
 */
/*!
 * @name Constants and macros for entire SIM_SOPT2 register
 */
/*@{*/
#define SIM_RD_SOPT2(base)       (SIM_SOPT2_REG(base))
#define SIM_WR_SOPT2(base, value) (SIM_SOPT2_REG(base) = (value))
#define SIM_RMW_SOPT2(base, mask, value) (SIM_WR_SOPT2(base, (SIM_RD_SOPT2(base) & ~(mask)) | (value)))
#define SIM_SET_SOPT2(base, value) (SIM_WR_SOPT2(base, SIM_RD_SOPT2(base) |  (value)))
#define SIM_CLR_SOPT2(base, value) (SIM_WR_SOPT2(base, SIM_RD_SOPT2(base) & ~(value)))
#define SIM_TOG_SOPT2(base, value) (SIM_WR_SOPT2(base, SIM_RD_SOPT2(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SOPT2 bitfields
 */

/*!
 * @name Register SIM_SOPT2, field RTCCLKOUTSEL[4] (RW)
 *
 * Selects either the RTC 1 Hz clock or the 32.768kHz clock to be output on the
 * RTC_CLKOUT pin.
 *
 * Values:
 * - 0b0 - RTC 1 Hz clock is output on the RTC_CLKOUT pin.
 * - 0b1 - RTC 32.768kHz clock is output on the RTC_CLKOUT pin.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT2_RTCCLKOUTSEL field. */
#define SIM_RD_SOPT2_RTCCLKOUTSEL(base) ((SIM_SOPT2_REG(base) & SIM_SOPT2_RTCCLKOUTSEL_MASK) >> SIM_SOPT2_RTCCLKOUTSEL_SHIFT)
#define SIM_BRD_SOPT2_RTCCLKOUTSEL(base) (BITBAND_ACCESS32(&SIM_SOPT2_REG(base), SIM_SOPT2_RTCCLKOUTSEL_SHIFT))

/*! @brief Set the RTCCLKOUTSEL field to a new value. */
#define SIM_WR_SOPT2_RTCCLKOUTSEL(base, value) (SIM_RMW_SOPT2(base, SIM_SOPT2_RTCCLKOUTSEL_MASK, SIM_SOPT2_RTCCLKOUTSEL(value)))
#define SIM_BWR_SOPT2_RTCCLKOUTSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT2_REG(base), SIM_SOPT2_RTCCLKOUTSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT2, field CLKOUTSEL[7:5] (RW)
 *
 * Selects the clock to output on the CLKOUT pin.
 *
 * Values:
 * - 0b000 - FlexBus CLKOUT
 * - 0b001 - Reserved
 * - 0b010 - Flash clock
 * - 0b011 - LPO clock (1 kHz)
 * - 0b100 - MCGIRCLK
 * - 0b101 - RTC 32.768kHz clock
 * - 0b110 - OSCERCLK0
 * - 0b111 - IRC 48 MHz clock
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT2_CLKOUTSEL field. */
#define SIM_RD_SOPT2_CLKOUTSEL(base) ((SIM_SOPT2_REG(base) & SIM_SOPT2_CLKOUTSEL_MASK) >> SIM_SOPT2_CLKOUTSEL_SHIFT)
#define SIM_BRD_SOPT2_CLKOUTSEL(base) (SIM_RD_SOPT2_CLKOUTSEL(base))

/*! @brief Set the CLKOUTSEL field to a new value. */
#define SIM_WR_SOPT2_CLKOUTSEL(base, value) (SIM_RMW_SOPT2(base, SIM_SOPT2_CLKOUTSEL_MASK, SIM_SOPT2_CLKOUTSEL(value)))
#define SIM_BWR_SOPT2_CLKOUTSEL(base, value) (SIM_WR_SOPT2_CLKOUTSEL(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT2, field FBSL[9:8] (RW)
 *
 * If flash security is enabled, then this field affects what CPU operations can
 * access off-chip via the FlexBus interface. This field has no effect if flash
 * security is not enabled.
 *
 * Values:
 * - 0b00 - All off-chip accesses (instruction and data) via the FlexBus are
 *     disallowed.
 * - 0b01 - All off-chip accesses (instruction and data) via the FlexBus are
 *     disallowed.
 * - 0b10 - Off-chip instruction accesses are disallowed. Data accesses are
 *     allowed.
 * - 0b11 - Off-chip instruction accesses and data accesses are allowed.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT2_FBSL field. */


/*!
 * @name Register SIM_SOPT2, field TRACECLKSEL[12] (RW)
 *
 * Selects the core/system clock or MCG output clock (MCGOUTCLK) as the trace
 * clock source.
 *
 * Values:
 * - 0b0 - MCGOUTCLK
 * - 0b1 - Core/system clock
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT2_TRACECLKSEL field. */
#define SIM_RD_SOPT2_TRACECLKSEL(base) ((SIM_SOPT2_REG(base) & SIM_SOPT2_TRACECLKSEL_MASK) >> SIM_SOPT2_TRACECLKSEL_SHIFT)
#define SIM_BRD_SOPT2_TRACECLKSEL(base) (BITBAND_ACCESS32(&SIM_SOPT2_REG(base), SIM_SOPT2_TRACECLKSEL_SHIFT))

/*! @brief Set the TRACECLKSEL field to a new value. */
#define SIM_WR_SOPT2_TRACECLKSEL(base, value) (SIM_RMW_SOPT2(base, SIM_SOPT2_TRACECLKSEL_MASK, SIM_SOPT2_TRACECLKSEL(value)))
#define SIM_BWR_SOPT2_TRACECLKSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT2_REG(base), SIM_SOPT2_TRACECLKSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT2, field PLLFLLSEL[17:16] (RW)
 *
 * Selects the high frequency clock for various peripheral clocking options.
 *
 * Values:
 * - 0b00 - MCGFLLCLK clock
 * - 0b01 - MCGPLLCLK clock
 * - 0b10 - Reserved
 * - 0b11 - IRC48 MHz clock
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT2_PLLFLLSEL field. */
#define SIM_RD_SOPT2_PLLFLLSEL(base) ((SIM_SOPT2_REG(base) & SIM_SOPT2_PLLFLLSEL_MASK) >> SIM_SOPT2_PLLFLLSEL_SHIFT)
#define SIM_BRD_SOPT2_PLLFLLSEL(base) (SIM_RD_SOPT2_PLLFLLSEL(base))

/*! @brief Set the PLLFLLSEL field to a new value. */
#define SIM_WR_SOPT2_PLLFLLSEL(base, value) (SIM_RMW_SOPT2(base, SIM_SOPT2_PLLFLLSEL_MASK, SIM_SOPT2_PLLFLLSEL(value)))
#define SIM_BWR_SOPT2_PLLFLLSEL(base, value) (SIM_WR_SOPT2_PLLFLLSEL(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT2, field USBSRC[18] (RW)
 *
 * Selects the clock source for the USB 48 MHz clock.
 *
 * Values:
 * - 0b0 - External bypass clock (USB_CLKIN).
 * - 0b1 - MCGFLLCLK , or MCGPLLCLK , or IRC48M clock as selected by
 *     SOPT2[PLLFLLSEL], and then divided by the USB fractional divider as configured by
 *     SIM_CLKDIV2[USBFRAC, USBDIV].
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT2_USBSRC field. */
#define SIM_RD_SOPT2_USBSRC(base) ((SIM_SOPT2_REG(base) & SIM_SOPT2_USBSRC_MASK) >> SIM_SOPT2_USBSRC_SHIFT)
#define SIM_BRD_SOPT2_USBSRC(base) (BITBAND_ACCESS32(&SIM_SOPT2_REG(base), SIM_SOPT2_USBSRC_SHIFT))

/*! @brief Set the USBSRC field to a new value. */
#define SIM_WR_SOPT2_USBSRC(base, value) (SIM_RMW_SOPT2(base, SIM_SOPT2_USBSRC_MASK, SIM_SOPT2_USBSRC(value)))
#define SIM_BWR_SOPT2_USBSRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT2_REG(base), SIM_SOPT2_USBSRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT2, field LPUARTSRC[27:26] (RW)
 *
 * Selects the clock source for the LPUART transmit and receive clock.
 *
 * Values:
 * - 0b00 - Clock disabled
 * - 0b01 - MCGFLLCLK , or MCGPLLCLK , or IRC48M clock as selected by
 *     SOPT2[PLLFLLSEL].
 * - 0b10 - OSCERCLK clock
 * - 0b11 - MCGIRCLK clock
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT2_LPUARTSRC field. */
#define SIM_RD_SOPT2_LPUARTSRC(base) ((SIM_SOPT2_REG(base) & SIM_SOPT2_LPUARTSRC_MASK) >> SIM_SOPT2_LPUARTSRC_SHIFT)
#define SIM_BRD_SOPT2_LPUARTSRC(base) (SIM_RD_SOPT2_LPUARTSRC(base))

/*! @brief Set the LPUARTSRC field to a new value. */
#define SIM_WR_SOPT2_LPUARTSRC(base, value) (SIM_RMW_SOPT2(base, SIM_SOPT2_LPUARTSRC_MASK, SIM_SOPT2_LPUARTSRC(value)))
#define SIM_BWR_SOPT2_LPUARTSRC(base, value) (SIM_WR_SOPT2_LPUARTSRC(base, value))
/*@}*/

/*******************************************************************************
 * SIM_SOPT4 - System Options Register 4
 ******************************************************************************/

/*!
 * @brief SIM_SOPT4 - System Options Register 4 (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_SOPT4 register
 */
/*@{*/
#define SIM_RD_SOPT4(base)       (SIM_SOPT4_REG(base))
#define SIM_WR_SOPT4(base, value) (SIM_SOPT4_REG(base) = (value))
#define SIM_RMW_SOPT4(base, mask, value) (SIM_WR_SOPT4(base, (SIM_RD_SOPT4(base) & ~(mask)) | (value)))
#define SIM_SET_SOPT4(base, value) (SIM_WR_SOPT4(base, SIM_RD_SOPT4(base) |  (value)))
#define SIM_CLR_SOPT4(base, value) (SIM_WR_SOPT4(base, SIM_RD_SOPT4(base) & ~(value)))
#define SIM_TOG_SOPT4(base, value) (SIM_WR_SOPT4(base, SIM_RD_SOPT4(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SOPT4 bitfields
 */

/*!
 * @name Register SIM_SOPT4, field FTM0FLT0[0] (RW)
 *
 * Selects the source of FTM0 fault 0. The pin source for fault 0 must be
 * configured for the FTM module fault function through the appropriate pin control
 * register in the port control module.
 *
 * Values:
 * - 0b0 - FTM0_FLT0 pin
 * - 0b1 - CMP0 out
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM0FLT0 field. */
#define SIM_RD_SOPT4_FTM0FLT0(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM0FLT0_MASK) >> SIM_SOPT4_FTM0FLT0_SHIFT)
#define SIM_BRD_SOPT4_FTM0FLT0(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0FLT0_SHIFT))

/*! @brief Set the FTM0FLT0 field to a new value. */
#define SIM_WR_SOPT4_FTM0FLT0(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM0FLT0_MASK, SIM_SOPT4_FTM0FLT0(value)))
#define SIM_BWR_SOPT4_FTM0FLT0(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0FLT0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM0FLT1[1] (RW)
 *
 * Selects the source of FTM0 fault 1. The pin source for fault 1 must be
 * configured for the FTM module fault function through the appropriate pin control
 * register in the port control module.
 *
 * Values:
 * - 0b0 - FTM0_FLT1 pin
 * - 0b1 - CMP1 out
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM0FLT1 field. */
#define SIM_RD_SOPT4_FTM0FLT1(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM0FLT1_MASK) >> SIM_SOPT4_FTM0FLT1_SHIFT)
#define SIM_BRD_SOPT4_FTM0FLT1(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0FLT1_SHIFT))

/*! @brief Set the FTM0FLT1 field to a new value. */
#define SIM_WR_SOPT4_FTM0FLT1(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM0FLT1_MASK, SIM_SOPT4_FTM0FLT1(value)))
#define SIM_BWR_SOPT4_FTM0FLT1(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0FLT1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM1FLT0[4] (RW)
 *
 * Selects the source of FTM1 fault 0. The pin source for fault 0 must be
 * configured for the FTM module fault function through the appropriate pin control
 * register in the port control module.
 *
 * Values:
 * - 0b0 - FTM1_FLT0 pin
 * - 0b1 - CMP0 out
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM1FLT0 field. */
#define SIM_RD_SOPT4_FTM1FLT0(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM1FLT0_MASK) >> SIM_SOPT4_FTM1FLT0_SHIFT)
#define SIM_BRD_SOPT4_FTM1FLT0(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM1FLT0_SHIFT))

/*! @brief Set the FTM1FLT0 field to a new value. */
#define SIM_WR_SOPT4_FTM1FLT0(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM1FLT0_MASK, SIM_SOPT4_FTM1FLT0(value)))
#define SIM_BWR_SOPT4_FTM1FLT0(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM1FLT0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM2FLT0[8] (RW)
 *
 * Selects the source of FTM2 fault 0. The pin source for fault 0 must be
 * configured for the FTM module fault function through the appropriate PORTx pin
 * control register.
 *
 * Values:
 * - 0b0 - FTM2_FLT0 pin
 * - 0b1 - CMP0 out
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM2FLT0 field. */
#define SIM_RD_SOPT4_FTM2FLT0(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM2FLT0_MASK) >> SIM_SOPT4_FTM2FLT0_SHIFT)
#define SIM_BRD_SOPT4_FTM2FLT0(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM2FLT0_SHIFT))

/*! @brief Set the FTM2FLT0 field to a new value. */
#define SIM_WR_SOPT4_FTM2FLT0(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM2FLT0_MASK, SIM_SOPT4_FTM2FLT0(value)))
#define SIM_BWR_SOPT4_FTM2FLT0(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM2FLT0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM3FLT0[12] (RW)
 *
 * Selects the source of FTM3 fault 0. The pin source for fault 0 must be
 * configured for the FTM module fault function through the appropriate PORTx pin
 * control register.
 *
 * Values:
 * - 0b0 - FTM3_FLT0 pin
 * - 0b1 - CMP0 out
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM3FLT0 field. */
#define SIM_RD_SOPT4_FTM3FLT0(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM3FLT0_MASK) >> SIM_SOPT4_FTM3FLT0_SHIFT)
#define SIM_BRD_SOPT4_FTM3FLT0(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3FLT0_SHIFT))

/*! @brief Set the FTM3FLT0 field to a new value. */
#define SIM_WR_SOPT4_FTM3FLT0(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM3FLT0_MASK, SIM_SOPT4_FTM3FLT0(value)))
#define SIM_BWR_SOPT4_FTM3FLT0(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3FLT0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM1CH0SRC[19:18] (RW)
 *
 * Selects the source for FTM1 channel 0 input capture. When the FTM is not in
 * input capture mode, clear this field.
 *
 * Values:
 * - 0b00 - FTM1_CH0 signal
 * - 0b01 - CMP0 output
 * - 0b10 - CMP1 output
 * - 0b11 - USB start of frame pulse
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM1CH0SRC field. */
#define SIM_RD_SOPT4_FTM1CH0SRC(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM1CH0SRC_MASK) >> SIM_SOPT4_FTM1CH0SRC_SHIFT)
#define SIM_BRD_SOPT4_FTM1CH0SRC(base) (SIM_RD_SOPT4_FTM1CH0SRC(base))

/*! @brief Set the FTM1CH0SRC field to a new value. */
#define SIM_WR_SOPT4_FTM1CH0SRC(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM1CH0SRC_MASK, SIM_SOPT4_FTM1CH0SRC(value)))
#define SIM_BWR_SOPT4_FTM1CH0SRC(base, value) (SIM_WR_SOPT4_FTM1CH0SRC(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM2CH0SRC[21:20] (RW)
 *
 * Selects the source for FTM2 channel 0 input capture. When the FTM is not in
 * input capture mode, clear this field.
 *
 * Values:
 * - 0b00 - FTM2_CH0 signal
 * - 0b01 - CMP0 output
 * - 0b10 - CMP1 output
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM2CH0SRC field. */
#define SIM_RD_SOPT4_FTM2CH0SRC(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM2CH0SRC_MASK) >> SIM_SOPT4_FTM2CH0SRC_SHIFT)
#define SIM_BRD_SOPT4_FTM2CH0SRC(base) (SIM_RD_SOPT4_FTM2CH0SRC(base))

/*! @brief Set the FTM2CH0SRC field to a new value. */
#define SIM_WR_SOPT4_FTM2CH0SRC(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM2CH0SRC_MASK, SIM_SOPT4_FTM2CH0SRC(value)))
#define SIM_BWR_SOPT4_FTM2CH0SRC(base, value) (SIM_WR_SOPT4_FTM2CH0SRC(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM2CH1SRC[22] (RW)
 *
 * Values:
 * - 0b0 - FTM2_CH1 signal
 * - 0b1 - Exclusive OR of FTM2_CH1, FTM2_CH0 and FTM1_CH1.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM2CH1SRC field. */
#define SIM_RD_SOPT4_FTM2CH1SRC(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM2CH1SRC_MASK) >> SIM_SOPT4_FTM2CH1SRC_SHIFT)
#define SIM_BRD_SOPT4_FTM2CH1SRC(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM2CH1SRC_SHIFT))

/*! @brief Set the FTM2CH1SRC field to a new value. */
#define SIM_WR_SOPT4_FTM2CH1SRC(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM2CH1SRC_MASK, SIM_SOPT4_FTM2CH1SRC(value)))
#define SIM_BWR_SOPT4_FTM2CH1SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM2CH1SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM0CLKSEL[24] (RW)
 *
 * Selects the external pin used to drive the clock to the FTM0 module. The
 * selected pin must also be configured for the FTM external clock function through
 * the appropriate pin control register in the port control module.
 *
 * Values:
 * - 0b0 - FTM_CLK0 pin
 * - 0b1 - FTM_CLK1 pin
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM0CLKSEL field. */
#define SIM_RD_SOPT4_FTM0CLKSEL(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM0CLKSEL_MASK) >> SIM_SOPT4_FTM0CLKSEL_SHIFT)
#define SIM_BRD_SOPT4_FTM0CLKSEL(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0CLKSEL_SHIFT))

/*! @brief Set the FTM0CLKSEL field to a new value. */
#define SIM_WR_SOPT4_FTM0CLKSEL(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM0CLKSEL_MASK, SIM_SOPT4_FTM0CLKSEL(value)))
#define SIM_BWR_SOPT4_FTM0CLKSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0CLKSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM1CLKSEL[25] (RW)
 *
 * Selects the external pin used to drive the clock to the FTM1 module. The
 * selected pin must also be configured for the FTM external clock function through
 * the appropriate pin control register in the port control module.
 *
 * Values:
 * - 0b0 - FTM_CLK0 pin
 * - 0b1 - FTM_CLK1 pin
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM1CLKSEL field. */
#define SIM_RD_SOPT4_FTM1CLKSEL(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM1CLKSEL_MASK) >> SIM_SOPT4_FTM1CLKSEL_SHIFT)
#define SIM_BRD_SOPT4_FTM1CLKSEL(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM1CLKSEL_SHIFT))

/*! @brief Set the FTM1CLKSEL field to a new value. */
#define SIM_WR_SOPT4_FTM1CLKSEL(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM1CLKSEL_MASK, SIM_SOPT4_FTM1CLKSEL(value)))
#define SIM_BWR_SOPT4_FTM1CLKSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM1CLKSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM2CLKSEL[26] (RW)
 *
 * Selects the external pin used to drive the clock to the FTM2 module. The
 * selected pin must also be configured for the FTM2 module external clock function
 * through the appropriate pin control register in the port control module.
 *
 * Values:
 * - 0b0 - FTM2 external clock driven by FTM_CLK0 pin.
 * - 0b1 - FTM2 external clock driven by FTM_CLK1 pin.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM2CLKSEL field. */
#define SIM_RD_SOPT4_FTM2CLKSEL(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM2CLKSEL_MASK) >> SIM_SOPT4_FTM2CLKSEL_SHIFT)
#define SIM_BRD_SOPT4_FTM2CLKSEL(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM2CLKSEL_SHIFT))

/*! @brief Set the FTM2CLKSEL field to a new value. */
#define SIM_WR_SOPT4_FTM2CLKSEL(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM2CLKSEL_MASK, SIM_SOPT4_FTM2CLKSEL(value)))
#define SIM_BWR_SOPT4_FTM2CLKSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM2CLKSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM3CLKSEL[27] (RW)
 *
 * Selects the external pin used to drive the clock to the FTM3 module. The
 * selected pin must also be configured for the FTM3 module external clock function
 * through the appropriate pin control register in the port control module.
 *
 * Values:
 * - 0b0 - FTM3 external clock driven by FTM_CLK0 pin.
 * - 0b1 - FTM3 external clock driven by FTM_CLK1 pin.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM3CLKSEL field. */
#define SIM_RD_SOPT4_FTM3CLKSEL(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM3CLKSEL_MASK) >> SIM_SOPT4_FTM3CLKSEL_SHIFT)
#define SIM_BRD_SOPT4_FTM3CLKSEL(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3CLKSEL_SHIFT))

/*! @brief Set the FTM3CLKSEL field to a new value. */
#define SIM_WR_SOPT4_FTM3CLKSEL(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM3CLKSEL_MASK, SIM_SOPT4_FTM3CLKSEL(value)))
#define SIM_BWR_SOPT4_FTM3CLKSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3CLKSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM0TRG0SRC[28] (RW)
 *
 * Selects the source of FTM0 hardware trigger 0.
 *
 * Values:
 * - 0b0 - HSCMP0 output drives FTM0 hardware trigger 0
 * - 0b1 - FTM1 channel match drives FTM0 hardware trigger 0
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM0TRG0SRC field. */
#define SIM_RD_SOPT4_FTM0TRG0SRC(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM0TRG0SRC_MASK) >> SIM_SOPT4_FTM0TRG0SRC_SHIFT)
#define SIM_BRD_SOPT4_FTM0TRG0SRC(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0TRG0SRC_SHIFT))

/*! @brief Set the FTM0TRG0SRC field to a new value. */
#define SIM_WR_SOPT4_FTM0TRG0SRC(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM0TRG0SRC_MASK, SIM_SOPT4_FTM0TRG0SRC(value)))
#define SIM_BWR_SOPT4_FTM0TRG0SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0TRG0SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM0TRG1SRC[29] (RW)
 *
 * Selects the source of FTM0 hardware trigger 1.
 *
 * Values:
 * - 0b0 - PDB output trigger 1 drives FTM0 hardware trigger 1
 * - 0b1 - FTM2 channel match drives FTM0 hardware trigger 1
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM0TRG1SRC field. */
#define SIM_RD_SOPT4_FTM0TRG1SRC(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM0TRG1SRC_MASK) >> SIM_SOPT4_FTM0TRG1SRC_SHIFT)
#define SIM_BRD_SOPT4_FTM0TRG1SRC(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0TRG1SRC_SHIFT))

/*! @brief Set the FTM0TRG1SRC field to a new value. */
#define SIM_WR_SOPT4_FTM0TRG1SRC(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM0TRG1SRC_MASK, SIM_SOPT4_FTM0TRG1SRC(value)))
#define SIM_BWR_SOPT4_FTM0TRG1SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM0TRG1SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM3TRG0SRC[30] (RW)
 *
 * Selects the source of FTM3 hardware trigger 0.
 *
 * Values:
 * - 0b0 - Reserved
 * - 0b1 - FTM1 channel match drives FTM3 hardware trigger 0
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM3TRG0SRC field. */
#define SIM_RD_SOPT4_FTM3TRG0SRC(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM3TRG0SRC_MASK) >> SIM_SOPT4_FTM3TRG0SRC_SHIFT)
#define SIM_BRD_SOPT4_FTM3TRG0SRC(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3TRG0SRC_SHIFT))

/*! @brief Set the FTM3TRG0SRC field to a new value. */
#define SIM_WR_SOPT4_FTM3TRG0SRC(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM3TRG0SRC_MASK, SIM_SOPT4_FTM3TRG0SRC(value)))
#define SIM_BWR_SOPT4_FTM3TRG0SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3TRG0SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT4, field FTM3TRG1SRC[31] (RW)
 *
 * Selects the source of FTM3 hardware trigger 1.
 *
 * Values:
 * - 0b0 - Reserved
 * - 0b1 - FTM2 channel match drives FTM3 hardware trigger 1
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT4_FTM3TRG1SRC field. */
#define SIM_RD_SOPT4_FTM3TRG1SRC(base) ((SIM_SOPT4_REG(base) & SIM_SOPT4_FTM3TRG1SRC_MASK) >> SIM_SOPT4_FTM3TRG1SRC_SHIFT)
#define SIM_BRD_SOPT4_FTM3TRG1SRC(base) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3TRG1SRC_SHIFT))

/*! @brief Set the FTM3TRG1SRC field to a new value. */
#define SIM_WR_SOPT4_FTM3TRG1SRC(base, value) (SIM_RMW_SOPT4(base, SIM_SOPT4_FTM3TRG1SRC_MASK, SIM_SOPT4_FTM3TRG1SRC(value)))
#define SIM_BWR_SOPT4_FTM3TRG1SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT4_REG(base), SIM_SOPT4_FTM3TRG1SRC_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SOPT5 - System Options Register 5
 ******************************************************************************/

/*!
 * @brief SIM_SOPT5 - System Options Register 5 (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_SOPT5 register
 */
/*@{*/
#define SIM_RD_SOPT5(base)       (SIM_SOPT5_REG(base))
#define SIM_WR_SOPT5(base, value) (SIM_SOPT5_REG(base) = (value))
#define SIM_RMW_SOPT5(base, mask, value) (SIM_WR_SOPT5(base, (SIM_RD_SOPT5(base) & ~(mask)) | (value)))
#define SIM_SET_SOPT5(base, value) (SIM_WR_SOPT5(base, SIM_RD_SOPT5(base) |  (value)))
#define SIM_CLR_SOPT5(base, value) (SIM_WR_SOPT5(base, SIM_RD_SOPT5(base) & ~(value)))
#define SIM_TOG_SOPT5(base, value) (SIM_WR_SOPT5(base, SIM_RD_SOPT5(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SOPT5 bitfields
 */

/*!
 * @name Register SIM_SOPT5, field UART0TXSRC[1:0] (RW)
 *
 * Selects the source for the UART 0 transmit data.
 *
 * Values:
 * - 0b00 - UART0_TX pin
 * - 0b01 - UART0_TX pin modulated with FTM1 channel 0 output
 * - 0b10 - UART0_TX pin modulated with FTM2 channel 0 output
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT5_UART0TXSRC field. */
#define SIM_RD_SOPT5_UART0TXSRC(base) ((SIM_SOPT5_REG(base) & SIM_SOPT5_UART0TXSRC_MASK) >> SIM_SOPT5_UART0TXSRC_SHIFT)
#define SIM_BRD_SOPT5_UART0TXSRC(base) (SIM_RD_SOPT5_UART0TXSRC(base))

/*! @brief Set the UART0TXSRC field to a new value. */
#define SIM_WR_SOPT5_UART0TXSRC(base, value) (SIM_RMW_SOPT5(base, SIM_SOPT5_UART0TXSRC_MASK, SIM_SOPT5_UART0TXSRC(value)))
#define SIM_BWR_SOPT5_UART0TXSRC(base, value) (SIM_WR_SOPT5_UART0TXSRC(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT5, field UART0RXSRC[3:2] (RW)
 *
 * Selects the source for the UART 0 receive data.
 *
 * Values:
 * - 0b00 - UART0_RX pin
 * - 0b01 - CMP0
 * - 0b10 - CMP1
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT5_UART0RXSRC field. */
#define SIM_RD_SOPT5_UART0RXSRC(base) ((SIM_SOPT5_REG(base) & SIM_SOPT5_UART0RXSRC_MASK) >> SIM_SOPT5_UART0RXSRC_SHIFT)
#define SIM_BRD_SOPT5_UART0RXSRC(base) (SIM_RD_SOPT5_UART0RXSRC(base))

/*! @brief Set the UART0RXSRC field to a new value. */
#define SIM_WR_SOPT5_UART0RXSRC(base, value) (SIM_RMW_SOPT5(base, SIM_SOPT5_UART0RXSRC_MASK, SIM_SOPT5_UART0RXSRC(value)))
#define SIM_BWR_SOPT5_UART0RXSRC(base, value) (SIM_WR_SOPT5_UART0RXSRC(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT5, field UART1TXSRC[5:4] (RW)
 *
 * Selects the source for the UART 1 transmit data.
 *
 * Values:
 * - 0b00 - UART1_TX pin
 * - 0b01 - UART1_TX pin modulated with FTM1 channel 0 output
 * - 0b10 - UART1_TX pin modulated with FTM2 channel 0 output
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT5_UART1TXSRC field. */
#define SIM_RD_SOPT5_UART1TXSRC(base) ((SIM_SOPT5_REG(base) & SIM_SOPT5_UART1TXSRC_MASK) >> SIM_SOPT5_UART1TXSRC_SHIFT)
#define SIM_BRD_SOPT5_UART1TXSRC(base) (SIM_RD_SOPT5_UART1TXSRC(base))

/*! @brief Set the UART1TXSRC field to a new value. */
#define SIM_WR_SOPT5_UART1TXSRC(base, value) (SIM_RMW_SOPT5(base, SIM_SOPT5_UART1TXSRC_MASK, SIM_SOPT5_UART1TXSRC(value)))
#define SIM_BWR_SOPT5_UART1TXSRC(base, value) (SIM_WR_SOPT5_UART1TXSRC(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT5, field UART1RXSRC[7:6] (RW)
 *
 * Selects the source for the UART 1 receive data.
 *
 * Values:
 * - 0b00 - UART1_RX pin
 * - 0b01 - CMP0
 * - 0b10 - CMP1
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT5_UART1RXSRC field. */
#define SIM_RD_SOPT5_UART1RXSRC(base) ((SIM_SOPT5_REG(base) & SIM_SOPT5_UART1RXSRC_MASK) >> SIM_SOPT5_UART1RXSRC_SHIFT)
#define SIM_BRD_SOPT5_UART1RXSRC(base) (SIM_RD_SOPT5_UART1RXSRC(base))

/*! @brief Set the UART1RXSRC field to a new value. */
#define SIM_WR_SOPT5_UART1RXSRC(base, value) (SIM_RMW_SOPT5(base, SIM_SOPT5_UART1RXSRC_MASK, SIM_SOPT5_UART1RXSRC(value)))
#define SIM_BWR_SOPT5_UART1RXSRC(base, value) (SIM_WR_SOPT5_UART1RXSRC(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT5, field LPUART0RXSRC[19:18] (RW)
 *
 * Selects the source for the LPUART0 receive data.
 *
 * Values:
 * - 0b00 - LPUART0_RX pin
 * - 0b01 - CMP0 output
 * - 0b10 - CMP1 output
 * - 0b11 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT5_LPUART0RXSRC field. */
#define SIM_RD_SOPT5_LPUART0RXSRC(base) ((SIM_SOPT5_REG(base) & SIM_SOPT5_LPUART0RXSRC_MASK) >> SIM_SOPT5_LPUART0RXSRC_SHIFT)
#define SIM_BRD_SOPT5_LPUART0RXSRC(base) (SIM_RD_SOPT5_LPUART0RXSRC(base))

/*! @brief Set the LPUART0RXSRC field to a new value. */
#define SIM_WR_SOPT5_LPUART0RXSRC(base, value) (SIM_RMW_SOPT5(base, SIM_SOPT5_LPUART0RXSRC_MASK, SIM_SOPT5_LPUART0RXSRC(value)))
#define SIM_BWR_SOPT5_LPUART0RXSRC(base, value) (SIM_WR_SOPT5_LPUART0RXSRC(base, value))
/*@}*/

/*******************************************************************************
 * SIM_SOPT7 - System Options Register 7
 ******************************************************************************/

/*!
 * @brief SIM_SOPT7 - System Options Register 7 (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_SOPT7 register
 */
/*@{*/
#define SIM_RD_SOPT7(base)       (SIM_SOPT7_REG(base))
#define SIM_WR_SOPT7(base, value) (SIM_SOPT7_REG(base) = (value))
#define SIM_RMW_SOPT7(base, mask, value) (SIM_WR_SOPT7(base, (SIM_RD_SOPT7(base) & ~(mask)) | (value)))
#define SIM_SET_SOPT7(base, value) (SIM_WR_SOPT7(base, SIM_RD_SOPT7(base) |  (value)))
#define SIM_CLR_SOPT7(base, value) (SIM_WR_SOPT7(base, SIM_RD_SOPT7(base) & ~(value)))
#define SIM_TOG_SOPT7(base, value) (SIM_WR_SOPT7(base, SIM_RD_SOPT7(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SOPT7 bitfields
 */

/*!
 * @name Register SIM_SOPT7, field ADC0TRGSEL[3:0] (RW)
 *
 * Selects the ADC0 trigger source when alternative triggers are functional in
 * stop and VLPS modes. .
 *
 * Values:
 * - 0b0000 - PDB external trigger pin input (PDB0_EXTRG)
 * - 0b0001 - High speed comparator 0 output
 * - 0b0010 - High speed comparator 1 output
 * - 0b0011 - Reserved
 * - 0b0100 - PIT trigger 0
 * - 0b0101 - PIT trigger 1
 * - 0b0110 - PIT trigger 2
 * - 0b0111 - PIT trigger 3
 * - 0b1000 - FTM0 trigger
 * - 0b1001 - FTM1 trigger
 * - 0b1010 - FTM2 trigger
 * - 0b1011 - FTM3 trigger
 * - 0b1100 - RTC alarm
 * - 0b1101 - RTC seconds
 * - 0b1110 - Low-power timer (LPTMR) trigger
 * - 0b1111 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT7_ADC0TRGSEL field. */
#define SIM_RD_SOPT7_ADC0TRGSEL(base) ((SIM_SOPT7_REG(base) & SIM_SOPT7_ADC0TRGSEL_MASK) >> SIM_SOPT7_ADC0TRGSEL_SHIFT)
#define SIM_BRD_SOPT7_ADC0TRGSEL(base) (SIM_RD_SOPT7_ADC0TRGSEL(base))

/*! @brief Set the ADC0TRGSEL field to a new value. */
#define SIM_WR_SOPT7_ADC0TRGSEL(base, value) (SIM_RMW_SOPT7(base, SIM_SOPT7_ADC0TRGSEL_MASK, SIM_SOPT7_ADC0TRGSEL(value)))
#define SIM_BWR_SOPT7_ADC0TRGSEL(base, value) (SIM_WR_SOPT7_ADC0TRGSEL(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT7, field ADC0PRETRGSEL[4] (RW)
 *
 * Selects the ADC0 pre-trigger source when alternative triggers are enabled
 * through ADC0ALTTRGEN.
 *
 * Values:
 * - 0b0 - Pre-trigger A
 * - 0b1 - Pre-trigger B
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT7_ADC0PRETRGSEL field. */
#define SIM_RD_SOPT7_ADC0PRETRGSEL(base) ((SIM_SOPT7_REG(base) & SIM_SOPT7_ADC0PRETRGSEL_MASK) >> SIM_SOPT7_ADC0PRETRGSEL_SHIFT)
#define SIM_BRD_SOPT7_ADC0PRETRGSEL(base) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC0PRETRGSEL_SHIFT))

/*! @brief Set the ADC0PRETRGSEL field to a new value. */
#define SIM_WR_SOPT7_ADC0PRETRGSEL(base, value) (SIM_RMW_SOPT7(base, SIM_SOPT7_ADC0PRETRGSEL_MASK, SIM_SOPT7_ADC0PRETRGSEL(value)))
#define SIM_BWR_SOPT7_ADC0PRETRGSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC0PRETRGSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT7, field ADC0ALTTRGEN[7] (RW)
 *
 * Enable alternative conversion triggers for ADC0.
 *
 * Values:
 * - 0b0 - PDB trigger selected for ADC0.
 * - 0b1 - Alternate trigger selected for ADC0.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT7_ADC0ALTTRGEN field. */
#define SIM_RD_SOPT7_ADC0ALTTRGEN(base) ((SIM_SOPT7_REG(base) & SIM_SOPT7_ADC0ALTTRGEN_MASK) >> SIM_SOPT7_ADC0ALTTRGEN_SHIFT)
#define SIM_BRD_SOPT7_ADC0ALTTRGEN(base) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC0ALTTRGEN_SHIFT))

/*! @brief Set the ADC0ALTTRGEN field to a new value. */
#define SIM_WR_SOPT7_ADC0ALTTRGEN(base, value) (SIM_RMW_SOPT7(base, SIM_SOPT7_ADC0ALTTRGEN_MASK, SIM_SOPT7_ADC0ALTTRGEN(value)))
#define SIM_BWR_SOPT7_ADC0ALTTRGEN(base, value) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC0ALTTRGEN_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT7, field ADC1TRGSEL[11:8] (RW)
 *
 * Selects the ADC1 trigger source when alternative triggers are functional in
 * stop and VLPS modes.
 *
 * Values:
 * - 0b0000 - PDB external trigger pin input (PDB0_EXTRG)
 * - 0b0001 - High speed comparator 0 output
 * - 0b0010 - High speed comparator 1 output
 * - 0b0011 - Reserved
 * - 0b0100 - PIT trigger 0
 * - 0b0101 - PIT trigger 1
 * - 0b0110 - PIT trigger 2
 * - 0b0111 - PIT trigger 3
 * - 0b1000 - FTM0 trigger
 * - 0b1001 - FTM1 trigger
 * - 0b1010 - FTM2 trigger
 * - 0b1011 - FTM3 trigger
 * - 0b1100 - RTC alarm
 * - 0b1101 - RTC seconds
 * - 0b1110 - Low-power timer (LPTMR) trigger
 * - 0b1111 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT7_ADC1TRGSEL field. */
#define SIM_RD_SOPT7_ADC1TRGSEL(base) ((SIM_SOPT7_REG(base) & SIM_SOPT7_ADC1TRGSEL_MASK) >> SIM_SOPT7_ADC1TRGSEL_SHIFT)
#define SIM_BRD_SOPT7_ADC1TRGSEL(base) (SIM_RD_SOPT7_ADC1TRGSEL(base))

/*! @brief Set the ADC1TRGSEL field to a new value. */
#define SIM_WR_SOPT7_ADC1TRGSEL(base, value) (SIM_RMW_SOPT7(base, SIM_SOPT7_ADC1TRGSEL_MASK, SIM_SOPT7_ADC1TRGSEL(value)))
#define SIM_BWR_SOPT7_ADC1TRGSEL(base, value) (SIM_WR_SOPT7_ADC1TRGSEL(base, value))
/*@}*/

/*!
 * @name Register SIM_SOPT7, field ADC1PRETRGSEL[12] (RW)
 *
 * Selects the ADC1 pre-trigger source when alternative triggers are enabled
 * through ADC1ALTTRGEN.
 *
 * Values:
 * - 0b0 - Pre-trigger A selected for ADC1.
 * - 0b1 - Pre-trigger B selected for ADC1.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT7_ADC1PRETRGSEL field. */
#define SIM_RD_SOPT7_ADC1PRETRGSEL(base) ((SIM_SOPT7_REG(base) & SIM_SOPT7_ADC1PRETRGSEL_MASK) >> SIM_SOPT7_ADC1PRETRGSEL_SHIFT)
#define SIM_BRD_SOPT7_ADC1PRETRGSEL(base) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC1PRETRGSEL_SHIFT))

/*! @brief Set the ADC1PRETRGSEL field to a new value. */
#define SIM_WR_SOPT7_ADC1PRETRGSEL(base, value) (SIM_RMW_SOPT7(base, SIM_SOPT7_ADC1PRETRGSEL_MASK, SIM_SOPT7_ADC1PRETRGSEL(value)))
#define SIM_BWR_SOPT7_ADC1PRETRGSEL(base, value) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC1PRETRGSEL_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT7, field ADC1ALTTRGEN[15] (RW)
 *
 * Enable alternative conversion triggers for ADC1.
 *
 * Values:
 * - 0b0 - PDB trigger selected for ADC1
 * - 0b1 - Alternate trigger selected for ADC1 as defined by ADC1TRGSEL.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT7_ADC1ALTTRGEN field. */
#define SIM_RD_SOPT7_ADC1ALTTRGEN(base) ((SIM_SOPT7_REG(base) & SIM_SOPT7_ADC1ALTTRGEN_MASK) >> SIM_SOPT7_ADC1ALTTRGEN_SHIFT)
#define SIM_BRD_SOPT7_ADC1ALTTRGEN(base) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC1ALTTRGEN_SHIFT))

/*! @brief Set the ADC1ALTTRGEN field to a new value. */
#define SIM_WR_SOPT7_ADC1ALTTRGEN(base, value) (SIM_RMW_SOPT7(base, SIM_SOPT7_ADC1ALTTRGEN_MASK, SIM_SOPT7_ADC1ALTTRGEN(value)))
#define SIM_BWR_SOPT7_ADC1ALTTRGEN(base, value) (BITBAND_ACCESS32(&SIM_SOPT7_REG(base), SIM_SOPT7_ADC1ALTTRGEN_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SOPT8 - System Options Register 8
 ******************************************************************************/

/*!
 * @brief SIM_SOPT8 - System Options Register 8 (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_SOPT8 register
 */
/*@{*/
#define SIM_RD_SOPT8(base)       (SIM_SOPT8_REG(base))
#define SIM_WR_SOPT8(base, value) (SIM_SOPT8_REG(base) = (value))
#define SIM_RMW_SOPT8(base, mask, value) (SIM_WR_SOPT8(base, (SIM_RD_SOPT8(base) & ~(mask)) | (value)))
#define SIM_SET_SOPT8(base, value) (SIM_WR_SOPT8(base, SIM_RD_SOPT8(base) |  (value)))
#define SIM_CLR_SOPT8(base, value) (SIM_WR_SOPT8(base, SIM_RD_SOPT8(base) & ~(value)))
#define SIM_TOG_SOPT8(base, value) (SIM_WR_SOPT8(base, SIM_RD_SOPT8(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SOPT8 bitfields
 */

/*!
 * @name Register SIM_SOPT8, field FTM0SYNCBIT[0] (RW)
 *
 * Values:
 * - 0b0 - No effect
 * - 0b1 - Write 1 to assert the TRIG0 input to FTM0, software must clear this
 *     bit to allow other trigger sources to assert.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0SYNCBIT field. */
#define SIM_RD_SOPT8_FTM0SYNCBIT(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0SYNCBIT_MASK) >> SIM_SOPT8_FTM0SYNCBIT_SHIFT)
#define SIM_BRD_SOPT8_FTM0SYNCBIT(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0SYNCBIT_SHIFT))

/*! @brief Set the FTM0SYNCBIT field to a new value. */
#define SIM_WR_SOPT8_FTM0SYNCBIT(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0SYNCBIT_MASK, SIM_SOPT8_FTM0SYNCBIT(value)))
#define SIM_BWR_SOPT8_FTM0SYNCBIT(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0SYNCBIT_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM1SYNCBIT[1] (RW)
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - Write 1 to assert the TRIG0 input to FTM1, software must clear this
 *     bit to allow other trigger sources to assert.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM1SYNCBIT field. */
#define SIM_RD_SOPT8_FTM1SYNCBIT(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM1SYNCBIT_MASK) >> SIM_SOPT8_FTM1SYNCBIT_SHIFT)
#define SIM_BRD_SOPT8_FTM1SYNCBIT(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM1SYNCBIT_SHIFT))

/*! @brief Set the FTM1SYNCBIT field to a new value. */
#define SIM_WR_SOPT8_FTM1SYNCBIT(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM1SYNCBIT_MASK, SIM_SOPT8_FTM1SYNCBIT(value)))
#define SIM_BWR_SOPT8_FTM1SYNCBIT(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM1SYNCBIT_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM2SYNCBIT[2] (RW)
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - Write 1 to assert the TRIG0 input to FTM2, software must clear this
 *     bit to allow other trigger sources to assert.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM2SYNCBIT field. */
#define SIM_RD_SOPT8_FTM2SYNCBIT(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM2SYNCBIT_MASK) >> SIM_SOPT8_FTM2SYNCBIT_SHIFT)
#define SIM_BRD_SOPT8_FTM2SYNCBIT(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM2SYNCBIT_SHIFT))

/*! @brief Set the FTM2SYNCBIT field to a new value. */
#define SIM_WR_SOPT8_FTM2SYNCBIT(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM2SYNCBIT_MASK, SIM_SOPT8_FTM2SYNCBIT(value)))
#define SIM_BWR_SOPT8_FTM2SYNCBIT(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM2SYNCBIT_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3SYNCBIT[3] (RW)
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - Write 1 to assert the TRIG0 input to FTM3, software must clear this
 *     bit to allow other trigger sources to assert.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3SYNCBIT field. */
#define SIM_RD_SOPT8_FTM3SYNCBIT(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3SYNCBIT_MASK) >> SIM_SOPT8_FTM3SYNCBIT_SHIFT)
#define SIM_BRD_SOPT8_FTM3SYNCBIT(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3SYNCBIT_SHIFT))

/*! @brief Set the FTM3SYNCBIT field to a new value. */
#define SIM_WR_SOPT8_FTM3SYNCBIT(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3SYNCBIT_MASK, SIM_SOPT8_FTM3SYNCBIT(value)))
#define SIM_BWR_SOPT8_FTM3SYNCBIT(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3SYNCBIT_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH0SRC[16] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH0 pin is output of FTM0 channel 0 output
 * - 0b1 - FTM0_CH0 pin is output of FTM0 channel 0 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH0SRC field. */
#define SIM_RD_SOPT8_FTM0OCH0SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH0SRC_MASK) >> SIM_SOPT8_FTM0OCH0SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH0SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH0SRC_SHIFT))

/*! @brief Set the FTM0OCH0SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH0SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH0SRC_MASK, SIM_SOPT8_FTM0OCH0SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH0SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH0SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH1SRC[17] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH1 pin is output of FTM0 channel 1 output
 * - 0b1 - FTM0_CH1 pin is output of FTM0 channel 1 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH1SRC field. */
#define SIM_RD_SOPT8_FTM0OCH1SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH1SRC_MASK) >> SIM_SOPT8_FTM0OCH1SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH1SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH1SRC_SHIFT))

/*! @brief Set the FTM0OCH1SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH1SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH1SRC_MASK, SIM_SOPT8_FTM0OCH1SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH1SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH1SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH2SRC[18] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH2 pin is output of FTM0 channel 2 output
 * - 0b1 - FTM0_CH2 pin is output of FTM0 channel 2 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH2SRC field. */
#define SIM_RD_SOPT8_FTM0OCH2SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH2SRC_MASK) >> SIM_SOPT8_FTM0OCH2SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH2SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH2SRC_SHIFT))

/*! @brief Set the FTM0OCH2SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH2SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH2SRC_MASK, SIM_SOPT8_FTM0OCH2SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH2SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH2SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH3SRC[19] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH3 pin is output of FTM0 channel 3 output
 * - 0b1 - FTM0_CH3 pin is output of FTM0 channel 3 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH3SRC field. */
#define SIM_RD_SOPT8_FTM0OCH3SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH3SRC_MASK) >> SIM_SOPT8_FTM0OCH3SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH3SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH3SRC_SHIFT))

/*! @brief Set the FTM0OCH3SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH3SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH3SRC_MASK, SIM_SOPT8_FTM0OCH3SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH3SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH3SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH4SRC[20] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH4 pin is output of FTM0 channel 4 output
 * - 0b1 - FTM0_CH4 pin is output of FTM0 channel 4 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH4SRC field. */
#define SIM_RD_SOPT8_FTM0OCH4SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH4SRC_MASK) >> SIM_SOPT8_FTM0OCH4SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH4SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH4SRC_SHIFT))

/*! @brief Set the FTM0OCH4SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH4SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH4SRC_MASK, SIM_SOPT8_FTM0OCH4SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH4SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH4SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH5SRC[21] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH5 pin is output of FTM0 channel 5 output
 * - 0b1 - FTM0_CH5 pin is output of FTM0 channel 5 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH5SRC field. */
#define SIM_RD_SOPT8_FTM0OCH5SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH5SRC_MASK) >> SIM_SOPT8_FTM0OCH5SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH5SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH5SRC_SHIFT))

/*! @brief Set the FTM0OCH5SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH5SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH5SRC_MASK, SIM_SOPT8_FTM0OCH5SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH5SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH5SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH6SRC[22] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH6 pin is output of FTM0 channel 6 output
 * - 0b1 - FTM0_CH6 pin is output of FTM0 channel 6 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH6SRC field. */
#define SIM_RD_SOPT8_FTM0OCH6SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH6SRC_MASK) >> SIM_SOPT8_FTM0OCH6SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH6SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH6SRC_SHIFT))

/*! @brief Set the FTM0OCH6SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH6SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH6SRC_MASK, SIM_SOPT8_FTM0OCH6SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH6SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH6SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM0OCH7SRC[23] (RW)
 *
 * Values:
 * - 0b0 - FTM0_CH7 pin is output of FTM0 channel 7 output
 * - 0b1 - FTM0_CH7 pin is output of FTM0 channel 7 output, modulated by FTM1
 *     channel 1 output
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM0OCH7SRC field. */
#define SIM_RD_SOPT8_FTM0OCH7SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM0OCH7SRC_MASK) >> SIM_SOPT8_FTM0OCH7SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM0OCH7SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH7SRC_SHIFT))

/*! @brief Set the FTM0OCH7SRC field to a new value. */
#define SIM_WR_SOPT8_FTM0OCH7SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM0OCH7SRC_MASK, SIM_SOPT8_FTM0OCH7SRC(value)))
#define SIM_BWR_SOPT8_FTM0OCH7SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM0OCH7SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH0SRC[24] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH0 pin is output of FTM3 channel 0 output
 * - 0b1 - FTM3_CH0 pin is output of FTM3 channel 0 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH0SRC field. */
#define SIM_RD_SOPT8_FTM3OCH0SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH0SRC_MASK) >> SIM_SOPT8_FTM3OCH0SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH0SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH0SRC_SHIFT))

/*! @brief Set the FTM3OCH0SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH0SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH0SRC_MASK, SIM_SOPT8_FTM3OCH0SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH0SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH0SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH1SRC[25] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH1 pin is output of FTM3 channel 1 output
 * - 0b1 - FTM3_CH1 pin is output of FTM3 channel 1 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH1SRC field. */
#define SIM_RD_SOPT8_FTM3OCH1SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH1SRC_MASK) >> SIM_SOPT8_FTM3OCH1SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH1SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH1SRC_SHIFT))

/*! @brief Set the FTM3OCH1SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH1SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH1SRC_MASK, SIM_SOPT8_FTM3OCH1SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH1SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH1SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH2SRC[26] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH2 pin is output of FTM3 channel 2 output
 * - 0b1 - FTM3_CH2 pin is output of FTM3 channel 2 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH2SRC field. */
#define SIM_RD_SOPT8_FTM3OCH2SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH2SRC_MASK) >> SIM_SOPT8_FTM3OCH2SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH2SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH2SRC_SHIFT))

/*! @brief Set the FTM3OCH2SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH2SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH2SRC_MASK, SIM_SOPT8_FTM3OCH2SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH2SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH2SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH3SRC[27] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH3 pin is output of FTM3 channel 3 output
 * - 0b1 - FTM3_CH3 pin is output of FTM3 channel 3 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH3SRC field. */
#define SIM_RD_SOPT8_FTM3OCH3SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH3SRC_MASK) >> SIM_SOPT8_FTM3OCH3SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH3SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH3SRC_SHIFT))

/*! @brief Set the FTM3OCH3SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH3SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH3SRC_MASK, SIM_SOPT8_FTM3OCH3SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH3SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH3SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH4SRC[28] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH4 pin is output of FTM3 channel 4 output
 * - 0b1 - FTM3_CH4 pin is output of FTM3 channel 4 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH4SRC field. */
#define SIM_RD_SOPT8_FTM3OCH4SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH4SRC_MASK) >> SIM_SOPT8_FTM3OCH4SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH4SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH4SRC_SHIFT))

/*! @brief Set the FTM3OCH4SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH4SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH4SRC_MASK, SIM_SOPT8_FTM3OCH4SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH4SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH4SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH5SRC[29] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH5 pin is output of FTM3 channel 5 output
 * - 0b1 - FTM3_CH5 pin is output of FTM3 channel 5 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH5SRC field. */
#define SIM_RD_SOPT8_FTM3OCH5SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH5SRC_MASK) >> SIM_SOPT8_FTM3OCH5SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH5SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH5SRC_SHIFT))

/*! @brief Set the FTM3OCH5SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH5SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH5SRC_MASK, SIM_SOPT8_FTM3OCH5SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH5SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH5SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH6SRC[30] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH6 pin is output of FTM3 channel 6 output
 * - 0b1 - FTM3_CH6 pin is output of FTM3 channel 6 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH6SRC field. */
#define SIM_RD_SOPT8_FTM3OCH6SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH6SRC_MASK) >> SIM_SOPT8_FTM3OCH6SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH6SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH6SRC_SHIFT))

/*! @brief Set the FTM3OCH6SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH6SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH6SRC_MASK, SIM_SOPT8_FTM3OCH6SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH6SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH6SRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SOPT8, field FTM3OCH7SRC[31] (RW)
 *
 * Values:
 * - 0b0 - FTM3_CH7 pin is output of FTM3 channel 7 output
 * - 0b1 - FTM3_CH7 pin is output of FTM3 channel 7 output modulated by FTM2
 *     channel 1 output.
 */
/*@{*/
/*! @brief Read current value of the SIM_SOPT8_FTM3OCH7SRC field. */
#define SIM_RD_SOPT8_FTM3OCH7SRC(base) ((SIM_SOPT8_REG(base) & SIM_SOPT8_FTM3OCH7SRC_MASK) >> SIM_SOPT8_FTM3OCH7SRC_SHIFT)
#define SIM_BRD_SOPT8_FTM3OCH7SRC(base) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH7SRC_SHIFT))

/*! @brief Set the FTM3OCH7SRC field to a new value. */
#define SIM_WR_SOPT8_FTM3OCH7SRC(base, value) (SIM_RMW_SOPT8(base, SIM_SOPT8_FTM3OCH7SRC_MASK, SIM_SOPT8_FTM3OCH7SRC(value)))
#define SIM_BWR_SOPT8_FTM3OCH7SRC(base, value) (BITBAND_ACCESS32(&SIM_SOPT8_REG(base), SIM_SOPT8_FTM3OCH7SRC_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SDID - System Device Identification Register
 ******************************************************************************/

/*!
 * @brief SIM_SDID - System Device Identification Register (RO)
 *
 * Reset value: 0x00000E80U
 */
/*!
 * @name Constants and macros for entire SIM_SDID register
 */
/*@{*/
#define SIM_RD_SDID(base)        (SIM_SDID_REG(base))
/*@}*/

/*
 * Constants & macros for individual SIM_SDID bitfields
 */

/*!
 * @name Register SIM_SDID, field PINID[3:0] (RO)
 *
 * Specifies the pincount of the device.
 *
 * Values:
 * - 0b0000 - Reserved
 * - 0b0001 - Reserved
 * - 0b0010 - 32-pin
 * - 0b0011 - Reserved
 * - 0b0100 - 48-pin
 * - 0b0101 - 64-pin
 * - 0b0110 - 80-pin
 * - 0b0111 - 81-pin or 121-pin
 * - 0b1000 - 100-pin
 * - 0b1001 - 121-pin
 * - 0b1010 - 144-pin
 * - 0b1011 - Custom pinout (WLCSP)
 * - 0b1100 - 169-pin
 * - 0b1101 - Reserved
 * - 0b1110 - 256-pin
 * - 0b1111 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SDID_PINID field. */
#define SIM_RD_SDID_PINID(base) ((SIM_SDID_REG(base) & SIM_SDID_PINID_MASK) >> SIM_SDID_PINID_SHIFT)
#define SIM_BRD_SDID_PINID(base) (SIM_RD_SDID_PINID(base))
/*@}*/

/*!
 * @name Register SIM_SDID, field FAMID[6:4] (RO)
 *
 * This field is maintained for compatibility only, but has been superceded by
 * the SERIESID, FAMILYID and SUBFAMID fields in this register.
 *
 * Values:
 * - 0b000 - K1x Family (without tamper)
 * - 0b001 - K2x Family (without tamper)
 * - 0b010 - K3x Family or K1x/K6x Family (with tamper)
 * - 0b011 - K4x Family or K2x Family (with tamper)
 * - 0b100 - K6x Family (without tamper)
 * - 0b101 - K7x Family
 * - 0b110 - Reserved
 * - 0b111 - Reserved
 */
/*@{*/
/*! @brief Read current value of the SIM_SDID_FAMID field. */
#define SIM_RD_SDID_FAMID(base) ((SIM_SDID_REG(base) & SIM_SDID_FAMID_MASK) >> SIM_SDID_FAMID_SHIFT)
#define SIM_BRD_SDID_FAMID(base) (SIM_RD_SDID_FAMID(base))
/*@}*/

/*!
 * @name Register SIM_SDID, field DIEID[11:7] (RO)
 *
 * Specifies the silicon feature set identication number for the device.
 */
/*@{*/
/*! @brief Read current value of the SIM_SDID_DIEID field. */
#define SIM_RD_SDID_DIEID(base) ((SIM_SDID_REG(base) & SIM_SDID_DIEID_MASK) >> SIM_SDID_DIEID_SHIFT)
#define SIM_BRD_SDID_DIEID(base) (SIM_RD_SDID_DIEID(base))
/*@}*/

/*!
 * @name Register SIM_SDID, field REVID[15:12] (RO)
 *
 * Specifies the silicon implementation number for the device.
 */
/*@{*/
/*! @brief Read current value of the SIM_SDID_REVID field. */
#define SIM_RD_SDID_REVID(base) ((SIM_SDID_REG(base) & SIM_SDID_REVID_MASK) >> SIM_SDID_REVID_SHIFT)
#define SIM_BRD_SDID_REVID(base) (SIM_RD_SDID_REVID(base))
/*@}*/

/*!
 * @name Register SIM_SDID, field SERIESID[23:20] (RO)
 *
 * Specifies the Kinetis series of the device.
 *
 * Values:
 * - 0b0000 - Kinetis K series
 * - 0b0001 - Kinetis L series
 * - 0b0101 - Kinetis W series
 * - 0b0110 - Kinetis V series
 */
/*@{*/
/*! @brief Read current value of the SIM_SDID_SERIESID field. */
#define SIM_RD_SDID_SERIESID(base) ((SIM_SDID_REG(base) & SIM_SDID_SERIESID_MASK) >> SIM_SDID_SERIESID_SHIFT)
#define SIM_BRD_SDID_SERIESID(base) (SIM_RD_SDID_SERIESID(base))
/*@}*/

/*!
 * @name Register SIM_SDID, field SUBFAMID[27:24] (RO)
 *
 * Specifies the Kinetis sub-family of the device.
 *
 * Values:
 * - 0b0000 - Kx0 Subfamily
 * - 0b0001 - Kx1 Subfamily (tamper detect)
 * - 0b0010 - Kx2 Subfamily
 * - 0b0011 - Kx3 Subfamily (tamper detect)
 * - 0b0100 - Kx4 Subfamily
 * - 0b0101 - Kx5 Subfamily (tamper detect)
 * - 0b0110 - Kx6 Subfamily
 */
/*@{*/
/*! @brief Read current value of the SIM_SDID_SUBFAMID field. */
#define SIM_RD_SDID_SUBFAMID(base) ((SIM_SDID_REG(base) & SIM_SDID_SUBFAMID_MASK) >> SIM_SDID_SUBFAMID_SHIFT)
#define SIM_BRD_SDID_SUBFAMID(base) (SIM_RD_SDID_SUBFAMID(base))
/*@}*/

/*!
 * @name Register SIM_SDID, field FAMILYID[31:28] (RO)
 *
 * Specifies the Kinetis family of the device.
 *
 * Values:
 * - 0b0001 - K1x Family
 * - 0b0010 - K2x Family
 * - 0b0011 - K3x Family
 * - 0b0100 - K4x Family
 * - 0b0110 - K6x Family
 * - 0b0111 - K7x Family
 */
/*@{*/
/*! @brief Read current value of the SIM_SDID_FAMILYID field. */
#define SIM_RD_SDID_FAMILYID(base) ((SIM_SDID_REG(base) & SIM_SDID_FAMILYID_MASK) >> SIM_SDID_FAMILYID_SHIFT)
#define SIM_BRD_SDID_FAMILYID(base) (SIM_RD_SDID_FAMILYID(base))
/*@}*/

/*******************************************************************************
 * SIM_SCGC4 - System Clock Gating Control Register 4
 ******************************************************************************/

/*!
 * @brief SIM_SCGC4 - System Clock Gating Control Register 4 (RW)
 *
 * Reset value: 0xF0100030U
 */
/*!
 * @name Constants and macros for entire SIM_SCGC4 register
 */
/*@{*/
#define SIM_RD_SCGC4(base)       (SIM_SCGC4_REG(base))
#define SIM_WR_SCGC4(base, value) (SIM_SCGC4_REG(base) = (value))
#define SIM_RMW_SCGC4(base, mask, value) (SIM_WR_SCGC4(base, (SIM_RD_SCGC4(base) & ~(mask)) | (value)))
#define SIM_SET_SCGC4(base, value) (SIM_WR_SCGC4(base, SIM_RD_SCGC4(base) |  (value)))
#define SIM_CLR_SCGC4(base, value) (SIM_WR_SCGC4(base, SIM_RD_SCGC4(base) & ~(value)))
#define SIM_TOG_SCGC4(base, value) (SIM_WR_SCGC4(base, SIM_RD_SCGC4(base) ^  (value)))
/*@}*/

/* Unified clock gate bit access macros */
#define SIM_SCGC_BIT_REG(base, index)        (*((volatile uint32_t *)&SIM_SCGC4_REG(base) + (((uint32_t)(index) >> 5) - 3U)))
#define SIM_SCGC_BIT_SHIFT(index)            ((uint32_t)(index) & ((1U << 5) - 1U))
#define SIM_RD_SCGC_BIT(base, index)         (SIM_SCGC_BIT_REG((base), (index)) & (1U << SIM_SCGC_BIT_SHIFT(index)))
#define SIM_BRD_SCGC_BIT(base, index)        (BITBAND_ACCESS32(&SIM_SCGC_BIT_REG((base), (index)), SIM_SCGC_BIT_SHIFT(index)))
#define SIM_WR_SCGC_BIT(base, index, value)  (SIM_SCGC_BIT_REG((base), (index)) = (SIM_SCGC_BIT_REG((base), (index)) & ~(1U << SIM_SCGC_BIT_SHIFT(index))) | ((uint32_t)(value) << SIM_SCGC_BIT_SHIFT(index)))
#define SIM_BWR_SCGC_BIT(base, index, value) (BITBAND_ACCESS32(&SIM_SCGC_BIT_REG((base), (index)), SIM_SCGC_BIT_SHIFT(index)) = (uint32_t)(value))

/*
 * Constants & macros for individual SIM_SCGC4 bitfields
 */

/*!
 * @name Register SIM_SCGC4, field EWM[1] (RW)
 *
 * This bit controls the clock gate to the EWM module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_EWM field. */
#define SIM_RD_SCGC4_EWM(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_EWM_MASK) >> SIM_SCGC4_EWM_SHIFT)
#define SIM_BRD_SCGC4_EWM(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_EWM_SHIFT))

/*! @brief Set the EWM field to a new value. */
#define SIM_WR_SCGC4_EWM(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_EWM_MASK, SIM_SCGC4_EWM(value)))
#define SIM_BWR_SCGC4_EWM(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_EWM_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field I2C0[6] (RW)
 *
 * This bit controls the clock gate to the I 2 C0 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_I2C0 field. */
#define SIM_RD_SCGC4_I2C0(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_I2C0_MASK) >> SIM_SCGC4_I2C0_SHIFT)
#define SIM_BRD_SCGC4_I2C0(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_I2C0_SHIFT))

/*! @brief Set the I2C0 field to a new value. */
#define SIM_WR_SCGC4_I2C0(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_I2C0_MASK, SIM_SCGC4_I2C0(value)))
#define SIM_BWR_SCGC4_I2C0(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_I2C0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field I2C1[7] (RW)
 *
 * This bit controls the clock gate to the I 2 C1 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_I2C1 field. */
#define SIM_RD_SCGC4_I2C1(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_I2C1_MASK) >> SIM_SCGC4_I2C1_SHIFT)
#define SIM_BRD_SCGC4_I2C1(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_I2C1_SHIFT))

/*! @brief Set the I2C1 field to a new value. */
#define SIM_WR_SCGC4_I2C1(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_I2C1_MASK, SIM_SCGC4_I2C1(value)))
#define SIM_BWR_SCGC4_I2C1(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_I2C1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field UART0[10] (RW)
 *
 * This bit controls the clock gate to the UART0 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_UART0 field. */
#define SIM_RD_SCGC4_UART0(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_UART0_MASK) >> SIM_SCGC4_UART0_SHIFT)
#define SIM_BRD_SCGC4_UART0(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_UART0_SHIFT))

/*! @brief Set the UART0 field to a new value. */
#define SIM_WR_SCGC4_UART0(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_UART0_MASK, SIM_SCGC4_UART0(value)))
#define SIM_BWR_SCGC4_UART0(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_UART0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field UART1[11] (RW)
 *
 * This bit controls the clock gate to the UART1 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_UART1 field. */
#define SIM_RD_SCGC4_UART1(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_UART1_MASK) >> SIM_SCGC4_UART1_SHIFT)
#define SIM_BRD_SCGC4_UART1(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_UART1_SHIFT))

/*! @brief Set the UART1 field to a new value. */
#define SIM_WR_SCGC4_UART1(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_UART1_MASK, SIM_SCGC4_UART1(value)))
#define SIM_BWR_SCGC4_UART1(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_UART1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field UART2[12] (RW)
 *
 * This bit controls the clock gate to the UART2 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_UART2 field. */
#define SIM_RD_SCGC4_UART2(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_UART2_MASK) >> SIM_SCGC4_UART2_SHIFT)
#define SIM_BRD_SCGC4_UART2(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_UART2_SHIFT))

/*! @brief Set the UART2 field to a new value. */
#define SIM_WR_SCGC4_UART2(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_UART2_MASK, SIM_SCGC4_UART2(value)))
#define SIM_BWR_SCGC4_UART2(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_UART2_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field USBOTG[18] (RW)
 *
 * This bit controls the clock gate to the USB module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_USBOTG field. */
#define SIM_RD_SCGC4_USBOTG(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_USBOTG_MASK) >> SIM_SCGC4_USBOTG_SHIFT)
#define SIM_BRD_SCGC4_USBOTG(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_USBOTG_SHIFT))

/*! @brief Set the USBOTG field to a new value. */
#define SIM_WR_SCGC4_USBOTG(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_USBOTG_MASK, SIM_SCGC4_USBOTG(value)))
#define SIM_BWR_SCGC4_USBOTG(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_USBOTG_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field CMP[19] (RW)
 *
 * This bit controls the clock gate to the comparator module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_CMP field. */
#define SIM_RD_SCGC4_CMP(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_CMP_MASK) >> SIM_SCGC4_CMP_SHIFT)
#define SIM_BRD_SCGC4_CMP(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_CMP_SHIFT))

/*! @brief Set the CMP field to a new value. */
#define SIM_WR_SCGC4_CMP(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_CMP_MASK, SIM_SCGC4_CMP(value)))
#define SIM_BWR_SCGC4_CMP(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_CMP_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC4, field VREF[20] (RW)
 *
 * This bit controls the clock gate to the VREF module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC4_VREF field. */
#define SIM_RD_SCGC4_VREF(base) ((SIM_SCGC4_REG(base) & SIM_SCGC4_VREF_MASK) >> SIM_SCGC4_VREF_SHIFT)
#define SIM_BRD_SCGC4_VREF(base) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_VREF_SHIFT))

/*! @brief Set the VREF field to a new value. */
#define SIM_WR_SCGC4_VREF(base, value) (SIM_RMW_SCGC4(base, SIM_SCGC4_VREF_MASK, SIM_SCGC4_VREF(value)))
#define SIM_BWR_SCGC4_VREF(base, value) (BITBAND_ACCESS32(&SIM_SCGC4_REG(base), SIM_SCGC4_VREF_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SCGC5 - System Clock Gating Control Register 5
 ******************************************************************************/

/*!
 * @brief SIM_SCGC5 - System Clock Gating Control Register 5 (RW)
 *
 * Reset value: 0x00040182U
 */
/*!
 * @name Constants and macros for entire SIM_SCGC5 register
 */
/*@{*/
#define SIM_RD_SCGC5(base)       (SIM_SCGC5_REG(base))
#define SIM_WR_SCGC5(base, value) (SIM_SCGC5_REG(base) = (value))
#define SIM_RMW_SCGC5(base, mask, value) (SIM_WR_SCGC5(base, (SIM_RD_SCGC5(base) & ~(mask)) | (value)))
#define SIM_SET_SCGC5(base, value) (SIM_WR_SCGC5(base, SIM_RD_SCGC5(base) |  (value)))
#define SIM_CLR_SCGC5(base, value) (SIM_WR_SCGC5(base, SIM_RD_SCGC5(base) & ~(value)))
#define SIM_TOG_SCGC5(base, value) (SIM_WR_SCGC5(base, SIM_RD_SCGC5(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SCGC5 bitfields
 */

/*!
 * @name Register SIM_SCGC5, field LPTMR[0] (RW)
 *
 * This bit controls software access to the Low Power Timer module.
 *
 * Values:
 * - 0b0 - Access disabled
 * - 0b1 - Access enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC5_LPTMR field. */
#define SIM_RD_SCGC5_LPTMR(base) ((SIM_SCGC5_REG(base) & SIM_SCGC5_LPTMR_MASK) >> SIM_SCGC5_LPTMR_SHIFT)
#define SIM_BRD_SCGC5_LPTMR(base) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_LPTMR_SHIFT))

/*! @brief Set the LPTMR field to a new value. */
#define SIM_WR_SCGC5_LPTMR(base, value) (SIM_RMW_SCGC5(base, SIM_SCGC5_LPTMR_MASK, SIM_SCGC5_LPTMR(value)))
#define SIM_BWR_SCGC5_LPTMR(base, value) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_LPTMR_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC5, field PORTA[9] (RW)
 *
 * This bit controls the clock gate to the Port A module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC5_PORTA field. */
#define SIM_RD_SCGC5_PORTA(base) ((SIM_SCGC5_REG(base) & SIM_SCGC5_PORTA_MASK) >> SIM_SCGC5_PORTA_SHIFT)
#define SIM_BRD_SCGC5_PORTA(base) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTA_SHIFT))

/*! @brief Set the PORTA field to a new value. */
#define SIM_WR_SCGC5_PORTA(base, value) (SIM_RMW_SCGC5(base, SIM_SCGC5_PORTA_MASK, SIM_SCGC5_PORTA(value)))
#define SIM_BWR_SCGC5_PORTA(base, value) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTA_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC5, field PORTB[10] (RW)
 *
 * This bit controls the clock gate to the Port B module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC5_PORTB field. */
#define SIM_RD_SCGC5_PORTB(base) ((SIM_SCGC5_REG(base) & SIM_SCGC5_PORTB_MASK) >> SIM_SCGC5_PORTB_SHIFT)
#define SIM_BRD_SCGC5_PORTB(base) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTB_SHIFT))

/*! @brief Set the PORTB field to a new value. */
#define SIM_WR_SCGC5_PORTB(base, value) (SIM_RMW_SCGC5(base, SIM_SCGC5_PORTB_MASK, SIM_SCGC5_PORTB(value)))
#define SIM_BWR_SCGC5_PORTB(base, value) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTB_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC5, field PORTC[11] (RW)
 *
 * This bit controls the clock gate to the Port C module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC5_PORTC field. */
#define SIM_RD_SCGC5_PORTC(base) ((SIM_SCGC5_REG(base) & SIM_SCGC5_PORTC_MASK) >> SIM_SCGC5_PORTC_SHIFT)
#define SIM_BRD_SCGC5_PORTC(base) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTC_SHIFT))

/*! @brief Set the PORTC field to a new value. */
#define SIM_WR_SCGC5_PORTC(base, value) (SIM_RMW_SCGC5(base, SIM_SCGC5_PORTC_MASK, SIM_SCGC5_PORTC(value)))
#define SIM_BWR_SCGC5_PORTC(base, value) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC5, field PORTD[12] (RW)
 *
 * This bit controls the clock gate to the Port D module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC5_PORTD field. */
#define SIM_RD_SCGC5_PORTD(base) ((SIM_SCGC5_REG(base) & SIM_SCGC5_PORTD_MASK) >> SIM_SCGC5_PORTD_SHIFT)
#define SIM_BRD_SCGC5_PORTD(base) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTD_SHIFT))

/*! @brief Set the PORTD field to a new value. */
#define SIM_WR_SCGC5_PORTD(base, value) (SIM_RMW_SCGC5(base, SIM_SCGC5_PORTD_MASK, SIM_SCGC5_PORTD(value)))
#define SIM_BWR_SCGC5_PORTD(base, value) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTD_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC5, field PORTE[13] (RW)
 *
 * This bit controls the clock gate to the Port E module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC5_PORTE field. */
#define SIM_RD_SCGC5_PORTE(base) ((SIM_SCGC5_REG(base) & SIM_SCGC5_PORTE_MASK) >> SIM_SCGC5_PORTE_SHIFT)
#define SIM_BRD_SCGC5_PORTE(base) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTE_SHIFT))

/*! @brief Set the PORTE field to a new value. */
#define SIM_WR_SCGC5_PORTE(base, value) (SIM_RMW_SCGC5(base, SIM_SCGC5_PORTE_MASK, SIM_SCGC5_PORTE(value)))
#define SIM_BWR_SCGC5_PORTE(base, value) (BITBAND_ACCESS32(&SIM_SCGC5_REG(base), SIM_SCGC5_PORTE_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SCGC6 - System Clock Gating Control Register 6
 ******************************************************************************/

/*!
 * @brief SIM_SCGC6 - System Clock Gating Control Register 6 (RW)
 *
 * Reset value: 0x40000001U
 */
/*!
 * @name Constants and macros for entire SIM_SCGC6 register
 */
/*@{*/
#define SIM_RD_SCGC6(base)       (SIM_SCGC6_REG(base))
#define SIM_WR_SCGC6(base, value) (SIM_SCGC6_REG(base) = (value))
#define SIM_RMW_SCGC6(base, mask, value) (SIM_WR_SCGC6(base, (SIM_RD_SCGC6(base) & ~(mask)) | (value)))
#define SIM_SET_SCGC6(base, value) (SIM_WR_SCGC6(base, SIM_RD_SCGC6(base) |  (value)))
#define SIM_CLR_SCGC6(base, value) (SIM_WR_SCGC6(base, SIM_RD_SCGC6(base) & ~(value)))
#define SIM_TOG_SCGC6(base, value) (SIM_WR_SCGC6(base, SIM_RD_SCGC6(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SCGC6 bitfields
 */

/*!
 * @name Register SIM_SCGC6, field FTF[0] (RW)
 *
 * This bit controls the clock gate to the flash memory. Flash reads are still
 * supported while the flash memory is clock gated, but entry into low power modes
 * and HSRUN mode is blocked.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_FTF field. */
#define SIM_RD_SCGC6_FTF(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_FTF_MASK) >> SIM_SCGC6_FTF_SHIFT)
#define SIM_BRD_SCGC6_FTF(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTF_SHIFT))

/*! @brief Set the FTF field to a new value. */
#define SIM_WR_SCGC6_FTF(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_FTF_MASK, SIM_SCGC6_FTF(value)))
#define SIM_BWR_SCGC6_FTF(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field DMAMUX[1] (RW)
 *
 * This bit controls the clock gate to the DMA Mux module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_DMAMUX field. */
#define SIM_RD_SCGC6_DMAMUX(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_DMAMUX_MASK) >> SIM_SCGC6_DMAMUX_SHIFT)
#define SIM_BRD_SCGC6_DMAMUX(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_DMAMUX_SHIFT))

/*! @brief Set the DMAMUX field to a new value. */
#define SIM_WR_SCGC6_DMAMUX(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_DMAMUX_MASK, SIM_SCGC6_DMAMUX(value)))
#define SIM_BWR_SCGC6_DMAMUX(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_DMAMUX_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field FTM3[6] (RW)
 *
 * This bit controls the clock gate to the FTM3 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_FTM3 field. */
#define SIM_RD_SCGC6_FTM3(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_FTM3_MASK) >> SIM_SCGC6_FTM3_SHIFT)
#define SIM_BRD_SCGC6_FTM3(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM3_SHIFT))

/*! @brief Set the FTM3 field to a new value. */
#define SIM_WR_SCGC6_FTM3(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_FTM3_MASK, SIM_SCGC6_FTM3(value)))
#define SIM_BWR_SCGC6_FTM3(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM3_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field ADC1[7] (RW)
 *
 * This bit controls the clock gate to the ADC1 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_ADC1 field. */
#define SIM_RD_SCGC6_ADC1(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_ADC1_MASK) >> SIM_SCGC6_ADC1_SHIFT)
#define SIM_BRD_SCGC6_ADC1(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_ADC1_SHIFT))

/*! @brief Set the ADC1 field to a new value. */
#define SIM_WR_SCGC6_ADC1(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_ADC1_MASK, SIM_SCGC6_ADC1(value)))
#define SIM_BWR_SCGC6_ADC1(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_ADC1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field DAC1[8] (RW)
 *
 * This bit controls the clock gate to the DAC1 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_DAC1 field. */
#define SIM_RD_SCGC6_DAC1(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_DAC1_MASK) >> SIM_SCGC6_DAC1_SHIFT)
#define SIM_BRD_SCGC6_DAC1(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_DAC1_SHIFT))

/*! @brief Set the DAC1 field to a new value. */
#define SIM_WR_SCGC6_DAC1(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_DAC1_MASK, SIM_SCGC6_DAC1(value)))
#define SIM_BWR_SCGC6_DAC1(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_DAC1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field RNGA[9] (RW)
 *
 * This bit controls the clock gate to the RNGA module.
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_RNGA field. */
#define SIM_RD_SCGC6_RNGA(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_RNGA_MASK) >> SIM_SCGC6_RNGA_SHIFT)
#define SIM_BRD_SCGC6_RNGA(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_RNGA_SHIFT))

/*! @brief Set the RNGA field to a new value. */
#define SIM_WR_SCGC6_RNGA(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_RNGA_MASK, SIM_SCGC6_RNGA(value)))
#define SIM_BWR_SCGC6_RNGA(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_RNGA_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field LPUART0[10] (RW)
 *
 * This bit controls the clock gate to the LPUART0 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_LPUART0 field. */
#define SIM_RD_SCGC6_LPUART0(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_LPUART0_MASK) >> SIM_SCGC6_LPUART0_SHIFT)
#define SIM_BRD_SCGC6_LPUART0(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_LPUART0_SHIFT))

/*! @brief Set the LPUART0 field to a new value. */
#define SIM_WR_SCGC6_LPUART0(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_LPUART0_MASK, SIM_SCGC6_LPUART0(value)))
#define SIM_BWR_SCGC6_LPUART0(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_LPUART0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field SPI0[12] (RW)
 *
 * This bit controls the clock gate to the SPI0 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_SPI0 field. */
#define SIM_RD_SCGC6_SPI0(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_SPI0_MASK) >> SIM_SCGC6_SPI0_SHIFT)
#define SIM_BRD_SCGC6_SPI0(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_SPI0_SHIFT))

/*! @brief Set the SPI0 field to a new value. */
#define SIM_WR_SCGC6_SPI0(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_SPI0_MASK, SIM_SCGC6_SPI0(value)))
#define SIM_BWR_SCGC6_SPI0(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_SPI0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field SPI1[13] (RW)
 *
 * This bit controls the clock gate to the SPI1 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_SPI1 field. */
#define SIM_RD_SCGC6_SPI1(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_SPI1_MASK) >> SIM_SCGC6_SPI1_SHIFT)
#define SIM_BRD_SCGC6_SPI1(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_SPI1_SHIFT))

/*! @brief Set the SPI1 field to a new value. */
#define SIM_WR_SCGC6_SPI1(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_SPI1_MASK, SIM_SCGC6_SPI1(value)))
#define SIM_BWR_SCGC6_SPI1(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_SPI1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field I2S[15] (RW)
 *
 * This bit controls the clock gate to the I 2 S module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_I2S field. */
#define SIM_RD_SCGC6_I2S(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_I2S_MASK) >> SIM_SCGC6_I2S_SHIFT)
#define SIM_BRD_SCGC6_I2S(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_I2S_SHIFT))

/*! @brief Set the I2S field to a new value. */
#define SIM_WR_SCGC6_I2S(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_I2S_MASK, SIM_SCGC6_I2S(value)))
#define SIM_BWR_SCGC6_I2S(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_I2S_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field CRC[18] (RW)
 *
 * This bit controls the clock gate to the CRC module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_CRC field. */
#define SIM_RD_SCGC6_CRC(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_CRC_MASK) >> SIM_SCGC6_CRC_SHIFT)
#define SIM_BRD_SCGC6_CRC(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_CRC_SHIFT))

/*! @brief Set the CRC field to a new value. */
#define SIM_WR_SCGC6_CRC(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_CRC_MASK, SIM_SCGC6_CRC(value)))
#define SIM_BWR_SCGC6_CRC(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_CRC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field PDB[22] (RW)
 *
 * This bit controls the clock gate to the PDB module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_PDB field. */
#define SIM_RD_SCGC6_PDB(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_PDB_MASK) >> SIM_SCGC6_PDB_SHIFT)
#define SIM_BRD_SCGC6_PDB(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_PDB_SHIFT))

/*! @brief Set the PDB field to a new value. */
#define SIM_WR_SCGC6_PDB(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_PDB_MASK, SIM_SCGC6_PDB(value)))
#define SIM_BWR_SCGC6_PDB(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_PDB_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field PIT[23] (RW)
 *
 * This bit controls the clock gate to the PIT module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_PIT field. */
#define SIM_RD_SCGC6_PIT(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_PIT_MASK) >> SIM_SCGC6_PIT_SHIFT)
#define SIM_BRD_SCGC6_PIT(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_PIT_SHIFT))

/*! @brief Set the PIT field to a new value. */
#define SIM_WR_SCGC6_PIT(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_PIT_MASK, SIM_SCGC6_PIT(value)))
#define SIM_BWR_SCGC6_PIT(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_PIT_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field FTM0[24] (RW)
 *
 * This bit controls the clock gate to the FTM0 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_FTM0 field. */
#define SIM_RD_SCGC6_FTM0(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_FTM0_MASK) >> SIM_SCGC6_FTM0_SHIFT)
#define SIM_BRD_SCGC6_FTM0(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM0_SHIFT))

/*! @brief Set the FTM0 field to a new value. */
#define SIM_WR_SCGC6_FTM0(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_FTM0_MASK, SIM_SCGC6_FTM0(value)))
#define SIM_BWR_SCGC6_FTM0(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field FTM1[25] (RW)
 *
 * This bit controls the clock gate to the FTM1 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_FTM1 field. */
#define SIM_RD_SCGC6_FTM1(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_FTM1_MASK) >> SIM_SCGC6_FTM1_SHIFT)
#define SIM_BRD_SCGC6_FTM1(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM1_SHIFT))

/*! @brief Set the FTM1 field to a new value. */
#define SIM_WR_SCGC6_FTM1(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_FTM1_MASK, SIM_SCGC6_FTM1(value)))
#define SIM_BWR_SCGC6_FTM1(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field FTM2[26] (RW)
 *
 * This bit controls the clock gate to the FTM2 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_FTM2 field. */
#define SIM_RD_SCGC6_FTM2(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_FTM2_MASK) >> SIM_SCGC6_FTM2_SHIFT)
#define SIM_BRD_SCGC6_FTM2(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM2_SHIFT))

/*! @brief Set the FTM2 field to a new value. */
#define SIM_WR_SCGC6_FTM2(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_FTM2_MASK, SIM_SCGC6_FTM2(value)))
#define SIM_BWR_SCGC6_FTM2(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_FTM2_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field ADC0[27] (RW)
 *
 * This bit controls the clock gate to the ADC0 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_ADC0 field. */
#define SIM_RD_SCGC6_ADC0(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_ADC0_MASK) >> SIM_SCGC6_ADC0_SHIFT)
#define SIM_BRD_SCGC6_ADC0(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_ADC0_SHIFT))

/*! @brief Set the ADC0 field to a new value. */
#define SIM_WR_SCGC6_ADC0(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_ADC0_MASK, SIM_SCGC6_ADC0(value)))
#define SIM_BWR_SCGC6_ADC0(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_ADC0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field RTC[29] (RW)
 *
 * This bit controls software access and interrupts to the RTC module.
 *
 * Values:
 * - 0b0 - Access and interrupts disabled
 * - 0b1 - Access and interrupts enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_RTC field. */
#define SIM_RD_SCGC6_RTC(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_RTC_MASK) >> SIM_SCGC6_RTC_SHIFT)
#define SIM_BRD_SCGC6_RTC(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_RTC_SHIFT))

/*! @brief Set the RTC field to a new value. */
#define SIM_WR_SCGC6_RTC(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_RTC_MASK, SIM_SCGC6_RTC(value)))
#define SIM_BWR_SCGC6_RTC(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_RTC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC6, field DAC0[31] (RW)
 *
 * This bit controls the clock gate to the DAC0 module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC6_DAC0 field. */
#define SIM_RD_SCGC6_DAC0(base) ((SIM_SCGC6_REG(base) & SIM_SCGC6_DAC0_MASK) >> SIM_SCGC6_DAC0_SHIFT)
#define SIM_BRD_SCGC6_DAC0(base) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_DAC0_SHIFT))

/*! @brief Set the DAC0 field to a new value. */
#define SIM_WR_SCGC6_DAC0(base, value) (SIM_RMW_SCGC6(base, SIM_SCGC6_DAC0_MASK, SIM_SCGC6_DAC0(value)))
#define SIM_BWR_SCGC6_DAC0(base, value) (BITBAND_ACCESS32(&SIM_SCGC6_REG(base), SIM_SCGC6_DAC0_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_SCGC7 - System Clock Gating Control Register 7
 ******************************************************************************/

/*!
 * @brief SIM_SCGC7 - System Clock Gating Control Register 7 (RW)
 *
 * Reset value: 0x00000002U
 */
/*!
 * @name Constants and macros for entire SIM_SCGC7 register
 */
/*@{*/
#define SIM_RD_SCGC7(base)       (SIM_SCGC7_REG(base))
#define SIM_WR_SCGC7(base, value) (SIM_SCGC7_REG(base) = (value))
#define SIM_RMW_SCGC7(base, mask, value) (SIM_WR_SCGC7(base, (SIM_RD_SCGC7(base) & ~(mask)) | (value)))
#define SIM_SET_SCGC7(base, value) (SIM_WR_SCGC7(base, SIM_RD_SCGC7(base) |  (value)))
#define SIM_CLR_SCGC7(base, value) (SIM_WR_SCGC7(base, SIM_RD_SCGC7(base) & ~(value)))
#define SIM_TOG_SCGC7(base, value) (SIM_WR_SCGC7(base, SIM_RD_SCGC7(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_SCGC7 bitfields
 */

/*!
 * @name Register SIM_SCGC7, field FLEXBUS[0] (RW)
 *
 * This bit controls the clock gate to the FlexBus module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC7_FLEXBUS field. */
#define SIM_RD_SCGC7_FLEXBUS(base) ((SIM_SCGC7_REG(base) & SIM_SCGC7_FLEXBUS_MASK) >> SIM_SCGC7_FLEXBUS_SHIFT)
#define SIM_BRD_SCGC7_FLEXBUS(base) (BITBAND_ACCESS32(&SIM_SCGC7_REG(base), SIM_SCGC7_FLEXBUS_SHIFT))

/*! @brief Set the FLEXBUS field to a new value. */
#define SIM_WR_SCGC7_FLEXBUS(base, value) (SIM_RMW_SCGC7(base, SIM_SCGC7_FLEXBUS_MASK, SIM_SCGC7_FLEXBUS(value)))
#define SIM_BWR_SCGC7_FLEXBUS(base, value) (BITBAND_ACCESS32(&SIM_SCGC7_REG(base), SIM_SCGC7_FLEXBUS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_SCGC7, field DMA[1] (RW)
 *
 * This bit controls the clock gate to the DMA module.
 *
 * Values:
 * - 0b0 - Clock disabled
 * - 0b1 - Clock enabled
 */
/*@{*/
/*! @brief Read current value of the SIM_SCGC7_DMA field. */
#define SIM_RD_SCGC7_DMA(base) ((SIM_SCGC7_REG(base) & SIM_SCGC7_DMA_MASK) >> SIM_SCGC7_DMA_SHIFT)
#define SIM_BRD_SCGC7_DMA(base) (BITBAND_ACCESS32(&SIM_SCGC7_REG(base), SIM_SCGC7_DMA_SHIFT))

/*! @brief Set the DMA field to a new value. */
#define SIM_WR_SCGC7_DMA(base, value) (SIM_RMW_SCGC7(base, SIM_SCGC7_DMA_MASK, SIM_SCGC7_DMA(value)))
#define SIM_BWR_SCGC7_DMA(base, value) (BITBAND_ACCESS32(&SIM_SCGC7_REG(base), SIM_SCGC7_DMA_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * SIM_CLKDIV1 - System Clock Divider Register 1
 ******************************************************************************/

/*!
 * @brief SIM_CLKDIV1 - System Clock Divider Register 1 (RW)
 *
 * Reset value: 0x00010000U
 *
 * When updating CLKDIV1, update all fields using the one write command.
 * Attempting to write an invalid clock ratio to the CLKDIV1 register will cause the
 * write to be ignored. The maximum divide ratio that can be programmed between
 * core/system clock and the other divided clocks is divide by 8. When OUTDIV1 equals
 * 0000 (divide by 1), the other dividers cannot be set higher than 0111 (divide
 * by 8). The CLKDIV1 register cannot be written to when the device is in VLPR
 * mode.
 */
/*!
 * @name Constants and macros for entire SIM_CLKDIV1 register
 */
/*@{*/
#define SIM_RD_CLKDIV1(base)     (SIM_CLKDIV1_REG(base))
#define SIM_WR_CLKDIV1(base, value) (SIM_CLKDIV1_REG(base) = (value))
#define SIM_RMW_CLKDIV1(base, mask, value) (SIM_WR_CLKDIV1(base, (SIM_RD_CLKDIV1(base) & ~(mask)) | (value)))
#define SIM_SET_CLKDIV1(base, value) (SIM_WR_CLKDIV1(base, SIM_RD_CLKDIV1(base) |  (value)))
#define SIM_CLR_CLKDIV1(base, value) (SIM_WR_CLKDIV1(base, SIM_RD_CLKDIV1(base) & ~(value)))
#define SIM_TOG_CLKDIV1(base, value) (SIM_WR_CLKDIV1(base, SIM_RD_CLKDIV1(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_CLKDIV1 bitfields
 */

/*!
 * @name Register SIM_CLKDIV1, field OUTDIV4[19:16] (RW)
 *
 * This field sets the divide value for the flash clock from MCGOUTCLK. At the
 * end of reset, it is loaded with either 0001 or 1111 depending on
 * FTF_FOPT[LPBOOT]. The flash clock frequency must be an integer divide of the system clock
 * frequency.
 *
 * Values:
 * - 0b0000 - Divide-by-1.
 * - 0b0001 - Divide-by-2.
 * - 0b0010 - Divide-by-3.
 * - 0b0011 - Divide-by-4.
 * - 0b0100 - Divide-by-5.
 * - 0b0101 - Divide-by-6.
 * - 0b0110 - Divide-by-7.
 * - 0b0111 - Divide-by-8.
 * - 0b1000 - Divide-by-9.
 * - 0b1001 - Divide-by-10.
 * - 0b1010 - Divide-by-11.
 * - 0b1011 - Divide-by-12.
 * - 0b1100 - Divide-by-13.
 * - 0b1101 - Divide-by-14.
 * - 0b1110 - Divide-by-15.
 * - 0b1111 - Divide-by-16.
 */
/*@{*/
/*! @brief Read current value of the SIM_CLKDIV1_OUTDIV4 field. */
#define SIM_RD_CLKDIV1_OUTDIV4(base) ((SIM_CLKDIV1_REG(base) & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT)
#define SIM_BRD_CLKDIV1_OUTDIV4(base) (SIM_RD_CLKDIV1_OUTDIV4(base))

/*! @brief Set the OUTDIV4 field to a new value. */
#define SIM_WR_CLKDIV1_OUTDIV4(base, value) (SIM_RMW_CLKDIV1(base, SIM_CLKDIV1_OUTDIV4_MASK, SIM_CLKDIV1_OUTDIV4(value)))
#define SIM_BWR_CLKDIV1_OUTDIV4(base, value) (SIM_WR_CLKDIV1_OUTDIV4(base, value))
/*@}*/

/*!
 * @name Register SIM_CLKDIV1, field OUTDIV3[23:20] (RW)
 *
 * This field sets the divide value for the FlexBus clock (external pin FB_CLK)
 * from MCGOUTCLK. At the end of reset, it is loaded with either 0001 or 1111
 * depending on FTF_FOPT[LPBOOT]. The FlexBus clock frequency must be an integer
 * divide of the system clock frequency.
 *
 * Values:
 * - 0b0000 - Divide-by-1.
 * - 0b0001 - Divide-by-2.
 * - 0b0010 - Divide-by-3.
 * - 0b0011 - Divide-by-4.
 * - 0b0100 - Divide-by-5.
 * - 0b0101 - Divide-by-6.
 * - 0b0110 - Divide-by-7.
 * - 0b0111 - Divide-by-8.
 * - 0b1000 - Divide-by-9.
 * - 0b1001 - Divide-by-10.
 * - 0b1010 - Divide-by-11.
 * - 0b1011 - Divide-by-12.
 * - 0b1100 - Divide-by-13.
 * - 0b1101 - Divide-by-14.
 * - 0b1110 - Divide-by-15.
 * - 0b1111 - Divide-by-16.
 */
/*@{*/

/*!
 * @name Register SIM_CLKDIV1, field OUTDIV2[27:24] (RW)
 *
 * This field sets the divide value for the bus clock from MCGOUTCLK. At the end
 * of reset, it is loaded with either 0000 or 0111 depending on
 * FTF_FOPT[LPBOOT]. The bus clock frequency must be an integer divide of the core/system clock
 * frequency.
 *
 * Values:
 * - 0b0000 - Divide-by-1.
 * - 0b0001 - Divide-by-2.
 * - 0b0010 - Divide-by-3.
 * - 0b0011 - Divide-by-4.
 * - 0b0100 - Divide-by-5.
 * - 0b0101 - Divide-by-6.
 * - 0b0110 - Divide-by-7.
 * - 0b0111 - Divide-by-8.
 * - 0b1000 - Divide-by-9.
 * - 0b1001 - Divide-by-10.
 * - 0b1010 - Divide-by-11.
 * - 0b1011 - Divide-by-12.
 * - 0b1100 - Divide-by-13.
 * - 0b1101 - Divide-by-14.
 * - 0b1110 - Divide-by-15.
 * - 0b1111 - Divide-by-16.
 */
/*@{*/
/*! @brief Read current value of the SIM_CLKDIV1_OUTDIV2 field. */
#define SIM_RD_CLKDIV1_OUTDIV2(base) ((SIM_CLKDIV1_REG(base) & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT)
#define SIM_BRD_CLKDIV1_OUTDIV2(base) (SIM_RD_CLKDIV1_OUTDIV2(base))

/*! @brief Set the OUTDIV2 field to a new value. */
#define SIM_WR_CLKDIV1_OUTDIV2(base, value) (SIM_RMW_CLKDIV1(base, SIM_CLKDIV1_OUTDIV2_MASK, SIM_CLKDIV1_OUTDIV2(value)))
#define SIM_BWR_CLKDIV1_OUTDIV2(base, value) (SIM_WR_CLKDIV1_OUTDIV2(base, value))
/*@}*/

/*!
 * @name Register SIM_CLKDIV1, field OUTDIV1[31:28] (RW)
 *
 * This field sets the divide value for the core/system clock from MCGOUTCLK. At
 * the end of reset, it is loaded with either 0000 or 0111 depending on
 * FTF_FOPT[LPBOOT].
 *
 * Values:
 * - 0b0000 - Divide-by-1.
 * - 0b0001 - Divide-by-2.
 * - 0b0010 - Divide-by-3.
 * - 0b0011 - Divide-by-4.
 * - 0b0100 - Divide-by-5.
 * - 0b0101 - Divide-by-6.
 * - 0b0110 - Divide-by-7.
 * - 0b0111 - Divide-by-8.
 * - 0b1000 - Divide-by-9.
 * - 0b1001 - Divide-by-10.
 * - 0b1010 - Divide-by-11.
 * - 0b1011 - Divide-by-12.
 * - 0b1100 - Divide-by-13.
 * - 0b1101 - Divide-by-14.
 * - 0b1110 - Divide-by-15.
 * - 0b1111 - Divide-by-16.
 */
/*@{*/
/*! @brief Read current value of the SIM_CLKDIV1_OUTDIV1 field. */
#define SIM_RD_CLKDIV1_OUTDIV1(base) ((SIM_CLKDIV1_REG(base) & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)
#define SIM_BRD_CLKDIV1_OUTDIV1(base) (SIM_RD_CLKDIV1_OUTDIV1(base))

/*! @brief Set the OUTDIV1 field to a new value. */
#define SIM_WR_CLKDIV1_OUTDIV1(base, value) (SIM_RMW_CLKDIV1(base, SIM_CLKDIV1_OUTDIV1_MASK, SIM_CLKDIV1_OUTDIV1(value)))
#define SIM_BWR_CLKDIV1_OUTDIV1(base, value) (SIM_WR_CLKDIV1_OUTDIV1(base, value))
/*@}*/

/*******************************************************************************
 * SIM_CLKDIV2 - System Clock Divider Register 2
 ******************************************************************************/

/*!
 * @brief SIM_CLKDIV2 - System Clock Divider Register 2 (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_CLKDIV2 register
 */
/*@{*/
#define SIM_RD_CLKDIV2(base)     (SIM_CLKDIV2_REG(base))
#define SIM_WR_CLKDIV2(base, value) (SIM_CLKDIV2_REG(base) = (value))
#define SIM_RMW_CLKDIV2(base, mask, value) (SIM_WR_CLKDIV2(base, (SIM_RD_CLKDIV2(base) & ~(mask)) | (value)))
#define SIM_SET_CLKDIV2(base, value) (SIM_WR_CLKDIV2(base, SIM_RD_CLKDIV2(base) |  (value)))
#define SIM_CLR_CLKDIV2(base, value) (SIM_WR_CLKDIV2(base, SIM_RD_CLKDIV2(base) & ~(value)))
#define SIM_TOG_CLKDIV2(base, value) (SIM_WR_CLKDIV2(base, SIM_RD_CLKDIV2(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_CLKDIV2 bitfields
 */

/*!
 * @name Register SIM_CLKDIV2, field USBFRAC[0] (RW)
 *
 * This field sets the fraction multiply value for the fractional clock divider
 * when the MCGFLLCLK/MCGPLLCLK clock is the USB clock source (SOPT2[USBSRC] =
 * 1). Divider output clock = Divider input clock * [ (USBFRAC+1) / (USBDIV+1) ]
 */
/*@{*/
/*! @brief Read current value of the SIM_CLKDIV2_USBFRAC field. */
#define SIM_RD_CLKDIV2_USBFRAC(base) ((SIM_CLKDIV2_REG(base) & SIM_CLKDIV2_USBFRAC_MASK) >> SIM_CLKDIV2_USBFRAC_SHIFT)
#define SIM_BRD_CLKDIV2_USBFRAC(base) (BITBAND_ACCESS32(&SIM_CLKDIV2_REG(base), SIM_CLKDIV2_USBFRAC_SHIFT))

/*! @brief Set the USBFRAC field to a new value. */
#define SIM_WR_CLKDIV2_USBFRAC(base, value) (SIM_RMW_CLKDIV2(base, SIM_CLKDIV2_USBFRAC_MASK, SIM_CLKDIV2_USBFRAC(value)))
#define SIM_BWR_CLKDIV2_USBFRAC(base, value) (BITBAND_ACCESS32(&SIM_CLKDIV2_REG(base), SIM_CLKDIV2_USBFRAC_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_CLKDIV2, field USBDIV[3:1] (RW)
 *
 * This field sets the divide value for the fractional clock divider when the
 * MCGFLLCLK/MCGPLLCLK clock is the USB clock source (SOPT2[USBSRC] = 1). Divider
 * output clock = Divider input clock * [ (USBFRAC+1) / (USBDIV+1) ]
 */
/*@{*/
/*! @brief Read current value of the SIM_CLKDIV2_USBDIV field. */
#define SIM_RD_CLKDIV2_USBDIV(base) ((SIM_CLKDIV2_REG(base) & SIM_CLKDIV2_USBDIV_MASK) >> SIM_CLKDIV2_USBDIV_SHIFT)
#define SIM_BRD_CLKDIV2_USBDIV(base) (SIM_RD_CLKDIV2_USBDIV(base))

/*! @brief Set the USBDIV field to a new value. */
#define SIM_WR_CLKDIV2_USBDIV(base, value) (SIM_RMW_CLKDIV2(base, SIM_CLKDIV2_USBDIV_MASK, SIM_CLKDIV2_USBDIV(value)))
#define SIM_BWR_CLKDIV2_USBDIV(base, value) (SIM_WR_CLKDIV2_USBDIV(base, value))
/*@}*/

/*******************************************************************************
 * SIM_FCFG1 - Flash Configuration Register 1
 ******************************************************************************/

/*!
 * @brief SIM_FCFG1 - Flash Configuration Register 1 (RW)
 *
 * Reset value: 0x0F0F0F00U
 *
 * The EESIZE and DEPART filelds are not applicable.
 */
/*!
 * @name Constants and macros for entire SIM_FCFG1 register
 */
/*@{*/
#define SIM_RD_FCFG1(base)       (SIM_FCFG1_REG(base))
#define SIM_WR_FCFG1(base, value) (SIM_FCFG1_REG(base) = (value))
#define SIM_RMW_FCFG1(base, mask, value) (SIM_WR_FCFG1(base, (SIM_RD_FCFG1(base) & ~(mask)) | (value)))
#define SIM_SET_FCFG1(base, value) (SIM_WR_FCFG1(base, SIM_RD_FCFG1(base) |  (value)))
#define SIM_CLR_FCFG1(base, value) (SIM_WR_FCFG1(base, SIM_RD_FCFG1(base) & ~(value)))
#define SIM_TOG_FCFG1(base, value) (SIM_WR_FCFG1(base, SIM_RD_FCFG1(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SIM_FCFG1 bitfields
 */

/*!
 * @name Register SIM_FCFG1, field FLASHDIS[0] (RW)
 *
 * Flash accesses are disabled (and generate a bus error) and the Flash memory
 * is placed in a low power state. This bit should not be changed during VLP
 * modes. Relocate the interrupt vectors out of Flash memory before disabling the
 * Flash.
 *
 * Values:
 * - 0b0 - Flash is enabled
 * - 0b1 - Flash is disabled
 */
/*@{*/
/*! @brief Read current value of the SIM_FCFG1_FLASHDIS field. */
#define SIM_RD_FCFG1_FLASHDIS(base) ((SIM_FCFG1_REG(base) & SIM_FCFG1_FLASHDIS_MASK) >> SIM_FCFG1_FLASHDIS_SHIFT)
#define SIM_BRD_FCFG1_FLASHDIS(base) (BITBAND_ACCESS32(&SIM_FCFG1_REG(base), SIM_FCFG1_FLASHDIS_SHIFT))

/*! @brief Set the FLASHDIS field to a new value. */
#define SIM_WR_FCFG1_FLASHDIS(base, value) (SIM_RMW_FCFG1(base, SIM_FCFG1_FLASHDIS_MASK, SIM_FCFG1_FLASHDIS(value)))
#define SIM_BWR_FCFG1_FLASHDIS(base, value) (BITBAND_ACCESS32(&SIM_FCFG1_REG(base), SIM_FCFG1_FLASHDIS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_FCFG1, field FLASHDOZE[1] (RW)
 *
 * When set, Flash memory is disabled for the duration of Wait mode. An attempt
 * by the DMA or other bus master to access the Flash when the Flash is disabled
 * will result in a bus error. This bit should be clear during VLP modes. The
 * Flash will be automatically enabled again at the end of Wait mode so interrupt
 * vectors do not need to be relocated out of Flash memory. The wakeup time from
 * Wait mode is extended when this bit is set.
 *
 * Values:
 * - 0b0 - Flash remains enabled during Wait mode
 * - 0b1 - Flash is disabled for the duration of Wait mode
 */
/*@{*/
/*! @brief Read current value of the SIM_FCFG1_FLASHDOZE field. */
#define SIM_RD_FCFG1_FLASHDOZE(base) ((SIM_FCFG1_REG(base) & SIM_FCFG1_FLASHDOZE_MASK) >> SIM_FCFG1_FLASHDOZE_SHIFT)
#define SIM_BRD_FCFG1_FLASHDOZE(base) (BITBAND_ACCESS32(&SIM_FCFG1_REG(base), SIM_FCFG1_FLASHDOZE_SHIFT))

/*! @brief Set the FLASHDOZE field to a new value. */
#define SIM_WR_FCFG1_FLASHDOZE(base, value) (SIM_RMW_FCFG1(base, SIM_FCFG1_FLASHDOZE_MASK, SIM_FCFG1_FLASHDOZE(value)))
#define SIM_BWR_FCFG1_FLASHDOZE(base, value) (BITBAND_ACCESS32(&SIM_FCFG1_REG(base), SIM_FCFG1_FLASHDOZE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SIM_FCFG1, field PFSIZE[27:24] (RO)
 *
 * This field specifies the amount of program flash memory available on the
 * device . Undefined values are reserved.
 *
 * Values:
 * - 0b0011 - 32 KB of program flash memory
 * - 0b0101 - 64 KB of program flash memory
 * - 0b0111 - 128 KB of program flash memory
 * - 0b1001 - 256 KB of program flash memory
 * - 0b1011 - 512 KB of program flash memory
 * - 0b1101 - 1024 KB of program flash memory
 * - 0b1111 - 512 KB of program flash memory
 */
/*@{*/
/*! @brief Read current value of the SIM_FCFG1_PFSIZE field. */
#define SIM_RD_FCFG1_PFSIZE(base) ((SIM_FCFG1_REG(base) & SIM_FCFG1_PFSIZE_MASK) >> SIM_FCFG1_PFSIZE_SHIFT)
#define SIM_BRD_FCFG1_PFSIZE(base) (SIM_RD_FCFG1_PFSIZE(base))
/*@}*/

/*******************************************************************************
 * SIM_FCFG2 - Flash Configuration Register 2
 ******************************************************************************/

/*!
 * @brief SIM_FCFG2 - Flash Configuration Register 2 (RO)
 *
 * Reset value: 0x7FFF0000U
 */
/*!
 * @name Constants and macros for entire SIM_FCFG2 register
 */
/*@{*/
#define SIM_RD_FCFG2(base)       (SIM_FCFG2_REG(base))
/*@}*/

/*
 * Constants & macros for individual SIM_FCFG2 bitfields
 */

/*!
 * @name Register SIM_FCFG2, field MAXADDR1[22:16] (RO)
 *
 * This field equals zero if there is only one program flash block, otherwise it
 * equals the value of the MAXADDR0 field. For example, with MAXADDR0 = MAXADDR1
 * = 0x20 the first invalid address of flash block 1 is 0x4_0000 + 0x4_0000.
 * This would be the MAXADDR1 value for a device with 512 KB program flash memory
 * across two flash blocks and no FlexNVM.
 */
/*@{*/
/*! @brief Read current value of the SIM_FCFG2_MAXADDR1 field. */
#define SIM_RD_FCFG2_MAXADDR1(base) ((SIM_FCFG2_REG(base) & SIM_FCFG2_MAXADDR1_MASK) >> SIM_FCFG2_MAXADDR1_SHIFT)
#define SIM_BRD_FCFG2_MAXADDR1(base) (SIM_RD_FCFG2_MAXADDR1(base))
/*@}*/

/*!
 * @name Register SIM_FCFG2, field MAXADDR0[30:24] (RO)
 *
 * This field concatenated with 13 trailing zeros indicates the first invalid
 * address of each program flash block. For example, if MAXADDR0 = 0x20 the first
 * invalid address of flash block 0 is 0x0004_0000. This would be the MAXADDR0
 * value for a device with 256 KB program flash in flash block 0.
 */
/*@{*/
/*! @brief Read current value of the SIM_FCFG2_MAXADDR0 field. */
#define SIM_RD_FCFG2_MAXADDR0(base) ((SIM_FCFG2_REG(base) & SIM_FCFG2_MAXADDR0_MASK) >> SIM_FCFG2_MAXADDR0_SHIFT)
#define SIM_BRD_FCFG2_MAXADDR0(base) (SIM_RD_FCFG2_MAXADDR0(base))
/*@}*/

/*******************************************************************************
 * SIM_UIDH - Unique Identification Register High
 ******************************************************************************/

/*!
 * @brief SIM_UIDH - Unique Identification Register High (RO)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_UIDH register
 */
/*@{*/
#define SIM_RD_UIDH(base)        (SIM_UIDH_REG(base))
/*@}*/

/*******************************************************************************
 * SIM_UIDMH - Unique Identification Register Mid-High
 ******************************************************************************/

/*!
 * @brief SIM_UIDMH - Unique Identification Register Mid-High (RO)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_UIDMH register
 */
/*@{*/
#define SIM_RD_UIDMH(base)       (SIM_UIDMH_REG(base))
/*@}*/

/*******************************************************************************
 * SIM_UIDML - Unique Identification Register Mid Low
 ******************************************************************************/

/*!
 * @brief SIM_UIDML - Unique Identification Register Mid Low (RO)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_UIDML register
 */
/*@{*/
#define SIM_RD_UIDML(base)       (SIM_UIDML_REG(base))
/*@}*/

/*******************************************************************************
 * SIM_UIDL - Unique Identification Register Low
 ******************************************************************************/

/*!
 * @brief SIM_UIDL - Unique Identification Register Low (RO)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire SIM_UIDL register
 */
/*@{*/
#define SIM_RD_UIDL(base)        (SIM_UIDL_REG(base))
/*@}*/

/*
 * MK22F51212 SMC
 *
 * System Mode Controller
 *
 * Registers defined in this header file:
 * - SMC_PMPROT - Power Mode Protection register
 * - SMC_PMCTRL - Power Mode Control register
 * - SMC_STOPCTRL - Stop Control Register
 * - SMC_PMSTAT - Power Mode Status register
 */

#define SMC_INSTANCE_COUNT (1U) /*!< Number of instances of the SMC module. */
#define SMC_IDX (0U) /*!< Instance number for SMC. */

/*******************************************************************************
 * SMC_PMPROT - Power Mode Protection register
 ******************************************************************************/

/*!
 * @brief SMC_PMPROT - Power Mode Protection register (RW)
 *
 * Reset value: 0x00U
 *
 * This register provides protection for entry into any low-power run or stop
 * mode. The enabling of the low-power run or stop mode occurs by configuring the
 * Power Mode Control register (PMCTRL). The PMPROT register can be written only
 * once after any system reset. If the MCU is configured for a disallowed or
 * reserved power mode, the MCU remains in its current power mode. For example, if the
 * MCU is in normal RUN mode and AVLP is 0, an attempt to enter VLPR mode using
 * PMCTRL[RUNM] is blocked and PMCTRL[RUNM] remains 00b, indicating the MCU is
 * still in Normal Run mode. This register is reset on Chip Reset not VLLS and by
 * reset types that trigger Chip Reset not VLLS. It is unaffected by reset types
 * that do not trigger Chip Reset not VLLS. See the Reset section details for more
 * information.
 */
/*!
 * @name Constants and macros for entire SMC_PMPROT register
 */
/*@{*/
#define SMC_RD_PMPROT(base)      (SMC_PMPROT_REG(base))
#define SMC_WR_PMPROT(base, value) (SMC_PMPROT_REG(base) = (value))
#define SMC_RMW_PMPROT(base, mask, value) (SMC_WR_PMPROT(base, (SMC_RD_PMPROT(base) & ~(mask)) | (value)))
#define SMC_SET_PMPROT(base, value) (SMC_WR_PMPROT(base, SMC_RD_PMPROT(base) |  (value)))
#define SMC_CLR_PMPROT(base, value) (SMC_WR_PMPROT(base, SMC_RD_PMPROT(base) & ~(value)))
#define SMC_TOG_PMPROT(base, value) (SMC_WR_PMPROT(base, SMC_RD_PMPROT(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual SMC_PMPROT bitfields
 */

/*!
 * @name Register SMC_PMPROT, field AVLLS[1] (RW)
 *
 * Provided the appropriate control bits are set up in PMCTRL, this write once
 * bit allows the MCU to enter any very-low-leakage stop mode (VLLSx).
 *
 * Values:
 * - 0b0 - Any VLLSx mode is not allowed
 * - 0b1 - Any VLLSx mode is allowed
 */
/*@{*/
/*! @brief Read current value of the SMC_PMPROT_AVLLS field. */
#define SMC_RD_PMPROT_AVLLS(base) ((SMC_PMPROT_REG(base) & SMC_PMPROT_AVLLS_MASK) >> SMC_PMPROT_AVLLS_SHIFT)
#define SMC_BRD_PMPROT_AVLLS(base) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_AVLLS_SHIFT))

/*! @brief Set the AVLLS field to a new value. */
#define SMC_WR_PMPROT_AVLLS(base, value) (SMC_RMW_PMPROT(base, SMC_PMPROT_AVLLS_MASK, SMC_PMPROT_AVLLS(value)))
#define SMC_BWR_PMPROT_AVLLS(base, value) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_AVLLS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SMC_PMPROT, field ALLS[3] (RW)
 *
 * Provided the appropriate control bits are set up in PMCTRL, this write-once
 * field allows the MCU to enter any low-leakage stop mode (LLS).
 *
 * Values:
 * - 0b0 - Any LLSx mode is not allowed
 * - 0b1 - Any LLSx mode is allowed
 */
/*@{*/
/*! @brief Read current value of the SMC_PMPROT_ALLS field. */
#define SMC_RD_PMPROT_ALLS(base) ((SMC_PMPROT_REG(base) & SMC_PMPROT_ALLS_MASK) >> SMC_PMPROT_ALLS_SHIFT)
#define SMC_BRD_PMPROT_ALLS(base) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_ALLS_SHIFT))

/*! @brief Set the ALLS field to a new value. */
#define SMC_WR_PMPROT_ALLS(base, value) (SMC_RMW_PMPROT(base, SMC_PMPROT_ALLS_MASK, SMC_PMPROT_ALLS(value)))
#define SMC_BWR_PMPROT_ALLS(base, value) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_ALLS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SMC_PMPROT, field AVLP[5] (RW)
 *
 * Provided the appropriate control bits are set up in PMCTRL, this write-once
 * field allows the MCU to enter any very-low-power mode (VLPR, VLPW, and VLPS).
 *
 * Values:
 * - 0b0 - VLPR, VLPW, and VLPS are not allowed.
 * - 0b1 - VLPR, VLPW, and VLPS are allowed.
 */
/*@{*/
/*! @brief Read current value of the SMC_PMPROT_AVLP field. */
#define SMC_RD_PMPROT_AVLP(base) ((SMC_PMPROT_REG(base) & SMC_PMPROT_AVLP_MASK) >> SMC_PMPROT_AVLP_SHIFT)
#define SMC_BRD_PMPROT_AVLP(base) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_AVLP_SHIFT))

/*! @brief Set the AVLP field to a new value. */
#define SMC_WR_PMPROT_AVLP(base, value) (SMC_RMW_PMPROT(base, SMC_PMPROT_AVLP_MASK, SMC_PMPROT_AVLP(value)))
#define SMC_BWR_PMPROT_AVLP(base, value) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_AVLP_SHIFT) = (value))
/*@}*/

/*!
 * @name Register SMC_PMPROT, field AHSRUN[7] (RW)
 *
 * Provided the appropriate control bits are set up in PMCTRL, this write-once
 * field allows the MCU to enter High Speed Run mode (HSRUN).
 *
 * Values:
 * - 0b0 - HSRUN is not allowed
 * - 0b1 - HSRUN is allowed
 */
/*@{*/
/*! @brief Read current value of the SMC_PMPROT_AHSRUN field. */
#define SMC_RD_PMPROT_AHSRUN(base) ((SMC_PMPROT_REG(base) & SMC_PMPROT_AHSRUN_MASK) >> SMC_PMPROT_AHSRUN_SHIFT)
#define SMC_BRD_PMPROT_AHSRUN(base) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_AHSRUN_SHIFT))

/*! @brief Set the AHSRUN field to a new value. */
#define SMC_WR_PMPROT_AHSRUN(base, value) (SMC_RMW_PMPROT(base, SMC_PMPROT_AHSRUN_MASK, SMC_PMPROT_AHSRUN(value)))
#define SMC_BWR_PMPROT_AHSRUN(base, value) (BITBAND_ACCESS8(&SMC_PMPROT_REG(base), SMC_PMPROT_AHSRUN_SHIFT) = (value))
/*@}*/

/*
 * MK22F51212 I2S
 *
 * Inter-IC Sound / Synchronous Audio Interface
 *
 * Registers defined in this header file:
 * - I2S_TCSR - SAI Transmit Control Register
 * - I2S_TCR1 - SAI Transmit Configuration 1 Register
 * - I2S_TCR2 - SAI Transmit Configuration 2 Register
 * - I2S_TCR3 - SAI Transmit Configuration 3 Register
 * - I2S_TCR4 - SAI Transmit Configuration 4 Register
 * - I2S_TCR5 - SAI Transmit Configuration 5 Register
 * - I2S_TDR - SAI Transmit Data Register
 * - I2S_TFR - SAI Transmit FIFO Register
 * - I2S_TMR - SAI Transmit Mask Register
 * - I2S_RCSR - SAI Receive Control Register
 * - I2S_RCR1 - SAI Receive Configuration 1 Register
 * - I2S_RCR2 - SAI Receive Configuration 2 Register
 * - I2S_RCR3 - SAI Receive Configuration 3 Register
 * - I2S_RCR4 - SAI Receive Configuration 4 Register
 * - I2S_RCR5 - SAI Receive Configuration 5 Register
 * - I2S_RDR - SAI Receive Data Register
 * - I2S_RFR - SAI Receive FIFO Register
 * - I2S_RMR - SAI Receive Mask Register
 * - I2S_MCR - SAI MCLK Control Register
 * - I2S_MDR - SAI MCLK Divide Register
 */

#define I2S_INSTANCE_COUNT (1U) /*!< Number of instances of the I2S module. */
#define I2S0_IDX (0U) /*!< Instance number for I2S0. */

/*******************************************************************************
 * I2S_TCSR - SAI Transmit Control Register
 ******************************************************************************/

/*!
 * @brief I2S_TCSR - SAI Transmit Control Register (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire I2S_TCSR register
 */
/*@{*/
#define I2S_RD_TCSR(base)        (I2S_TCSR_REG(base))
#define I2S_WR_TCSR(base, value) (I2S_TCSR_REG(base) = (value))
#define I2S_RMW_TCSR(base, mask, value) (I2S_WR_TCSR(base, (I2S_RD_TCSR(base) & ~(mask)) | (value)))
#define I2S_SET_TCSR(base, value) (I2S_WR_TCSR(base, I2S_RD_TCSR(base) |  (value)))
#define I2S_CLR_TCSR(base, value) (I2S_WR_TCSR(base, I2S_RD_TCSR(base) & ~(value)))
#define I2S_TOG_TCSR(base, value) (I2S_WR_TCSR(base, I2S_RD_TCSR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_TCSR bitfields
 */

/*!
 * @name Register I2S_TCSR, field FRDE[0] (RW)
 *
 * Enables/disables DMA requests.
 *
 * Values:
 * - 0b0 - Disables the DMA request.
 * - 0b1 - Enables the DMA request.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FRDE field. */
#define I2S_RD_TCSR_FRDE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FRDE_MASK) >> I2S_TCSR_FRDE_SHIFT)
#define I2S_BRD_TCSR_FRDE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FRDE_SHIFT))

/*! @brief Set the FRDE field to a new value. */
#define I2S_WR_TCSR_FRDE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_FRDE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_FRDE(value)))
#define I2S_BWR_TCSR_FRDE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FRDE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FWDE[1] (RW)
 *
 * Enables/disables DMA requests.
 *
 * Values:
 * - 0b0 - Disables the DMA request.
 * - 0b1 - Enables the DMA request.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FWDE field. */
#define I2S_RD_TCSR_FWDE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FWDE_MASK) >> I2S_TCSR_FWDE_SHIFT)
#define I2S_BRD_TCSR_FWDE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FWDE_SHIFT))

/*! @brief Set the FWDE field to a new value. */
#define I2S_WR_TCSR_FWDE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_FWDE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_FWDE(value)))
#define I2S_BWR_TCSR_FWDE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FWDE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FRIE[8] (RW)
 *
 * Enables/disables FIFO request interrupts.
 *
 * Values:
 * - 0b0 - Disables the interrupt.
 * - 0b1 - Enables the interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FRIE field. */
#define I2S_RD_TCSR_FRIE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FRIE_MASK) >> I2S_TCSR_FRIE_SHIFT)
#define I2S_BRD_TCSR_FRIE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FRIE_SHIFT))

/*! @brief Set the FRIE field to a new value. */
#define I2S_WR_TCSR_FRIE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_FRIE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_FRIE(value)))
#define I2S_BWR_TCSR_FRIE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FRIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FWIE[9] (RW)
 *
 * Enables/disables FIFO warning interrupts.
 *
 * Values:
 * - 0b0 - Disables the interrupt.
 * - 0b1 - Enables the interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FWIE field. */
#define I2S_RD_TCSR_FWIE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FWIE_MASK) >> I2S_TCSR_FWIE_SHIFT)
#define I2S_BRD_TCSR_FWIE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FWIE_SHIFT))

/*! @brief Set the FWIE field to a new value. */
#define I2S_WR_TCSR_FWIE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_FWIE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_FWIE(value)))
#define I2S_BWR_TCSR_FWIE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FWIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FEIE[10] (RW)
 *
 * Enables/disables FIFO error interrupts.
 *
 * Values:
 * - 0b0 - Disables the interrupt.
 * - 0b1 - Enables the interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FEIE field. */
#define I2S_RD_TCSR_FEIE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FEIE_MASK) >> I2S_TCSR_FEIE_SHIFT)
#define I2S_BRD_TCSR_FEIE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FEIE_SHIFT))

/*! @brief Set the FEIE field to a new value. */
#define I2S_WR_TCSR_FEIE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_FEIE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_FEIE(value)))
#define I2S_BWR_TCSR_FEIE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FEIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field SEIE[11] (RW)
 *
 * Enables/disables sync error interrupts.
 *
 * Values:
 * - 0b0 - Disables interrupt.
 * - 0b1 - Enables interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_SEIE field. */
#define I2S_RD_TCSR_SEIE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_SEIE_MASK) >> I2S_TCSR_SEIE_SHIFT)
#define I2S_BRD_TCSR_SEIE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_SEIE_SHIFT))

/*! @brief Set the SEIE field to a new value. */
#define I2S_WR_TCSR_SEIE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_SEIE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_SEIE(value)))
#define I2S_BWR_TCSR_SEIE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_SEIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field WSIE[12] (RW)
 *
 * Enables/disables word start interrupts.
 *
 * Values:
 * - 0b0 - Disables interrupt.
 * - 0b1 - Enables interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_WSIE field. */
#define I2S_RD_TCSR_WSIE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_WSIE_MASK) >> I2S_TCSR_WSIE_SHIFT)
#define I2S_BRD_TCSR_WSIE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_WSIE_SHIFT))

/*! @brief Set the WSIE field to a new value. */
#define I2S_WR_TCSR_WSIE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_WSIE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_WSIE(value)))
#define I2S_BWR_TCSR_WSIE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_WSIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FRF[16] (RO)
 *
 * Indicates that the number of words in an enabled transmit channel FIFO is
 * less than or equal to the transmit FIFO watermark.
 *
 * Values:
 * - 0b0 - Transmit FIFO watermark has not been reached.
 * - 0b1 - Transmit FIFO watermark has been reached.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FRF field. */
#define I2S_RD_TCSR_FRF(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FRF_MASK) >> I2S_TCSR_FRF_SHIFT)
#define I2S_BRD_TCSR_FRF(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FRF_SHIFT))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FWF[17] (RO)
 *
 * Indicates that an enabled transmit FIFO is empty.
 *
 * Values:
 * - 0b0 - No enabled transmit FIFO is empty.
 * - 0b1 - Enabled transmit FIFO is empty.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FWF field. */
#define I2S_RD_TCSR_FWF(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FWF_MASK) >> I2S_TCSR_FWF_SHIFT)
#define I2S_BRD_TCSR_FWF(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FWF_SHIFT))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FEF[18] (W1C)
 *
 * Indicates that an enabled transmit FIFO has underrun. Write a logic 1 to this
 * field to clear this flag.
 *
 * Values:
 * - 0b0 - Transmit underrun not detected.
 * - 0b1 - Transmit underrun detected.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_FEF field. */
#define I2S_RD_TCSR_FEF(base) ((I2S_TCSR_REG(base) & I2S_TCSR_FEF_MASK) >> I2S_TCSR_FEF_SHIFT)
#define I2S_BRD_TCSR_FEF(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FEF_SHIFT))

/*! @brief Set the FEF field to a new value. */
#define I2S_WR_TCSR_FEF(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_FEF(value)))
#define I2S_BWR_TCSR_FEF(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FEF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field SEF[19] (W1C)
 *
 * Indicates that an error in the externally-generated frame sync has been
 * detected. Write a logic 1 to this field to clear this flag.
 *
 * Values:
 * - 0b0 - Sync error not detected.
 * - 0b1 - Frame sync error detected.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_SEF field. */
#define I2S_RD_TCSR_SEF(base) ((I2S_TCSR_REG(base) & I2S_TCSR_SEF_MASK) >> I2S_TCSR_SEF_SHIFT)
#define I2S_BRD_TCSR_SEF(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_SEF_SHIFT))

/*! @brief Set the SEF field to a new value. */
#define I2S_WR_TCSR_SEF(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_SEF_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_SEF(value)))
#define I2S_BWR_TCSR_SEF(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_SEF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field WSF[20] (W1C)
 *
 * Indicates that the start of the configured word has been detected. Write a
 * logic 1 to this field to clear this flag.
 *
 * Values:
 * - 0b0 - Start of word not detected.
 * - 0b1 - Start of word detected.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_WSF field. */
#define I2S_RD_TCSR_WSF(base) ((I2S_TCSR_REG(base) & I2S_TCSR_WSF_MASK) >> I2S_TCSR_WSF_SHIFT)
#define I2S_BRD_TCSR_WSF(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_WSF_SHIFT))

/*! @brief Set the WSF field to a new value. */
#define I2S_WR_TCSR_WSF(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_WSF_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK), I2S_TCSR_WSF(value)))
#define I2S_BWR_TCSR_WSF(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_WSF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field SR[24] (RW)
 *
 * When set, resets the internal transmitter logic including the FIFO pointers.
 * Software-visible registers are not affected, except for the status registers.
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - Software reset.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_SR field. */
#define I2S_RD_TCSR_SR(base) ((I2S_TCSR_REG(base) & I2S_TCSR_SR_MASK) >> I2S_TCSR_SR_SHIFT)
#define I2S_BRD_TCSR_SR(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_SR_SHIFT))

/*! @brief Set the SR field to a new value. */
#define I2S_WR_TCSR_SR(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_SR_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_SR(value)))
#define I2S_BWR_TCSR_SR(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_SR_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field FR[25] (WORZ)
 *
 * Resets the FIFO pointers. Reading this field will always return zero. FIFO
 * pointers should only be reset when the transmitter is disabled or the FIFO error
 * flag is set.
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - FIFO reset.
 */
/*@{*/
/*! @brief Set the FR field to a new value. */
#define I2S_WR_TCSR_FR(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_FR_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_FR(value)))
#define I2S_BWR_TCSR_FR(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_FR_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field BCE[28] (RW)
 *
 * Enables the transmit bit clock, separately from the TE. This field is
 * automatically set whenever TE is set. When software clears this field, the transmit
 * bit clock remains enabled, and this bit remains set, until the end of the
 * current frame.
 *
 * Values:
 * - 0b0 - Transmit bit clock is disabled.
 * - 0b1 - Transmit bit clock is enabled.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_BCE field. */
#define I2S_RD_TCSR_BCE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_BCE_MASK) >> I2S_TCSR_BCE_SHIFT)
#define I2S_BRD_TCSR_BCE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_BCE_SHIFT))

/*! @brief Set the BCE field to a new value. */
#define I2S_WR_TCSR_BCE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_BCE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_BCE(value)))
#define I2S_BWR_TCSR_BCE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_BCE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field DBGE[29] (RW)
 *
 * Enables/disables transmitter operation in Debug mode. The transmit bit clock
 * is not affected by debug mode.
 *
 * Values:
 * - 0b0 - Transmitter is disabled in Debug mode, after completing the current
 *     frame.
 * - 0b1 - Transmitter is enabled in Debug mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_DBGE field. */
#define I2S_RD_TCSR_DBGE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_DBGE_MASK) >> I2S_TCSR_DBGE_SHIFT)
#define I2S_BRD_TCSR_DBGE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_DBGE_SHIFT))

/*! @brief Set the DBGE field to a new value. */
#define I2S_WR_TCSR_DBGE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_DBGE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_DBGE(value)))
#define I2S_BWR_TCSR_DBGE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_DBGE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field STOPE[30] (RW)
 *
 * Configures transmitter operation in Stop mode. This field is ignored and the
 * transmitter is disabled in all low-leakage stop modes.
 *
 * Values:
 * - 0b0 - Transmitter disabled in Stop mode.
 * - 0b1 - Transmitter enabled in Stop mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_STOPE field. */
#define I2S_RD_TCSR_STOPE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_STOPE_MASK) >> I2S_TCSR_STOPE_SHIFT)
#define I2S_BRD_TCSR_STOPE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_STOPE_SHIFT))

/*! @brief Set the STOPE field to a new value. */
#define I2S_WR_TCSR_STOPE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_STOPE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_STOPE(value)))
#define I2S_BWR_TCSR_STOPE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_STOPE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCSR, field TE[31] (RW)
 *
 * Enables/disables the transmitter. When software clears this field, the
 * transmitter remains enabled, and this bit remains set, until the end of the current
 * frame.
 *
 * Values:
 * - 0b0 - Transmitter is disabled.
 * - 0b1 - Transmitter is enabled, or transmitter has been disabled and has not
 *     yet reached end of frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCSR_TE field. */
#define I2S_RD_TCSR_TE(base) ((I2S_TCSR_REG(base) & I2S_TCSR_TE_MASK) >> I2S_TCSR_TE_SHIFT)
#define I2S_BRD_TCSR_TE(base) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_TE_SHIFT))

/*! @brief Set the TE field to a new value. */
#define I2S_WR_TCSR_TE(base, value) (I2S_RMW_TCSR(base, (I2S_TCSR_TE_MASK | I2S_TCSR_FEF_MASK | I2S_TCSR_SEF_MASK | I2S_TCSR_WSF_MASK), I2S_TCSR_TE(value)))
#define I2S_BWR_TCSR_TE(base, value) (BITBAND_ACCESS32(&I2S_TCSR_REG(base), I2S_TCSR_TE_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * I2S_TCR1 - SAI Transmit Configuration 1 Register
 ******************************************************************************/

/*!
 * @brief I2S_TCR1 - SAI Transmit Configuration 1 Register (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire I2S_TCR1 register
 */
/*@{*/
#define I2S_RD_TCR1(base)        (I2S_TCR1_REG(base))
#define I2S_WR_TCR1(base, value) (I2S_TCR1_REG(base) = (value))
#define I2S_RMW_TCR1(base, mask, value) (I2S_WR_TCR1(base, (I2S_RD_TCR1(base) & ~(mask)) | (value)))
#define I2S_SET_TCR1(base, value) (I2S_WR_TCR1(base, I2S_RD_TCR1(base) |  (value)))
#define I2S_CLR_TCR1(base, value) (I2S_WR_TCR1(base, I2S_RD_TCR1(base) & ~(value)))
#define I2S_TOG_TCR1(base, value) (I2S_WR_TCR1(base, I2S_RD_TCR1(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_TCR1 bitfields
 */

/*!
 * @name Register I2S_TCR1, field TFW[2:0] (RW)
 *
 * Configures the watermark level for all enabled transmit channels.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR1_TFW field. */
#define I2S_RD_TCR1_TFW(base) ((I2S_TCR1_REG(base) & I2S_TCR1_TFW_MASK) >> I2S_TCR1_TFW_SHIFT)
#define I2S_BRD_TCR1_TFW(base) (I2S_RD_TCR1_TFW(base))

/*! @brief Set the TFW field to a new value. */
#define I2S_WR_TCR1_TFW(base, value) (I2S_RMW_TCR1(base, I2S_TCR1_TFW_MASK, I2S_TCR1_TFW(value)))
#define I2S_BWR_TCR1_TFW(base, value) (I2S_WR_TCR1_TFW(base, value))
/*@}*/

/*******************************************************************************
 * I2S_TCR2 - SAI Transmit Configuration 2 Register
 ******************************************************************************/

/*!
 * @brief I2S_TCR2 - SAI Transmit Configuration 2 Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register must not be altered when TCSR[TE] is set.
 */
/*!
 * @name Constants and macros for entire I2S_TCR2 register
 */
/*@{*/
#define I2S_RD_TCR2(base)        (I2S_TCR2_REG(base))
#define I2S_WR_TCR2(base, value) (I2S_TCR2_REG(base) = (value))
#define I2S_RMW_TCR2(base, mask, value) (I2S_WR_TCR2(base, (I2S_RD_TCR2(base) & ~(mask)) | (value)))
#define I2S_SET_TCR2(base, value) (I2S_WR_TCR2(base, I2S_RD_TCR2(base) |  (value)))
#define I2S_CLR_TCR2(base, value) (I2S_WR_TCR2(base, I2S_RD_TCR2(base) & ~(value)))
#define I2S_TOG_TCR2(base, value) (I2S_WR_TCR2(base, I2S_RD_TCR2(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_TCR2 bitfields
 */

/*!
 * @name Register I2S_TCR2, field DIV[7:0] (RW)
 *
 * Divides down the audio master clock to generate the bit clock when configured
 * for an internal bit clock. The division value is (DIV + 1) * 2.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR2_DIV field. */
#define I2S_RD_TCR2_DIV(base) ((I2S_TCR2_REG(base) & I2S_TCR2_DIV_MASK) >> I2S_TCR2_DIV_SHIFT)
#define I2S_BRD_TCR2_DIV(base) (I2S_RD_TCR2_DIV(base))

/*! @brief Set the DIV field to a new value. */
#define I2S_WR_TCR2_DIV(base, value) (I2S_RMW_TCR2(base, I2S_TCR2_DIV_MASK, I2S_TCR2_DIV(value)))
#define I2S_BWR_TCR2_DIV(base, value) (I2S_WR_TCR2_DIV(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR2, field BCD[24] (RW)
 *
 * Configures the direction of the bit clock.
 *
 * Values:
 * - 0b0 - Bit clock is generated externally in Slave mode.
 * - 0b1 - Bit clock is generated internally in Master mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR2_BCD field. */
#define I2S_RD_TCR2_BCD(base) ((I2S_TCR2_REG(base) & I2S_TCR2_BCD_MASK) >> I2S_TCR2_BCD_SHIFT)
#define I2S_BRD_TCR2_BCD(base) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCD_SHIFT))

/*! @brief Set the BCD field to a new value. */
#define I2S_WR_TCR2_BCD(base, value) (I2S_RMW_TCR2(base, I2S_TCR2_BCD_MASK, I2S_TCR2_BCD(value)))
#define I2S_BWR_TCR2_BCD(base, value) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCD_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR2, field BCP[25] (RW)
 *
 * Configures the polarity of the bit clock.
 *
 * Values:
 * - 0b0 - Bit clock is active high with drive outputs on rising edge and sample
 *     inputs on falling edge.
 * - 0b1 - Bit clock is active low with drive outputs on falling edge and sample
 *     inputs on rising edge.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR2_BCP field. */
#define I2S_RD_TCR2_BCP(base) ((I2S_TCR2_REG(base) & I2S_TCR2_BCP_MASK) >> I2S_TCR2_BCP_SHIFT)
#define I2S_BRD_TCR2_BCP(base) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCP_SHIFT))

/*! @brief Set the BCP field to a new value. */
#define I2S_WR_TCR2_BCP(base, value) (I2S_RMW_TCR2(base, I2S_TCR2_BCP_MASK, I2S_TCR2_BCP(value)))
#define I2S_BWR_TCR2_BCP(base, value) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCP_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR2, field MSEL[27:26] (RW)
 *
 * Selects the audio Master Clock option used to generate an internally
 * generated bit clock. This field has no effect when configured for an externally
 * generated bit clock. Depending on the device, some Master Clock options might not be
 * available. See the chip configuration details for the availability and
 * chip-specific meaning of each option.
 *
 * Values:
 * - 0b00 - Bus Clock selected.
 * - 0b01 - Master Clock (MCLK) 1 option selected.
 * - 0b10 - Master Clock (MCLK) 2 option selected.
 * - 0b11 - Master Clock (MCLK) 3 option selected.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR2_MSEL field. */
#define I2S_RD_TCR2_MSEL(base) ((I2S_TCR2_REG(base) & I2S_TCR2_MSEL_MASK) >> I2S_TCR2_MSEL_SHIFT)
#define I2S_BRD_TCR2_MSEL(base) (I2S_RD_TCR2_MSEL(base))

/*! @brief Set the MSEL field to a new value. */
#define I2S_WR_TCR2_MSEL(base, value) (I2S_RMW_TCR2(base, I2S_TCR2_MSEL_MASK, I2S_TCR2_MSEL(value)))
#define I2S_BWR_TCR2_MSEL(base, value) (I2S_WR_TCR2_MSEL(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR2, field BCI[28] (RW)
 *
 * When this field is set and using an internally generated bit clock in either
 * synchronous or asynchronous mode, the bit clock actually used by the
 * transmitter is delayed by the pad output delay (the transmitter is clocked by the pad
 * input as if the clock was externally generated). This has the effect of
 * decreasing the data input setup time, but increasing the data output valid time. The
 * slave mode timing from the datasheet should be used for the transmitter when
 * this bit is set. In synchronous mode, this bit allows the transmitter to use
 * the slave mode timing from the datasheet, while the receiver uses the master
 * mode timing. This field has no effect when configured for an externally generated
 * bit clock .
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - Internal logic is clocked as if bit clock was externally generated.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR2_BCI field. */
#define I2S_RD_TCR2_BCI(base) ((I2S_TCR2_REG(base) & I2S_TCR2_BCI_MASK) >> I2S_TCR2_BCI_SHIFT)
#define I2S_BRD_TCR2_BCI(base) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCI_SHIFT))

/*! @brief Set the BCI field to a new value. */
#define I2S_WR_TCR2_BCI(base, value) (I2S_RMW_TCR2(base, I2S_TCR2_BCI_MASK, I2S_TCR2_BCI(value)))
#define I2S_BWR_TCR2_BCI(base, value) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCI_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR2, field BCS[29] (RW)
 *
 * This field swaps the bit clock used by the transmitter. When the transmitter
 * is configured in asynchronous mode and this bit is set, the transmitter is
 * clocked by the receiver bit clock (SAI_RX_BCLK). This allows the transmitter and
 * receiver to share the same bit clock, but the transmitter continues to use the
 * transmit frame sync (SAI_TX_SYNC). When the transmitter is configured in
 * synchronous mode, the transmitter BCS field and receiver BCS field must be set to
 * the same value. When both are set, the transmitter and receiver are both
 * clocked by the transmitter bit clock (SAI_TX_BCLK) but use the receiver frame sync
 * (SAI_RX_SYNC).
 *
 * Values:
 * - 0b0 - Use the normal bit clock source.
 * - 0b1 - Swap the bit clock source.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR2_BCS field. */
#define I2S_RD_TCR2_BCS(base) ((I2S_TCR2_REG(base) & I2S_TCR2_BCS_MASK) >> I2S_TCR2_BCS_SHIFT)
#define I2S_BRD_TCR2_BCS(base) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCS_SHIFT))

/*! @brief Set the BCS field to a new value. */
#define I2S_WR_TCR2_BCS(base, value) (I2S_RMW_TCR2(base, I2S_TCR2_BCS_MASK, I2S_TCR2_BCS(value)))
#define I2S_BWR_TCR2_BCS(base, value) (BITBAND_ACCESS32(&I2S_TCR2_REG(base), I2S_TCR2_BCS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR2, field SYNC[31:30] (RW)
 *
 * Configures between asynchronous and synchronous modes of operation. When
 * configured for a synchronous mode of operation, the receiver must be configured
 * for asynchronous operation.
 *
 * Values:
 * - 0b00 - Asynchronous mode.
 * - 0b01 - Synchronous with receiver.
 * - 0b10 - Synchronous with another SAI transmitter.
 * - 0b11 - Synchronous with another SAI receiver.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR2_SYNC field. */
#define I2S_RD_TCR2_SYNC(base) ((I2S_TCR2_REG(base) & I2S_TCR2_SYNC_MASK) >> I2S_TCR2_SYNC_SHIFT)
#define I2S_BRD_TCR2_SYNC(base) (I2S_RD_TCR2_SYNC(base))

/*! @brief Set the SYNC field to a new value. */
#define I2S_WR_TCR2_SYNC(base, value) (I2S_RMW_TCR2(base, I2S_TCR2_SYNC_MASK, I2S_TCR2_SYNC(value)))
#define I2S_BWR_TCR2_SYNC(base, value) (I2S_WR_TCR2_SYNC(base, value))
/*@}*/

/*******************************************************************************
 * I2S_TCR3 - SAI Transmit Configuration 3 Register
 ******************************************************************************/

/*!
 * @brief I2S_TCR3 - SAI Transmit Configuration 3 Register (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire I2S_TCR3 register
 */
/*@{*/
#define I2S_RD_TCR3(base)        (I2S_TCR3_REG(base))
#define I2S_WR_TCR3(base, value) (I2S_TCR3_REG(base) = (value))
#define I2S_RMW_TCR3(base, mask, value) (I2S_WR_TCR3(base, (I2S_RD_TCR3(base) & ~(mask)) | (value)))
#define I2S_SET_TCR3(base, value) (I2S_WR_TCR3(base, I2S_RD_TCR3(base) |  (value)))
#define I2S_CLR_TCR3(base, value) (I2S_WR_TCR3(base, I2S_RD_TCR3(base) & ~(value)))
#define I2S_TOG_TCR3(base, value) (I2S_WR_TCR3(base, I2S_RD_TCR3(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_TCR3 bitfields
 */

/*!
 * @name Register I2S_TCR3, field WDFL[3:0] (RW)
 *
 * Configures which word sets the start of word flag. The value written must be
 * one less than the word number. For example, writing 0 configures the first
 * word in the frame. When configured to a value greater than TCR4[FRSZ], then the
 * start of word flag is never set.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR3_WDFL field. */
#define I2S_RD_TCR3_WDFL(base) ((I2S_TCR3_REG(base) & I2S_TCR3_WDFL_MASK) >> I2S_TCR3_WDFL_SHIFT)
#define I2S_BRD_TCR3_WDFL(base) (I2S_RD_TCR3_WDFL(base))

/*! @brief Set the WDFL field to a new value. */
#define I2S_WR_TCR3_WDFL(base, value) (I2S_RMW_TCR3(base, I2S_TCR3_WDFL_MASK, I2S_TCR3_WDFL(value)))
#define I2S_BWR_TCR3_WDFL(base, value) (I2S_WR_TCR3_WDFL(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR3, field TCE[16] (RW)
 *
 * Enables the corresponding data channel for transmit operation. A channel must
 * be enabled before its FIFO is accessed. Changing this field will take effect
 * immediately for generating the FIFO request and warning flags, but at the end
 * of each frame for transmit operation.
 *
 * Values:
 * - 0b0 - Transmit data channel N is disabled.
 * - 0b1 - Transmit data channel N is enabled.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR3_TCE field. */
#define I2S_RD_TCR3_TCE(base) ((I2S_TCR3_REG(base) & I2S_TCR3_TCE_MASK) >> I2S_TCR3_TCE_SHIFT)
#define I2S_BRD_TCR3_TCE(base) (BITBAND_ACCESS32(&I2S_TCR3_REG(base), I2S_TCR3_TCE_SHIFT))

/*! @brief Set the TCE field to a new value. */
#define I2S_WR_TCR3_TCE(base, value) (I2S_RMW_TCR3(base, I2S_TCR3_TCE_MASK, I2S_TCR3_TCE(value)))
#define I2S_BWR_TCR3_TCE(base, value) (BITBAND_ACCESS32(&I2S_TCR3_REG(base), I2S_TCR3_TCE_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * I2S_TCR4 - SAI Transmit Configuration 4 Register
 ******************************************************************************/

/*!
 * @brief I2S_TCR4 - SAI Transmit Configuration 4 Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register must not be altered when TCSR[TE] is set.
 */
/*!
 * @name Constants and macros for entire I2S_TCR4 register
 */
/*@{*/
#define I2S_RD_TCR4(base)        (I2S_TCR4_REG(base))
#define I2S_WR_TCR4(base, value) (I2S_TCR4_REG(base) = (value))
#define I2S_RMW_TCR4(base, mask, value) (I2S_WR_TCR4(base, (I2S_RD_TCR4(base) & ~(mask)) | (value)))
#define I2S_SET_TCR4(base, value) (I2S_WR_TCR4(base, I2S_RD_TCR4(base) |  (value)))
#define I2S_CLR_TCR4(base, value) (I2S_WR_TCR4(base, I2S_RD_TCR4(base) & ~(value)))
#define I2S_TOG_TCR4(base, value) (I2S_WR_TCR4(base, I2S_RD_TCR4(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_TCR4 bitfields
 */

/*!
 * @name Register I2S_TCR4, field FSD[0] (RW)
 *
 * Configures the direction of the frame sync.
 *
 * Values:
 * - 0b0 - Frame sync is generated externally in Slave mode.
 * - 0b1 - Frame sync is generated internally in Master mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_FSD field. */
#define I2S_RD_TCR4_FSD(base) ((I2S_TCR4_REG(base) & I2S_TCR4_FSD_MASK) >> I2S_TCR4_FSD_SHIFT)
#define I2S_BRD_TCR4_FSD(base) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FSD_SHIFT))

/*! @brief Set the FSD field to a new value. */
#define I2S_WR_TCR4_FSD(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_FSD_MASK, I2S_TCR4_FSD(value)))
#define I2S_BWR_TCR4_FSD(base, value) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FSD_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field FSP[1] (RW)
 *
 * Configures the polarity of the frame sync.
 *
 * Values:
 * - 0b0 - Frame sync is active high.
 * - 0b1 - Frame sync is active low.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_FSP field. */
#define I2S_RD_TCR4_FSP(base) ((I2S_TCR4_REG(base) & I2S_TCR4_FSP_MASK) >> I2S_TCR4_FSP_SHIFT)
#define I2S_BRD_TCR4_FSP(base) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FSP_SHIFT))

/*! @brief Set the FSP field to a new value. */
#define I2S_WR_TCR4_FSP(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_FSP_MASK, I2S_TCR4_FSP(value)))
#define I2S_BWR_TCR4_FSP(base, value) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FSP_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field ONDEM[2] (RW)
 *
 * When set, and the frame sync is generated internally, a frame sync is only
 * generated when the FIFO warning flag is clear.
 *
 * Values:
 * - 0b0 - Internal frame sync is generated continuously.
 * - 0b1 - Internal frame sync is generated when the FIFO warning flag is clear.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_ONDEM field. */
#define I2S_RD_TCR4_ONDEM(base) ((I2S_TCR4_REG(base) & I2S_TCR4_ONDEM_MASK) >> I2S_TCR4_ONDEM_SHIFT)
#define I2S_BRD_TCR4_ONDEM(base) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_ONDEM_SHIFT))

/*! @brief Set the ONDEM field to a new value. */
#define I2S_WR_TCR4_ONDEM(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_ONDEM_MASK, I2S_TCR4_ONDEM(value)))
#define I2S_BWR_TCR4_ONDEM(base, value) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_ONDEM_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field FSE[3] (RW)
 *
 * Values:
 * - 0b0 - Frame sync asserts with the first bit of the frame.
 * - 0b1 - Frame sync asserts one bit before the first bit of the frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_FSE field. */
#define I2S_RD_TCR4_FSE(base) ((I2S_TCR4_REG(base) & I2S_TCR4_FSE_MASK) >> I2S_TCR4_FSE_SHIFT)
#define I2S_BRD_TCR4_FSE(base) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FSE_SHIFT))

/*! @brief Set the FSE field to a new value. */
#define I2S_WR_TCR4_FSE(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_FSE_MASK, I2S_TCR4_FSE(value)))
#define I2S_BWR_TCR4_FSE(base, value) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FSE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field MF[4] (RW)
 *
 * Configures whether the LSB or the MSB is transmitted first.
 *
 * Values:
 * - 0b0 - LSB is transmitted first.
 * - 0b1 - MSB is transmitted first.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_MF field. */
#define I2S_RD_TCR4_MF(base) ((I2S_TCR4_REG(base) & I2S_TCR4_MF_MASK) >> I2S_TCR4_MF_SHIFT)
#define I2S_BRD_TCR4_MF(base) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_MF_SHIFT))

/*! @brief Set the MF field to a new value. */
#define I2S_WR_TCR4_MF(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_MF_MASK, I2S_TCR4_MF(value)))
#define I2S_BWR_TCR4_MF(base, value) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_MF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field SYWD[12:8] (RW)
 *
 * Configures the length of the frame sync in number of bit clocks. The value
 * written must be one less than the number of bit clocks. For example, write 0 for
 * the frame sync to assert for one bit clock only. The sync width cannot be
 * configured longer than the first word of the frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_SYWD field. */
#define I2S_RD_TCR4_SYWD(base) ((I2S_TCR4_REG(base) & I2S_TCR4_SYWD_MASK) >> I2S_TCR4_SYWD_SHIFT)
#define I2S_BRD_TCR4_SYWD(base) (I2S_RD_TCR4_SYWD(base))

/*! @brief Set the SYWD field to a new value. */
#define I2S_WR_TCR4_SYWD(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_SYWD_MASK, I2S_TCR4_SYWD(value)))
#define I2S_BWR_TCR4_SYWD(base, value) (I2S_WR_TCR4_SYWD(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field FRSZ[19:16] (RW)
 *
 * Configures the number of words in each frame. The value written must be one
 * less than the number of words in the frame. For example, write 0 for one word
 * per frame. The maximum supported frame size is 16 words.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_FRSZ field. */
#define I2S_RD_TCR4_FRSZ(base) ((I2S_TCR4_REG(base) & I2S_TCR4_FRSZ_MASK) >> I2S_TCR4_FRSZ_SHIFT)
#define I2S_BRD_TCR4_FRSZ(base) (I2S_RD_TCR4_FRSZ(base))

/*! @brief Set the FRSZ field to a new value. */
#define I2S_WR_TCR4_FRSZ(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_FRSZ_MASK, I2S_TCR4_FRSZ(value)))
#define I2S_BWR_TCR4_FRSZ(base, value) (I2S_WR_TCR4_FRSZ(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field FPACK[25:24] (RW)
 *
 * Enables packing of 8-bit data or 16-bit data into each 32-bit FIFO word. If
 * the word size is greater than 8-bit or 16-bit then only the first 8-bit or
 * 16-bits are loaded from the FIFO. The first word in each frame always starts with
 * a new 32-bit FIFO word and the first bit shifted must be configured within the
 * first packed word. When FIFO packing is enabled, the FIFO write pointer will
 * only increment when the full 32-bit FIFO word has been written by software.
 *
 * Values:
 * - 0b00 - FIFO packing is disabled
 * - 0b01 - Reserved
 * - 0b10 - 8-bit FIFO packing is enabled
 * - 0b11 - 16-bit FIFO packing is enabled
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_FPACK field. */
#define I2S_RD_TCR4_FPACK(base) ((I2S_TCR4_REG(base) & I2S_TCR4_FPACK_MASK) >> I2S_TCR4_FPACK_SHIFT)
#define I2S_BRD_TCR4_FPACK(base) (I2S_RD_TCR4_FPACK(base))

/*! @brief Set the FPACK field to a new value. */
#define I2S_WR_TCR4_FPACK(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_FPACK_MASK, I2S_TCR4_FPACK(value)))
#define I2S_BWR_TCR4_FPACK(base, value) (I2S_WR_TCR4_FPACK(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR4, field FCONT[28] (RW)
 *
 * Configures when the SAI will continue transmitting after a FIFO error has
 * been detected.
 *
 * Values:
 * - 0b0 - On FIFO error, the SAI will continue from the start of the next frame
 *     after the FIFO error flag has been cleared.
 * - 0b1 - On FIFO error, the SAI will continue from the same word that caused
 *     the FIFO error to set after the FIFO warning flag has been cleared.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR4_FCONT field. */
#define I2S_RD_TCR4_FCONT(base) ((I2S_TCR4_REG(base) & I2S_TCR4_FCONT_MASK) >> I2S_TCR4_FCONT_SHIFT)
#define I2S_BRD_TCR4_FCONT(base) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FCONT_SHIFT))

/*! @brief Set the FCONT field to a new value. */
#define I2S_WR_TCR4_FCONT(base, value) (I2S_RMW_TCR4(base, I2S_TCR4_FCONT_MASK, I2S_TCR4_FCONT(value)))
#define I2S_BWR_TCR4_FCONT(base, value) (BITBAND_ACCESS32(&I2S_TCR4_REG(base), I2S_TCR4_FCONT_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * I2S_TCR5 - SAI Transmit Configuration 5 Register
 ******************************************************************************/

/*!
 * @brief I2S_TCR5 - SAI Transmit Configuration 5 Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register must not be altered when TCSR[TE] is set.
 */
/*!
 * @name Constants and macros for entire I2S_TCR5 register
 */
/*@{*/
#define I2S_RD_TCR5(base)        (I2S_TCR5_REG(base))
#define I2S_WR_TCR5(base, value) (I2S_TCR5_REG(base) = (value))
#define I2S_RMW_TCR5(base, mask, value) (I2S_WR_TCR5(base, (I2S_RD_TCR5(base) & ~(mask)) | (value)))
#define I2S_SET_TCR5(base, value) (I2S_WR_TCR5(base, I2S_RD_TCR5(base) |  (value)))
#define I2S_CLR_TCR5(base, value) (I2S_WR_TCR5(base, I2S_RD_TCR5(base) & ~(value)))
#define I2S_TOG_TCR5(base, value) (I2S_WR_TCR5(base, I2S_RD_TCR5(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_TCR5 bitfields
 */

/*!
 * @name Register I2S_TCR5, field FBT[12:8] (RW)
 *
 * Configures the bit index for the first bit transmitted for each word in the
 * frame. If configured for MSB First, the index of the next bit transmitted is
 * one less than the current bit transmitted. If configured for LSB First, the
 * index of the next bit transmitted is one more than the current bit transmitted.
 * The value written must be greater than or equal to the word width when
 * configured for MSB First. The value written must be less than or equal to 31-word width
 * when configured for LSB First.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR5_FBT field. */
#define I2S_RD_TCR5_FBT(base) ((I2S_TCR5_REG(base) & I2S_TCR5_FBT_MASK) >> I2S_TCR5_FBT_SHIFT)
#define I2S_BRD_TCR5_FBT(base) (I2S_RD_TCR5_FBT(base))

/*! @brief Set the FBT field to a new value. */
#define I2S_WR_TCR5_FBT(base, value) (I2S_RMW_TCR5(base, I2S_TCR5_FBT_MASK, I2S_TCR5_FBT(value)))
#define I2S_BWR_TCR5_FBT(base, value) (I2S_WR_TCR5_FBT(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR5, field W0W[20:16] (RW)
 *
 * Configures the number of bits in the first word in each frame. The value
 * written must be one less than the number of bits in the first word. Word width of
 * less than 8 bits is not supported if there is only one word per frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR5_W0W field. */
#define I2S_RD_TCR5_W0W(base) ((I2S_TCR5_REG(base) & I2S_TCR5_W0W_MASK) >> I2S_TCR5_W0W_SHIFT)
#define I2S_BRD_TCR5_W0W(base) (I2S_RD_TCR5_W0W(base))

/*! @brief Set the W0W field to a new value. */
#define I2S_WR_TCR5_W0W(base, value) (I2S_RMW_TCR5(base, I2S_TCR5_W0W_MASK, I2S_TCR5_W0W(value)))
#define I2S_BWR_TCR5_W0W(base, value) (I2S_WR_TCR5_W0W(base, value))
/*@}*/

/*!
 * @name Register I2S_TCR5, field WNW[28:24] (RW)
 *
 * Configures the number of bits in each word, for each word except the first in
 * the frame. The value written must be one less than the number of bits per
 * word. Word width of less than 8 bits is not supported.
 */
/*@{*/
/*! @brief Read current value of the I2S_TCR5_WNW field. */
#define I2S_RD_TCR5_WNW(base) ((I2S_TCR5_REG(base) & I2S_TCR5_WNW_MASK) >> I2S_TCR5_WNW_SHIFT)
#define I2S_BRD_TCR5_WNW(base) (I2S_RD_TCR5_WNW(base))

/*! @brief Set the WNW field to a new value. */
#define I2S_WR_TCR5_WNW(base, value) (I2S_RMW_TCR5(base, I2S_TCR5_WNW_MASK, I2S_TCR5_WNW(value)))
#define I2S_BWR_TCR5_WNW(base, value) (I2S_WR_TCR5_WNW(base, value))
/*@}*/

/*******************************************************************************
 * I2S_TDR - SAI Transmit Data Register
 ******************************************************************************/

/*!
 * @brief I2S_TDR - SAI Transmit Data Register (WORZ)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire I2S_TDR register
 */
/*@{*/
#define I2S_RD_TDR(base, index)  (I2S_TDR_REG(base, index))
#define I2S_WR_TDR(base, index, value) (I2S_TDR_REG(base, index) = (value))
#define I2S_RMW_TDR(base, index, mask, value) (I2S_WR_TDR(base, index, (I2S_RD_TDR(base, index) & ~(mask)) | (value)))
/*@}*/

/*******************************************************************************
 * I2S_TFR - SAI Transmit FIFO Register
 ******************************************************************************/

/*!
 * @brief I2S_TFR - SAI Transmit FIFO Register (RO)
 *
 * Reset value: 0x00000000U
 *
 * The MSB of the read and write pointers is used to distinguish between FIFO
 * full and empty conditions. If the read and write pointers are identical, then
 * the FIFO is empty. If the read and write pointers are identical except for the
 * MSB, then the FIFO is full.
 */
/*!
 * @name Constants and macros for entire I2S_TFR register
 */
/*@{*/
#define I2S_RD_TFR(base, index)  (I2S_TFR_REG(base, index))
/*@}*/

/*
 * Constants & macros for individual I2S_TFR bitfields
 */

/*!
 * @name Register I2S_TFR, field RFP[3:0] (RO)
 *
 * FIFO read pointer for transmit data channel.
 */
/*@{*/
/*! @brief Read current value of the I2S_TFR_RFP field. */
#define I2S_RD_TFR_RFP(base, index) ((I2S_TFR_REG(base, index) & I2S_TFR_RFP_MASK) >> I2S_TFR_RFP_SHIFT)
#define I2S_BRD_TFR_RFP(base, index) (I2S_RD_TFR_RFP(base, index))
/*@}*/

/*!
 * @name Register I2S_TFR, field WFP[19:16] (RO)
 *
 * FIFO write pointer for transmit data channel.
 */
/*@{*/
/*! @brief Read current value of the I2S_TFR_WFP field. */
#define I2S_RD_TFR_WFP(base, index) ((I2S_TFR_REG(base, index) & I2S_TFR_WFP_MASK) >> I2S_TFR_WFP_SHIFT)
#define I2S_BRD_TFR_WFP(base, index) (I2S_RD_TFR_WFP(base, index))
/*@}*/

/*******************************************************************************
 * I2S_TMR - SAI Transmit Mask Register
 ******************************************************************************/

/*!
 * @brief I2S_TMR - SAI Transmit Mask Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register is double-buffered and updates: When TCSR[TE] is first set At
 * the end of each frame. This allows the masked words in each frame to change
 * from frame to frame.
 */
/*!
 * @name Constants and macros for entire I2S_TMR register
 */
/*@{*/
#define I2S_RD_TMR(base)         (I2S_TMR_REG(base))
#define I2S_WR_TMR(base, value)  (I2S_TMR_REG(base) = (value))
#define I2S_RMW_TMR(base, mask, value) (I2S_WR_TMR(base, (I2S_RD_TMR(base) & ~(mask)) | (value)))
#define I2S_SET_TMR(base, value) (I2S_WR_TMR(base, I2S_RD_TMR(base) |  (value)))
#define I2S_CLR_TMR(base, value) (I2S_WR_TMR(base, I2S_RD_TMR(base) & ~(value)))
#define I2S_TOG_TMR(base, value) (I2S_WR_TMR(base, I2S_RD_TMR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_TMR bitfields
 */

/*!
 * @name Register I2S_TMR, field TWM[15:0] (RW)
 *
 * Configures whether the transmit word is masked (transmit data pin tristated
 * and transmit data not read from FIFO) for the corresponding word in the frame.
 *
 * Values:
 * - 0b0000000000000000 - Word N is enabled.
 * - 0b0000000000000001 - Word N is masked. The transmit data pins are
 *     tri-stated when masked.
 */
/*@{*/
/*! @brief Read current value of the I2S_TMR_TWM field. */
#define I2S_RD_TMR_TWM(base) ((I2S_TMR_REG(base) & I2S_TMR_TWM_MASK) >> I2S_TMR_TWM_SHIFT)
#define I2S_BRD_TMR_TWM(base) (I2S_RD_TMR_TWM(base))

/*! @brief Set the TWM field to a new value. */
#define I2S_WR_TMR_TWM(base, value) (I2S_RMW_TMR(base, I2S_TMR_TWM_MASK, I2S_TMR_TWM(value)))
#define I2S_BWR_TMR_TWM(base, value) (I2S_WR_TMR_TWM(base, value))
/*@}*/

/*******************************************************************************
 * I2S_RCSR - SAI Receive Control Register
 ******************************************************************************/

/*!
 * @brief I2S_RCSR - SAI Receive Control Register (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire I2S_RCSR register
 */
/*@{*/
#define I2S_RD_RCSR(base)        (I2S_RCSR_REG(base))
#define I2S_WR_RCSR(base, value) (I2S_RCSR_REG(base) = (value))
#define I2S_RMW_RCSR(base, mask, value) (I2S_WR_RCSR(base, (I2S_RD_RCSR(base) & ~(mask)) | (value)))
#define I2S_SET_RCSR(base, value) (I2S_WR_RCSR(base, I2S_RD_RCSR(base) |  (value)))
#define I2S_CLR_RCSR(base, value) (I2S_WR_RCSR(base, I2S_RD_RCSR(base) & ~(value)))
#define I2S_TOG_RCSR(base, value) (I2S_WR_RCSR(base, I2S_RD_RCSR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_RCSR bitfields
 */

/*!
 * @name Register I2S_RCSR, field FRDE[0] (RW)
 *
 * Enables/disables DMA requests.
 *
 * Values:
 * - 0b0 - Disables the DMA request.
 * - 0b1 - Enables the DMA request.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FRDE field. */
#define I2S_RD_RCSR_FRDE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FRDE_MASK) >> I2S_RCSR_FRDE_SHIFT)
#define I2S_BRD_RCSR_FRDE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FRDE_SHIFT))

/*! @brief Set the FRDE field to a new value. */
#define I2S_WR_RCSR_FRDE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_FRDE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_FRDE(value)))
#define I2S_BWR_RCSR_FRDE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FRDE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FWDE[1] (RW)
 *
 * Enables/disables DMA requests.
 *
 * Values:
 * - 0b0 - Disables the DMA request.
 * - 0b1 - Enables the DMA request.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FWDE field. */
#define I2S_RD_RCSR_FWDE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FWDE_MASK) >> I2S_RCSR_FWDE_SHIFT)
#define I2S_BRD_RCSR_FWDE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FWDE_SHIFT))

/*! @brief Set the FWDE field to a new value. */
#define I2S_WR_RCSR_FWDE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_FWDE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_FWDE(value)))
#define I2S_BWR_RCSR_FWDE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FWDE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FRIE[8] (RW)
 *
 * Enables/disables FIFO request interrupts.
 *
 * Values:
 * - 0b0 - Disables the interrupt.
 * - 0b1 - Enables the interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FRIE field. */
#define I2S_RD_RCSR_FRIE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FRIE_MASK) >> I2S_RCSR_FRIE_SHIFT)
#define I2S_BRD_RCSR_FRIE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FRIE_SHIFT))

/*! @brief Set the FRIE field to a new value. */
#define I2S_WR_RCSR_FRIE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_FRIE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_FRIE(value)))
#define I2S_BWR_RCSR_FRIE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FRIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FWIE[9] (RW)
 *
 * Enables/disables FIFO warning interrupts.
 *
 * Values:
 * - 0b0 - Disables the interrupt.
 * - 0b1 - Enables the interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FWIE field. */
#define I2S_RD_RCSR_FWIE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FWIE_MASK) >> I2S_RCSR_FWIE_SHIFT)
#define I2S_BRD_RCSR_FWIE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FWIE_SHIFT))

/*! @brief Set the FWIE field to a new value. */
#define I2S_WR_RCSR_FWIE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_FWIE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_FWIE(value)))
#define I2S_BWR_RCSR_FWIE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FWIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FEIE[10] (RW)
 *
 * Enables/disables FIFO error interrupts.
 *
 * Values:
 * - 0b0 - Disables the interrupt.
 * - 0b1 - Enables the interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FEIE field. */
#define I2S_RD_RCSR_FEIE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FEIE_MASK) >> I2S_RCSR_FEIE_SHIFT)
#define I2S_BRD_RCSR_FEIE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FEIE_SHIFT))

/*! @brief Set the FEIE field to a new value. */
#define I2S_WR_RCSR_FEIE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_FEIE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_FEIE(value)))
#define I2S_BWR_RCSR_FEIE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FEIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field SEIE[11] (RW)
 *
 * Enables/disables sync error interrupts.
 *
 * Values:
 * - 0b0 - Disables interrupt.
 * - 0b1 - Enables interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_SEIE field. */
#define I2S_RD_RCSR_SEIE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_SEIE_MASK) >> I2S_RCSR_SEIE_SHIFT)
#define I2S_BRD_RCSR_SEIE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_SEIE_SHIFT))

/*! @brief Set the SEIE field to a new value. */
#define I2S_WR_RCSR_SEIE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_SEIE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_SEIE(value)))
#define I2S_BWR_RCSR_SEIE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_SEIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field WSIE[12] (RW)
 *
 * Enables/disables word start interrupts.
 *
 * Values:
 * - 0b0 - Disables interrupt.
 * - 0b1 - Enables interrupt.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_WSIE field. */
#define I2S_RD_RCSR_WSIE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_WSIE_MASK) >> I2S_RCSR_WSIE_SHIFT)
#define I2S_BRD_RCSR_WSIE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_WSIE_SHIFT))

/*! @brief Set the WSIE field to a new value. */
#define I2S_WR_RCSR_WSIE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_WSIE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_WSIE(value)))
#define I2S_BWR_RCSR_WSIE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_WSIE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FRF[16] (RO)
 *
 * Indicates that the number of words in an enabled receive channel FIFO is
 * greater than the receive FIFO watermark.
 *
 * Values:
 * - 0b0 - Receive FIFO watermark not reached.
 * - 0b1 - Receive FIFO watermark has been reached.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FRF field. */
#define I2S_RD_RCSR_FRF(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FRF_MASK) >> I2S_RCSR_FRF_SHIFT)
#define I2S_BRD_RCSR_FRF(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FRF_SHIFT))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FWF[17] (RO)
 *
 * Indicates that an enabled receive FIFO is full.
 *
 * Values:
 * - 0b0 - No enabled receive FIFO is full.
 * - 0b1 - Enabled receive FIFO is full.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FWF field. */
#define I2S_RD_RCSR_FWF(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FWF_MASK) >> I2S_RCSR_FWF_SHIFT)
#define I2S_BRD_RCSR_FWF(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FWF_SHIFT))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FEF[18] (W1C)
 *
 * Indicates that an enabled receive FIFO has overflowed. Write a logic 1 to
 * this field to clear this flag.
 *
 * Values:
 * - 0b0 - Receive overflow not detected.
 * - 0b1 - Receive overflow detected.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_FEF field. */
#define I2S_RD_RCSR_FEF(base) ((I2S_RCSR_REG(base) & I2S_RCSR_FEF_MASK) >> I2S_RCSR_FEF_SHIFT)
#define I2S_BRD_RCSR_FEF(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FEF_SHIFT))

/*! @brief Set the FEF field to a new value. */
#define I2S_WR_RCSR_FEF(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_FEF(value)))
#define I2S_BWR_RCSR_FEF(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FEF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field SEF[19] (W1C)
 *
 * Indicates that an error in the externally-generated frame sync has been
 * detected. Write a logic 1 to this field to clear this flag.
 *
 * Values:
 * - 0b0 - Sync error not detected.
 * - 0b1 - Frame sync error detected.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_SEF field. */
#define I2S_RD_RCSR_SEF(base) ((I2S_RCSR_REG(base) & I2S_RCSR_SEF_MASK) >> I2S_RCSR_SEF_SHIFT)
#define I2S_BRD_RCSR_SEF(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_SEF_SHIFT))

/*! @brief Set the SEF field to a new value. */
#define I2S_WR_RCSR_SEF(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_SEF_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_SEF(value)))
#define I2S_BWR_RCSR_SEF(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_SEF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field WSF[20] (W1C)
 *
 * Indicates that the start of the configured word has been detected. Write a
 * logic 1 to this field to clear this flag.
 *
 * Values:
 * - 0b0 - Start of word not detected.
 * - 0b1 - Start of word detected.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_WSF field. */
#define I2S_RD_RCSR_WSF(base) ((I2S_RCSR_REG(base) & I2S_RCSR_WSF_MASK) >> I2S_RCSR_WSF_SHIFT)
#define I2S_BRD_RCSR_WSF(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_WSF_SHIFT))

/*! @brief Set the WSF field to a new value. */
#define I2S_WR_RCSR_WSF(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_WSF_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK), I2S_RCSR_WSF(value)))
#define I2S_BWR_RCSR_WSF(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_WSF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field SR[24] (RW)
 *
 * Resets the internal receiver logic including the FIFO pointers.
 * Software-visible registers are not affected, except for the status registers.
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - Software reset.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_SR field. */
#define I2S_RD_RCSR_SR(base) ((I2S_RCSR_REG(base) & I2S_RCSR_SR_MASK) >> I2S_RCSR_SR_SHIFT)
#define I2S_BRD_RCSR_SR(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_SR_SHIFT))

/*! @brief Set the SR field to a new value. */
#define I2S_WR_RCSR_SR(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_SR_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_SR(value)))
#define I2S_BWR_RCSR_SR(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_SR_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field FR[25] (WORZ)
 *
 * Resets the FIFO pointers. Reading this field will always return zero. FIFO
 * pointers should only be reset when the receiver is disabled or the FIFO error
 * flag is set.
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - FIFO reset.
 */
/*@{*/
/*! @brief Set the FR field to a new value. */
#define I2S_WR_RCSR_FR(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_FR_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_FR(value)))
#define I2S_BWR_RCSR_FR(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_FR_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field BCE[28] (RW)
 *
 * Enables the receive bit clock, separately from RE. This field is
 * automatically set whenever RE is set. When software clears this field, the receive bit
 * clock remains enabled, and this field remains set, until the end of the current
 * frame.
 *
 * Values:
 * - 0b0 - Receive bit clock is disabled.
 * - 0b1 - Receive bit clock is enabled.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_BCE field. */
#define I2S_RD_RCSR_BCE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_BCE_MASK) >> I2S_RCSR_BCE_SHIFT)
#define I2S_BRD_RCSR_BCE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_BCE_SHIFT))

/*! @brief Set the BCE field to a new value. */
#define I2S_WR_RCSR_BCE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_BCE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_BCE(value)))
#define I2S_BWR_RCSR_BCE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_BCE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field DBGE[29] (RW)
 *
 * Enables/disables receiver operation in Debug mode. The receive bit clock is
 * not affected by Debug mode.
 *
 * Values:
 * - 0b0 - Receiver is disabled in Debug mode, after completing the current
 *     frame.
 * - 0b1 - Receiver is enabled in Debug mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_DBGE field. */
#define I2S_RD_RCSR_DBGE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_DBGE_MASK) >> I2S_RCSR_DBGE_SHIFT)
#define I2S_BRD_RCSR_DBGE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_DBGE_SHIFT))

/*! @brief Set the DBGE field to a new value. */
#define I2S_WR_RCSR_DBGE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_DBGE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_DBGE(value)))
#define I2S_BWR_RCSR_DBGE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_DBGE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field STOPE[30] (RW)
 *
 * Configures receiver operation in Stop mode. This bit is ignored and the
 * receiver is disabled in all low-leakage stop modes.
 *
 * Values:
 * - 0b0 - Receiver disabled in Stop mode.
 * - 0b1 - Receiver enabled in Stop mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_STOPE field. */
#define I2S_RD_RCSR_STOPE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_STOPE_MASK) >> I2S_RCSR_STOPE_SHIFT)
#define I2S_BRD_RCSR_STOPE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_STOPE_SHIFT))

/*! @brief Set the STOPE field to a new value. */
#define I2S_WR_RCSR_STOPE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_STOPE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_STOPE(value)))
#define I2S_BWR_RCSR_STOPE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_STOPE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCSR, field RE[31] (RW)
 *
 * Enables/disables the receiver. When software clears this field, the receiver
 * remains enabled, and this bit remains set, until the end of the current frame.
 *
 * Values:
 * - 0b0 - Receiver is disabled.
 * - 0b1 - Receiver is enabled, or receiver has been disabled and has not yet
 *     reached end of frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCSR_RE field. */
#define I2S_RD_RCSR_RE(base) ((I2S_RCSR_REG(base) & I2S_RCSR_RE_MASK) >> I2S_RCSR_RE_SHIFT)
#define I2S_BRD_RCSR_RE(base) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_RE_SHIFT))

/*! @brief Set the RE field to a new value. */
#define I2S_WR_RCSR_RE(base, value) (I2S_RMW_RCSR(base, (I2S_RCSR_RE_MASK | I2S_RCSR_FEF_MASK | I2S_RCSR_SEF_MASK | I2S_RCSR_WSF_MASK), I2S_RCSR_RE(value)))
#define I2S_BWR_RCSR_RE(base, value) (BITBAND_ACCESS32(&I2S_RCSR_REG(base), I2S_RCSR_RE_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * I2S_RCR1 - SAI Receive Configuration 1 Register
 ******************************************************************************/

/*!
 * @brief I2S_RCR1 - SAI Receive Configuration 1 Register (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire I2S_RCR1 register
 */
/*@{*/
#define I2S_RD_RCR1(base)        (I2S_RCR1_REG(base))
#define I2S_WR_RCR1(base, value) (I2S_RCR1_REG(base) = (value))
#define I2S_RMW_RCR1(base, mask, value) (I2S_WR_RCR1(base, (I2S_RD_RCR1(base) & ~(mask)) | (value)))
#define I2S_SET_RCR1(base, value) (I2S_WR_RCR1(base, I2S_RD_RCR1(base) |  (value)))
#define I2S_CLR_RCR1(base, value) (I2S_WR_RCR1(base, I2S_RD_RCR1(base) & ~(value)))
#define I2S_TOG_RCR1(base, value) (I2S_WR_RCR1(base, I2S_RD_RCR1(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_RCR1 bitfields
 */

/*!
 * @name Register I2S_RCR1, field RFW[2:0] (RW)
 *
 * Configures the watermark level for all enabled receiver channels.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR1_RFW field. */
#define I2S_RD_RCR1_RFW(base) ((I2S_RCR1_REG(base) & I2S_RCR1_RFW_MASK) >> I2S_RCR1_RFW_SHIFT)
#define I2S_BRD_RCR1_RFW(base) (I2S_RD_RCR1_RFW(base))

/*! @brief Set the RFW field to a new value. */
#define I2S_WR_RCR1_RFW(base, value) (I2S_RMW_RCR1(base, I2S_RCR1_RFW_MASK, I2S_RCR1_RFW(value)))
#define I2S_BWR_RCR1_RFW(base, value) (I2S_WR_RCR1_RFW(base, value))
/*@}*/

/*******************************************************************************
 * I2S_RCR2 - SAI Receive Configuration 2 Register
 ******************************************************************************/

/*!
 * @brief I2S_RCR2 - SAI Receive Configuration 2 Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register must not be altered when RCSR[RE] is set.
 */
/*!
 * @name Constants and macros for entire I2S_RCR2 register
 */
/*@{*/
#define I2S_RD_RCR2(base)        (I2S_RCR2_REG(base))
#define I2S_WR_RCR2(base, value) (I2S_RCR2_REG(base) = (value))
#define I2S_RMW_RCR2(base, mask, value) (I2S_WR_RCR2(base, (I2S_RD_RCR2(base) & ~(mask)) | (value)))
#define I2S_SET_RCR2(base, value) (I2S_WR_RCR2(base, I2S_RD_RCR2(base) |  (value)))
#define I2S_CLR_RCR2(base, value) (I2S_WR_RCR2(base, I2S_RD_RCR2(base) & ~(value)))
#define I2S_TOG_RCR2(base, value) (I2S_WR_RCR2(base, I2S_RD_RCR2(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_RCR2 bitfields
 */

/*!
 * @name Register I2S_RCR2, field DIV[7:0] (RW)
 *
 * Divides down the audio master clock to generate the bit clock when configured
 * for an internal bit clock. The division value is (DIV + 1) * 2.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR2_DIV field. */
#define I2S_RD_RCR2_DIV(base) ((I2S_RCR2_REG(base) & I2S_RCR2_DIV_MASK) >> I2S_RCR2_DIV_SHIFT)
#define I2S_BRD_RCR2_DIV(base) (I2S_RD_RCR2_DIV(base))

/*! @brief Set the DIV field to a new value. */
#define I2S_WR_RCR2_DIV(base, value) (I2S_RMW_RCR2(base, I2S_RCR2_DIV_MASK, I2S_RCR2_DIV(value)))
#define I2S_BWR_RCR2_DIV(base, value) (I2S_WR_RCR2_DIV(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR2, field BCD[24] (RW)
 *
 * Configures the direction of the bit clock.
 *
 * Values:
 * - 0b0 - Bit clock is generated externally in Slave mode.
 * - 0b1 - Bit clock is generated internally in Master mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR2_BCD field. */
#define I2S_RD_RCR2_BCD(base) ((I2S_RCR2_REG(base) & I2S_RCR2_BCD_MASK) >> I2S_RCR2_BCD_SHIFT)
#define I2S_BRD_RCR2_BCD(base) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCD_SHIFT))

/*! @brief Set the BCD field to a new value. */
#define I2S_WR_RCR2_BCD(base, value) (I2S_RMW_RCR2(base, I2S_RCR2_BCD_MASK, I2S_RCR2_BCD(value)))
#define I2S_BWR_RCR2_BCD(base, value) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCD_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR2, field BCP[25] (RW)
 *
 * Configures the polarity of the bit clock.
 *
 * Values:
 * - 0b0 - Bit Clock is active high with drive outputs on rising edge and sample
 *     inputs on falling edge.
 * - 0b1 - Bit Clock is active low with drive outputs on falling edge and sample
 *     inputs on rising edge.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR2_BCP field. */
#define I2S_RD_RCR2_BCP(base) ((I2S_RCR2_REG(base) & I2S_RCR2_BCP_MASK) >> I2S_RCR2_BCP_SHIFT)
#define I2S_BRD_RCR2_BCP(base) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCP_SHIFT))

/*! @brief Set the BCP field to a new value. */
#define I2S_WR_RCR2_BCP(base, value) (I2S_RMW_RCR2(base, I2S_RCR2_BCP_MASK, I2S_RCR2_BCP(value)))
#define I2S_BWR_RCR2_BCP(base, value) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCP_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR2, field MSEL[27:26] (RW)
 *
 * Selects the audio Master Clock option used to generate an internally
 * generated bit clock. This field has no effect when configured for an externally
 * generated bit clock. Depending on the device, some Master Clock options might not be
 * available. See the chip configuration details for the availability and
 * chip-specific meaning of each option.
 *
 * Values:
 * - 0b00 - Bus Clock selected.
 * - 0b01 - Master Clock (MCLK) 1 option selected.
 * - 0b10 - Master Clock (MCLK) 2 option selected.
 * - 0b11 - Master Clock (MCLK) 3 option selected.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR2_MSEL field. */
#define I2S_RD_RCR2_MSEL(base) ((I2S_RCR2_REG(base) & I2S_RCR2_MSEL_MASK) >> I2S_RCR2_MSEL_SHIFT)
#define I2S_BRD_RCR2_MSEL(base) (I2S_RD_RCR2_MSEL(base))

/*! @brief Set the MSEL field to a new value. */
#define I2S_WR_RCR2_MSEL(base, value) (I2S_RMW_RCR2(base, I2S_RCR2_MSEL_MASK, I2S_RCR2_MSEL(value)))
#define I2S_BWR_RCR2_MSEL(base, value) (I2S_WR_RCR2_MSEL(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR2, field BCI[28] (RW)
 *
 * When this field is set and using an internally generated bit clock in either
 * synchronous or asynchronous mode, the bit clock actually used by the receiver
 * is delayed by the pad output delay (the receiver is clocked by the pad input
 * as if the clock was externally generated). This has the effect of decreasing
 * the data input setup time, but increasing the data output valid time. The slave
 * mode timing from the datasheet should be used for the receiver when this bit
 * is set. In synchronous mode, this bit allows the receiver to use the slave mode
 * timing from the datasheet, while the transmitter uses the master mode timing.
 * This field has no effect when configured for an externally generated bit
 * clock .
 *
 * Values:
 * - 0b0 - No effect.
 * - 0b1 - Internal logic is clocked as if bit clock was externally generated.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR2_BCI field. */
#define I2S_RD_RCR2_BCI(base) ((I2S_RCR2_REG(base) & I2S_RCR2_BCI_MASK) >> I2S_RCR2_BCI_SHIFT)
#define I2S_BRD_RCR2_BCI(base) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCI_SHIFT))

/*! @brief Set the BCI field to a new value. */
#define I2S_WR_RCR2_BCI(base, value) (I2S_RMW_RCR2(base, I2S_RCR2_BCI_MASK, I2S_RCR2_BCI(value)))
#define I2S_BWR_RCR2_BCI(base, value) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCI_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR2, field BCS[29] (RW)
 *
 * This field swaps the bit clock used by the receiver. When the receiver is
 * configured in asynchronous mode and this bit is set, the receiver is clocked by
 * the transmitter bit clock (SAI_TX_BCLK). This allows the transmitter and
 * receiver to share the same bit clock, but the receiver continues to use the receiver
 * frame sync (SAI_RX_SYNC). When the receiver is configured in synchronous
 * mode, the transmitter BCS field and receiver BCS field must be set to the same
 * value. When both are set, the transmitter and receiver are both clocked by the
 * receiver bit clock (SAI_RX_BCLK) but use the transmitter frame sync
 * (SAI_TX_SYNC).
 *
 * Values:
 * - 0b0 - Use the normal bit clock source.
 * - 0b1 - Swap the bit clock source.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR2_BCS field. */
#define I2S_RD_RCR2_BCS(base) ((I2S_RCR2_REG(base) & I2S_RCR2_BCS_MASK) >> I2S_RCR2_BCS_SHIFT)
#define I2S_BRD_RCR2_BCS(base) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCS_SHIFT))

/*! @brief Set the BCS field to a new value. */
#define I2S_WR_RCR2_BCS(base, value) (I2S_RMW_RCR2(base, I2S_RCR2_BCS_MASK, I2S_RCR2_BCS(value)))
#define I2S_BWR_RCR2_BCS(base, value) (BITBAND_ACCESS32(&I2S_RCR2_REG(base), I2S_RCR2_BCS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR2, field SYNC[31:30] (RW)
 *
 * Configures between asynchronous and synchronous modes of operation. When
 * configured for a synchronous mode of operation, the transmitter must be configured
 * for asynchronous operation.
 *
 * Values:
 * - 0b00 - Asynchronous mode.
 * - 0b01 - Synchronous with transmitter.
 * - 0b10 - Synchronous with another SAI receiver.
 * - 0b11 - Synchronous with another SAI transmitter.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR2_SYNC field. */
#define I2S_RD_RCR2_SYNC(base) ((I2S_RCR2_REG(base) & I2S_RCR2_SYNC_MASK) >> I2S_RCR2_SYNC_SHIFT)
#define I2S_BRD_RCR2_SYNC(base) (I2S_RD_RCR2_SYNC(base))

/*! @brief Set the SYNC field to a new value. */
#define I2S_WR_RCR2_SYNC(base, value) (I2S_RMW_RCR2(base, I2S_RCR2_SYNC_MASK, I2S_RCR2_SYNC(value)))
#define I2S_BWR_RCR2_SYNC(base, value) (I2S_WR_RCR2_SYNC(base, value))
/*@}*/

/*******************************************************************************
 * I2S_RCR3 - SAI Receive Configuration 3 Register
 ******************************************************************************/

/*!
 * @brief I2S_RCR3 - SAI Receive Configuration 3 Register (RW)
 *
 * Reset value: 0x00000000U
 */
/*!
 * @name Constants and macros for entire I2S_RCR3 register
 */
/*@{*/
#define I2S_RD_RCR3(base)        (I2S_RCR3_REG(base))
#define I2S_WR_RCR3(base, value) (I2S_RCR3_REG(base) = (value))
#define I2S_RMW_RCR3(base, mask, value) (I2S_WR_RCR3(base, (I2S_RD_RCR3(base) & ~(mask)) | (value)))
#define I2S_SET_RCR3(base, value) (I2S_WR_RCR3(base, I2S_RD_RCR3(base) |  (value)))
#define I2S_CLR_RCR3(base, value) (I2S_WR_RCR3(base, I2S_RD_RCR3(base) & ~(value)))
#define I2S_TOG_RCR3(base, value) (I2S_WR_RCR3(base, I2S_RD_RCR3(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_RCR3 bitfields
 */

/*!
 * @name Register I2S_RCR3, field WDFL[3:0] (RW)
 *
 * Configures which word the start of word flag is set. The value written should
 * be one less than the word number (for example, write zero to configure for
 * the first word in the frame). When configured to a value greater than the Frame
 * Size field, then the start of word flag is never set.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR3_WDFL field. */
#define I2S_RD_RCR3_WDFL(base) ((I2S_RCR3_REG(base) & I2S_RCR3_WDFL_MASK) >> I2S_RCR3_WDFL_SHIFT)
#define I2S_BRD_RCR3_WDFL(base) (I2S_RD_RCR3_WDFL(base))

/*! @brief Set the WDFL field to a new value. */
#define I2S_WR_RCR3_WDFL(base, value) (I2S_RMW_RCR3(base, I2S_RCR3_WDFL_MASK, I2S_RCR3_WDFL(value)))
#define I2S_BWR_RCR3_WDFL(base, value) (I2S_WR_RCR3_WDFL(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR3, field RCE[16] (RW)
 *
 * Enables the corresponding data channel for receive operation. A channel must
 * be enabled before its FIFO is accessed. Changing this field will take effect
 * immediately for generating the FIFO request and warning flags, but at the end
 * of each frame for receive operation.
 *
 * Values:
 * - 0b0 - Receive data channel N is disabled.
 * - 0b1 - Receive data channel N is enabled.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR3_RCE field. */
#define I2S_RD_RCR3_RCE(base) ((I2S_RCR3_REG(base) & I2S_RCR3_RCE_MASK) >> I2S_RCR3_RCE_SHIFT)
#define I2S_BRD_RCR3_RCE(base) (BITBAND_ACCESS32(&I2S_RCR3_REG(base), I2S_RCR3_RCE_SHIFT))

/*! @brief Set the RCE field to a new value. */
#define I2S_WR_RCR3_RCE(base, value) (I2S_RMW_RCR3(base, I2S_RCR3_RCE_MASK, I2S_RCR3_RCE(value)))
#define I2S_BWR_RCR3_RCE(base, value) (BITBAND_ACCESS32(&I2S_RCR3_REG(base), I2S_RCR3_RCE_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * I2S_RCR4 - SAI Receive Configuration 4 Register
 ******************************************************************************/

/*!
 * @brief I2S_RCR4 - SAI Receive Configuration 4 Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register must not be altered when RCSR[RE] is set.
 */
/*!
 * @name Constants and macros for entire I2S_RCR4 register
 */
/*@{*/
#define I2S_RD_RCR4(base)        (I2S_RCR4_REG(base))
#define I2S_WR_RCR4(base, value) (I2S_RCR4_REG(base) = (value))
#define I2S_RMW_RCR4(base, mask, value) (I2S_WR_RCR4(base, (I2S_RD_RCR4(base) & ~(mask)) | (value)))
#define I2S_SET_RCR4(base, value) (I2S_WR_RCR4(base, I2S_RD_RCR4(base) |  (value)))
#define I2S_CLR_RCR4(base, value) (I2S_WR_RCR4(base, I2S_RD_RCR4(base) & ~(value)))
#define I2S_TOG_RCR4(base, value) (I2S_WR_RCR4(base, I2S_RD_RCR4(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_RCR4 bitfields
 */

/*!
 * @name Register I2S_RCR4, field FSD[0] (RW)
 *
 * Configures the direction of the frame sync.
 *
 * Values:
 * - 0b0 - Frame Sync is generated externally in Slave mode.
 * - 0b1 - Frame Sync is generated internally in Master mode.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_FSD field. */
#define I2S_RD_RCR4_FSD(base) ((I2S_RCR4_REG(base) & I2S_RCR4_FSD_MASK) >> I2S_RCR4_FSD_SHIFT)
#define I2S_BRD_RCR4_FSD(base) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FSD_SHIFT))

/*! @brief Set the FSD field to a new value. */
#define I2S_WR_RCR4_FSD(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_FSD_MASK, I2S_RCR4_FSD(value)))
#define I2S_BWR_RCR4_FSD(base, value) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FSD_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field FSP[1] (RW)
 *
 * Configures the polarity of the frame sync.
 *
 * Values:
 * - 0b0 - Frame sync is active high.
 * - 0b1 - Frame sync is active low.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_FSP field. */
#define I2S_RD_RCR4_FSP(base) ((I2S_RCR4_REG(base) & I2S_RCR4_FSP_MASK) >> I2S_RCR4_FSP_SHIFT)
#define I2S_BRD_RCR4_FSP(base) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FSP_SHIFT))

/*! @brief Set the FSP field to a new value. */
#define I2S_WR_RCR4_FSP(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_FSP_MASK, I2S_RCR4_FSP(value)))
#define I2S_BWR_RCR4_FSP(base, value) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FSP_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field ONDEM[2] (RW)
 *
 * When set, and the frame sync is generated internally, a frame sync is only
 * generated when the FIFO warning flag is clear.
 *
 * Values:
 * - 0b0 - Internal frame sync is generated continuously.
 * - 0b1 - Internal frame sync is generated when the FIFO warning flag is clear.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_ONDEM field. */
#define I2S_RD_RCR4_ONDEM(base) ((I2S_RCR4_REG(base) & I2S_RCR4_ONDEM_MASK) >> I2S_RCR4_ONDEM_SHIFT)
#define I2S_BRD_RCR4_ONDEM(base) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_ONDEM_SHIFT))

/*! @brief Set the ONDEM field to a new value. */
#define I2S_WR_RCR4_ONDEM(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_ONDEM_MASK, I2S_RCR4_ONDEM(value)))
#define I2S_BWR_RCR4_ONDEM(base, value) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_ONDEM_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field FSE[3] (RW)
 *
 * Values:
 * - 0b0 - Frame sync asserts with the first bit of the frame.
 * - 0b1 - Frame sync asserts one bit before the first bit of the frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_FSE field. */
#define I2S_RD_RCR4_FSE(base) ((I2S_RCR4_REG(base) & I2S_RCR4_FSE_MASK) >> I2S_RCR4_FSE_SHIFT)
#define I2S_BRD_RCR4_FSE(base) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FSE_SHIFT))

/*! @brief Set the FSE field to a new value. */
#define I2S_WR_RCR4_FSE(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_FSE_MASK, I2S_RCR4_FSE(value)))
#define I2S_BWR_RCR4_FSE(base, value) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FSE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field MF[4] (RW)
 *
 * Configures whether the LSB or the MSB is received first.
 *
 * Values:
 * - 0b0 - LSB is received first.
 * - 0b1 - MSB is received first.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_MF field. */
#define I2S_RD_RCR4_MF(base) ((I2S_RCR4_REG(base) & I2S_RCR4_MF_MASK) >> I2S_RCR4_MF_SHIFT)
#define I2S_BRD_RCR4_MF(base) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_MF_SHIFT))

/*! @brief Set the MF field to a new value. */
#define I2S_WR_RCR4_MF(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_MF_MASK, I2S_RCR4_MF(value)))
#define I2S_BWR_RCR4_MF(base, value) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_MF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field SYWD[12:8] (RW)
 *
 * Configures the length of the frame sync in number of bit clocks. The value
 * written must be one less than the number of bit clocks. For example, write 0 for
 * the frame sync to assert for one bit clock only. The sync width cannot be
 * configured longer than the first word of the frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_SYWD field. */
#define I2S_RD_RCR4_SYWD(base) ((I2S_RCR4_REG(base) & I2S_RCR4_SYWD_MASK) >> I2S_RCR4_SYWD_SHIFT)
#define I2S_BRD_RCR4_SYWD(base) (I2S_RD_RCR4_SYWD(base))

/*! @brief Set the SYWD field to a new value. */
#define I2S_WR_RCR4_SYWD(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_SYWD_MASK, I2S_RCR4_SYWD(value)))
#define I2S_BWR_RCR4_SYWD(base, value) (I2S_WR_RCR4_SYWD(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field FRSZ[19:16] (RW)
 *
 * Configures the number of words in each frame. The value written must be one
 * less than the number of words in the frame. For example, write 0 for one word
 * per frame. The maximum supported frame size is 16 words.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_FRSZ field. */
#define I2S_RD_RCR4_FRSZ(base) ((I2S_RCR4_REG(base) & I2S_RCR4_FRSZ_MASK) >> I2S_RCR4_FRSZ_SHIFT)
#define I2S_BRD_RCR4_FRSZ(base) (I2S_RD_RCR4_FRSZ(base))

/*! @brief Set the FRSZ field to a new value. */
#define I2S_WR_RCR4_FRSZ(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_FRSZ_MASK, I2S_RCR4_FRSZ(value)))
#define I2S_BWR_RCR4_FRSZ(base, value) (I2S_WR_RCR4_FRSZ(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field FPACK[25:24] (RW)
 *
 * Enables packing of 8-bit data or 16-bit data into each 32-bit FIFO word. If
 * the word size is greater than 8-bit or 16-bit then only the first 8-bit or
 * 16-bits are stored to the FIFO. The first word in each frame always starts with a
 * new 32-bit FIFO word and the first bit shifted must be configured within the
 * first packed word. When FIFO packing is enabled, the FIFO read pointer will
 * only increment when the full 32-bit FIFO word has been read by software.
 *
 * Values:
 * - 0b00 - FIFO packing is disabled
 * - 0b01 - Reserved.
 * - 0b10 - 8-bit FIFO packing is enabled
 * - 0b11 - 16-bit FIFO packing is enabled
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_FPACK field. */
#define I2S_RD_RCR4_FPACK(base) ((I2S_RCR4_REG(base) & I2S_RCR4_FPACK_MASK) >> I2S_RCR4_FPACK_SHIFT)
#define I2S_BRD_RCR4_FPACK(base) (I2S_RD_RCR4_FPACK(base))

/*! @brief Set the FPACK field to a new value. */
#define I2S_WR_RCR4_FPACK(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_FPACK_MASK, I2S_RCR4_FPACK(value)))
#define I2S_BWR_RCR4_FPACK(base, value) (I2S_WR_RCR4_FPACK(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR4, field FCONT[28] (RW)
 *
 * Configures when the SAI will continue receiving after a FIFO error has been
 * detected.
 *
 * Values:
 * - 0b0 - On FIFO error, the SAI will continue from the start of the next frame
 *     after the FIFO error flag has been cleared.
 * - 0b1 - On FIFO error, the SAI will continue from the same word that caused
 *     the FIFO error to set after the FIFO warning flag has been cleared.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR4_FCONT field. */
#define I2S_RD_RCR4_FCONT(base) ((I2S_RCR4_REG(base) & I2S_RCR4_FCONT_MASK) >> I2S_RCR4_FCONT_SHIFT)
#define I2S_BRD_RCR4_FCONT(base) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FCONT_SHIFT))

/*! @brief Set the FCONT field to a new value. */
#define I2S_WR_RCR4_FCONT(base, value) (I2S_RMW_RCR4(base, I2S_RCR4_FCONT_MASK, I2S_RCR4_FCONT(value)))
#define I2S_BWR_RCR4_FCONT(base, value) (BITBAND_ACCESS32(&I2S_RCR4_REG(base), I2S_RCR4_FCONT_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * I2S_RCR5 - SAI Receive Configuration 5 Register
 ******************************************************************************/

/*!
 * @brief I2S_RCR5 - SAI Receive Configuration 5 Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register must not be altered when RCSR[RE] is set.
 */
/*!
 * @name Constants and macros for entire I2S_RCR5 register
 */
/*@{*/
#define I2S_RD_RCR5(base)        (I2S_RCR5_REG(base))
#define I2S_WR_RCR5(base, value) (I2S_RCR5_REG(base) = (value))
#define I2S_RMW_RCR5(base, mask, value) (I2S_WR_RCR5(base, (I2S_RD_RCR5(base) & ~(mask)) | (value)))
#define I2S_SET_RCR5(base, value) (I2S_WR_RCR5(base, I2S_RD_RCR5(base) |  (value)))
#define I2S_CLR_RCR5(base, value) (I2S_WR_RCR5(base, I2S_RD_RCR5(base) & ~(value)))
#define I2S_TOG_RCR5(base, value) (I2S_WR_RCR5(base, I2S_RD_RCR5(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_RCR5 bitfields
 */

/*!
 * @name Register I2S_RCR5, field FBT[12:8] (RW)
 *
 * Configures the bit index for the first bit received for each word in the
 * frame. If configured for MSB First, the index of the next bit received is one less
 * than the current bit received. If configured for LSB First, the index of the
 * next bit received is one more than the current bit received. The value written
 * must be greater than or equal to the word width when configured for MSB
 * First. The value written must be less than or equal to 31-word width when
 * configured for LSB First.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR5_FBT field. */
#define I2S_RD_RCR5_FBT(base) ((I2S_RCR5_REG(base) & I2S_RCR5_FBT_MASK) >> I2S_RCR5_FBT_SHIFT)
#define I2S_BRD_RCR5_FBT(base) (I2S_RD_RCR5_FBT(base))

/*! @brief Set the FBT field to a new value. */
#define I2S_WR_RCR5_FBT(base, value) (I2S_RMW_RCR5(base, I2S_RCR5_FBT_MASK, I2S_RCR5_FBT(value)))
#define I2S_BWR_RCR5_FBT(base, value) (I2S_WR_RCR5_FBT(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR5, field W0W[20:16] (RW)
 *
 * Configures the number of bits in the first word in each frame. The value
 * written must be one less than the number of bits in the first word. Word width of
 * less than 8 bits is not supported if there is only one word per frame.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR5_W0W field. */
#define I2S_RD_RCR5_W0W(base) ((I2S_RCR5_REG(base) & I2S_RCR5_W0W_MASK) >> I2S_RCR5_W0W_SHIFT)
#define I2S_BRD_RCR5_W0W(base) (I2S_RD_RCR5_W0W(base))

/*! @brief Set the W0W field to a new value. */
#define I2S_WR_RCR5_W0W(base, value) (I2S_RMW_RCR5(base, I2S_RCR5_W0W_MASK, I2S_RCR5_W0W(value)))
#define I2S_BWR_RCR5_W0W(base, value) (I2S_WR_RCR5_W0W(base, value))
/*@}*/

/*!
 * @name Register I2S_RCR5, field WNW[28:24] (RW)
 *
 * Configures the number of bits in each word, for each word except the first in
 * the frame. The value written must be one less than the number of bits per
 * word. Word width of less than 8 bits is not supported.
 */
/*@{*/
/*! @brief Read current value of the I2S_RCR5_WNW field. */
#define I2S_RD_RCR5_WNW(base) ((I2S_RCR5_REG(base) & I2S_RCR5_WNW_MASK) >> I2S_RCR5_WNW_SHIFT)
#define I2S_BRD_RCR5_WNW(base) (I2S_RD_RCR5_WNW(base))

/*! @brief Set the WNW field to a new value. */
#define I2S_WR_RCR5_WNW(base, value) (I2S_RMW_RCR5(base, I2S_RCR5_WNW_MASK, I2S_RCR5_WNW(value)))
#define I2S_BWR_RCR5_WNW(base, value) (I2S_WR_RCR5_WNW(base, value))
/*@}*/

/*******************************************************************************
 * I2S_RDR - SAI Receive Data Register
 ******************************************************************************/

/*!
 * @brief I2S_RDR - SAI Receive Data Register (RO)
 *
 * Reset value: 0x00000000U
 *
 * Reading this register introduces one additional peripheral clock wait state
 * on each read.
 */
/*!
 * @name Constants and macros for entire I2S_RDR register
 */
/*@{*/
#define I2S_RD_RDR(base, index)  (I2S_RDR_REG(base, index))
/*@}*/

/*******************************************************************************
 * I2S_RFR - SAI Receive FIFO Register
 ******************************************************************************/

/*!
 * @brief I2S_RFR - SAI Receive FIFO Register (RO)
 *
 * Reset value: 0x00000000U
 *
 * The MSB of the read and write pointers is used to distinguish between FIFO
 * full and empty conditions. If the read and write pointers are identical, then
 * the FIFO is empty. If the read and write pointers are identical except for the
 * MSB, then the FIFO is full.
 */
/*!
 * @name Constants and macros for entire I2S_RFR register
 */
/*@{*/
#define I2S_RD_RFR(base, index)  (I2S_RFR_REG(base, index))
/*@}*/

/*
 * Constants & macros for individual I2S_RFR bitfields
 */

/*!
 * @name Register I2S_RFR, field RFP[3:0] (RO)
 *
 * FIFO read pointer for receive data channel.
 */
/*@{*/
/*! @brief Read current value of the I2S_RFR_RFP field. */
#define I2S_RD_RFR_RFP(base, index) ((I2S_RFR_REG(base, index) & I2S_RFR_RFP_MASK) >> I2S_RFR_RFP_SHIFT)
#define I2S_BRD_RFR_RFP(base, index) (I2S_RD_RFR_RFP(base, index))
/*@}*/

/*!
 * @name Register I2S_RFR, field WFP[19:16] (RO)
 *
 * FIFO write pointer for receive data channel.
 */
/*@{*/
/*! @brief Read current value of the I2S_RFR_WFP field. */
#define I2S_RD_RFR_WFP(base, index) ((I2S_RFR_REG(base, index) & I2S_RFR_WFP_MASK) >> I2S_RFR_WFP_SHIFT)
#define I2S_BRD_RFR_WFP(base, index) (I2S_RD_RFR_WFP(base, index))
/*@}*/

/*******************************************************************************
 * I2S_RMR - SAI Receive Mask Register
 ******************************************************************************/

/*!
 * @brief I2S_RMR - SAI Receive Mask Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * This register is double-buffered and updates: When RCSR[RE] is first set At
 * the end of each frame This allows the masked words in each frame to change from
 * frame to frame.
 */
/*!
 * @name Constants and macros for entire I2S_RMR register
 */
/*@{*/
#define I2S_RD_RMR(base)         (I2S_RMR_REG(base))
#define I2S_WR_RMR(base, value)  (I2S_RMR_REG(base) = (value))
#define I2S_RMW_RMR(base, mask, value) (I2S_WR_RMR(base, (I2S_RD_RMR(base) & ~(mask)) | (value)))
#define I2S_SET_RMR(base, value) (I2S_WR_RMR(base, I2S_RD_RMR(base) |  (value)))
#define I2S_CLR_RMR(base, value) (I2S_WR_RMR(base, I2S_RD_RMR(base) & ~(value)))
#define I2S_TOG_RMR(base, value) (I2S_WR_RMR(base, I2S_RD_RMR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_RMR bitfields
 */

/*!
 * @name Register I2S_RMR, field RWM[15:0] (RW)
 *
 * Configures whether the receive word is masked (received data ignored and not
 * written to receive FIFO) for the corresponding word in the frame.
 *
 * Values:
 * - 0b0000000000000000 - Word N is enabled.
 * - 0b0000000000000001 - Word N is masked.
 */
/*@{*/
/*! @brief Read current value of the I2S_RMR_RWM field. */
#define I2S_RD_RMR_RWM(base) ((I2S_RMR_REG(base) & I2S_RMR_RWM_MASK) >> I2S_RMR_RWM_SHIFT)
#define I2S_BRD_RMR_RWM(base) (I2S_RD_RMR_RWM(base))

/*! @brief Set the RWM field to a new value. */
#define I2S_WR_RMR_RWM(base, value) (I2S_RMW_RMR(base, I2S_RMR_RWM_MASK, I2S_RMR_RWM(value)))
#define I2S_BWR_RMR_RWM(base, value) (I2S_WR_RMR_RWM(base, value))
/*@}*/

/*******************************************************************************
 * I2S_MCR - SAI MCLK Control Register
 ******************************************************************************/

/*!
 * @brief I2S_MCR - SAI MCLK Control Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * The MCLK Control Register (MCR) controls the clock source and direction of
 * the audio master clock.
 */
/*!
 * @name Constants and macros for entire I2S_MCR register
 */
/*@{*/
#define I2S_RD_MCR(base)         (I2S_MCR_REG(base))
#define I2S_WR_MCR(base, value)  (I2S_MCR_REG(base) = (value))
#define I2S_RMW_MCR(base, mask, value) (I2S_WR_MCR(base, (I2S_RD_MCR(base) & ~(mask)) | (value)))
#define I2S_SET_MCR(base, value) (I2S_WR_MCR(base, I2S_RD_MCR(base) |  (value)))
#define I2S_CLR_MCR(base, value) (I2S_WR_MCR(base, I2S_RD_MCR(base) & ~(value)))
#define I2S_TOG_MCR(base, value) (I2S_WR_MCR(base, I2S_RD_MCR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_MCR bitfields
 */

/*!
 * @name Register I2S_MCR, field MICS[25:24] (RW)
 *
 * Selects the clock input to the MCLK divider. This field cannot be changed
 * while the MCLK divider is enabled. See the chip configuration details for
 * information about the connections to these inputs.
 *
 * Values:
 * - 0b00 - MCLK divider input clock 0 selected.
 * - 0b01 - MCLK divider input clock 1 selected.
 * - 0b10 - MCLK divider input clock 2 selected.
 * - 0b11 - MCLK divider input clock 3 selected.
 */
/*@{*/
/*! @brief Read current value of the I2S_MCR_MICS field. */
#define I2S_RD_MCR_MICS(base) ((I2S_MCR_REG(base) & I2S_MCR_MICS_MASK) >> I2S_MCR_MICS_SHIFT)
#define I2S_BRD_MCR_MICS(base) (I2S_RD_MCR_MICS(base))

/*! @brief Set the MICS field to a new value. */
#define I2S_WR_MCR_MICS(base, value) (I2S_RMW_MCR(base, I2S_MCR_MICS_MASK, I2S_MCR_MICS(value)))
#define I2S_BWR_MCR_MICS(base, value) (I2S_WR_MCR_MICS(base, value))
/*@}*/

/*!
 * @name Register I2S_MCR, field MOE[30] (RW)
 *
 * Enables the MCLK divider and configures the MCLK signal pin as an output.
 * When software clears this field, it remains set until the MCLK divider is fully
 * disabled.
 *
 * Values:
 * - 0b0 - MCLK signal pin is configured as an input that bypasses the MCLK
 *     divider.
 * - 0b1 - MCLK signal pin is configured as an output from the MCLK divider and
 *     the MCLK divider is enabled.
 */
/*@{*/
/*! @brief Read current value of the I2S_MCR_MOE field. */
#define I2S_RD_MCR_MOE(base) ((I2S_MCR_REG(base) & I2S_MCR_MOE_MASK) >> I2S_MCR_MOE_SHIFT)
#define I2S_BRD_MCR_MOE(base) (BITBAND_ACCESS32(&I2S_MCR_REG(base), I2S_MCR_MOE_SHIFT))

/*! @brief Set the MOE field to a new value. */
#define I2S_WR_MCR_MOE(base, value) (I2S_RMW_MCR(base, I2S_MCR_MOE_MASK, I2S_MCR_MOE(value)))
#define I2S_BWR_MCR_MOE(base, value) (BITBAND_ACCESS32(&I2S_MCR_REG(base), I2S_MCR_MOE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register I2S_MCR, field DUF[31] (RO)
 *
 * Provides the status of on-the-fly updates to the MCLK divider ratio.
 *
 * Values:
 * - 0b0 - MCLK divider ratio is not being updated currently.
 * - 0b1 - MCLK divider ratio is updating on-the-fly. Further updates to the
 *     MCLK divider ratio are blocked while this flag remains set.
 */
/*@{*/
/*! @brief Read current value of the I2S_MCR_DUF field. */
#define I2S_RD_MCR_DUF(base) ((I2S_MCR_REG(base) & I2S_MCR_DUF_MASK) >> I2S_MCR_DUF_SHIFT)
#define I2S_BRD_MCR_DUF(base) (BITBAND_ACCESS32(&I2S_MCR_REG(base), I2S_MCR_DUF_SHIFT))
/*@}*/

/*******************************************************************************
 * I2S_MDR - SAI MCLK Divide Register
 ******************************************************************************/

/*!
 * @brief I2S_MDR - SAI MCLK Divide Register (RW)
 *
 * Reset value: 0x00000000U
 *
 * The MCLK Divide Register (MDR) configures the MCLK divide ratio. Although the
 * MDR can be changed when the MCLK divider clock is enabled, additional writes
 * to the MDR are blocked while MCR[DUF] is set. Writes to the MDR when the MCLK
 * divided clock is disabled do not set MCR[DUF].
 */
/*!
 * @name Constants and macros for entire I2S_MDR register
 */
/*@{*/
#define I2S_RD_MDR(base)         (I2S_MDR_REG(base))
#define I2S_WR_MDR(base, value)  (I2S_MDR_REG(base) = (value))
#define I2S_RMW_MDR(base, mask, value) (I2S_WR_MDR(base, (I2S_RD_MDR(base) & ~(mask)) | (value)))
#define I2S_SET_MDR(base, value) (I2S_WR_MDR(base, I2S_RD_MDR(base) |  (value)))
#define I2S_CLR_MDR(base, value) (I2S_WR_MDR(base, I2S_RD_MDR(base) & ~(value)))
#define I2S_TOG_MDR(base, value) (I2S_WR_MDR(base, I2S_RD_MDR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual I2S_MDR bitfields
 */

/*!
 * @name Register I2S_MDR, field DIVIDE[11:0] (RW)
 *
 * Sets the MCLK divide ratio such that: MCLK output = MCLK input * ( (FRACT +
 * 1) / (DIVIDE + 1) ). FRACT must be set equal or less than the value in the
 * DIVIDE field.
 */
/*@{*/
/*! @brief Read current value of the I2S_MDR_DIVIDE field. */
#define I2S_RD_MDR_DIVIDE(base) ((I2S_MDR_REG(base) & I2S_MDR_DIVIDE_MASK) >> I2S_MDR_DIVIDE_SHIFT)
#define I2S_BRD_MDR_DIVIDE(base) (I2S_RD_MDR_DIVIDE(base))

/*! @brief Set the DIVIDE field to a new value. */
#define I2S_WR_MDR_DIVIDE(base, value) (I2S_RMW_MDR(base, I2S_MDR_DIVIDE_MASK, I2S_MDR_DIVIDE(value)))
#define I2S_BWR_MDR_DIVIDE(base, value) (I2S_WR_MDR_DIVIDE(base, value))
/*@}*/

/*!
 * @name Register I2S_MDR, field FRACT[19:12] (RW)
 *
 * Sets the MCLK divide ratio such that: MCLK output = MCLK input * ( (FRACT +
 * 1) / (DIVIDE + 1) ). FRACT must be set equal or less than the value in the
 * DIVIDE field.
 */
/*@{*/
/*! @brief Read current value of the I2S_MDR_FRACT field. */
#define I2S_RD_MDR_FRACT(base) ((I2S_MDR_REG(base) & I2S_MDR_FRACT_MASK) >> I2S_MDR_FRACT_SHIFT)
#define I2S_BRD_MDR_FRACT(base) (I2S_RD_MDR_FRACT(base))

/*! @brief Set the FRACT field to a new value. */
#define I2S_WR_MDR_FRACT(base, value) (I2S_RMW_MDR(base, I2S_MDR_FRACT_MASK, I2S_MDR_FRACT(value)))
#define I2S_BWR_MDR_FRACT(base, value) (I2S_WR_MDR_FRACT(base, value))
/*@}*/

#define I2S_TCSR_FRDE_SHIFT                      0
#define I2S_TCSR_FWDE_SHIFT                      1
#define I2S_TCSR_FRIE_SHIFT                      8
#define I2S_TCSR_FWIE_SHIFT                      9
#define I2S_TCSR_FEIE_SHIFT                      10
#define I2S_TCSR_SEIE_SHIFT                      11
#define I2S_TCSR_WSIE_SHIFT                      12
#define I2S_TCSR_FRF_SHIFT                       16
#define I2S_TCSR_FWF_SHIFT                       17
#define I2S_TCSR_FEF_SHIFT                       18
#define I2S_TCSR_SEF_SHIFT                       19
#define I2S_TCSR_WSF_SHIFT                       20
#define I2S_TCSR_SR_SHIFT                        24
#define I2S_TCSR_FR_SHIFT                        25
#define I2S_TCSR_BCE_SHIFT                       28
#define I2S_TCSR_DBGE_SHIFT                      29
#define I2S_TCSR_STOPE_SHIFT                     30
#define I2S_TCSR_TE_SHIFT                        31
#define I2S_TCR1_TFW_SHIFT                       0
#define I2S_TCR2_DIV_SHIFT                       0
#define I2S_TCR2_BCD_SHIFT                       24
#define I2S_TCR2_BCP_SHIFT                       25
#define I2S_TCR2_MSEL_SHIFT                      26
#define I2S_TCR2_BCI_SHIFT                       28
#define I2S_TCR2_BCS_SHIFT                       29
#define I2S_TCR2_SYNC_SHIFT                      30
#define I2S_TCR3_WDFL_SHIFT                      0
#define I2S_TCR3_TCE_SHIFT                       16
#define I2S_TCR4_FSD_SHIFT                       0
#define I2S_TCR4_FSP_SHIFT                       1
#define I2S_TCR4_ONDEM_SHIFT                     2
#define I2S_TCR4_FSE_SHIFT                       3
#define I2S_TCR4_MF_SHIFT                        4
#define I2S_TCR4_SYWD_SHIFT                      8
#define I2S_TCR4_FRSZ_SHIFT                      16
#define I2S_TCR4_FPACK_SHIFT                     24
#define I2S_TCR4_FCONT_SHIFT                     28
#define I2S_TCR5_FBT_SHIFT                       8
#define I2S_TCR5_W0W_SHIFT                       16
#define I2S_TCR5_WNW_SHIFT                       24
#define I2S_TDR_TDR_SHIFT                        0
#define I2S_TFR_RFP_SHIFT                        0
#define I2S_TFR_WFP_SHIFT                        16
#define I2S_TMR_TWM_SHIFT                        0
#define I2S_RCSR_FRDE_SHIFT                      0
#define I2S_RCSR_FWDE_SHIFT                      1
#define I2S_RCSR_FRIE_SHIFT                      8
#define I2S_RCSR_FWIE_SHIFT                      9
#define I2S_RCSR_FEIE_SHIFT                      10
#define I2S_RCSR_SEIE_SHIFT                      11
#define I2S_RCSR_WSIE_SHIFT                      12
#define I2S_RCSR_FRF_SHIFT                       16
#define I2S_RCSR_FWF_SHIFT                       17
#define I2S_RCSR_FEF_SHIFT                       18
#define I2S_RCSR_SEF_SHIFT                       19
#define I2S_RCSR_WSF_SHIFT                       20
#define I2S_RCSR_SR_SHIFT                        24
#define I2S_RCSR_FR_SHIFT                        25
#define I2S_RCSR_BCE_SHIFT                       28
#define I2S_RCSR_DBGE_SHIFT                      29
#define I2S_RCSR_STOPE_SHIFT                     30
#define I2S_RCSR_RE_SHIFT                        31
#define I2S_RCR1_RFW_SHIFT                       0
#define I2S_RCR2_DIV_SHIFT                       0
#define I2S_RCR2_BCD_SHIFT                       24
#define I2S_RCR2_BCP_SHIFT                       25
#define I2S_RCR2_MSEL_SHIFT                      26
#define I2S_RCR2_BCI_SHIFT                       28
#define I2S_RCR2_BCS_SHIFT                       29
#define I2S_RCR2_SYNC_SHIFT                      30
#define I2S_RCR3_WDFL_SHIFT                      0
#define I2S_RCR3_RCE_SHIFT                       16
#define I2S_RCR4_FSD_SHIFT                       0
#define I2S_RCR4_FSP_SHIFT                       1
#define I2S_RCR4_ONDEM_SHIFT                     2
#define I2S_RCR4_FSE_SHIFT                       3
#define I2S_RCR4_MF_SHIFT                        4
#define I2S_RCR4_SYWD_SHIFT                      8
#define I2S_RCR4_FRSZ_SHIFT                      16
#define I2S_RCR4_FPACK_SHIFT                     24
#define I2S_RCR4_FCONT_SHIFT                     28
#define I2S_RCR5_FBT_SHIFT                       8
#define I2S_RCR5_W0W_SHIFT                       16
#define I2S_RCR5_WNW_SHIFT                       24
#define I2S_RDR_RDR_SHIFT                        0
#define I2S_RFR_RFP_SHIFT                        0
#define I2S_RFR_WFP_SHIFT                        16
#define I2S_RMR_RWM_SHIFT                        0
#define I2S_MCR_MICS_SHIFT                       24
#define I2S_MCR_MOE_SHIFT                        30
#define I2S_MCR_DUF_SHIFT                        31
#define I2S_MDR_DIVIDE_SHIFT                     0
#define I2S_MDR_FRACT_SHIFT                      12


/*!
 * @}
 */ /* end of group I2S_Peripheral_Access_Layer */

/*
 * MK22F51212 MCG
 *
 * Multipurpose Clock Generator module
 *
 * Registers defined in this header file:
 * - MCG_C1 - MCG Control 1 Register
 * - MCG_C2 - MCG Control 2 Register
 * - MCG_C3 - MCG Control 3 Register
 * - MCG_C4 - MCG Control 4 Register
 * - MCG_C5 - MCG Control 5 Register
 * - MCG_C6 - MCG Control 6 Register
 * - MCG_S - MCG Status Register
 * - MCG_SC - MCG Status and Control Register
 * - MCG_ATCVH - MCG Auto Trim Compare Value High Register
 * - MCG_ATCVL - MCG Auto Trim Compare Value Low Register
 * - MCG_C7 - MCG Control 7 Register
 * - MCG_C8 - MCG Control 8 Register
 */

#define MCG_INSTANCE_COUNT (1U) /*!< Number of instances of the MCG module. */
#define MCG_IDX (0U) /*!< Instance number for MCG. */

/*******************************************************************************
 * MCG_C1 - MCG Control 1 Register
 ******************************************************************************/

/*!
 * @brief MCG_C1 - MCG Control 1 Register (RW)
 *
 * Reset value: 0x04U
 */
/*!
 * @name Constants and macros for entire MCG_C1 register
 */
/*@{*/
#define MCG_RD_C1(base)          (MCG_C1_REG(base))
#define MCG_WR_C1(base, value)   (MCG_C1_REG(base) = (value))
#define MCG_RMW_C1(base, mask, value) (MCG_WR_C1(base, (MCG_RD_C1(base) & ~(mask)) | (value)))
#define MCG_SET_C1(base, value)  (MCG_WR_C1(base, MCG_RD_C1(base) |  (value)))
#define MCG_CLR_C1(base, value)  (MCG_WR_C1(base, MCG_RD_C1(base) & ~(value)))
#define MCG_TOG_C1(base, value)  (MCG_WR_C1(base, MCG_RD_C1(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_C1 bitfields
 */

/*!
 * @name Register MCG_C1, field IREFSTEN[0] (RW)
 *
 * Controls whether or not the internal reference clock remains enabled when the
 * MCG enters Stop mode.
 *
 * Values:
 * - 0b0 - Internal reference clock is disabled in Stop mode.
 * - 0b1 - Internal reference clock is enabled in Stop mode if IRCLKEN is set or
 *     if MCG is in FEI, FBI, or BLPI modes before entering Stop mode.
 */
/*@{*/
/*! @brief Read current value of the MCG_C1_IREFSTEN field. */
#define MCG_RD_C1_IREFSTEN(base) ((MCG_C1_REG(base) & MCG_C1_IREFSTEN_MASK) >> MCG_C1_IREFSTEN_SHIFT)
#define MCG_BRD_C1_IREFSTEN(base) (BITBAND_ACCESS8(&MCG_C1_REG(base), MCG_C1_IREFSTEN_SHIFT))

/*! @brief Set the IREFSTEN field to a new value. */
#define MCG_WR_C1_IREFSTEN(base, value) (MCG_RMW_C1(base, MCG_C1_IREFSTEN_MASK, MCG_C1_IREFSTEN(value)))
#define MCG_BWR_C1_IREFSTEN(base, value) (BITBAND_ACCESS8(&MCG_C1_REG(base), MCG_C1_IREFSTEN_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C1, field IRCLKEN[1] (RW)
 *
 * Enables the internal reference clock for use as MCGIRCLK.
 *
 * Values:
 * - 0b0 - MCGIRCLK inactive.
 * - 0b1 - MCGIRCLK active.
 */
/*@{*/
/*! @brief Read current value of the MCG_C1_IRCLKEN field. */
#define MCG_RD_C1_IRCLKEN(base) ((MCG_C1_REG(base) & MCG_C1_IRCLKEN_MASK) >> MCG_C1_IRCLKEN_SHIFT)
#define MCG_BRD_C1_IRCLKEN(base) (BITBAND_ACCESS8(&MCG_C1_REG(base), MCG_C1_IRCLKEN_SHIFT))

/*! @brief Set the IRCLKEN field to a new value. */
#define MCG_WR_C1_IRCLKEN(base, value) (MCG_RMW_C1(base, MCG_C1_IRCLKEN_MASK, MCG_C1_IRCLKEN(value)))
#define MCG_BWR_C1_IRCLKEN(base, value) (BITBAND_ACCESS8(&MCG_C1_REG(base), MCG_C1_IRCLKEN_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C1, field IREFS[2] (RW)
 *
 * Selects the reference clock source for the FLL.
 *
 * Values:
 * - 0b0 - External reference clock is selected.
 * - 0b1 - The slow internal reference clock is selected.
 */
/*@{*/
/*! @brief Read current value of the MCG_C1_IREFS field. */
#define MCG_RD_C1_IREFS(base) ((MCG_C1_REG(base) & MCG_C1_IREFS_MASK) >> MCG_C1_IREFS_SHIFT)
#define MCG_BRD_C1_IREFS(base) (BITBAND_ACCESS8(&MCG_C1_REG(base), MCG_C1_IREFS_SHIFT))

/*! @brief Set the IREFS field to a new value. */
#define MCG_WR_C1_IREFS(base, value) (MCG_RMW_C1(base, MCG_C1_IREFS_MASK, MCG_C1_IREFS(value)))
#define MCG_BWR_C1_IREFS(base, value) (BITBAND_ACCESS8(&MCG_C1_REG(base), MCG_C1_IREFS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C1, field FRDIV[5:3] (RW)
 *
 * Selects the amount to divide down the external reference clock for the FLL.
 * The resulting frequency must be in the range 31.25 kHz to 39.0625 kHz (This is
 * required when FLL/DCO is the clock source for MCGOUTCLK . In FBE mode, it is
 * not required to meet this range, but it is recommended in the cases when trying
 * to enter a FLL mode from FBE).
 *
 * Values:
 * - 0b000 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 1; for all other RANGE
 *     values, Divide Factor is 32.
 * - 0b001 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 2; for all other RANGE
 *     values, Divide Factor is 64.
 * - 0b010 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 4; for all other RANGE
 *     values, Divide Factor is 128.
 * - 0b011 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 8; for all other RANGE
 *     values, Divide Factor is 256.
 * - 0b100 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 16; for all other RANGE
 *     values, Divide Factor is 512.
 * - 0b101 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 32; for all other RANGE
 *     values, Divide Factor is 1024.
 * - 0b110 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 64; for all other RANGE
 *     values, Divide Factor is 1280 .
 * - 0b111 - If RANGE = 0 or OSCSEL=1 , Divide Factor is 128; for all other
 *     RANGE values, Divide Factor is 1536 .
 */
/*@{*/
/*! @brief Read current value of the MCG_C1_FRDIV field. */
#define MCG_RD_C1_FRDIV(base) ((MCG_C1_REG(base) & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT)
#define MCG_BRD_C1_FRDIV(base) (MCG_RD_C1_FRDIV(base))

/*! @brief Set the FRDIV field to a new value. */
#define MCG_WR_C1_FRDIV(base, value) (MCG_RMW_C1(base, MCG_C1_FRDIV_MASK, MCG_C1_FRDIV(value)))
#define MCG_BWR_C1_FRDIV(base, value) (MCG_WR_C1_FRDIV(base, value))
/*@}*/

/*!
 * @name Register MCG_C1, field CLKS[7:6] (RW)
 *
 * Selects the clock source for MCGOUTCLK .
 *
 * Values:
 * - 0b00 - Encoding 0 - Output of FLL or PLL is selected (depends on PLLS
 *     control bit).
 * - 0b01 - Encoding 1 - Internal reference clock is selected.
 * - 0b10 - Encoding 2 - External reference clock is selected.
 * - 0b11 - Encoding 3 - Reserved.
 */
/*@{*/
/*! @brief Read current value of the MCG_C1_CLKS field. */
#define MCG_RD_C1_CLKS(base) ((MCG_C1_REG(base) & MCG_C1_CLKS_MASK) >> MCG_C1_CLKS_SHIFT)
#define MCG_BRD_C1_CLKS(base) (MCG_RD_C1_CLKS(base))

/*! @brief Set the CLKS field to a new value. */
#define MCG_WR_C1_CLKS(base, value) (MCG_RMW_C1(base, MCG_C1_CLKS_MASK, MCG_C1_CLKS(value)))
#define MCG_BWR_C1_CLKS(base, value) (MCG_WR_C1_CLKS(base, value))
/*@}*/

/*******************************************************************************
 * MCG_C2 - MCG Control 2 Register
 ******************************************************************************/

/*!
 * @brief MCG_C2 - MCG Control 2 Register (RW)
 *
 * Reset value: 0x80U
 */
/*!
 * @name Constants and macros for entire MCG_C2 register
 */
/*@{*/
#define MCG_RD_C2(base)          (MCG_C2_REG(base))
#define MCG_WR_C2(base, value)   (MCG_C2_REG(base) = (value))
#define MCG_RMW_C2(base, mask, value) (MCG_WR_C2(base, (MCG_RD_C2(base) & ~(mask)) | (value)))
#define MCG_SET_C2(base, value)  (MCG_WR_C2(base, MCG_RD_C2(base) |  (value)))
#define MCG_CLR_C2(base, value)  (MCG_WR_C2(base, MCG_RD_C2(base) & ~(value)))
#define MCG_TOG_C2(base, value)  (MCG_WR_C2(base, MCG_RD_C2(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_C2 bitfields
 */

/*!
 * @name Register MCG_C2, field IRCS[0] (RW)
 *
 * Selects between the fast or slow internal reference clock source.
 *
 * Values:
 * - 0b0 - Slow internal reference clock selected.
 * - 0b1 - Fast internal reference clock selected.
 */
/*@{*/
/*! @brief Read current value of the MCG_C2_IRCS field. */
#define MCG_RD_C2_IRCS(base) ((MCG_C2_REG(base) & MCG_C2_IRCS_MASK) >> MCG_C2_IRCS_SHIFT)
#define MCG_BRD_C2_IRCS(base) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_IRCS_SHIFT))

/*! @brief Set the IRCS field to a new value. */
#define MCG_WR_C2_IRCS(base, value) (MCG_RMW_C2(base, MCG_C2_IRCS_MASK, MCG_C2_IRCS(value)))
#define MCG_BWR_C2_IRCS(base, value) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_IRCS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C2, field LP[1] (RW)
 *
 * Controls whether the FLL or PLL is disabled in BLPI and BLPE modes. In FBE or
 * PBE modes, setting this bit to 1 will transition the MCG into BLPE mode; in
 * FBI mode, setting this bit to 1 will transition the MCG into BLPI mode. In any
 * other MCG mode, LP bit has no affect.
 *
 * Values:
 * - 0b0 - FLL or PLL is not disabled in bypass modes.
 * - 0b1 - FLL or PLL is disabled in bypass modes (lower power)
 */
/*@{*/
/*! @brief Read current value of the MCG_C2_LP field. */
#define MCG_RD_C2_LP(base)   ((MCG_C2_REG(base) & MCG_C2_LP_MASK) >> MCG_C2_LP_SHIFT)
#define MCG_BRD_C2_LP(base)  (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_LP_SHIFT))

/*! @brief Set the LP field to a new value. */
#define MCG_WR_C2_LP(base, value) (MCG_RMW_C2(base, MCG_C2_LP_MASK, MCG_C2_LP(value)))
#define MCG_BWR_C2_LP(base, value) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_LP_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C2, field EREFS[2] (RW)
 *
 * Selects the source for the external reference clock. See the Oscillator (OSC)
 * chapter for more details.
 *
 * Values:
 * - 0b0 - External reference clock requested.
 * - 0b1 - Oscillator requested.
 */
/*@{*/
/*! @brief Read current value of the MCG_C2_EREFS field. */
#define MCG_RD_C2_EREFS(base) ((MCG_C2_REG(base) & MCG_C2_EREFS_MASK) >> MCG_C2_EREFS_SHIFT)
#define MCG_BRD_C2_EREFS(base) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_EREFS_SHIFT))

/*! @brief Set the EREFS field to a new value. */
#define MCG_WR_C2_EREFS(base, value) (MCG_RMW_C2(base, MCG_C2_EREFS_MASK, MCG_C2_EREFS(value)))
#define MCG_BWR_C2_EREFS(base, value) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_EREFS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C2, field HGO[3] (RW)
 *
 * Controls the crystal oscillator mode of operation. See the Oscillator (OSC)
 * chapter for more details.
 *
 * Values:
 * - 0b0 - Configure crystal oscillator for low-power operation.
 * - 0b1 - Configure crystal oscillator for high-gain operation.
 */
/*@{*/
/*! @brief Read current value of the MCG_C2_HGO field. */
#define MCG_RD_C2_HGO(base)  ((MCG_C2_REG(base) & MCG_C2_HGO_MASK) >> MCG_C2_HGO_SHIFT)
#define MCG_BRD_C2_HGO(base) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_HGO_SHIFT))

/*! @brief Set the HGO field to a new value. */
#define MCG_WR_C2_HGO(base, value) (MCG_RMW_C2(base, MCG_C2_HGO_MASK, MCG_C2_HGO(value)))
#define MCG_BWR_C2_HGO(base, value) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_HGO_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C2, field RANGE[5:4] (RW)
 *
 * Selects the frequency range for the crystal oscillator or external clock
 * source. See the Oscillator (OSC) chapter for more details and the device data
 * sheet for the frequency ranges used.
 *
 * Values:
 * - 0b00 - Encoding 0 - Low frequency range selected for the crystal oscillator
 *     .
 * - 0b01 - Encoding 1 - High frequency range selected for the crystal
 *     oscillator .
 */
/*@{*/
/*! @brief Read current value of the MCG_C2_RANGE field. */
#define MCG_RD_C2_RANGE(base) ((MCG_C2_REG(base) & MCG_C2_RANGE_MASK) >> MCG_C2_RANGE_SHIFT)
#define MCG_BRD_C2_RANGE(base) (MCG_RD_C2_RANGE(base))

/*! @brief Set the RANGE field to a new value. */
#define MCG_WR_C2_RANGE(base, value) (MCG_RMW_C2(base, MCG_C2_RANGE_MASK, MCG_C2_RANGE(value)))
#define MCG_BWR_C2_RANGE(base, value) (MCG_WR_C2_RANGE(base, value))
/*@}*/

/*!
 * @name Register MCG_C2, field FCFTRIM[6] (RW)
 *
 * FCFTRIM controls the smallest adjustment of the fast internal reference clock
 * frequency. Setting FCFTRIM increases the period and clearing FCFTRIM
 * decreases the period by the smallest amount possible. If an FCFTRIM value stored in
 * nonvolatile memory is to be used, it is your responsibility to copy that value
 * from the nonvolatile memory location to this bit.
 */
/*@{*/
/*! @brief Read current value of the MCG_C2_FCFTRIM field. */
#define MCG_RD_C2_FCFTRIM(base) ((MCG_C2_REG(base) & MCG_C2_FCFTRIM_MASK) >> MCG_C2_FCFTRIM_SHIFT)
#define MCG_BRD_C2_FCFTRIM(base) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_FCFTRIM_SHIFT))

/*! @brief Set the FCFTRIM field to a new value. */
#define MCG_WR_C2_FCFTRIM(base, value) (MCG_RMW_C2(base, MCG_C2_FCFTRIM_MASK, MCG_C2_FCFTRIM(value)))
#define MCG_BWR_C2_FCFTRIM(base, value) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_FCFTRIM_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C2, field LOCRE0[7] (RW)
 *
 * Determines whether an interrupt or a reset request is made following a loss
 * of OSC0 external reference clock. The LOCRE0 only has an affect when CME0 is
 * set.
 *
 * Values:
 * - 0b0 - Interrupt request is generated on a loss of OSC0 external reference
 *     clock.
 * - 0b1 - Generate a reset request on a loss of OSC0 external reference clock.
 */
/*@{*/
/*! @brief Read current value of the MCG_C2_LOCRE0 field. */
#define MCG_RD_C2_LOCRE0(base) ((MCG_C2_REG(base) & MCG_C2_LOCRE0_MASK) >> MCG_C2_LOCRE0_SHIFT)
#define MCG_BRD_C2_LOCRE0(base) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_LOCRE0_SHIFT))

/*! @brief Set the LOCRE0 field to a new value. */
#define MCG_WR_C2_LOCRE0(base, value) (MCG_RMW_C2(base, MCG_C2_LOCRE0_MASK, MCG_C2_LOCRE0(value)))
#define MCG_BWR_C2_LOCRE0(base, value) (BITBAND_ACCESS8(&MCG_C2_REG(base), MCG_C2_LOCRE0_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * MCG_C3 - MCG Control 3 Register
 ******************************************************************************/

/*!
 * @brief MCG_C3 - MCG Control 3 Register (RW)
 *
 * Reset value: 0x00U
 */
/*!
 * @name Constants and macros for entire MCG_C3 register
 */
/*@{*/
#define MCG_RD_C3(base)          (MCG_C3_REG(base))
#define MCG_WR_C3(base, value)   (MCG_C3_REG(base) = (value))
#define MCG_RMW_C3(base, mask, value) (MCG_WR_C3(base, (MCG_RD_C3(base) & ~(mask)) | (value)))
#define MCG_SET_C3(base, value)  (MCG_WR_C3(base, MCG_RD_C3(base) |  (value)))
#define MCG_CLR_C3(base, value)  (MCG_WR_C3(base, MCG_RD_C3(base) & ~(value)))
#define MCG_TOG_C3(base, value)  (MCG_WR_C3(base, MCG_RD_C3(base) ^  (value)))
/*@}*/

/*******************************************************************************
 * MCG_C4 - MCG Control 4 Register
 ******************************************************************************/

/*!
 * @brief MCG_C4 - MCG Control 4 Register (RW)
 *
 * Reset value: 0x00U
 *
 * Reset values for DRST and DMX32 bits are 0.
 */
/*!
 * @name Constants and macros for entire MCG_C4 register
 */
/*@{*/
#define MCG_RD_C4(base)          (MCG_C4_REG(base))
#define MCG_WR_C4(base, value)   (MCG_C4_REG(base) = (value))
#define MCG_RMW_C4(base, mask, value) (MCG_WR_C4(base, (MCG_RD_C4(base) & ~(mask)) | (value)))
#define MCG_SET_C4(base, value)  (MCG_WR_C4(base, MCG_RD_C4(base) |  (value)))
#define MCG_CLR_C4(base, value)  (MCG_WR_C4(base, MCG_RD_C4(base) & ~(value)))
#define MCG_TOG_C4(base, value)  (MCG_WR_C4(base, MCG_RD_C4(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_C4 bitfields
 */

/*!
 * @name Register MCG_C4, field SCFTRIM[0] (RW)
 *
 * SCFTRIM A value for SCFTRIM is loaded during reset from a factory programmed
 * location . controls the smallest adjustment of the slow internal reference
 * clock frequency. Setting SCFTRIM increases the period and clearing SCFTRIM
 * decreases the period by the smallest amount possible. If an SCFTRIM value stored in
 * nonvolatile memory is to be used, it is your responsibility to copy that value
 * from the nonvolatile memory location to this bit.
 */
/*@{*/
/*! @brief Read current value of the MCG_C4_SCFTRIM field. */
#define MCG_RD_C4_SCFTRIM(base) ((MCG_C4_REG(base) & MCG_C4_SCFTRIM_MASK) >> MCG_C4_SCFTRIM_SHIFT)
#define MCG_BRD_C4_SCFTRIM(base) (BITBAND_ACCESS8(&MCG_C4_REG(base), MCG_C4_SCFTRIM_SHIFT))

/*! @brief Set the SCFTRIM field to a new value. */
#define MCG_WR_C4_SCFTRIM(base, value) (MCG_RMW_C4(base, MCG_C4_SCFTRIM_MASK, MCG_C4_SCFTRIM(value)))
#define MCG_BWR_C4_SCFTRIM(base, value) (BITBAND_ACCESS8(&MCG_C4_REG(base), MCG_C4_SCFTRIM_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C4, field FCTRIM[4:1] (RW)
 *
 * FCTRIM A value for FCTRIM is loaded during reset from a factory programmed
 * location. controls the fast internal reference clock frequency by controlling
 * the fast internal reference clock period. The FCTRIM bits are binary weighted,
 * that is, bit 1 adjusts twice as much as bit 0. Increasing the binary value
 * increases the period, and decreasing the value decreases the period. If an
 * FCTRIM[3:0] value stored in nonvolatile memory is to be used, it is your
 * responsibility to copy that value from the nonvolatile memory location to this register.
 */
/*@{*/
/*! @brief Read current value of the MCG_C4_FCTRIM field. */
#define MCG_RD_C4_FCTRIM(base) ((MCG_C4_REG(base) & MCG_C4_FCTRIM_MASK) >> MCG_C4_FCTRIM_SHIFT)
#define MCG_BRD_C4_FCTRIM(base) (MCG_RD_C4_FCTRIM(base))

/*! @brief Set the FCTRIM field to a new value. */
#define MCG_WR_C4_FCTRIM(base, value) (MCG_RMW_C4(base, MCG_C4_FCTRIM_MASK, MCG_C4_FCTRIM(value)))
#define MCG_BWR_C4_FCTRIM(base, value) (MCG_WR_C4_FCTRIM(base, value))
/*@}*/

/*!
 * @name Register MCG_C4, field DRST_DRS[6:5] (RW)
 *
 * The DRS bits select the frequency range for the FLL output, DCOOUT. When the
 * LP bit is set, writes to the DRS bits are ignored. The DRST read field
 * indicates the current frequency range for DCOOUT. The DRST field does not update
 * immediately after a write to the DRS field due to internal synchronization between
 * clock domains. See the DCO Frequency Range table for more details.
 *
 * Values:
 * - 0b00 - Encoding 0 - Low range (reset default).
 * - 0b01 - Encoding 1 - Mid range.
 * - 0b10 - Encoding 2 - Mid-high range.
 * - 0b11 - Encoding 3 - High range.
 */
/*@{*/
/*! @brief Read current value of the MCG_C4_DRST_DRS field. */
#define MCG_RD_C4_DRST_DRS(base) ((MCG_C4_REG(base) & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT)
#define MCG_BRD_C4_DRST_DRS(base) (MCG_RD_C4_DRST_DRS(base))

/*! @brief Set the DRST_DRS field to a new value. */
#define MCG_WR_C4_DRST_DRS(base, value) (MCG_RMW_C4(base, MCG_C4_DRST_DRS_MASK, MCG_C4_DRST_DRS(value)))
#define MCG_BWR_C4_DRST_DRS(base, value) (MCG_WR_C4_DRST_DRS(base, value))
/*@}*/

/*!
 * @name Register MCG_C4, field DMX32[7] (RW)
 *
 * The DMX32 bit controls whether the DCO frequency range is narrowed to its
 * maximum frequency with a 32.768 kHz reference. The following table identifies
 * settings for the DCO frequency range. The system clocks derived from this source
 * should not exceed their specified maximums. DRST_DRS DMX32 Reference Range FLL
 * Factor DCO Range 00 0 31.25-39.0625 kHz 640 20-25 MHz 1 32.768 kHz 732 24 MHz
 * 01 0 31.25-39.0625 kHz 1280 40-50 MHz 1 32.768 kHz 1464 48 MHz 10 0
 * 31.25-39.0625 kHz 1920 60-75 MHz 1 32.768 kHz 2197 72 MHz 11 0 31.25-39.0625 kHz 2560
 * 80-100 MHz 1 32.768 kHz 2929 96 MHz
 *
 * Values:
 * - 0b0 - DCO has a default range of 25%.
 * - 0b1 - DCO is fine-tuned for maximum frequency with 32.768 kHz reference.
 */
/*@{*/
/*! @brief Read current value of the MCG_C4_DMX32 field. */
#define MCG_RD_C4_DMX32(base) ((MCG_C4_REG(base) & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT)
#define MCG_BRD_C4_DMX32(base) (BITBAND_ACCESS8(&MCG_C4_REG(base), MCG_C4_DMX32_SHIFT))

/*! @brief Set the DMX32 field to a new value. */
#define MCG_WR_C4_DMX32(base, value) (MCG_RMW_C4(base, MCG_C4_DMX32_MASK, MCG_C4_DMX32(value)))
#define MCG_BWR_C4_DMX32(base, value) (BITBAND_ACCESS8(&MCG_C4_REG(base), MCG_C4_DMX32_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * MCG_C5 - MCG Control 5 Register
 ******************************************************************************/

/*!
 * @brief MCG_C5 - MCG Control 5 Register (RW)
 *
 * Reset value: 0x00U
 */
/*!
 * @name Constants and macros for entire MCG_C5 register
 */
/*@{*/
#define MCG_RD_C5(base)          (MCG_C5_REG(base))
#define MCG_WR_C5(base, value)   (MCG_C5_REG(base) = (value))
#define MCG_RMW_C5(base, mask, value) (MCG_WR_C5(base, (MCG_RD_C5(base) & ~(mask)) | (value)))
#define MCG_SET_C5(base, value)  (MCG_WR_C5(base, MCG_RD_C5(base) |  (value)))
#define MCG_CLR_C5(base, value)  (MCG_WR_C5(base, MCG_RD_C5(base) & ~(value)))
#define MCG_TOG_C5(base, value)  (MCG_WR_C5(base, MCG_RD_C5(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_C5 bitfields
 */

/*!
 * @name Register MCG_C5, field PRDIV0[4:0] (RW)
 *
 * Selects the amount to divide down the external reference clock for the PLL.
 * The resulting frequency must be in the range of 2 MHz to 4 MHz. After the PLL
 * is enabled (by setting either PLLCLKEN 0 or PLLS), the PRDIV 0 value must not
 * be changed when LOCK0 is zero. PLL External Reference Divide Factor PRDIV 0
 * Divide Factor PRDIV 0 Divide Factor PRDIV 0 Divide Factor PRDIV 0 Divide Factor
 * 00000 1 01000 9 10000 17 11000 25 00001 2 01001 10 10001 18 11001 Reserved
 * 00010 3 01010 11 10010 19 11010 Reserved 00011 4 01011 12 10011 20 11011 Reserved
 * 00100 5 01100 13 10100 21 11100 Reserved 00101 6 01101 14 10101 22 11101
 * Reserved 00110 7 01110 15 10110 23 11110 Reserved 00111 8 01111 16 10111 24 11111
 * Reserved
 */
/*@{*/
/*! @brief Read current value of the MCG_C5_PRDIV0 field. */
#define MCG_RD_C5_PRDIV0(base) ((MCG_C5_REG(base) & MCG_C5_PRDIV0_MASK) >> MCG_C5_PRDIV0_SHIFT)
#define MCG_BRD_C5_PRDIV0(base) (MCG_RD_C5_PRDIV0(base))

/*! @brief Set the PRDIV0 field to a new value. */
#define MCG_WR_C5_PRDIV0(base, value) (MCG_RMW_C5(base, MCG_C5_PRDIV0_MASK, MCG_C5_PRDIV0(value)))
#define MCG_BWR_C5_PRDIV0(base, value) (MCG_WR_C5_PRDIV0(base, value))
/*@}*/

/*!
 * @name Register MCG_C5, field PLLSTEN0[5] (RW)
 *
 * Enables the PLL Clock during Normal Stop. In Low Power Stop mode, the PLL
 * clock gets disabled even if PLLSTEN 0 =1. All other power modes, PLLSTEN 0 bit
 * has no affect and does not enable the PLL Clock to run if it is written to 1.
 *
 * Values:
 * - 0b0 - MCGPLLCLK is disabled in any of the Stop modes.
 * - 0b1 - MCGPLLCLK is enabled if system is in Normal Stop mode.
 */
/*@{*/
/*! @brief Read current value of the MCG_C5_PLLSTEN0 field. */
#define MCG_RD_C5_PLLSTEN0(base) ((MCG_C5_REG(base) & MCG_C5_PLLSTEN0_MASK) >> MCG_C5_PLLSTEN0_SHIFT)
#define MCG_BRD_C5_PLLSTEN0(base) (BITBAND_ACCESS8(&MCG_C5_REG(base), MCG_C5_PLLSTEN0_SHIFT))

/*! @brief Set the PLLSTEN0 field to a new value. */
#define MCG_WR_C5_PLLSTEN0(base, value) (MCG_RMW_C5(base, MCG_C5_PLLSTEN0_MASK, MCG_C5_PLLSTEN0(value)))
#define MCG_BWR_C5_PLLSTEN0(base, value) (BITBAND_ACCESS8(&MCG_C5_REG(base), MCG_C5_PLLSTEN0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C5, field PLLCLKEN0[6] (RW)
 *
 * Enables the PLL independent of PLLS and enables the PLL clock for use as
 * MCGPLLCLK. (PRDIV 0 needs to be programmed to the correct divider to generate a
 * PLL reference clock in the range of 2 - 4 MHz range prior to setting the
 * PLLCLKEN 0 bit). Setting PLLCLKEN 0 will enable the external oscillator if not
 * already enabled. Whenever the PLL is being enabled by means of the PLLCLKEN 0 bit,
 * and the external oscillator is being used as the reference clock, the OSCINIT 0
 * bit should be checked to make sure it is set.
 *
 * Values:
 * - 0b0 - MCGPLLCLK is inactive.
 * - 0b1 - MCGPLLCLK is active.
 */
/*@{*/
/*! @brief Read current value of the MCG_C5_PLLCLKEN0 field. */
#define MCG_RD_C5_PLLCLKEN0(base) ((MCG_C5_REG(base) & MCG_C5_PLLCLKEN0_MASK) >> MCG_C5_PLLCLKEN0_SHIFT)
#define MCG_BRD_C5_PLLCLKEN0(base) (BITBAND_ACCESS8(&MCG_C5_REG(base), MCG_C5_PLLCLKEN0_SHIFT))

/*! @brief Set the PLLCLKEN0 field to a new value. */
#define MCG_WR_C5_PLLCLKEN0(base, value) (MCG_RMW_C5(base, MCG_C5_PLLCLKEN0_MASK, MCG_C5_PLLCLKEN0(value)))
#define MCG_BWR_C5_PLLCLKEN0(base, value) (BITBAND_ACCESS8(&MCG_C5_REG(base), MCG_C5_PLLCLKEN0_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * MCG_C6 - MCG Control 6 Register
 ******************************************************************************/

/*!
 * @brief MCG_C6 - MCG Control 6 Register (RW)
 *
 * Reset value: 0x00U
 */
/*!
 * @name Constants and macros for entire MCG_C6 register
 */
/*@{*/
#define MCG_RD_C6(base)          (MCG_C6_REG(base))
#define MCG_WR_C6(base, value)   (MCG_C6_REG(base) = (value))
#define MCG_RMW_C6(base, mask, value) (MCG_WR_C6(base, (MCG_RD_C6(base) & ~(mask)) | (value)))
#define MCG_SET_C6(base, value)  (MCG_WR_C6(base, MCG_RD_C6(base) |  (value)))
#define MCG_CLR_C6(base, value)  (MCG_WR_C6(base, MCG_RD_C6(base) & ~(value)))
#define MCG_TOG_C6(base, value)  (MCG_WR_C6(base, MCG_RD_C6(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_C6 bitfields
 */

/*!
 * @name Register MCG_C6, field VDIV0[4:0] (RW)
 *
 * Selects the amount to divide the VCO output of the PLL. The VDIV 0 bits
 * establish the multiplication factor (M) applied to the reference clock frequency.
 * After the PLL is enabled (by setting either PLLCLKEN 0 or PLLS), the VDIV 0
 * value must not be changed when LOCK 0 is zero. PLL VCO Divide Factor VDIV 0
 * Multiply Factor VDIV 0 Multiply Factor VDIV 0 Multiply Factor VDIV 0 Multiply
 * Factor 00000 24 01000 32 10000 40 11000 48 00001 25 01001 33 10001 41 11001 49
 * 00010 26 01010 34 10010 42 11010 50 00011 27 01011 35 10011 43 11011 51 00100 28
 * 01100 36 10100 44 11100 52 00101 29 01101 37 10101 45 11101 53 00110 30 01110
 * 38 10110 46 11110 54 00111 31 01111 39 10111 47 11111 55
 */
/*@{*/
/*! @brief Read current value of the MCG_C6_VDIV0 field. */
#define MCG_RD_C6_VDIV0(base) ((MCG_C6_REG(base) & MCG_C6_VDIV0_MASK) >> MCG_C6_VDIV0_SHIFT)
#define MCG_BRD_C6_VDIV0(base) (MCG_RD_C6_VDIV0(base))

/*! @brief Set the VDIV0 field to a new value. */
#define MCG_WR_C6_VDIV0(base, value) (MCG_RMW_C6(base, MCG_C6_VDIV0_MASK, MCG_C6_VDIV0(value)))
#define MCG_BWR_C6_VDIV0(base, value) (MCG_WR_C6_VDIV0(base, value))
/*@}*/

/*!
 * @name Register MCG_C6, field CME0[5] (RW)
 *
 * Enables the loss of clock monitoring circuit for the OSC0 external reference
 * mux select. The LOCRE0 bit will determine if a interrupt or a reset request is
 * generated following a loss of OSC0 indication. The CME0 bit must only be set
 * to a logic 1 when the MCG is in an operational mode that uses the external
 * clock (FEE, FBE, PEE, PBE, or BLPE) . Whenever the CME0 bit is set to a logic 1,
 * the value of the RANGE0 bits in the C2 register should not be changed. CME0
 * bit should be set to a logic 0 before the MCG enters any Stop mode. Otherwise, a
 * reset request may occur while in Stop mode. CME0 should also be set to a
 * logic 0 before entering VLPR or VLPW power modes if the MCG is in BLPE mode.
 *
 * Values:
 * - 0b0 - External clock monitor is disabled for OSC0.
 * - 0b1 - External clock monitor is enabled for OSC0.
 */
/*@{*/
/*! @brief Read current value of the MCG_C6_CME0 field. */
#define MCG_RD_C6_CME0(base) ((MCG_C6_REG(base) & MCG_C6_CME0_MASK) >> MCG_C6_CME0_SHIFT)
#define MCG_BRD_C6_CME0(base) (BITBAND_ACCESS8(&MCG_C6_REG(base), MCG_C6_CME0_SHIFT))

/*! @brief Set the CME0 field to a new value. */
#define MCG_WR_C6_CME0(base, value) (MCG_RMW_C6(base, MCG_C6_CME0_MASK, MCG_C6_CME0(value)))
#define MCG_BWR_C6_CME0(base, value) (BITBAND_ACCESS8(&MCG_C6_REG(base), MCG_C6_CME0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C6, field PLLS[6] (RW)
 *
 * Controls whether the PLL or FLL output is selected as the MCG source when
 * CLKS[1:0]=00. If the PLLS bit is cleared and PLLCLKEN 0 is not set, the PLL is
 * disabled in all modes. If the PLLS is set, the FLL is disabled in all modes.
 *
 * Values:
 * - 0b0 - FLL is selected.
 * - 0b1 - PLL is selected (PRDIV 0 need to be programmed to the correct divider
 *     to generate a PLL reference clock in the range of 2-4 MHz prior to
 *     setting the PLLS bit).
 */
/*@{*/
/*! @brief Read current value of the MCG_C6_PLLS field. */
#define MCG_RD_C6_PLLS(base) ((MCG_C6_REG(base) & MCG_C6_PLLS_MASK) >> MCG_C6_PLLS_SHIFT)
#define MCG_BRD_C6_PLLS(base) (BITBAND_ACCESS8(&MCG_C6_REG(base), MCG_C6_PLLS_SHIFT))

/*! @brief Set the PLLS field to a new value. */
#define MCG_WR_C6_PLLS(base, value) (MCG_RMW_C6(base, MCG_C6_PLLS_MASK, MCG_C6_PLLS(value)))
#define MCG_BWR_C6_PLLS(base, value) (BITBAND_ACCESS8(&MCG_C6_REG(base), MCG_C6_PLLS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C6, field LOLIE0[7] (RW)
 *
 * Determines if an interrupt request is made following a loss of lock
 * indication. This bit only has an effect when LOLS 0 is set.
 *
 * Values:
 * - 0b0 - No interrupt request is generated on loss of lock.
 * - 0b1 - Generate an interrupt request on loss of lock.
 */
/*@{*/
/*! @brief Read current value of the MCG_C6_LOLIE0 field. */
#define MCG_RD_C6_LOLIE0(base) ((MCG_C6_REG(base) & MCG_C6_LOLIE0_MASK) >> MCG_C6_LOLIE0_SHIFT)
#define MCG_BRD_C6_LOLIE0(base) (BITBAND_ACCESS8(&MCG_C6_REG(base), MCG_C6_LOLIE0_SHIFT))

/*! @brief Set the LOLIE0 field to a new value. */
#define MCG_WR_C6_LOLIE0(base, value) (MCG_RMW_C6(base, MCG_C6_LOLIE0_MASK, MCG_C6_LOLIE0(value)))
#define MCG_BWR_C6_LOLIE0(base, value) (BITBAND_ACCESS8(&MCG_C6_REG(base), MCG_C6_LOLIE0_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * MCG_S - MCG Status Register
 ******************************************************************************/

/*!
 * @brief MCG_S - MCG Status Register (RW)
 *
 * Reset value: 0x10U
 */
/*!
 * @name Constants and macros for entire MCG_S register
 */
/*@{*/
#define MCG_RD_S(base)           (MCG_S_REG(base))
#define MCG_WR_S(base, value)    (MCG_S_REG(base) = (value))
#define MCG_RMW_S(base, mask, value) (MCG_WR_S(base, (MCG_RD_S(base) & ~(mask)) | (value)))
#define MCG_SET_S(base, value)   (MCG_WR_S(base, MCG_RD_S(base) |  (value)))
#define MCG_CLR_S(base, value)   (MCG_WR_S(base, MCG_RD_S(base) & ~(value)))
#define MCG_TOG_S(base, value)   (MCG_WR_S(base, MCG_RD_S(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_S bitfields
 */

/*!
 * @name Register MCG_S, field IRCST[0] (RO)
 *
 * The IRCST bit indicates the current source for the internal reference clock
 * select clock (IRCSCLK). The IRCST bit does not update immediately after a write
 * to the IRCS bit due to internal synchronization between clock domains. The
 * IRCST bit will only be updated if the internal reference clock is enabled,
 * either by the MCG being in a mode that uses the IRC or by setting the C1[IRCLKEN]
 * bit .
 *
 * Values:
 * - 0b0 - Source of internal reference clock is the slow clock (32 kHz IRC).
 * - 0b1 - Source of internal reference clock is the fast clock (4 MHz IRC).
 */
/*@{*/
/*! @brief Read current value of the MCG_S_IRCST field. */
#define MCG_RD_S_IRCST(base) ((MCG_S_REG(base) & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT)
#define MCG_BRD_S_IRCST(base) (BITBAND_ACCESS8(&MCG_S_REG(base), MCG_S_IRCST_SHIFT))
/*@}*/

/*!
 * @name Register MCG_S, field OSCINIT0[1] (RO)
 *
 * This bit, which resets to 0, is set to 1 after the initialization cycles of
 * the crystal oscillator clock have completed. After being set, the bit is
 * cleared to 0 if the OSC is subsequently disabled. See the OSC module's detailed
 * description for more information.
 */
/*@{*/
/*! @brief Read current value of the MCG_S_OSCINIT0 field. */
#define MCG_RD_S_OSCINIT0(base) ((MCG_S_REG(base) & MCG_S_OSCINIT0_MASK) >> MCG_S_OSCINIT0_SHIFT)
#define MCG_BRD_S_OSCINIT0(base) (BITBAND_ACCESS8(&MCG_S_REG(base), MCG_S_OSCINIT0_SHIFT))
/*@}*/

/*!
 * @name Register MCG_S, field CLKST[3:2] (RO)
 *
 * These bits indicate the current clock mode. The CLKST bits do not update
 * immediately after a write to the CLKS bits due to internal synchronization between
 * clock domains.
 *
 * Values:
 * - 0b00 - Encoding 0 - Output of the FLL is selected (reset default).
 * - 0b01 - Encoding 1 - Internal reference clock is selected.
 * - 0b10 - Encoding 2 - External reference clock is selected.
 * - 0b11 - Encoding 3 - Output of the PLL is selected.
 */
/*@{*/
/*! @brief Read current value of the MCG_S_CLKST field. */
#define MCG_RD_S_CLKST(base) ((MCG_S_REG(base) & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT)
#define MCG_BRD_S_CLKST(base) (MCG_RD_S_CLKST(base))
/*@}*/

/*!
 * @name Register MCG_S, field IREFST[4] (RO)
 *
 * This bit indicates the current source for the FLL reference clock. The IREFST
 * bit does not update immediately after a write to the IREFS bit due to
 * internal synchronization between clock domains.
 *
 * Values:
 * - 0b0 - Source of FLL reference clock is the external reference clock.
 * - 0b1 - Source of FLL reference clock is the internal reference clock.
 */
/*@{*/
/*! @brief Read current value of the MCG_S_IREFST field. */
#define MCG_RD_S_IREFST(base) ((MCG_S_REG(base) & MCG_S_IREFST_MASK) >> MCG_S_IREFST_SHIFT)
#define MCG_BRD_S_IREFST(base) (BITBAND_ACCESS8(&MCG_S_REG(base), MCG_S_IREFST_SHIFT))
/*@}*/

/*!
 * @name Register MCG_S, field PLLST[5] (RO)
 *
 * This bit indicates the clock source selected by PLLS . The PLLST bit does not
 * update immediately after a write to the PLLS bit due to internal
 * synchronization between clock domains.
 *
 * Values:
 * - 0b0 - Source of PLLS clock is FLL clock.
 * - 0b1 - Source of PLLS clock is PLL output clock.
 */
/*@{*/
/*! @brief Read current value of the MCG_S_PLLST field. */
#define MCG_RD_S_PLLST(base) ((MCG_S_REG(base) & MCG_S_PLLST_MASK) >> MCG_S_PLLST_SHIFT)
#define MCG_BRD_S_PLLST(base) (BITBAND_ACCESS8(&MCG_S_REG(base), MCG_S_PLLST_SHIFT))
/*@}*/

/*!
 * @name Register MCG_S, field LOCK0[6] (RO)
 *
 * This bit indicates whether the PLL has acquired lock. Lock detection is only
 * enabled when the PLL is enabled (either through clock mode selection or
 * PLLCLKEN0=1 setting). While the PLL clock is locking to the desired frequency, the
 * MCG PLL clock (MCGPLLCLK) will be gated off until the LOCK bit gets asserted.
 * If the lock status bit is set, changing the value of the PRDIV0 [4:0] bits in
 * the C5 register or the VDIV0[4:0] bits in the C6 register causes the lock
 * status bit to clear and stay cleared until the PLL has reacquired lock. Loss of PLL
 * reference clock will also cause the LOCK0 bit to clear until the PLL has
 * reacquired lock. Entry into LLS, VLPS, or regular Stop with PLLSTEN=0 also causes
 * the lock status bit to clear and stay cleared until the Stop mode is exited
 * and the PLL has reacquired lock. Any time the PLL is enabled and the LOCK0 bit
 * is cleared, the MCGPLLCLK will be gated off until the LOCK0 bit is asserted
 * again.
 *
 * Values:
 * - 0b0 - PLL is currently unlocked.
 * - 0b1 - PLL is currently locked.
 */
/*@{*/
/*! @brief Read current value of the MCG_S_LOCK0 field. */
#define MCG_RD_S_LOCK0(base) ((MCG_S_REG(base) & MCG_S_LOCK0_MASK) >> MCG_S_LOCK0_SHIFT)
#define MCG_BRD_S_LOCK0(base) (BITBAND_ACCESS8(&MCG_S_REG(base), MCG_S_LOCK0_SHIFT))
/*@}*/

/*!
 * @name Register MCG_S, field LOLS0[7] (W1C)
 *
 * This bit is a sticky bit indicating the lock status for the PLL. LOLS is set
 * if after acquiring lock, the PLL output frequency has fallen outside the lock
 * exit frequency tolerance, D unl . LOLIE determines whether an interrupt
 * request is made when LOLS is set. LOLRE determines whether a reset request is made
 * when LOLS is set. This bit is cleared by reset or by writing a logic 1 to it
 * when set. Writing a logic 0 to this bit has no effect.
 *
 * Values:
 * - 0b0 - PLL has not lost lock since LOLS 0 was last cleared.
 * - 0b1 - PLL has lost lock since LOLS 0 was last cleared.
 */
/*@{*/
/*! @brief Read current value of the MCG_S_LOLS0 field. */
#define MCG_RD_S_LOLS0(base) ((MCG_S_REG(base) & MCG_S_LOLS0_MASK) >> MCG_S_LOLS0_SHIFT)
#define MCG_BRD_S_LOLS0(base) (BITBAND_ACCESS8(&MCG_S_REG(base), MCG_S_LOLS0_SHIFT))

/*! @brief Set the LOLS0 field to a new value. */
#define MCG_WR_S_LOLS0(base, value) (MCG_RMW_S(base, MCG_S_LOLS0_MASK, MCG_S_LOLS0(value)))
#define MCG_BWR_S_LOLS0(base, value) (BITBAND_ACCESS8(&MCG_S_REG(base), MCG_S_LOLS0_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * MCG_SC - MCG Status and Control Register
 ******************************************************************************/

/*!
 * @brief MCG_SC - MCG Status and Control Register (RW)
 *
 * Reset value: 0x02U
 */
/*!
 * @name Constants and macros for entire MCG_SC register
 */
/*@{*/
#define MCG_RD_SC(base)          (MCG_SC_REG(base))
#define MCG_WR_SC(base, value)   (MCG_SC_REG(base) = (value))
#define MCG_RMW_SC(base, mask, value) (MCG_WR_SC(base, (MCG_RD_SC(base) & ~(mask)) | (value)))
#define MCG_SET_SC(base, value)  (MCG_WR_SC(base, MCG_RD_SC(base) |  (value)))
#define MCG_CLR_SC(base, value)  (MCG_WR_SC(base, MCG_RD_SC(base) & ~(value)))
#define MCG_TOG_SC(base, value)  (MCG_WR_SC(base, MCG_RD_SC(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_SC bitfields
 */

/*!
 * @name Register MCG_SC, field LOCS0[0] (W1C)
 *
 * The LOCS0 indicates when a loss of OSC0 reference clock has occurred. The
 * LOCS0 bit only has an effect when CME0 is set. This bit is cleared by writing a
 * logic 1 to it when set.
 *
 * Values:
 * - 0b0 - Loss of OSC0 has not occurred.
 * - 0b1 - Loss of OSC0 has occurred.
 */
/*@{*/
/*! @brief Read current value of the MCG_SC_LOCS0 field. */
#define MCG_RD_SC_LOCS0(base) ((MCG_SC_REG(base) & MCG_SC_LOCS0_MASK) >> MCG_SC_LOCS0_SHIFT)
#define MCG_BRD_SC_LOCS0(base) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_LOCS0_SHIFT))

/*! @brief Set the LOCS0 field to a new value. */
#define MCG_WR_SC_LOCS0(base, value) (MCG_RMW_SC(base, MCG_SC_LOCS0_MASK, MCG_SC_LOCS0(value)))
#define MCG_BWR_SC_LOCS0(base, value) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_LOCS0_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_SC, field FCRDIV[3:1] (RW)
 *
 * Selects the amount to divide down the fast internal reference clock. The
 * resulting frequency will be in the range 31.25 kHz to 4 MHz (Note: Changing the
 * divider when the Fast IRC is enabled is not supported).
 *
 * Values:
 * - 0b000 - Divide Factor is 1
 * - 0b001 - Divide Factor is 2.
 * - 0b010 - Divide Factor is 4.
 * - 0b011 - Divide Factor is 8.
 * - 0b100 - Divide Factor is 16
 * - 0b101 - Divide Factor is 32
 * - 0b110 - Divide Factor is 64
 * - 0b111 - Divide Factor is 128.
 */
/*@{*/
/*! @brief Read current value of the MCG_SC_FCRDIV field. */
#define MCG_RD_SC_FCRDIV(base) ((MCG_SC_REG(base) & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT)
#define MCG_BRD_SC_FCRDIV(base) (MCG_RD_SC_FCRDIV(base))

/*! @brief Set the FCRDIV field to a new value. */
#define MCG_WR_SC_FCRDIV(base, value) (MCG_RMW_SC(base, (MCG_SC_FCRDIV_MASK | MCG_SC_LOCS0_MASK), MCG_SC_FCRDIV(value)))
#define MCG_BWR_SC_FCRDIV(base, value) (MCG_WR_SC_FCRDIV(base, value))
/*@}*/

/*!
 * @name Register MCG_SC, field FLTPRSRV[4] (RW)
 *
 * This bit will prevent the FLL filter values from resetting allowing the FLL
 * output frequency to remain the same during clock mode changes where the FLL/DCO
 * output is still valid. (Note: This requires that the FLL reference frequency
 * to remain the same as what it was prior to the new clock mode switch.
 * Otherwise FLL filter and frequency values will change.)
 *
 * Values:
 * - 0b0 - FLL filter and FLL frequency will reset on changes to currect clock
 *     mode.
 * - 0b1 - Fll filter and FLL frequency retain their previous values during new
 *     clock mode change.
 */
/*@{*/
/*! @brief Read current value of the MCG_SC_FLTPRSRV field. */
#define MCG_RD_SC_FLTPRSRV(base) ((MCG_SC_REG(base) & MCG_SC_FLTPRSRV_MASK) >> MCG_SC_FLTPRSRV_SHIFT)
#define MCG_BRD_SC_FLTPRSRV(base) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_FLTPRSRV_SHIFT))

/*! @brief Set the FLTPRSRV field to a new value. */
#define MCG_WR_SC_FLTPRSRV(base, value) (MCG_RMW_SC(base, (MCG_SC_FLTPRSRV_MASK | MCG_SC_LOCS0_MASK), MCG_SC_FLTPRSRV(value)))
#define MCG_BWR_SC_FLTPRSRV(base, value) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_FLTPRSRV_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_SC, field ATMF[5] (RW)
 *
 * Fail flag for the Automatic Trim Machine (ATM). This bit asserts when the
 * Automatic Trim Machine is enabled, ATME=1, and a write to the C1, C3, C4, and SC
 * registers is detected or the MCG enters into any Stop mode. A write to ATMF
 * clears the flag.
 *
 * Values:
 * - 0b0 - Automatic Trim Machine completed normally.
 * - 0b1 - Automatic Trim Machine failed.
 */
/*@{*/
/*! @brief Read current value of the MCG_SC_ATMF field. */
#define MCG_RD_SC_ATMF(base) ((MCG_SC_REG(base) & MCG_SC_ATMF_MASK) >> MCG_SC_ATMF_SHIFT)
#define MCG_BRD_SC_ATMF(base) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_ATMF_SHIFT))

/*! @brief Set the ATMF field to a new value. */
#define MCG_WR_SC_ATMF(base, value) (MCG_RMW_SC(base, (MCG_SC_ATMF_MASK | MCG_SC_LOCS0_MASK), MCG_SC_ATMF(value)))
#define MCG_BWR_SC_ATMF(base, value) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_ATMF_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_SC, field ATMS[6] (RW)
 *
 * Selects the IRCS clock for Auto Trim Test.
 *
 * Values:
 * - 0b0 - 32 kHz Internal Reference Clock selected.
 * - 0b1 - 4 MHz Internal Reference Clock selected.
 */
/*@{*/
/*! @brief Read current value of the MCG_SC_ATMS field. */
#define MCG_RD_SC_ATMS(base) ((MCG_SC_REG(base) & MCG_SC_ATMS_MASK) >> MCG_SC_ATMS_SHIFT)
#define MCG_BRD_SC_ATMS(base) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_ATMS_SHIFT))

/*! @brief Set the ATMS field to a new value. */
#define MCG_WR_SC_ATMS(base, value) (MCG_RMW_SC(base, (MCG_SC_ATMS_MASK | MCG_SC_LOCS0_MASK), MCG_SC_ATMS(value)))
#define MCG_BWR_SC_ATMS(base, value) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_ATMS_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_SC, field ATME[7] (RW)
 *
 * Enables the Auto Trim Machine to start automatically trimming the selected
 * Internal Reference Clock. ATME deasserts after the Auto Trim Machine has
 * completed trimming all trim bits of the IRCS clock selected by the ATMS bit. Writing
 * to C1, C3, C4, and SC registers or entering Stop mode aborts the auto trim
 * operation and clears this bit.
 *
 * Values:
 * - 0b0 - Auto Trim Machine disabled.
 * - 0b1 - Auto Trim Machine enabled.
 */
/*@{*/
/*! @brief Read current value of the MCG_SC_ATME field. */
#define MCG_RD_SC_ATME(base) ((MCG_SC_REG(base) & MCG_SC_ATME_MASK) >> MCG_SC_ATME_SHIFT)
#define MCG_BRD_SC_ATME(base) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_ATME_SHIFT))

/*! @brief Set the ATME field to a new value. */
#define MCG_WR_SC_ATME(base, value) (MCG_RMW_SC(base, (MCG_SC_ATME_MASK | MCG_SC_LOCS0_MASK), MCG_SC_ATME(value)))
#define MCG_BWR_SC_ATME(base, value) (BITBAND_ACCESS8(&MCG_SC_REG(base), MCG_SC_ATME_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * MCG_ATCVH - MCG Auto Trim Compare Value High Register
 ******************************************************************************/

/*!
 * @brief MCG_ATCVH - MCG Auto Trim Compare Value High Register (RW)
 *
 * Reset value: 0x00U
 */
/*!
 * @name Constants and macros for entire MCG_ATCVH register
 */
/*@{*/
#define MCG_RD_ATCVH(base)       (MCG_ATCVH_REG(base))
#define MCG_WR_ATCVH(base, value) (MCG_ATCVH_REG(base) = (value))
#define MCG_RMW_ATCVH(base, mask, value) (MCG_WR_ATCVH(base, (MCG_RD_ATCVH(base) & ~(mask)) | (value)))
#define MCG_SET_ATCVH(base, value) (MCG_WR_ATCVH(base, MCG_RD_ATCVH(base) |  (value)))
#define MCG_CLR_ATCVH(base, value) (MCG_WR_ATCVH(base, MCG_RD_ATCVH(base) & ~(value)))
#define MCG_TOG_ATCVH(base, value) (MCG_WR_ATCVH(base, MCG_RD_ATCVH(base) ^  (value)))
/*@}*/

/*******************************************************************************
 * MCG_ATCVL - MCG Auto Trim Compare Value Low Register
 ******************************************************************************/

/*!
 * @brief MCG_ATCVL - MCG Auto Trim Compare Value Low Register (RW)
 *
 * Reset value: 0x00U
 */
/*!
 * @name Constants and macros for entire MCG_ATCVL register
 */
/*@{*/
#define MCG_RD_ATCVL(base)       (MCG_ATCVL_REG(base))
#define MCG_WR_ATCVL(base, value) (MCG_ATCVL_REG(base) = (value))
#define MCG_RMW_ATCVL(base, mask, value) (MCG_WR_ATCVL(base, (MCG_RD_ATCVL(base) & ~(mask)) | (value)))
#define MCG_SET_ATCVL(base, value) (MCG_WR_ATCVL(base, MCG_RD_ATCVL(base) |  (value)))
#define MCG_CLR_ATCVL(base, value) (MCG_WR_ATCVL(base, MCG_RD_ATCVL(base) & ~(value)))
#define MCG_TOG_ATCVL(base, value) (MCG_WR_ATCVL(base, MCG_RD_ATCVL(base) ^  (value)))
/*@}*/

/*******************************************************************************
 * MCG_C7 - MCG Control 7 Register
 ******************************************************************************/

/*!
 * @brief MCG_C7 - MCG Control 7 Register (RW)
 *
 * Reset value: 0x00U
 */
/*!
 * @name Constants and macros for entire MCG_C7 register
 */
/*@{*/
#define MCG_RD_C7(base)          (MCG_C7_REG(base))
#define MCG_WR_C7(base, value)   (MCG_C7_REG(base) = (value))
#define MCG_RMW_C7(base, mask, value) (MCG_WR_C7(base, (MCG_RD_C7(base) & ~(mask)) | (value)))
#define MCG_SET_C7(base, value)  (MCG_WR_C7(base, MCG_RD_C7(base) |  (value)))
#define MCG_CLR_C7(base, value)  (MCG_WR_C7(base, MCG_RD_C7(base) & ~(value)))
#define MCG_TOG_C7(base, value)  (MCG_WR_C7(base, MCG_RD_C7(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_C7 bitfields
 */

/*!
 * @name Register MCG_C7, field OSCSEL[1:0] (RW)
 *
 * Selects the MCG FLL external reference clock
 *
 * Values:
 * - 0b00 - Selects Oscillator (OSCCLK0).
 * - 0b01 - Selects 32 kHz RTC Oscillator.
 * - 0b10 - Selects Oscillator (OSCCLK1).
 * - 0b11 - RESERVED
 */
/*@{*/
/*! @brief Read current value of the MCG_C7_OSCSEL field. */
#define MCG_RD_C7_OSCSEL(base) ((MCG_C7_REG(base) & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT)
#define MCG_BRD_C7_OSCSEL(base) (MCG_RD_C7_OSCSEL(base))

/*! @brief Set the OSCSEL field to a new value. */
#define MCG_WR_C7_OSCSEL(base, value) (MCG_RMW_C7(base, MCG_C7_OSCSEL_MASK, MCG_C7_OSCSEL(value)))
#define MCG_BWR_C7_OSCSEL(base, value) (MCG_WR_C7_OSCSEL(base, value))
/*@}*/

/*******************************************************************************
 * MCG_C8 - MCG Control 8 Register
 ******************************************************************************/

/*!
 * @brief MCG_C8 - MCG Control 8 Register (RW)
 *
 * Reset value: 0x80U
 */
/*!
 * @name Constants and macros for entire MCG_C8 register
 */
/*@{*/
#define MCG_RD_C8(base)          (MCG_C8_REG(base))
#define MCG_WR_C8(base, value)   (MCG_C8_REG(base) = (value))
#define MCG_RMW_C8(base, mask, value) (MCG_WR_C8(base, (MCG_RD_C8(base) & ~(mask)) | (value)))
#define MCG_SET_C8(base, value)  (MCG_WR_C8(base, MCG_RD_C8(base) |  (value)))
#define MCG_CLR_C8(base, value)  (MCG_WR_C8(base, MCG_RD_C8(base) & ~(value)))
#define MCG_TOG_C8(base, value)  (MCG_WR_C8(base, MCG_RD_C8(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual MCG_C8 bitfields
 */

/*!
 * @name Register MCG_C8, field LOCS1[0] (W1C)
 *
 * This bit indicates when a loss of clock has occurred. This bit is cleared by
 * writing a logic 1 to it when set.
 *
 * Values:
 * - 0b0 - Loss of RTC has not occur.
 * - 0b1 - Loss of RTC has occur
 */
/*@{*/
/*! @brief Read current value of the MCG_C8_LOCS1 field. */
#define MCG_RD_C8_LOCS1(base) ((MCG_C8_REG(base) & MCG_C8_LOCS1_MASK) >> MCG_C8_LOCS1_SHIFT)
#define MCG_BRD_C8_LOCS1(base) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_LOCS1_SHIFT))

/*! @brief Set the LOCS1 field to a new value. */
#define MCG_WR_C8_LOCS1(base, value) (MCG_RMW_C8(base, MCG_C8_LOCS1_MASK, MCG_C8_LOCS1(value)))
#define MCG_BWR_C8_LOCS1(base, value) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_LOCS1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C8, field CME1[5] (RW)
 *
 * Enables the loss of clock monitoring circuit for the output of the RTC
 * external reference clock. The LOCRE1 bit will determine whether an interrupt or a
 * reset request is generated following a loss of RTC clock indication. The CME1
 * bit should be set to a logic 1 when the MCG is in an operational mode that uses
 * the RTC as its external reference clock or if the RTC is operational. CME1 bit
 * must be set to a logic 0 before the MCG enters any Stop mode. Otherwise, a
 * reset request may occur when in Stop mode. CME1 should also be set to a logic 0
 * before entering VLPR or VLPW power modes.
 *
 * Values:
 * - 0b0 - External clock monitor is disabled for RTC clock.
 * - 0b1 - External clock monitor is enabled for RTC clock.
 */
/*@{*/
/*! @brief Read current value of the MCG_C8_CME1 field. */
#define MCG_RD_C8_CME1(base) ((MCG_C8_REG(base) & MCG_C8_CME1_MASK) >> MCG_C8_CME1_SHIFT)
#define MCG_BRD_C8_CME1(base) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_CME1_SHIFT))

/*! @brief Set the CME1 field to a new value. */
#define MCG_WR_C8_CME1(base, value) (MCG_RMW_C8(base, (MCG_C8_CME1_MASK | MCG_C8_LOCS1_MASK), MCG_C8_CME1(value)))
#define MCG_BWR_C8_CME1(base, value) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_CME1_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C8, field LOLRE[6] (RW)
 *
 * Determines if an interrupt or a reset request is made following a PLL loss of
 * lock.
 *
 * Values:
 * - 0b0 - Interrupt request is generated on a PLL loss of lock indication. The
 *     PLL loss of lock interrupt enable bit must also be set to generate the
 *     interrupt request.
 * - 0b1 - Generate a reset request on a PLL loss of lock indication.
 */
/*@{*/
/*! @brief Read current value of the MCG_C8_LOLRE field. */
#define MCG_RD_C8_LOLRE(base) ((MCG_C8_REG(base) & MCG_C8_LOLRE_MASK) >> MCG_C8_LOLRE_SHIFT)
#define MCG_BRD_C8_LOLRE(base) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_LOLRE_SHIFT))

/*! @brief Set the LOLRE field to a new value. */
#define MCG_WR_C8_LOLRE(base, value) (MCG_RMW_C8(base, (MCG_C8_LOLRE_MASK | MCG_C8_LOCS1_MASK), MCG_C8_LOLRE(value)))
#define MCG_BWR_C8_LOLRE(base, value) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_LOLRE_SHIFT) = (value))
/*@}*/

/*!
 * @name Register MCG_C8, field LOCRE1[7] (RW)
 *
 * Determines if a interrupt or a reset request is made following a loss of RTC
 * external reference clock. The LOCRE1 only has an affect when CME1 is set.
 *
 * Values:
 * - 0b0 - Interrupt request is generated on a loss of RTC external reference
 *     clock.
 * - 0b1 - Generate a reset request on a loss of RTC external reference clock
 */
/*@{*/
/*! @brief Read current value of the MCG_C8_LOCRE1 field. */
#define MCG_RD_C8_LOCRE1(base) ((MCG_C8_REG(base) & MCG_C8_LOCRE1_MASK) >> MCG_C8_LOCRE1_SHIFT)
#define MCG_BRD_C8_LOCRE1(base) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_LOCRE1_SHIFT))

/*! @brief Set the LOCRE1 field to a new value. */
#define MCG_WR_C8_LOCRE1(base, value) (MCG_RMW_C8(base, (MCG_C8_LOCRE1_MASK | MCG_C8_LOCS1_MASK), MCG_C8_LOCRE1(value)))
#define MCG_BWR_C8_LOCRE1(base, value) (BITBAND_ACCESS8(&MCG_C8_REG(base), MCG_C8_LOCRE1_SHIFT) = (value))
/*@}*/

#define MCG_C1_IREFSTEN_SHIFT                    0
#define MCG_C1_IRCLKEN_SHIFT                     1
#define MCG_C1_IREFS_SHIFT                       2
#define MCG_C1_FRDIV_SHIFT                       3
#define MCG_C1_CLKS_SHIFT                        6
#define MCG_C2_IRCS_SHIFT                        0
#define MCG_C2_LP_SHIFT                          1
#define MCG_C2_EREFS_SHIFT                       2
#define MCG_C2_HGO_SHIFT                         3
#define MCG_C2_RANGE_SHIFT                       4
#define MCG_C2_FCFTRIM_SHIFT                     6
#define MCG_C2_LOCRE0_SHIFT                      7
#define MCG_C3_SCTRIM_SHIFT                      0
#define MCG_C4_SCFTRIM_SHIFT                     0
#define MCG_C4_FCTRIM_SHIFT                      1
#define MCG_C4_DRST_DRS_SHIFT                    5
#define MCG_C4_DMX32_SHIFT                       7
#define MCG_C5_PRDIV0_SHIFT                      0
#define MCG_C5_PLLSTEN0_SHIFT                    5
#define MCG_C5_PLLCLKEN0_SHIFT                   6
#define MCG_C6_VDIV0_SHIFT                       0
#define MCG_C6_CME0_SHIFT                        5
#define MCG_C6_PLLS_SHIFT                        6
#define MCG_C6_LOLIE0_SHIFT                      7
#define MCG_S_IRCST_SHIFT                        0
#define MCG_S_OSCINIT0_SHIFT                     1
#define MCG_S_CLKST_SHIFT                        2
#define MCG_S_IREFST_SHIFT                       4
#define MCG_S_PLLST_SHIFT                        5
#define MCG_S_LOCK0_SHIFT                        6
#define MCG_S_LOLS0_SHIFT                        7
#define MCG_SC_LOCS0_SHIFT                       0
#define MCG_SC_FCRDIV_SHIFT                      1
#define MCG_SC_FLTPRSRV_SHIFT                    4
#define MCG_SC_ATMF_SHIFT                        5
#define MCG_SC_ATMS_SHIFT                        6
#define MCG_SC_ATME_SHIFT                        7
#define MCG_ATCVH_ATCVH_SHIFT                    0
#define MCG_ATCVL_ATCVL_SHIFT                    0
#define MCG_C7_OSCSEL_SHIFT                      0
#define MCG_C8_LOCS1_SHIFT                       0
#define MCG_C8_CME1_SHIFT                        5
#define MCG_C8_LOLRE_SHIFT                       6
#define MCG_C8_LOCRE1_SHIFT                      7

#define MCG_C1_IREFSTEN_MASK                     0x1u
#define MCG_C1_IRCLKEN_MASK                      0x2u
//#define MCG_C1_IREFS_MASK                        0x4u
#define MCG_C1_FRDIV_MASK                        0x38u
#define MCG_C1_CLKS_MASK                         0xC0u
#define MCG_C2_IRCS_MASK                         0x1u
#define MCG_C2_LP_MASK                           0x2u
#define MCG_C2_EREFS_MASK                        0x4u
#define MCG_C2_HGO_MASK                          0x8u
#define MCG_C2_RANGE_MASK                        0x30u
#define MCG_C2_FCFTRIM_MASK                      0x40u
#define MCG_C2_LOCRE0_MASK                       0x80u
#define MCG_C3_SCTRIM_MASK                       0xFFu
#define MCG_C4_SCFTRIM_MASK                      0x1u
#define MCG_C4_FCTRIM_MASK                       0x1Eu
#define MCG_C4_DRST_DRS_MASK                     0x60u
#define MCG_C4_DMX32_MASK                        0x80u
#define MCG_C5_PRDIV0_MASK                       0x1Fu
#define MCG_C5_PLLSTEN0_MASK                     0x20u
#define MCG_C5_PLLCLKEN0_MASK                    0x40u
#define MCG_C6_VDIV0_MASK                        0x1Fu
#define MCG_C6_CME0_MASK                         0x20u
#define MCG_C6_PLLS_MASK                         0x40u
#define MCG_C6_LOLIE0_MASK                       0x80u
#define MCG_S_IRCST_MASK                         0x1u
#define MCG_S_OSCINIT0_MASK                      0x2u
#define MCG_S_CLKST_MASK                         0xCu
#define MCG_S_IREFST_MASK                        0x10u
#define MCG_S_PLLST_MASK                         0x20u
#define MCG_S_LOCK0_MASK                         0x40u
#define MCG_S_LOLS0_MASK                         0x80u
#define MCG_SC_LOCS0_MASK                        0x1u
#define MCG_SC_FCRDIV_MASK                       0xEu
#define MCG_SC_FLTPRSRV_MASK                     0x10u
#define MCG_SC_ATMF_MASK                         0x20u
#define MCG_SC_ATMS_MASK                         0x40u
#define MCG_SC_ATME_MASK                         0x80u
#define MCG_ATCVH_ATCVH_MASK                     0xFFu
#define MCG_ATCVL_ATCVL_MASK                     0xFFu
#define MCG_C7_OSCSEL_MASK                       0x3u
#define MCG_C8_LOCS1_MASK                        0x1u
#define MCG_C8_CME1_MASK                         0x20u
#define MCG_C8_LOLRE_MASK                        0x40u
#define MCG_C8_LOCRE1_MASK                       0x80u

#define FTM_INSTANCE_COUNT (4U) /*!< Number of instances of the FTM module. */

/*
 * MK22F51212 OSC
 *
 * Oscillator
 *
 * Registers defined in this header file:
 * - OSC_CR - OSC Control Register
 * - OSC_DIV - OSC_DIV
 */

#define OSC_INSTANCE_COUNT (1U) /*!< Number of instances of the OSC module. */
#define OSC_IDX (0U) /*!< Instance number for OSC. */

/*******************************************************************************
 * OSC_CR - OSC Control Register
 ******************************************************************************/

/*!
 * @brief OSC_CR - OSC Control Register (RW)
 *
 * Reset value: 0x00U
 *
 * After OSC is enabled and starts generating the clocks, the configurations
 * such as low power and frequency range, must not be changed.
 */
/*!
 * @name Constants and macros for entire OSC_CR register
 */
/*@{*/
#define OSC_RD_CR(base)          (OSC_CR_REG(base))
#define OSC_WR_CR(base, value)   (OSC_CR_REG(base) = (value))
#define OSC_RMW_CR(base, mask, value) (OSC_WR_CR(base, (OSC_RD_CR(base) & ~(mask)) | (value)))
#define OSC_SET_CR(base, value)  (OSC_WR_CR(base, OSC_RD_CR(base) |  (value)))
#define OSC_CLR_CR(base, value)  (OSC_WR_CR(base, OSC_RD_CR(base) & ~(value)))
#define OSC_TOG_CR(base, value)  (OSC_WR_CR(base, OSC_RD_CR(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual OSC_CR bitfields
 */

/*!
 * @name Register OSC_CR, field SC16P[0] (RW)
 *
 * Configures the oscillator load.
 *
 * Values:
 * - 0b0 - Disable the selection.
 * - 0b1 - Add 16 pF capacitor to the oscillator load.
 */
/*@{*/
/*! @brief Read current value of the OSC_CR_SC16P field. */
#define OSC_RD_CR_SC16P(base) ((OSC_CR_REG(base) & OSC_CR_SC16P_MASK) >> OSC_CR_SC16P_SHIFT)
#define OSC_BRD_CR_SC16P(base) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC16P_SHIFT))

/*! @brief Set the SC16P field to a new value. */
#define OSC_WR_CR_SC16P(base, value) (OSC_RMW_CR(base, OSC_CR_SC16P_MASK, OSC_CR_SC16P(value)))
#define OSC_BWR_CR_SC16P(base, value) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC16P_SHIFT) = (value))
/*@}*/

/*!
 * @name Register OSC_CR, field SC8P[1] (RW)
 *
 * Configures the oscillator load.
 *
 * Values:
 * - 0b0 - Disable the selection.
 * - 0b1 - Add 8 pF capacitor to the oscillator load.
 */
/*@{*/
/*! @brief Read current value of the OSC_CR_SC8P field. */
#define OSC_RD_CR_SC8P(base) ((OSC_CR_REG(base) & OSC_CR_SC8P_MASK) >> OSC_CR_SC8P_SHIFT)
#define OSC_BRD_CR_SC8P(base) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC8P_SHIFT))

/*! @brief Set the SC8P field to a new value. */
#define OSC_WR_CR_SC8P(base, value) (OSC_RMW_CR(base, OSC_CR_SC8P_MASK, OSC_CR_SC8P(value)))
#define OSC_BWR_CR_SC8P(base, value) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC8P_SHIFT) = (value))
/*@}*/

/*!
 * @name Register OSC_CR, field SC4P[2] (RW)
 *
 * Configures the oscillator load.
 *
 * Values:
 * - 0b0 - Disable the selection.
 * - 0b1 - Add 4 pF capacitor to the oscillator load.
 */
/*@{*/
/*! @brief Read current value of the OSC_CR_SC4P field. */
#define OSC_RD_CR_SC4P(base) ((OSC_CR_REG(base) & OSC_CR_SC4P_MASK) >> OSC_CR_SC4P_SHIFT)
#define OSC_BRD_CR_SC4P(base) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC4P_SHIFT))

/*! @brief Set the SC4P field to a new value. */
#define OSC_WR_CR_SC4P(base, value) (OSC_RMW_CR(base, OSC_CR_SC4P_MASK, OSC_CR_SC4P(value)))
#define OSC_BWR_CR_SC4P(base, value) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC4P_SHIFT) = (value))
/*@}*/

/*!
 * @name Register OSC_CR, field SC2P[3] (RW)
 *
 * Configures the oscillator load.
 *
 * Values:
 * - 0b0 - Disable the selection.
 * - 0b1 - Add 2 pF capacitor to the oscillator load.
 */
/*@{*/
/*! @brief Read current value of the OSC_CR_SC2P field. */
#define OSC_RD_CR_SC2P(base) ((OSC_CR_REG(base) & OSC_CR_SC2P_MASK) >> OSC_CR_SC2P_SHIFT)
#define OSC_BRD_CR_SC2P(base) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC2P_SHIFT))

/*! @brief Set the SC2P field to a new value. */
#define OSC_WR_CR_SC2P(base, value) (OSC_RMW_CR(base, OSC_CR_SC2P_MASK, OSC_CR_SC2P(value)))
#define OSC_BWR_CR_SC2P(base, value) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_SC2P_SHIFT) = (value))
/*@}*/

/*!
 * @name Register OSC_CR, field EREFSTEN[5] (RW)
 *
 * Controls whether or not the external reference clock (OSCERCLK) remains
 * enabled when MCU enters Stop mode.
 *
 * Values:
 * - 0b0 - External reference clock is disabled in Stop mode.
 * - 0b1 - External reference clock stays enabled in Stop mode if ERCLKEN is set
 *     before entering Stop mode.
 */
/*@{*/
/*! @brief Read current value of the OSC_CR_EREFSTEN field. */
#define OSC_RD_CR_EREFSTEN(base) ((OSC_CR_REG(base) & OSC_CR_EREFSTEN_MASK) >> OSC_CR_EREFSTEN_SHIFT)
#define OSC_BRD_CR_EREFSTEN(base) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_EREFSTEN_SHIFT))

/*! @brief Set the EREFSTEN field to a new value. */
#define OSC_WR_CR_EREFSTEN(base, value) (OSC_RMW_CR(base, OSC_CR_EREFSTEN_MASK, OSC_CR_EREFSTEN(value)))
#define OSC_BWR_CR_EREFSTEN(base, value) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_EREFSTEN_SHIFT) = (value))
/*@}*/

/*!
 * @name Register OSC_CR, field ERCLKEN[7] (RW)
 *
 * Enables external reference clock (OSCERCLK).
 *
 * Values:
 * - 0b0 - External reference clock is inactive.
 * - 0b1 - External reference clock is enabled.
 */
/*@{*/
/*! @brief Read current value of the OSC_CR_ERCLKEN field. */
#define OSC_RD_CR_ERCLKEN(base) ((OSC_CR_REG(base) & OSC_CR_ERCLKEN_MASK) >> OSC_CR_ERCLKEN_SHIFT)
#define OSC_BRD_CR_ERCLKEN(base) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_ERCLKEN_SHIFT))

/*! @brief Set the ERCLKEN field to a new value. */
#define OSC_WR_CR_ERCLKEN(base, value) (OSC_RMW_CR(base, OSC_CR_ERCLKEN_MASK, OSC_CR_ERCLKEN(value)))
#define OSC_BWR_CR_ERCLKEN(base, value) (BITBAND_ACCESS8(&OSC_CR_REG(base), OSC_CR_ERCLKEN_SHIFT) = (value))
/*@}*/

/*******************************************************************************
 * OSC_DIV - OSC_DIV
 ******************************************************************************/

/*!
 * @brief OSC_DIV - OSC_DIV (RW)
 *
 * Reset value: 0x00U
 *
 * OSC CLock divider register.
 */
/*!
 * @name Constants and macros for entire OSC_DIV register
 */
/*@{*/
#define OSC_RD_DIV(base)         (OSC_DIV_REG(base))
#define OSC_WR_DIV(base, value)  (OSC_DIV_REG(base) = (value))
#define OSC_RMW_DIV(base, mask, value) (OSC_WR_DIV(base, (OSC_RD_DIV(base) & ~(mask)) | (value)))
#define OSC_SET_DIV(base, value) (OSC_WR_DIV(base, OSC_RD_DIV(base) |  (value)))
#define OSC_CLR_DIV(base, value) (OSC_WR_DIV(base, OSC_RD_DIV(base) & ~(value)))
#define OSC_TOG_DIV(base, value) (OSC_WR_DIV(base, OSC_RD_DIV(base) ^  (value)))
/*@}*/

/*
 * Constants & macros for individual OSC_DIV bitfields
 */

/*!
 * @name Register OSC_DIV, field ERPS[7:6] (RW)
 *
 * ERCLK prescaler. These two bits are used to divide the ERCLK output. The
 * un-divided ERCLK output is not affected by these two bits.
 *
 * Values:
 * - 0b00 - The divisor ratio is 1.
 * - 0b01 - The divisor ratio is 2.
 * - 0b10 - The divisor ratio is 4.
 * - 0b11 - The divisor ratio is 8.
 */
/*@{*/
/*! @brief Read current value of the OSC_DIV_ERPS field. */
#define OSC_RD_DIV_ERPS(base) ((OSC_DIV_REG(base) & OSC_DIV_ERPS_MASK) >> OSC_DIV_ERPS_SHIFT)
#define OSC_BRD_DIV_ERPS(base) (OSC_RD_DIV_ERPS(base))

/*! @brief Set the ERPS field to a new value. */
#define OSC_WR_DIV_ERPS(base, value) (OSC_RMW_DIV(base, OSC_DIV_ERPS_MASK, OSC_DIV_ERPS(value)))
#define OSC_BWR_DIV_ERPS(base, value) (OSC_WR_DIV_ERPS(base, value))
/*@}*/

#define OSC_CR_SC16P_MASK                        0x1u
#define OSC_CR_SC8P_MASK                         0x2u
#define OSC_CR_SC4P_MASK                         0x4u
#define OSC_CR_SC2P_MASK                         0x8u

#define OSC_CR_SC16P_SHIFT                       0
#define OSC_CR_SC8P_SHIFT                        1
#define OSC_CR_SC4P_SHIFT                        2
#define OSC_CR_SC2P_SHIFT                        3
#define OSC_CR_EREFSTEN_SHIFT                    5
#define OSC_CR_ERCLKEN_SHIFT                     7
#define OSC_DIV_ERPS_SHIFT                       6

#define RCM_SRS0_WAKEUP_MASK                     0x1u
#define PMC_REGSC_ACKISO_MASK                    0x8u
#define SIM_SCGC5_LPTMR_MASK                     0x1u
#define LPTMR_CSR_TCF_MASK                       0x80u
#define LPTMR_PSR_PBYP_MASK                      0x4u
#define LPTMR_CSR_TEN_MASK                       0x1u
#define SIM_CLKDIV2_USBFRAC_MASK                 0x1u

#define SIM_SCGC6_RTC_MASK                       0x20000000u
#define RTC_CR_OSCE_MASK                         0x100u
#define RTC_CR_SC2P_MASK                         0x2000u
#define RTC_CR_SC4P_MASK                         0x1000u  
#define RTC_CR_SC8P_MASK                         0x800u
#define RTC_CR_SC16P_MASK                        0x400u
#define RTC_CR_CLKO_MASK                         0x200u

#define SIM_SCGC5_PORTA_MASK                     0x200u 
#define PORT_PCR_ISF_MASK                        0x1000000u 

#endif
