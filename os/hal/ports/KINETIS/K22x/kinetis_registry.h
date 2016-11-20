/*
    ChibiOS - Copyright (C) 2014 Derek Mulcahy
                        (C) 2016 flabbergast <s3+flabbergast@sdfeu.org>

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
 * @file    K22x/kinetis_registry.h
 * @brief   K22x capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef KINETIS_REGISTRY_H_
#define KINETIS_REGISTRY_H_

#if !defined(K22x) || defined(__DOXYGEN__)
#define K22x
#endif

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    K22x capabilities
 * @{
 */
/*===========================================================================*/
/* K22x5                                                                     */
/*===========================================================================*/
#if defined(K22x10) || defined(__DOXYGEN__)

/**
 * @brief   Maximum system and core clock (f_SYS) frequency.
 */
#define KINETIS_SYSCLK_MAX      100000000L

/**
 * @brief   Maximum bus clock (f_BUS) frequency.
 */
#define KINETIS_BUSCLK_MAX      50000000L

/**
 * @brief   Maximum flash clock (f_FLASH) frequency.
 */
#define KINETIS_FLASHCLK_MAX    25000000L

/* ADC attributes.*/
#define KINETIS_HAS_ADC0            TRUE
#define KINETIS_ADC0_IRQ_VECTOR     VectorDC
#define KINETIS_HAS_ADC1            TRUE
#define KINETIS_ADC1_IRQ_VECTOR     Vector164

/* DAC attributes.*/
#define KINETIS_HAS_DAC0            TRUE
#define KINETIS_DAC0_IRQ_VECTOR     Vector120

/* DMA attributes.*/
#define KINETIS_DMA0_IRQ_VECTOR     Vector40
#define KINETIS_DMA1_IRQ_VECTOR     Vector44
#define KINETIS_DMA2_IRQ_VECTOR     Vector48
#define KINETIS_DMA3_IRQ_VECTOR     Vector4C
#define KINETIS_HAS_DMA_ERROR_IRQ   FALSE

/* EXT attributes.*/
#define KINETIS_PORTA_IRQ_VECTOR    Vector12C
#define KINETIS_PORTB_IRQ_VECTOR    Vector130
#define KINETIS_PORTC_IRQ_VECTOR    Vector134
#define KINETIS_PORTD_IRQ_VECTOR    Vector138
#define KINETIS_PORTE_IRQ_VECTOR    Vector13C
#define KINETIS_EXT_HAS_COMMON_CD_IRQ   FALSE
#define KINETIS_EXT_HAS_COMMON_BCDE_IRQ FALSE
#define KINETIS_GPIO_HAS_OPENDRAIN  TRUE

/* I2C attributes.*/
#define KINETIS_HAS_I2C0            TRUE
#define KINETIS_I2C0_IRQ_VECTOR     VectorA0
#define KINETIS_HAS_I2C1            TRUE
#define KINETIS_I2C1_IRQ_VECTOR     VectorA4

/* Serial attributes.*/
#define KINETIS_HAS_SERIAL0         TRUE
#define KINETIS_SERIAL0_IRQ_VECTOR  VectorBC
#define KINETIS_HAS_SERIAL1         TRUE
#define KINETIS_SERIAL1_IRQ_VECTOR  VectorC4
#define KINETIS_HAS_SERIAL2         TRUE
#define KINETIS_SERIAL2_IRQ_VECTOR  VectorCC
#define KINETIS_HAS_SERIAL_ERROR_IRQ TRUE
#define KINETIS_SERIAL0_ERROR_IRQ_VECTOR VectorC0
#define KINETIS_SERIAL1_ERROR_IRQ_VECTOR VectorC8
#define KINETIS_SERIAL2_ERROR_IRQ_VECTOR VectorD0
#define KINETIS_SERIAL0_IS_LPUART   FALSE
#define KINETIS_SERIAL0_IS_UARTLP   FALSE
#define KINETIS_SERIAL1_IS_LPUART   FALSE
#define KINETIS_HAS_SERIAL3         TRUE
#define KINETIS_SERIAL3_IRQ_VECTOR  VectorB8
#define KINETIS_SERIAL3_IS_LPUART   TRUE

/* SPI attributes.*/
#define KINETIS_HAS_SPI0            TRUE
#define KINETIS_SPI0_IRQ_VECTOR     VectorA8
#define KINETIS_HAS_SPI1            TRUE
#define KINETIS_SPI1_IRQ_VECTOR     VectorAC

/* FlexTimer attributes.*/
#define KINETIS_FTM0_CHANNELS 8
#define KINETIS_FTM1_CHANNELS 2
#define KINETIS_FTM2_CHANNELS 2

#define KINETIS_FTM0_IRQ_VECTOR     VectorE8
#define KINETIS_FTM1_IRQ_VECTOR     VectorEC
#define KINETIS_HAS_FTM2            TRUE
#define KINETIS_FTM2_IRQ_VECTOR     VectorF0

/* GPT attributes.*/
#define KINETIS_HAS_PIT0            TRUE
#define KINETIS_PIT0_IRQ_VECTOR     Vector100
#define KINETIS_HAS_PIT1            TRUE
#define KINETIS_PIT1_IRQ_VECTOR     Vector104
#define KINETIS_HAS_PIT2            TRUE
#define KINETIS_PIT2_IRQ_VECTOR     Vector108
#define KINETIS_HAS_PIT3            TRUE
#define KINETIS_PIT3_IRQ_VECTOR     Vector10C
#define KINETIS_HAS_PIT_COMMON_IRQ  FALSE

/* USB attributes.*/
#define KINETIS_HAS_USB             TRUE
#define KINETIS_USB_IRQ_VECTOR      Vector114
#define KINETIS_USB0_IS_USBOTG      TRUE
#define KINETIS_HAS_USB_CLOCK_RECOVERY FALSE

/* LPTMR attributes.*/
#define KINETIS_LPTMR0_IRQ_VECTOR   Vector128
/** @} */


#endif /* defined(K22x10) || defined(__DOXYGEN__) */


#endif /* KINETIS_REGISTRY_H_ */

/** @} */
