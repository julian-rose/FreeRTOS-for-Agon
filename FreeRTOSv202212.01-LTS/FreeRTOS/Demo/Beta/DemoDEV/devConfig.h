/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2024 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 *  Please ensure to read the configuration and relevant port sections of the
 *  online documentation.
 *
 *  http://www.FreeRTOS.org - Documentation, latest information, license and
 *  contact details.
 *
 * devConfig.h for Agon Light
 *
 * Device Configuration for FreeRTOS for Agon Light by Julian Rose, 2024, 
 * with copyright assigned to Amazon Web Services as for FreeRTOS version 10.5.
 *
 * DEV application-specific C function definitions.
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * Definitions to configure DEV functions in devapi.c
 * Each definition is assigned a constant value
 * For 'configUSE*' the values are 0 (omit) or 1 (include) that function set
 *  in the build
 * Omitting unnecessary operations reduces the built code size
 *
 * MOS includes simple functions to access UART1 and the I2C pins at the Agon
 * rear GPIO connector. But these are not safeguarded, and there is no support
 * for SPI or GPIO. Devapi fills in these omissions. 
*/


#if !defined( DEVCONFIG_H )
#define DEVCONFIG_H


/* Devices
 *   configUSE_DRV_UART      0 = MOS API non-buffered, single character i/o
 *                           1 = DEV API safeguarded, buffered multi-character i/o
 *   configUSE_DRV_I2C       0 = MOS API Single Master mode
 *                           1 = DEV API Multi-Master mode safeguarded
 *   configUSE_DRV_SPI       0 = disable support (MOS only supports SD-card)
 *                           1 = DEV API safeguarded interface
 *   configUSE_DRV_GPIO      0 = disable support (MOS does not support GPIO)
 *                           1 = DEV API safeguarded interface
*/
#define configUSE_DRV_UART               0
#define configUSE_DRV_I2C                1
#define configUSE_DRV_SPI                0
#define configUSE_DRV_GPIO               0


/* Safeguarding
     configUSE_DEV_SAFEGUARDS  1 = check the pin number range 
                                   check alloction of pins does not overlap
                                   you might opt for this in a debug build
                               0 = smaller and faster code, but unsafeguarded
                                   you might opt for this in a release build
*/
#define configUSE_DEV_SAFEGUARDS         1


/* Interrupts
     configUSE_FAST_INTERRUPTS 1 = Application-sourced FAST_INTERRUPT_HANDLER 
                                     ISR is stored in IVT. 
                                   No DEV_MAJOR or DEV_MINOR params.
                                   Need a dedicated ISR per device
                                   ISR exit with a RETI.L epilogue (refer to 
                                     devgpio::gpioisr)
                               0 = Application-sourced INTERRUPT_HANDLER is 
                                     called from DEV API ISR.
                                   DEV_MAJOR, DEV_MINOR params.
                                   Devices may share a common handler
                                   All in C language
*/
#define configUSE_FAST_INTERRUPTS        0


/* UART
     configDRV_UART_BUFFER_SZ         UART DEV maintains separate Rx and Tx 
                                      buffers, each of configDRV_UART_BUFFER_SZ 
                                      bytes in size.
                                      Values in range 16..1024
     configDRV_UART_UNBUFFERED_DELAY  uart_read and uart_write timeout in ticks
                                      before returning incomplete.transaction.
                                      uart_read_buffered and uart_write_buffered
                                      always return without waiting.
                                      Values in range 0..portMAX_DELAY
*/
#define configDRV_UART_BUFFER_SZ         128
#define configDRV_UART_UNBUFFERED_DELAY  ( configTICK_RATE_HZ * 1 )


/* I2C
     configDRV_I2C_BUFFER_NUM         I2C DEV maintains configDRV_I2C_BUFFER_NUM
                                      Slave Receive role buffers. As a guide,
                                      this may correspond to the number of attached 
                                      devices, or to the noisiest device data rate.
                                      Values in range 1..128
     configDRV_I2C_BUFFER_SZ          Each Slave Receive buffer is of size
                                      configDRV_I2C_BUFFER_SZ bytes. As a guide
                                      this may correspond to the longest message
                                      that can be received from any device.
                                      Values in range 1..255
     configDRV_I2C_TRANSITION_RETRY   In case of Bus R/W State Transition errors,
                                      typically caused by a bus conflict, DEV I2C 
                                      can auto retry a number of times before 
                                      returning an error code.
                                      Values in range 0..8
     configDRV_I2C_MAX_DELAY          i2c_readm and i2c_writem timeout in ticks
                                      before returning incomplete.transaction.
                                      Values in range 0..portMAX_DELAY
*/
#define configDRV_I2C_BUFFER_NUM          2
#define configDRV_I2C_BUFFER_SZ          32
#define configDRV_I2C_TRANSITION_RETRY    1
#define configDRV_I2C_MAX_DELAY          ( configTICK_RATE_HZ * 1 )

#endif /* DEVCONFIG_H */
