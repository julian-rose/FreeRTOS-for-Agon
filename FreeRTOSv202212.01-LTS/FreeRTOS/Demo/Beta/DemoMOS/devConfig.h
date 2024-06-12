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
 *   configUSE_DRV_UART      0 = use basic MOS interface in mosapi.h, but DEV
 *                               won't be able to safeguard access
 *                           1 = use safeguarded DEV interface
 *   configUSE_DRV_I2C       0 = use basic MOS interface in mosapi.h, but DEV
 *                               won't be able to safeguard access
 *                           1 = use safeguarded DEV interface
 *   configUSE_DRV_SPI       0 = disable support
 *                           1 = enable safeguarded DEV interface
 *   configUSE_DRV_GPIO      0 = disable support
 *                           1 = enable safeguarded DEV interface
*/
#define configUSE_DRV_UART      1
#define configUSE_DRV_I2C       1
#define configUSE_DRV_SPI       1
#define configUSE_DRV_GPIO      1


#endif /* DEVCONFIG_H */
