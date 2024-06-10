;*
;* FreeRTOS Kernel V10.5.1
;* Copyright (C) 2024 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
;*
;* SPDX-License-Identifier: MIT
;*
;* Permission is hereby granted, free of charge, to any person obtaining a copy of
;* this software and associated documentation files (the "Software"), to deal in
;* the Software without restriction, including without limitation the rights to
;* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
;* the Software, and to permit persons to whom the Software is furnished to do so,
;* subject to the following conditions:
;*
;* The above copyright notice and this permission notice shall be included in all
;* copies or substantial portions of the Software.
;*
;* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
;* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
;* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
;* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
;* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
;*
;* https://www.FreeRTOS.org
;* https://github.com/FreeRTOS
;*
;*  Please ensure to read the configuration and relevant port sections of the
;*  online documentation.
;*
;*  http://www.FreeRTOS.org - Documentation, latest information, license and
;*  contact details.
;*
;* mosConfig.inc for Agon Light
;*
;* MOS Configuration for FreeRTOS for Agon Light by Julian Rose, 2024, 
;* with copyright assigned to Amazon Web Services as for FreeRTOS version 10.5.
;*
;* MOS application-specific definitions.  
;* Target:  Agon Light (eZ80)
;* Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
;*           ZDSII in ADL mode
;*
;* Definitions to configure MOS for each FreeRTOS / MOS application 
;* Each definition is assigned a constant value
;* For 'configUSE*' the values are 0 (omit) or 1 (include) that function set
;*  in the build
;* Omitting unnecessary operations reduces the built code size
;
IFDEF MOSCONFIG_H

ELSE

MOSCONFIG_H:

; File and Directory function group
;
configUSE_MOS_FILE_OPS                   EQU 1
configUSE_MOS_DIR_OPS                    EQU 1
configUSE_MOS_BUFFERED_FILE_OPS          EQU 1
configUSE_MOS_SYSVARS                    EQU 1

; FAT File and Directory function group
;
configUSE_FFS_FILE_OPS                   EQU 1
configUSE_FFS_DIR_OPS                    EQU 1

; Devices function group
;
configUSE_MOS_KEYBOARD_OPS               EQU 1
configUSE_MOS_DEVICE_RTC                 EQU 1
configUSE_MOS_DEVICE_UART                EQU 1
configUSE_MOS_DEVICE_I2C                 EQU 1


ENDIF ;MOSCONFIG_H
