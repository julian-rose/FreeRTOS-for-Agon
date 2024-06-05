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
 * mosapi.c for Agon Light
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The definitions in this file support the MOS API
 * for Agon Light (and comptaibles) and the ZDSII compiler
 * Zilog eZ80 ANSI C Compiler Version 3.4 (19101101).
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include "mosapi.h"


/*----- Global Names --------------------------------------------------------*/
    /* A place to hold the global MOS-errno strings;
         https://github.com/breakintoprogram/agon-mos/blob/main/src/mos.c
         also                               /agon-mos-main/src_fatfs/ff.h */
MOS_ERRNOS const mos_errnos[ ]= 
{
    { MOS_ERR_OK,                 "OK" },
    { MOS_ERR_SDCARD_ACCESS,      "Error accessing SD card" },
    { MOS_ERR_ASSERT,             "Assertion failed" },
    { MOS_ERR_SDCARD_FAIL,        "SD card failure" },
    { MOS_ERR_FILE_NOT_EXIST,     "Could not find file" },
    { MOS_ERR_PATH_NOT_EXIST,     "Could not find path" },
    { MOS_ERR_PATHNAME_INVALID,   "Invalid path name" },
    { MOS_ERR_DIRECTORY_FULL,     "Access denied or directory full" },
    { MOS_ERR_ACCESS_DENIED,      "Access denied" },
    { MOS_ERR_INVALID_FILE,       "Invalid file/directory object" },
    { MOS_ERR_SDCARD_WRITE,       "SD card is write protected" },
    { MOS_ERR_DRIVE_NUM,          "Logical drive number is invalid" },
    { MOS_ERR_VOL_WORKAREA,       "Volume has no work area" },
    { MOS_ERR_VOL_FILESYSTEM,     "No valid FAT volume" },
    { MOS_ERR_MKFS,               "Error occurred during mkfs" },
    { MOS_ERR_VOL_TIMEOUT,        "Volume timeout" },
    { MOS_ERR_VOL_LOCKED,         "Volume locked" },
    { MOS_ERR_LFN_BUF,            "LFN working buffer could not be allocated" },
    { MOS_ERR_TOO_MANY_FILES,     "Too many open files" },
    { MOS_ERR_PARAM_INVALID,      "Invalid parameter" },
    { MOS_ERR_COMMAND_INVALID,    "Invalid command" },
    { MOS_ERR_EXECUTABLE_INVALID, "Invalid executable" },
};


/*----- Private functions ---------------------------------------------------*/



/*----- Function definitions ------------------------------------------------*/



/*---------------------------------------------------------------------------*/
