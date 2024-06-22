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
 * devi2c.c for Agon Light
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The definitions in this file support the DEV API (I2C low-level)
 * for Agon Light (and comptaibles) and the ZDSII compiler
 * Zilog eZ80 ANSI C Compiler Version 3.4 (19101101).
 * Created 20.Jun.2024 by Julian Rose for Agon Light port
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <eZ80F92.h>

#include "FreeRTOS.h"
#include "devConfig.h"
#include "devapi.h"
#include "devapil.h"


#if( 1 == configUSE_DRV_I2C )

/*---- Global Variables -----------------------------------------------------*/
/* Changing any global variable needs to be done within
    portENTER_CRITICAL( );
    {
    }
    portEXIT_CRITICAL( );
*/


/*----- Private functions ---------------------------------------------------*/
/*------ I2C I/O value functions -------------------------------------------*/


/*------ I2C low-level functions -------------------------------------------*/
/* These routines normally called from devapi.c                              */

/* i2c_dev_open
   Device-specific i2c open function for minor device configuration */
POSIX_ERRNO i2c_dev_open(
                       DEV_MODE const mode
                   )
{
    return( POSIX_ERRNO_ENONE );
}


/* i2c_dev_close
   Device-specific i2c close function for device shutdown */
void i2c_dev_close(
                void
            )
{
    // 1. Detach any interrupt handler
}


/* i2c_dev_read
   Device-specific i2c read function */
POSIX_ERRNO i2c_dev_read(
                       void * const buffer,
                       size_t const num_bytes_to_read,
                       size_t * num_bytes_read
                   )
{
    return( POSIX_ERRNO_ENONE );
}


/* i2c_dev_write
   Device-specific I2C write function */
POSIX_ERRNO i2c_dev_write(
                       void * const buffer,
                       size_t const num_bytes_to_write,
                       size_t * num_bytes_written
                   )
{
    return( POSIX_ERRNO_ENONE );
}


#endif  /* 1 == configUSE_DRV_I2C */
