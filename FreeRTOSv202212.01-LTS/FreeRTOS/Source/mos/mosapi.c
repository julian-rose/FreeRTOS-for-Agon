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
 * Created May/2024 by Julian Rose for Agon Light port
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "mosapi.h"


/*----- Function definitions ------------------------------------------------*/
/* mos_printerr
 *   Invoke MOS function 0Fh "mos_getError".
 *   Parameters:
 *     callstr - a text string to precede the error string
 *     err - a MOS_ERRNO number
*/
void mos_printerr( char const * const callstr, MOS_ERRNO const err )
{
    void *buf;
    
#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* Safeguard Zilog library function calls from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( 128 );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ 128 ];
        buf = buff;
    }
#   endif

    if( NULL != buf )
    {
        /* safeguard Zilog library function calls from concurrent access */
        portENTER_CRITICAL( );
        {
            ( void )memset( buf, 0, 128 );
        }
        portEXIT_CRITICAL( );

        mos_geterror( err, buf, 128 );
        ( void )printf( "\r\n%s > Err : %d : %s\r\n", callstr, err, buf );

#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            /* safeguard Zilog library function calls from concurrent access */
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\n%s > Err : %d\r\n", callstr, err );
    }
}


/*---------------------------------------------------------------------------*/
