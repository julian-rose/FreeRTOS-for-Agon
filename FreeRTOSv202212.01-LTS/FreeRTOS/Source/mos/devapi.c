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
 * devapi.c for Agon Light
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The definitions in this file support the DEV API
 * for Agon Light (and comptaibles)
 * Created 11.Jun.2024 by Julian Rose for Agon Light port
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include <eZ80F92.h>

#include "FreeRTOS.h"
#include "devConfig.h"
#include "devapi.h"
#include "devapil.h"


/*----- Global Names --------------------------------------------------------*/
#if( 1 == configUSE_DEV_DEVICE_DRIVERS )

#if( 1 == configUSE_DEV_SAFEGUARDS )

PIN_NUM const assigned_pins[ NUM_DEV_MAJOR ]
                           [ NUM_DEV_MINOR ]=
{
    { /* DEV_NUM_GPIO */
      GPIO_13, GPIO_14, GPIO_15, GPIO_16, 
      GPIO_17, GPIO_18, GPIO_19, GPIO_20,
      GPIO_21, GPIO_22, GPIO_23, GPIO_24, 
      0, GPIO_26, 0, 0,                   // GPIO_25 cannot be assigned
      0, 0, 0, 0
    },

    { /* DEV_NUM_UART */
      0, 0, 0, 0, 
      UART_1_TXD, UART_1_RXD, UART_1_RTS, UART_1_CTS,
      UART_1_DTR, UART_1_DSR, UART_1_DCD, UART_1_RI,
      0, 0, 0, 0, 
      0, 0, 0, 0
    },

    { /* DEV_NUM_SPI */
      0, 0, 0, 0, 
      0, 0, 0, 0,
      0, 0, 0, 0,
      SPI_SS, 0, SPI_MISO, 0,
      0, 0, SPI_CLK, SPI_MOSI
    },

    { /* DEV_NUM_I2C */
      0, 0, 0, 0, 
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0, 
      I2C_SDA, I2C_SCL, 0, 0
    }
};

#endif  /*( 1 == configUSE_DEV_SAFEGUARDS )*/


PORT_BITMAP const portmap[ NUM_DEV_MINOR ]=
{
    {  PORT_D, 4 },  // GPIO_13
    {  PORT_D, 5 },  // GPIO_14
    {  PORT_D, 6 },  // GPIO_15
    {  PORT_D, 7 },  // GPIO_16
    {  PORT_C, 0 },  // GPIO_17
    {  PORT_C, 1 },  // GPIO_18
    {  PORT_C, 2 },  // GPIO_19
    {  PORT_C, 3 },  // GPIO_20
    {  PORT_C, 4 },  // GPIO_21
    {  PORT_C, 5 },  // GPIO_22
    {  PORT_C, 6 },  // GPIO_23
    {  PORT_C, 7 },  // GPIO_24
    {  PORT_B, 2 },  // GPIO_25 cannot be assigned
    {  PORT_B, 5 },  // GPIO_26
    
    /* DEV API also controls PORT_B, 4 for SPI, but this is not taken to 
       pinout. It is used as MicroSD_CS, left asserted by MOS. DEV API shall
       de-assert then re-assert PB4 on each SPI operation. */
};


/*---- Global Variables -----------------------------------------------------*/
/* Changing any global variable needs to be done within
    portENTER_CRITICAL( );
    {
    }
    portEXIT_CRITICAL( );
*/
#if( 1 == configUSE_DEV_SAFEGUARDS )

PIN_STATE pinstate[ NUM_DEV_MINOR ]=
{
    PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE,
    PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE,
    PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE,
    PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE,
    PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE, PIN_STATE_FREE
};

#endif  /* ( 1 == configUSE_DEV_SAFEGUARDS ) */

DEV_MODE pinmode[ NUM_DEV_MINOR ]=
{
    DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED,
    DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED,
    DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED,
    DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED,
    DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED,
    DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED,
    DEV_MODE_UNBUFFERED, DEV_MODE_UNBUFFERED
};

#endif  /* ( 1 == configUSE_DEV_DEVICE_DRIVERS ) */


/*----- Private functions ---------------------------------------------------*/
#if( 1 == configUSE_DEV_DEVICE_DRIVERS )


/*------ Generic Pin allocation and free functions --------------------------*/
#if( 1 == configUSE_DEV_SAFEGUARDS )

static POSIX_ERRNO pins_alloc(
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       DEV_MODE const mode 
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    unsigned int const mnr =( minor - PIN_NUM_START );
    int i;

    switch( major )
    {
        case DEV_NUM_GPIO:
        {
            if(( GPIO_13 > minor )||( GPIO_26 < minor ))
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( GPIO_25 == minor )   // PB2 (cannot be assigned due to MOS SPI software)
            {
                ret = POSIX_ERRNO_EADDRINUSE;
            }
            else
            if( 0 == assigned_pins[ DEV_NUM_GPIO ][ mnr ])
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( PIN_STATE_FREE != pinstate[ mnr ])
            {
                ret = POSIX_ERRNO_EBUSY;
            }
            else
            {
                pinstate[ mnr ]= PIN_STATE_INUSE;
                pinmode[ mnr ]= mode;
            }
        }
        break;

        case DEV_NUM_SPI:
        {
            if(( PIN_STATE_FREE == pinstate[ SPI_SS - PIN_NUM_START ])&&
               ( PIN_STATE_FREE == pinstate[ SPI_MISO - PIN_NUM_START ])&&
               ( PIN_STATE_FREE == pinstate[ SPI_CLK - PIN_NUM_START ])&&
               ( PIN_STATE_FREE == pinstate[ SPI_MOSI - PIN_NUM_START ]))
            {
                pinstate[ SPI_SS - PIN_NUM_START ]= PIN_STATE_INUSE;
                pinmode[ SPI_SS - PIN_NUM_START ]= mode;
                pinstate[ SPI_MISO - PIN_NUM_START ]= PIN_STATE_INUSE;
                pinmode[ SPI_MISO - PIN_NUM_START ]= mode;
                pinstate[ SPI_CLK - PIN_NUM_START ]= PIN_STATE_INUSE;
                pinmode[ SPI_CLK - PIN_NUM_START ]= mode;
                pinstate[ SPI_MOSI - PIN_NUM_START ]= PIN_STATE_INUSE;
                pinmode[ SPI_MOSI - PIN_NUM_START ]= mode;
            }
            else
            {
                ret = POSIX_ERRNO_EBUSY;
            }
        }
        break;

        case DEV_NUM_I2C:
        {
            if(( PIN_STATE_FREE == pinstate[ I2C_SDA - PIN_NUM_START ])&&
               ( PIN_STATE_FREE == pinstate[ I2C_SCL - PIN_NUM_START ]))
            {
                pinstate[ I2C_SDA - PIN_NUM_START ]= PIN_STATE_INUSE;
                pinmode[ I2C_SDA - PIN_NUM_START  ]= mode;
                pinstate[ I2C_SCL - PIN_NUM_START]= PIN_STATE_INUSE;
                pinmode[ I2C_SCL - PIN_NUM_START ]= mode;
            }
            else
            {
                ret = POSIX_ERRNO_EBUSY;
            }
        }
        break;

        case DEV_NUM_UART:
        {
            if( 0 == minor )
            {
                ret = POSIX_ERRNO_EBUSY;  /* eZ80 - VDP link */
            }
            else
            if( 1 == minor )
            {
                PIN_NUM end;
            
                if( DEV_MODE_UART_HALF_MODEM & mode )  // HALF or HALF_NULL
                {
                    end = UART_1_CTS;
                }
                else
                if(( DEV_MODE_UART_FULL_MODEM & mode )||  // FULL or FULL_NULL
                   ( DEV_MODE_UART_LOOPBACK & mode ))
                {
                    end = UART_1_RI;
                }
                else
                {
                    end = UART_1_RXD;
                }

                for( i = UART_1_TXD; end >= i; i++ )
                {
                    unsigned int const mnri =( i - PIN_NUM_START );

                    if( PIN_STATE_FREE != pinstate[ mnri ])
                    {
                        ret = POSIX_ERRNO_EBUSY;
                        break;
                    }
                }
            
                if( POSIX_ERRNO_EBUSY != ret )
                {
                    for( i = UART_1_TXD; end >= i; i++ )
                    {
                        unsigned int const mnri =( i - PIN_NUM_START );

                        pinstate[ mnri ]= PIN_STATE_INUSE;
                        pinmode[ mnri ]= mode;
                    }
                }
            }
            else
            {
                ret = POSIX_ERRNO_ENODEV;
            }
        }
        break;
        
        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( ret );
}


static POSIX_ERRNO pins_free(
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    unsigned int const mnr =( minor - PIN_NUM_START );
    int i;

    switch( major )
    {
        case DEV_NUM_GPIO:
        {
            if( 0 == assigned_pins[ major ][ mnr ])
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( PIN_STATE_FREE == pinstate[ mnr ])
            {
                ret = POSIX_ERRNO_ENOTCONN;
            }
            else
            {
                pinstate[ mnr ]= PIN_STATE_FREE;
                pinmode[ mnr ]= DEV_MODE_UNBUFFERED;
            }
        }
        break;

        case DEV_NUM_SPI:
        {
            if(( PIN_STATE_INUSE == pinstate[ SPI_SS - PIN_NUM_START ])&&
               ( PIN_STATE_INUSE == pinstate[ SPI_MISO - PIN_NUM_START ])&&
               ( PIN_STATE_INUSE == pinstate[ SPI_CLK - PIN_NUM_START ])&&
               ( PIN_STATE_INUSE == pinstate[ SPI_MOSI - PIN_NUM_START ]))
            {
                pinstate[ SPI_SS - PIN_NUM_START ]= PIN_STATE_FREE;
                pinmode[ SPI_SS - PIN_NUM_START ]= DEV_MODE_UNBUFFERED;
                pinstate[ SPI_MISO - PIN_NUM_START ]= PIN_STATE_FREE;
                pinmode[ SPI_MISO - PIN_NUM_START ]= DEV_MODE_UNBUFFERED;
                pinstate[ SPI_CLK - PIN_NUM_START ]= PIN_STATE_FREE;
                pinmode[ SPI_CLK - PIN_NUM_START ]= DEV_MODE_UNBUFFERED;
                pinstate[ SPI_MOSI - PIN_NUM_START ]= PIN_STATE_FREE;
                pinmode[ SPI_MOSI - PIN_NUM_START ]= DEV_MODE_UNBUFFERED;
            }
            else
            {
                ret = POSIX_ERRNO_ENOTCONN;
            }
        }
        break;

        case DEV_NUM_I2C:
        {
            if(( PIN_STATE_INUSE == pinstate[ I2C_SDA - PIN_NUM_START ])&&
               ( PIN_STATE_INUSE == pinstate[ I2C_SCL - PIN_NUM_START ]))
            {
                pinstate[ I2C_SDA - PIN_NUM_START ]= PIN_STATE_FREE;
                pinmode[ I2C_SDA - PIN_NUM_START ]= DEV_MODE_UNBUFFERED;
                pinstate[ I2C_SCL - PIN_NUM_START ]= PIN_STATE_FREE;
                pinmode[ I2C_SCL - PIN_NUM_START ]= DEV_MODE_UNBUFFERED;
            }
            else
            {
                ret = POSIX_ERRNO_ENOTCONN;
            }
        }
        break;

        case DEV_NUM_UART:
        {
            if( 0 == minor )
            {
                ret = POSIX_ERRNO_EBUSY;  /* eZ80 - VDP link */
            }
            else
            if( 1 == minor )
            {
                PIN_NUM end;

                if(( DEV_MODE_UART_FULL_MODEM & pinmode[ UART_1_DTR - PIN_NUM_START ])||  // FULL or FULL_NULL
                   ( DEV_MODE_UART_LOOPBACK & pinmode[ UART_1_DTR - PIN_NUM_START ]))
                {
                    end = UART_1_RI;
                }
                else
                if( DEV_MODE_UART_HALF_MODEM & pinmode[ UART_1_RTS - PIN_NUM_START ]) // HALF or HALF_NULL
                {
                    end = UART_1_CTS;
                }
                else
                {
                    end = UART_1_RXD;
                }
            
                for( i = UART_1_TXD; end >= i; i++ )
                {
                    unsigned int const mnri =( i - PIN_NUM_START );

                    if( PIN_STATE_FREE == pinstate[ mnri ])
                    {
                        ret = POSIX_ERRNO_ENOTCONN;
                        break;
                    }
                }
            
                if( POSIX_ERRNO_ENOTCONN != ret )
                {
                    for( i = UART_1_TXD; end >= i; i++ )
                    {
                        unsigned int const mnri =( i - PIN_NUM_START );

                        pinstate[ mnri ]= PIN_STATE_FREE;
                        pinmode[ mnri ]= DEV_MODE_UNBUFFERED;
                    }
                }
            }
            else
            {
                ret = POSIX_ERRNO_ENODEV;
            }
        }
        break;
        
        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( ret );
}

#endif  /*( 1 == configUSE_DEV_SAFEGUARDS )*/


/*------ Private Generic Driver Functions -----------------------------------*/

#if( 1 == configUSE_DEV_SAFEGUARDS )

/* dev_open
   Generic function to Open a device for i/o
   Checks the major and minor device numbers range
   Checks that the required minor device pins are free
   Invokes the device-specific open function for minor device configuration */
static POSIX_ERRNO dev_open( 
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       DEV_MODE const mode,
                       ...
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    va_list args;

    /* 1. allocate the device pins; 
          this also tests major, minor numbers,
          and if the device is already open */
    portENTER_CRITICAL( );
    {
        ret = pins_alloc( major, minor, mode );
    }
    portEXIT_CRITICAL( );

    /* 2. open the device */
    if( POSIX_ERRNO_ENONE == ret )
    {
        va_start( args, mode );

        switch( major )
        {
            case DEV_NUM_GPIO:
            {
                void * genarg;  // 24-bit int-sized argument (including pointers)
                genarg = va_arg( args, void * );
                va_end( args );

#               if( 1 == configUSE_DRV_GPIO )||( 1 == configUSE_DRV_SPI )
                {
                    ret = gpio_dev_open( minor, mode, genarg );
                }
#               endif /*( 1 == configUSE_DRV_GPIO )*/
            }
            break;

            case DEV_NUM_UART:
            {
                void * params;  // 24-bit int-sized argument (including pointers)
                params = va_arg( args, void * );
                va_end( args );

#               if( 1 == configUSE_DRV_UART )
                {
                    ret = uart_dev_open( mode, params );
                }
#               endif /*( 1 == configUSE_DRV_UART )*/
            }
            break;

            case DEV_NUM_SPI:
            {
                void * params;  // 24-bit int-sized argument (including pointers)
                params = va_arg( args, void * );
                va_end( args );

#               if( 1 == configUSE_DRV_SPI )
                {
                    ret = spi_dev_open( minor, mode, params );
                }
#               endif /*( 1 == configUSE_DRV_SPI )*/
            }
            break;

            case DEV_NUM_I2C:
            {
                void * slaveAddr;
                void * i2cHandler;  // 24-bit int-sized argument (including pointers)
                slaveAddr = va_arg( args, void* );
                i2cHandler = va_arg( args, void * );
                va_end( args );

#               if( 1 == configUSE_DRV_I2C )
                {
                    ret = i2c_dev_open( mode, slaveAddr, i2cHandler );
                }
#               endif /*( 1 == configUSE_DRV_I2C )*/
            }
            break;

            default:  /* already tested major number in pins_assign */
            {
                va_end( args );
            }
            break;
        }

        if( POSIX_ERRNO_ENONE != ret )
        {
            portENTER_CRITICAL( );
            {
                pins_free( major, minor );
            }
            portEXIT_CRITICAL( );
        }
    }

    return( ret );
}


/* dev_close
   Generic function to Close a previously opened device
   Checks the major and minor device numbers range
   Checks that the required minor device pins are free
   Invokes the device-specific close function for minor device closure */
static void dev_close( 
                DEV_NUM_MAJOR const major,
                DEV_NUM_MINOR const minor
            )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    /* 1. close the device and detach any interrupt handlers */
    switch( major )
    {
        case DEV_NUM_GPIO:
        {
            if(( GPIO_13 > minor )||( GPIO_26 < minor ))
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( GPIO_25 == minor )   // PB2 (cannot be assigned due to MOS SPI software)
            {
                ret = POSIX_ERRNO_EADDRINUSE;
            }
            else
            if( DEV_MODE_UNBUFFERED == pinmode[( minor - PIN_NUM_START )])
            {
                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
#               if( 1 == configUSE_DRV_GPIO )||( 1 == configUSE_DRV_SPI )
                {
                    gpio_dev_close( minor );
                }
#               endif /*( 1 == configUSE_DRV_GPIO )*/
            }
        }
        break;

        case DEV_NUM_UART:
        {
            if( DEV_MODE_UNBUFFERED == pinmode[( UART_1_TXD - PIN_NUM_START )])
            {
                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
#               if( 1 == configUSE_DRV_UART )
                {
                    uart_dev_close( );
                }
#               endif /*( 1 == configUSE_DRV_UART )*/
            }
        }
        break;

        case DEV_NUM_SPI:
        {
            if( DEV_MODE_UNBUFFERED == pinmode[( SPI_SS - PIN_NUM_START )])
            {
                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
#               if( 1 == configUSE_DRV_SPI )
                {
                    spi_dev_close( minor );
                }
#               endif /*( 1 == configUSE_DRV_SPI )*/
            }
        }
        break;

        case DEV_NUM_I2C:
        {
            if(( I2C_MINOR_MASTER != minor )&&( I2C_MINOR_SLAVE != minor ))
            {
                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
#               if( 1 == configUSE_DRV_I2C )
                {
                    i2c_dev_close( );
                }
#               endif /*( 1 == configUSE_DRV_I2C )*/
            }
        }
        break;

        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    /* 2. Free the allocated pins */
    if(( POSIX_ERRNO_ENODEV != ret )&&
       ( POSIX_ERRNO_EADDRINUSE != ret )&&
       ( POSIX_ERRNO_ENSRCH != ret ))
    {
        portENTER_CRITICAL( );
        {
            pins_free( major, minor );
        }
        portEXIT_CRITICAL( );
    }
}


/* dev_read
   Read data from a previously opened device
   Calling task may block if opened in UART_MODE_UNBUFFERED */
static POSIX_ERRNO dev_read( 
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       void * const buffer,
                       size_t const num_bytes_to_read,
                       size_t * num_bytes_read,
                       POSIX_ERRNO *result
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    switch( major )
    {
        case DEV_NUM_GPIO:
        {
            if(( GPIO_13 > minor )||( GPIO_26 < minor ))
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( GPIO_25 == minor )   // PB2 (cannot be assigned due to MOS SPI software)
            {
                ret = POSIX_ERRNO_EADDRINUSE;
            }
            else
            if( DEV_MODE_UNBUFFERED == pinmode[( minor - PIN_NUM_START )])
            {
                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
#               if( 1 == configUSE_DRV_GPIO )||( 1 == configUSE_DRV_SPI )
                {
                    ret = gpio_dev_read( minor, buffer );
                }
#               endif /*( 1 == configUSE_DRV_GPIO )*/
            }
        }
        break;

        case DEV_NUM_UART:
        {
#           if( 1 == configUSE_DRV_UART )
            {
                ret = uart_dev_read(
                          buffer, 
                          num_bytes_to_read,
                          num_bytes_read,
                          result
                );
            }
#           endif /*( 1 == configUSE_DRV_UART )*/
        }
        break;

        case DEV_NUM_SPI:
        {
            /* performed within spi_read */
        }
        break;

        case DEV_NUM_I2C:
        {
#           if( 1 == configUSE_DRV_I2C )
            {
                if( I2C_MINOR_MASTER == minor )
                {
                    ret = i2c_dev_readm(
                              buffer, 
                              num_bytes_to_read,
                              num_bytes_read,
                              result );
                }
                else
                if( I2C_MINOR_SLAVE == minor )
                {
                    ret = i2c_dev_reads(
                              buffer, 
                              num_bytes_to_read,
                              num_bytes_read,
                              result );
                }
                else
                {
                    ret = POSIX_ERRNO_ENODEV;
                }
            }
#           endif /*( 1 == configUSE_DRV_I2C )*/
        }
        break;

        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( ret );
}


/* dev_write
   Write data to a previously opened device
   Calling task may block if opened in UART_MODE_UNBUFFERED */
static POSIX_ERRNO dev_write( 
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       void * const buffer,
                       size_t const num_bytes_to_write,
                       size_t * const num_bytes_written,
                       POSIX_ERRNO *result
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    switch( major )
    {
        case DEV_NUM_GPIO:
        {
            if(( GPIO_13 > minor )||( GPIO_26 < minor ))
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( GPIO_25 == minor )   // PB2 (cannot be assigned due to MOS SPI software)
            {
                ret = POSIX_ERRNO_EADDRINUSE;
            }
            else
            if( DEV_MODE_UNBUFFERED == pinmode[( minor - PIN_NUM_START )])
            {
                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
#               if(( 1 == configUSE_DRV_GPIO )||( 1 == configUSE_DRV_SPI ))
                {
                    ret = gpio_dev_write( minor, *(( unsigned char * )buffer ));
                }
#               endif /*( 1 == configUSE_DRV_GPIO )*/
            }
        }
        break;

        case DEV_NUM_UART:
        {
#           if( 1 == configUSE_DRV_UART )
            {
                ret = uart_dev_write(
                          buffer, 
                          num_bytes_to_write,
                          num_bytes_written,
                          result
                );
            }
#           endif /*( 1 == configUSE_DRV_UART )*/
        }
        break;

        case DEV_NUM_SPI:
        {
            if( DEV_MODE_UNBUFFERED & pinmode[( SPI_SS - PIN_NUM_START )])
            {
                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
#               if( 1 == configUSE_DRV_SPI )
                {
                    ret = spi_dev_write( minor, buffer, num_bytes_to_write );
                }
#               endif /*( 1 == configUSE_DRV_SPI )*/
            }
        }
        break;

        case DEV_NUM_I2C:
        {
#           if( 1 == configUSE_DRV_I2C )
            {
                if( I2C_MINOR_MASTER == minor )
                {
                    ret = i2c_dev_writem(
                              buffer, 
                              num_bytes_to_write,
                              num_bytes_written,
                              result );
                }
                else
                if( I2C_MINOR_SLAVE == minor )
                {
                    ret = i2c_dev_writes(
                              buffer, 
                              num_bytes_to_write,
                              num_bytes_written,
                              result );
                }
                else
                {
                    ret = POSIX_ERRNO_ENODEV;
                }
            }
#           endif /*( 1 == configUSE_DRV_I2C )*/
        }
        break;

        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( ret );
}


/* dev_poll
   Retrieve buffers state in previously opened device */
static POSIX_ERRNO dev_poll( 
                       DEV_NUM_MAJOR const major,
                       unsigned short const minor,
                       size_t * num_tx_bytes_buffered,  // write buffer status
                       size_t * num_rx_bytes_buffered,  // read buffer status
                       void * status                    // general status word
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    switch( major )
    {
        case DEV_NUM_GPIO:
        {
            if(( GPIO_13 > minor )||( GPIO_26 < minor ))
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( GPIO_25 == minor )   // PB2 (cannot be assigned due to MOS SPI software)
            {
                ret = POSIX_ERRNO_EADDRINUSE;
            }
            else
            {
                if( num_tx_bytes_buffered ) *num_tx_bytes_buffered = 0;
                if( num_rx_bytes_buffered ) *num_rx_bytes_buffered = 0;
                if( status ) *( unsigned char * )status = 0;
            }
        }
        break;

        case DEV_NUM_UART:
        {
            if( 1 != minor )
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            {
#               if( 1 == configUSE_DRV_UART )
                {
                    ret = uart_dev_poll(
                              num_tx_bytes_buffered,   // write buffer status
                              num_tx_bytes_buffered,  // read buffer status
                              status               // general status word
                    );
                }
#               else
                {
                    if( num_tx_bytes_buffered ) *num_tx_bytes_buffered = 0;
                    if( num_rx_bytes_buffered ) *num_rx_bytes_buffered = 0;
                    if( status ) *( unsigned char * )status = 0;
                }
#               endif /*( 1 == configUSE_DRV_UART )*/
            }
        }
        break;

        case DEV_NUM_SPI:
        {
             if( num_tx_bytes_buffered ) *num_tx_bytes_buffered = 0;
             if( num_rx_bytes_buffered ) *num_rx_bytes_buffered = 0;
             if( status ) *( unsigned char * )status = 0;
        }
        break;

        case DEV_NUM_I2C:
        {
             if( num_tx_bytes_buffered ) *num_tx_bytes_buffered = 0;
             if( num_rx_bytes_buffered ) *num_rx_bytes_buffered = 0;
             if( status ) *( unsigned char * )status = 0;
        }
        break;

        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( ret );
}


/* dev_ioctl
   Perform an I/O control operation on a previously opened device
     I/O operations can be one of three types:
       Executive - in which param is generally NULL
       Set - in which param is an input
       Get - in which param is an output */
static POSIX_ERRNO dev_ioctl( 
                       DEV_NUM_MAJOR const major,
                       unsigned short const minor,
                       DEV_IOCTL const cmd,          // operation to perform
                       void * param                  // operation arguments
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    switch( major )
    {
        case DEV_NUM_GPIO:
        {
            if(( GPIO_13 > minor )||( GPIO_26 < minor ))
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            if( GPIO_25 == minor )   // PB2 (cannot be assigned due to MOS SPI software)
            {
                ret = POSIX_ERRNO_EADDRINUSE;
            }
            else
            {
                ret = POSIX_ERRNO_EIO;
            }
        }
        break;

        case DEV_NUM_UART:
        {
            if( 1 != minor )
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            {
#               if( 1 == configUSE_DRV_UART )
                {
                    ret = uart_dev_ioctl(
                              cmd,
                              param
                    );
                }
#               endif /*( 1 == configUSE_DRV_UART )*/
            }
        }
        break;

        case DEV_NUM_SPI:
        {
            ret = POSIX_ERRNO_EIO;
        }
        break;

        case DEV_NUM_I2C:
        {
            if(( I2C_MINOR_SLAVE != minor )&&( I2C_MINOR_MASTER != minor ))
            {
                ret = POSIX_ERRNO_ENODEV;
            }
            else
            {
#               if( 1 == configUSE_DRV_I2C )
                {
                    ret = i2c_dev_ioctl(
                              cmd,
                              param
                    );
                }
#               endif /*( 1 == configUSE_DRV_I2C )*/
            }
        }
        break;

        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( ret );
}

#endif  /*( 1 == configUSE_DEV_SAFEGUARDS )*/

#endif  /* configUSE_DEV_DEVICE_DRIVERS */


/*----- DEV API User Function definitions -----------------------------------*/
#if( 1 == configUSE_DRV_GPIO )||( 1 == configUSE_DRV_SPI )
    /* DEV_API: gpio_open
       Open GPIO for i/o
       Get any mode-specific parameters
       Invoke either generic or low-level driver as configured */
POSIX_ERRNO gpio_open( 
                GPIO_PIN_NUM const pin, 
                DEV_MODE const mode,
                ...
            )
{
    POSIX_ERRNO res;
    va_list args;

    
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d : mode = 0x%x\r\n", "devapi.c", __LINE__, mode );
#   endif
    
    va_start( args, mode );

    switch( mode )
    {
        case DEV_MODE_GPIO_INTRDE:
        case DEV_MODE_GPIO_INTRLOW:
        case DEV_MODE_GPIO_INTRHIGH:
        case DEV_MODE_GPIO_INTRFALL:
        case DEV_MODE_GPIO_INTRRISE:
        {
            void * interrupt_handler;

            interrupt_handler = va_arg( args, void* );
            va_end( args );

#           if defined( _DEBUG )&& 0
            {
                ( void )printf( "%s : %d : interrupt_handler = %p\r\n", 
                                "devapi.c", __LINE__, interrupt_handler );    
            }
#           endif
#           if( 1 == configUSE_DEV_SAFEGUARDS )
            {
                res = dev_open( DEV_NUM_GPIO, pin, mode, interrupt_handler );
            }
#           else
            {
                res = gpio_dev_open( pin, mode, interrupt_handler );
            }
#           endif
        }
        break;

        case DEV_MODE_GPIO_OUT:
        {
            int init_value;

            init_value = va_arg( args, int );
            va_end( args );

#           if( 1 == configUSE_DEV_SAFEGUARDS )
            {
                res = dev_open( DEV_NUM_GPIO, pin, mode, init_value );
            }
#           else
            {
                res = gpio_dev_open( pin, mode, init_value );
            }
#           endif
        }
        break;

        case DEV_MODE_GPIO_IN:
        case DEV_MODE_GPIO_DIO:
        case DEV_MODE_GPIO_SIO:
        case DEV_MODE_GPIO_ALTFUNC:
        {
            va_end( args );

#           if( 1 == configUSE_DEV_SAFEGUARDS )
            {
                res = dev_open( DEV_NUM_GPIO, pin, mode, NULL );
            }
#           else
            {
                res = gpio_dev_open( pin, mode );
            }
#           endif
        }
        break;

        default:
        {
            va_end( args );
            res = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( res );
}


    /* DEV_API: gpio_close
       Close previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_close( 
                GPIO_PIN_NUM const pin
            )
{
#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        dev_close( DEV_NUM_GPIO, pin );
    }
#   else
    {
        gpio_dev_close( pin );
    }
#   endif

    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: gpio_read
       Read data from previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_read(
                GPIO_PIN_NUM const pin,
                unsigned char * const buffer
            )
{
#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        dev_read( DEV_NUM_GPIO, pin, buffer, 1, NULL, NULL );
    }
#   else
    {
        gpio_dev_read( pin, buffer );
    }
#   endif

    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: gpio_write
       Write data to previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_write(
                GPIO_PIN_NUM const pin,
                unsigned char const val
            )
{
#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        dev_write( DEV_NUM_GPIO, pin, &val, 1, NULL, NULL );
    }
#   else
    {
        gpio_dev_write( pin, val );
    }
#   endif

    return( POSIX_ERRNO_ENONE );
}


#endif  /* 1 == configUSE_DRV_GPIO */


#if( 1 == configUSE_DRV_I2C )
/* DEV_API: i2c_open
   Open I2C for i/o */
POSIX_ERRNO i2c_open( 
                DEV_MODE const mode,
                ...
            )
{
    POSIX_ERRNO res;
    va_list args;

    
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d : mode = 0x%x\r\n", "devapi.c", __LINE__, mode );
#   endif
    
    va_start( args, mode );

    switch( mode )
    {
        case DEV_MODE_I2C_MULTI_MASTER :
        {
            I2C_ADDR * slaveAddr;
            void * i2cHandler;

            slaveAddr = va_arg( args, void* );
            i2cHandler = va_arg( args, void* );  // may be NULL
            va_end( args );

#           if defined( _DEBUG )&& 0
            {
                ( void )printf( "%s : %d : slaveAddr = %p : handler = %p\r\n", 
                                "devapi.c", __LINE__, slaveAddr, i2cHandler );    
            }
#           endif
#           if( 1 == configUSE_DEV_SAFEGUARDS )
            {
                res = dev_open( DEV_NUM_I2C, I2C_MINOR_SLAVE, 
                                mode, slaveAddr, i2cHandler );
            }
#           else
            {
                res = i2c_dev_open( mode, slaveAddr, i2cHandler );
            }
#           endif
        }
        break;

        case DEV_MODE_I2C_SINGLE_MASTER :
        {
            va_end( args );

#           if( 1 == configUSE_DEV_SAFEGUARDS )
            {
                res = dev_open( DEV_NUM_I2C, I2C_MINOR_MASTER, 
                                mode, NULL, NULL );
            }
#           else
            {
                res = i2c_dev_open( mode, NULL, NULL );
            }
#           endif
        }
        break;

        default:
        {
            va_end( args );
            res = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    return( res );
}


/* DEV_API: i2c_close
   Close previously opened I2C */
POSIX_ERRNO i2c_close( 
                void 
            )
{
#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        dev_close( DEV_NUM_I2C, I2C_MINOR_MASTER );
    }
#   else
    {
        i2c_dev_close( );
    }
#   endif

    return( POSIX_ERRNO_ENONE );
}


/* DEV_API: i2c_readm (Master Receive)
   Inititate a Read from a target slave, through previously opened I2C.
   The target slave address and target register address are embedded in the 
   I2C_MSG parameter.
   The slave address can be a 7-bit [1 byte] or a 10-bit [2 byte] value.
   All non-zero register addresses are valid; register address zero is ignored.
*/
POSIX_ERRNO i2c_readm(
                I2C_MSG const * message,
                size_t const msgLen,
                size_t * num_bytes_buffered,
                POSIX_ERRNO * result
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {        
        ret = dev_read( 
                  DEV_NUM_I2C, I2C_MINOR_MASTER, 
                  message, msgLen,
                  num_bytes_buffered, result );
    }
#   else
    {
        ret = i2c_dev_readm( message, msgLen, num_bytes_buffered, result );
    }
#   endif
    
    return( ret );
}


/* DEV_API: i2c_writem (Master Transmit)
   Initiate a Write to a target slave through previously opened I2C
   The target slave address is embedded in the I2C_MSG parameter */
POSIX_ERRNO i2c_writem(
                I2C_MSG const * const message,
                size_t const msgLen,
                size_t * num_bytes_sent,
                POSIX_ERRNO * result
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {        
        ret = dev_write( 
                  DEV_NUM_I2C, I2C_MINOR_MASTER, 
                  message, msgLen,
                  num_bytes_sent, result );
    }
#   else
    {
        ret = i2c_dev_writem( message, msgLen, num_bytes_sent, result );
    }
#   endif
    
    return( ret );
}


/* DEV_API: i2c_reads (Slave Receive)
   Application calls i2c_reads to collect data received from a remote 
   master Slave Receive messages. */
POSIX_ERRNO i2c_reads(
                unsigned char const * buffer,
                size_t const num_bytes_to_read,
                size_t * num_bytes_buffered,
                POSIX_ERRNO * result
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        
        ret = dev_read( 
                  DEV_NUM_I2C, I2C_MINOR_SLAVE, 
                  buffer, num_bytes_to_read,
                  num_bytes_buffered, result );
    }
#   else
    {
        ret = i2c_dev_reads( buffer, msgLen, num_bytes_buffered, result );
    }
#   endif
    
    return( ret );
}


/* DEV_API: i2c_writes (Slave Transmit)
   ISR Handler Task (i2cHandler) calls i2c_writes to respond to a Slave
   Transmit request from a remote bus-master.
   This function would not normally be invoked from other tasks. */
POSIX_ERRNO i2c_writes(
                unsigned char const * const buffer,
                size_t const num_bytes_to_write,
                size_t * num_bytes_sent,
                POSIX_ERRNO * result
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_write( 
                  DEV_NUM_I2C, I2C_MINOR_SLAVE, 
                  buffer, num_bytes_to_write,
                  num_bytes_sent, result );
    }
#   else
    {
        ret = i2c_dev_writes( message, msgLen, num_bytes_sent, result );
    }
#   endif
    
    return( ret );
}


/* DEV_API: i2c_ioctl
       Perform an IO Control function on previously opened I2C */
POSIX_ERRNO i2c_ioctl(
                DEV_IOCTL const cmd,
                void * param
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_ioctl( 
                  DEV_NUM_I2C, I2C_MINOR_MASTER, cmd, param
              );
    }
#   else
    {
        ret = i2c_dev_ioctl(
                  cmd, param
              );
    }
#   endif
    
    return( ret );
}

#endif /* 1 == configUSE_DRV_I2C */


#if( 1 == configUSE_DRV_SPI )
/* DEV_API: spi_open
   Open SPI for i/o */
POSIX_ERRNO spi_open( 
                GPIO_PIN_NUM const slaveSelect, 
                DEV_MODE const mode,
                SPI_PARAMS const * params
            )
{
    POSIX_ERRNO ret;

    switch( DEV_MODE_SPI_MASK & mode )
    {
        case DEV_MODE_SPI_DEFAULT :
        {
#           if( 1 == configUSE_DEV_SAFEGUARDS )
            {
                ret = dev_open( DEV_NUM_SPI, slaveSelect, mode, params );
            }
#           else
            {
                ret = spi_dev_open( slaveSelect, mode, params );
            }
#           endif
        }
        break;

        default :
        {
            ret = POSIX_ERRNO_EINVAL;
        }
        break;
    }

    return( ret );
}


/* DEV_API: spi_close
   Close previously opened SPI */
POSIX_ERRNO spi_close( 
                GPIO_PIN_NUM const slaveSelect
            )
{
#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        dev_close( DEV_NUM_SPI, slaveSelect );
    }
#   else
    {
        spi_dev_close( slaveSelect );
    }
#   endif

    return( POSIX_ERRNO_ENONE );
}


/* DEV_API: spi_read
   Read data from a previously opened SPI.
   This is an uncommon read API and so for configUSE_DEV_SAFEGUARDS we bypass 
     calling dev_read, and instead do those checks locally. */
POSIX_ERRNO spi_read(
                GPIO_PIN_NUM const slaveSelect,
                unsigned char * tx_buffer,
                unsigned char * rx_buffer,
                size_t const num_bytes_to_transceive
            )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        if( DEV_MODE_UNBUFFERED == pinmode[( SPI_SS - PIN_NUM_START )])
        {
            ret = POSIX_ERRNO_ENSRCH;
        }
    }
#   endif
    
    if( POSIX_ERRNO_ENONE == ret )
    {
        ret = spi_dev_read( 
                  slaveSelect, 
                  tx_buffer, 
                  rx_buffer, 
                  num_bytes_to_transceive );
    }
    
    return( ret );
}


/* DEV_API: spi_write
   Write data to a previously opened SPI */
POSIX_ERRNO spi_write(
                GPIO_PIN_NUM const slaveSelect,
                void * const buffer,
                size_t const num_bytes_to_write
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_write( 
                  DEV_NUM_SPI, slaveSelect, 
                  buffer, num_bytes_to_write, NULL, NULL );
    }
#   else
    {
        ret = spi_dev_write( slaveSelect, buffer, num_bytes_to_write );
    }
#   endif
    
    return( ret );
}
#endif /* 1 == configUSE_DRV_SPI */


#if( 1 == configUSE_DRV_UART )
/* DEV_API: uart_open
   Open UART1 for i/o */
POSIX_ERRNO uart_open( 
                DEV_MODE const mode,
                UART_PARAMS const * params
            )
{
    POSIX_ERRNO ret;

        // mode may also contain DEV_MODE_BUFFERED_MASK
    switch( DEV_MODE_UART_MASK & mode )
    {
        case DEV_MODE_UART_NO_FLOWCONTROL :
        case DEV_MODE_UART_SW_FLOWCONTROL :
        case DEV_MODE_UART_HALF_MODEM :
        case DEV_MODE_UART_HALF_NULL_MODEM :
        case DEV_MODE_UART_FULL_MODEM :
        case DEV_MODE_UART_FULL_NULL_MODEM :
        case DEV_MODE_UART_LOOPBACK :
        {
#           if( 1 == configUSE_DEV_SAFEGUARDS )
            {
                ret = dev_open( DEV_NUM_UART, 1, mode, params );
            }
#           else
            {
                ret = uart_dev_open( mode, params );
            }
#           endif
        }
        break;

        default :
        {
            ret = POSIX_ERRNO_EINVAL;
        }
        break;
    }

    return( ret );
}


/* DEV_API: uart_close
   Close previously opened UART1 */
POSIX_ERRNO uart_close( 
                void 
            )
{
#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        dev_close( DEV_NUM_UART, 1 );
    }
#   else
    {
        uart_dev_close( );
    }
#   endif

    return( POSIX_ERRNO_ENONE );
}


/* DEV_API: uart_read
     Read data from a previously opened UART1
     Calling task may block until either the read is complete or an error
       transpires. */
POSIX_ERRNO uart_read(
                void * const buffer,
                size_t const num_bytes_to_read
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_read( 
                  DEV_NUM_UART, 1, 
                  buffer, num_bytes_to_read, NULL, NULL );
    }
#   else
    {
        ret = uart_dev_read( buffer, num_bytes_to_read, NULL, NULL );
    }
#   endif
    
    return( ret );
}


/* DEV_API: uart_read_buffered
   Read data from previously opened UART1
   If the uart is opened with DEV_MODE_BUFFERED in the mode parameters,
     the calling task will not block, but return immediately with any data
     that has been received at the time of calling. It is like a poll that
     receives data immediately.
   If the uart is opened without DEV_MODE_BUFFERED in the mode parameters,
     then the behaviour is like uart_read and the calling task may block. */
POSIX_ERRNO uart_read_buffered(
                void * const buffer,
                size_t const num_bytes_to_read,
                size_t * num_bytes_buffered,
                POSIX_ERRNO * result
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_read( 
                  DEV_NUM_UART, 1, 
                   buffer, num_bytes_to_read, num_bytes_buffered, result );
    }
#   else
    {
        ret = uart_dev_read( 
                  buffer, num_bytes_to_read, num_bytes_buffered, result );
    }
#   endif
    
    return( ret );
}


/* DEV_API: uart_getch
       Read a single byte from the previously opened UART1
       Calling task may block; on error 0xff is returned 
       (0xff may be a valid read too, but this api cannot return ret) */
char uart_getch(
         void
     )
{
    POSIX_ERRNO ret;
    char ch;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_read( DEV_NUM_UART, 1, &ch, 1, NULL, NULL );
    }
#   else
    {
        ret = uart_dev_read( &ch, 1, NULL, NULL );
    }
#   endif
    
    if( POSIX_ERRNO_ENONE != ret )
    {
        ch = 0xff;
    }
    
    return( ch );
}


/* DEV_API: uart_write
     Write data to a previously opened UART1
     Calling task will block until either the write is complete or an error
       happens. */
POSIX_ERRNO uart_write(
                void * const buffer,
                size_t const num_bytes_to_write
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_write( 
                  DEV_NUM_UART, 1, 
                  buffer, num_bytes_to_write, NULL, NULL );
    }
#   else
    {
        ret = uart_dev_write( buffer, num_bytes_to_write, NULL, NULL );
    }
#   endif
    
    return( ret );
}


/* DEV_API: uart_write_buffered
     Write data to a previously opened UART1
     If the uart is opened with DEV_MODE_BUFFERED in the mode parameters,
       the calling task will not block, but return immediately.
     If the uart is opened without DEV_MODE_BUFFERED in the mode parameters,
       then the behaviour is like uart_write and the calling task may block.  */
POSIX_ERRNO uart_write_buffered(
                void * const buffer,
                size_t const num_bytes_to_write,
                size_t * num_bytes_buffered,
                POSIX_ERRNO * result
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_write( 
                  DEV_NUM_UART, 1, 
                  buffer, num_bytes_to_write, num_bytes_buffered, result );
    }
#   else
    {
        ret = uart_dev_write( 
                  buffer, num_bytes_to_write, num_bytes_buffered, result );
    }
#   endif
    
    return( ret );
}


    /* DEV_API: uart_putch
       Write a single byte to the previously opened UART1
       Calling task will return immediately; user must check return value
       Defined in devapi.c */
POSIX_ERRNO uart_putch(
                char const ch
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_write( 
                  DEV_NUM_UART, 1, 
                  &ch, 1, NULL, NULL );
    }
#   else
    {
        ret = uart_dev_write( &ch, 1, NULL, NULL );
    }
#   endif
    
    return( ret );
}


/* DEV_API: uart_poll
   Poll previously opened UART1 */
POSIX_ERRNO uart_poll(
                size_t * num_tx_bytes_buffered,  // write buffer status
                size_t * num_rx_bytes_buffered,  // read buffer status
                UART_MODEM_STATUS *modem_status  // modem activity
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_poll( 
                  DEV_NUM_UART, 1, 
                  num_tx_bytes_buffered,
                  num_rx_bytes_buffered,
                  modem_status
                  );
    }
#   else
    {
        ret = uart_dev_poll(
                  num_tx_bytes_buffered,
                  num_rx_bytes_buffered,
                  modem_status
                  );
    }
#   endif
    
    return( ret );
}


/* DEV_API: uart_ioctl
       Perform an IO Control function on previously opened UART1 */
POSIX_ERRNO uart_ioctl(
                DEV_IOCTL const cmd,
                void * param
            )
{
    POSIX_ERRNO ret;

#   if( 1 == configUSE_DEV_SAFEGUARDS )
    {
        ret = dev_ioctl( 
                  DEV_NUM_UART, 1, cmd, param
                  );
    }
#   else
    {
        ret = uart_dev_ioctl(
                  cmd, param
                  );
    }
#   endif
    
    return( ret );
}

#endif /* configUSE_DRV_UART */
