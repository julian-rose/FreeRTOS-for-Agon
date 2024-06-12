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
 * for Agon Light (and comptaibles) and the ZDSII compiler
 * Zilog eZ80 ANSI C Compiler Version 3.4 (19101101).
 * Created 11/Jun/2024 by Julian Rose for Agon Light port
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "devConfig.h"
#include "devapi.h"


/*----- Constants -----------------------------------------------------------*/
#define configUSE_DEV_DEVICE_DRIVERS                   \
            ( configUSE_DRV_UART | configUSE_DRV_SPI | \
              configUSE_DRV_I2C | configUSE_DRV_GPIO )


/*----- Enumeration Types ---------------------------------------------------*/
/*  Connector pinout
      Extended enumeration of GPIO_PIN_NUM.
      Pin numbers follow Agon Light2; and Agon Origins. 
      (Different pin numbers are used on Console8.)
      Unenumerated pins 1..5 are power
                   pins 6..12 are ESP32 GPIO (not yet accessible through VDP protocol
                   pins 33..34 are power
*/
typedef enum _pin_num
{
    /* Pin Name      Pin#     Function      Ez80 pin name
    --------------   ----     -----------   -------------*/
    GPIO_13        = 13,   // GPIO pin 13   (PD4)
    UART_0_DTR     = 13,   // UART0 DTR     (PD4)
    GPIO_14        = 14,   // GPIO pin 14   (PD5)
    UART_0_DSR     = 14,   // UART0 DSR     (PD5)
    GPIO_15        = 15,   // GPIO pin 15   (PD6)
    UART_0_DCD     = 15,   // UART0 DCD     (PD6)
    GPIO_16        = 16,   // GPIO pin 16   (PD7)
    UART_0_RI      = 16,   // UART0 RI      (PD7)
    GPIO_17        = 17,   // GPIO pin 17   (PC0)
    UART_1_TXD     = 17,   // UART1 TxD     (PC0)
    GPIO_18        = 18,   // GPIO pin 18   (PC1)
    UART_1_RXD     = 18,   // UART1 RxD     (PC1)
    GPIO_19        = 19,   // GPIO pin 19   (PC2)
    UART_1_RTS     = 19,   // UART1 RTS     (PC2)
    GPIO_20        = 20,   // GPIO pin 20   (PC3)
    UART_1_CTS     = 20,   // UART1 CTS     (PC3)
    GPIO_21        = 21,   // GPIO pin 21   (PC4)
    UART_1_DTR     = 21,   // UART1 DTR     (PC4)
    GPIO_22        = 22,   // GPIO pin 22   (PC5)
    UART_1_DSR     = 22,   // UART1 DSR     (PC5)
    GPIO_23        = 23,   // GPIO pin 23   (PC6)
    UART_1_DCD     = 23,   // UART1 DCD     (PC6)
    GPIO_24        = 24,   // GPIO pin 24   (PC7)
    UART_1_RI      = 24,   // UART1 RI      (PC7)
    GPIO_25        = 25,   // GPIO pin 25   (PB2)
    SPI_SS         = 25,   // SPI SS        (PB2)
    GPIO_26        = 26,   // GPIO pin 26   (PB5)
    SPI_MISO       = 27,   // SPI MISO      (PB6)
    SYS_CLKOUT     = 28,   // SYSCLK        (PHI)
    I2C_SDA        = 29,   // I2C SDA       (SDA)
    I2C_SCL        = 30,   // I2C SCL       (SCL)
    SPI_CLK        = 31,   // SPI CLK       (PB3)
    SPI_MOSI       = 32,   // SPI MOSI      (PB7)

    PIN_NUM_END

} PIN_NUM;


typedef enum _gpio_state
{
    GPIO_STATE_FREE = 0,
    GPIO_STATE_INUSE

} GPIO_STATE;


typedef enum _dev_num_major
{
    DEV_NUM_GPIO = 1,
    DEV_NUM_UART = 2,
    DEV_NUM_SPI = 3,
    DEV_NUM_I2C = 4

} DEV_NUM_MAJOR;


/*----- Type Definitions ----------------------------------------------------*/
typedef struct _dev_pin
{
    DEV_NUM_MAJOR majnum;
    GPIO_PIN_NUM pinnum;  // same as device minor number
    GPIO_STATE state;

} DEV_PIN;


/*----- Global Names --------------------------------------------------------*/
static DEV_PIN devpin[ GPIO_PIN_NUM_END ];


/*----- Private functions ---------------------------------------------------*/
#if( 1 == configUSE_DEV_DEVICE_DRIVERS )

/* DEV_API: dev_open
       Open a device for i/o
       Defined in mosapi.c */

static POSIX_ERRNO dev_open( 
                       DEV_NUM_MAJOR const major,
                       unsigned short const minor,  // used by gpio, 0 otherwise
                       DEV_MODE const mode
                   )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: dev_close
       Close a previously opened device
       Defined in mosapi.c */
static void dev_close( 
                DEV_NUM_MAJOR const major,
                unsigned short const minor
            )
{
}


    /* DEV_API: dev_read
       Read data from a previously opened device
       Calling task may block if opened in UART_MODE_UNBUFFERED
       Defined in mosapi.c */
static POSIX_ERRNO dev_read( 
                       DEV_NUM_MAJOR const major,
                       unsigned short const minor,
                       void * const buffer,
                       size_t const num_bytes_to_read,
                       size_t * const num_bytes_read
                   )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: dev_write
       Write data to a previously opened device
       Calling task may block if opened in UART_MODE_UNBUFFERED
       Defined in mosapi.c */
static POSIX_ERRNO dev_write( 
                       DEV_NUM_MAJOR const major,
                       unsigned short const minor,
                       void * const buffer,
                       size_t const num_bytes_to_write,
                       size_t * const num_bytes_written
                   )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: dev_poll
       Retrieve buffers state in previously opened device */
static POSIX_ERRNO dev_poll( 
                       DEV_NUM_MAJOR const major,
                       unsigned short const minor,
                       size_t * num_bytes_buffered,    // content of uart input buffer
                       size_t * num_bytes_free         // free space in uart output buffer
                   )
{
    return( POSIX_ERRNO_ENONE );
}

#endif /* configUSE_DEV_DEVICE_DRIVERS */


/*----- Function definitions ------------------------------------------------*/
#if( 1 == configUSE_DRV_UART )
    /* DEV_API: uart_open
       Open UART1 for i/o
       Defined in devapi.c */
POSIX_ERRNO uart_open( 
                void 
            )
{
//             To enable interrupts assign a handler

    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: uart_close
       Close previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_close( 
                void 
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: uart_read
       Read data from previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_read(
                void * const buffer,
                size_t const num_bytes_to_read
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: uart_write
       Read data from previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_write(
                void * const buffer,
                size_t const num_bytes_to_write,
                size_t * num_bytes_buffered
            )
{
    return( POSIX_ERRNO_ENONE );
}


/* DEV_API: uart_poll
       Poll previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_poll(
                size_t * num_bytes_to_read,    // content of uart input buffer
                size_t * num_bytes_to_write    // free space in uart output buffer
            )
{
    return( POSIX_ERRNO_ENONE );
}
#endif /* configUSE_DRV_UART */


#if( 1 == configUSE_DRV_I2C )
    /* DEV_API: i2c_open
       Open I2C for i/o
       Defined in devapi.c */
POSIX_ERRNO i2c_open( 
                void 
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: i2c_close
       Close previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_close( 
                void 
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: i2c_read
       Read data from previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_read(
                void * const buffer,
                size_t const num_bytes_to_read
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: i2c_write
       Read data from previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_write(
                void * const buffer,
                size_t const num_bytes_to_write
            )
{
    return( POSIX_ERRNO_ENONE );
}
#endif /* 1 == configUSE_DRV_I2C */


#if( 1 == configUSE_DRV_SPI )
    /* DEV_API: spi_open
       Open SPI for i/o
       Defined in devapi.c */
POSIX_ERRNO spi_open( 
                void 
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: spi_close
       Close previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_close( 
                void 
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: spi_read
       Read data from previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_read(
                void * const buffer,
                size_t const num_bytes_to_read
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: spi_write
       Read data from previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_write(
                void * const buffer,
                size_t const num_bytes_to_write
            )
{
    return( POSIX_ERRNO_ENONE );
}
#endif /* 1 == configUSE_DRV_SPI */


#if( 1 == configUSE_DRV_GPIO )
    /* DEV_API: gpio_open
       Open GPIO for i/o
       Defined in devapi.c */
POSIX_ERRNO gpio_open( 
                GPIO_PIN_NUM const pin 
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: gpio_close
       Close previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_close( 
                void 
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: gpio_read
       Read data from previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_read(
                void * const buffer
            )
{
    return( POSIX_ERRNO_ENONE );
}


    /* DEV_API: gpio_write
       Read data from previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_write(
                void * const buffer
            )
{
    return( POSIX_ERRNO_ENONE );
}
#endif  /* 1 == configUSE_DRV_GPIO */
