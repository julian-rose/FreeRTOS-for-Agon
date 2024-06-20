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
 * devapi.h for Agon Light
 *
 * DEV API for FreeRTOS for Agon Light by Julian Rose, 2024, with copyright 
 * assigned to Amazon Web Services as for FreeRTOS version 10.5.
 *
 * DEV API specific definitions (which extend the MOS API towards POSIX)
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * Definitions for DEV interface functions for FreeRTOS for Agon Light 
 * (and compatibles).
*/

#ifndef DEVAPI_H
#define DEVAPI_H

#ifndef DEVCONFIG_H
# include "devConfig.h"    /* application-specific DEV configuration */
#endif


/*----- Enumeration Types ---------------------------------------------------*/
/* The POSIX errno domain overlays the MOS_ERR domain.
   In both domains value 0 = ok and other values are errors.
   But to distinguish the domains, we left shift the POSIX errors into the second 
   byte of a short (and an int, enum) */
typedef enum _posix_errno
{
    POSIX_ERRNO_ENONE           =(   0 << 8 ),  /* No errors */
    POSIX_ERRNO_ENOPERM         =(   1 << 8 ),  /* User permission denied */
    POSIX_ERRNO_ENOENT          =(   2 << 8 ),  /* No such file or directory */
    POSIX_ERRNO_ENSRCH          =(   3 << 8 ),  /* No such process */
    POSIX_ERRNO_EINTR           =(   4 << 8 ),  /* Interrupted system call */
    POSIX_ERRNO_EIO             =(   5 << 8 ),  /* I/O error */
    POSIX_ERRNO_ENXIO           =(   6 << 8 ),  /* Extended I/O error */
    POSIX_ERRNO_ETOOBIG         =(   7 << 8 ),  /* Argument too long */
    POSIX_ERRNO_NOEXEC          =(   8 << 8 ),  /* Exec format error */
    POSIX_ERRNO_EBADF           =(   9 << 8 ),  /* Bad file number */
    POSIX_ERRNO_ECHILD          =(  10 << 8 ),  /* No child process */
    POSIX_ERRNO_EWOULDBLOCK     =(  11 << 8 ),  /* Operation would block */
    POSIX_ERRNO_ENOMEM          =(  12 << 8 ),  /* Not enough memory */
    POSIX_ERRNO_EACCES          =(  13 << 8 ),  /* Permission denied */
    POSIX_ERRNO_EFAULT          =(  14 << 8 ),  /* Bad address */
    POSIX_ERRNO_NOTBLK          =(  15 << 8 ),  /* Not a block device */
    POSIX_ERRNO_EBUSY           =(  16 << 8 ),  /* Device busy */
    POSIX_ERRNO_EEXIST          =(  17 << 8 ),  /* File exists */
    POSIX_ERRNO_EXDEV           =(  18 << 8 ),  /* Cross-device link */
    POSIX_ERRNO_ENODEV          =(  19 << 8 ),  /* No such device */
    POSIX_ERRNO_ENOTDIR         =(  20 << 8 ),  /* Not a directory */
    POSIX_ERRNO_EISDIR          =(  21 << 8 ),  /* Is a directory */
    POSIX_ERRNO_EINVAL          =(  22 << 8 ),  /* Invalid argument */
    POSIX_ERRNO_ENFILE          =(  23 << 8 ),  /* Too many files */
    POSIX_ERRNO_ENOTTY          =(  25 << 8 ),  /* Not a serial device */
    POSIX_ERRNO_EFBUSY          =(  26 << 8 ),  /* File busy */
    POSIX_ERRNO_EFBIG           =(  27 << 8 ),  /* File too large */
    POSIX_ERRNO_ENOSPC          =(  28 << 8 ),  /* No space left on device */
    POSIX_ERRNO_ESPIPE          =(  29 << 8 ),  /* Illegal seek */
    POSIX_ERRNO_EROFS           =(  30 << 8 ),  /* Read only file system */
    POSIX_ERRNO_EMLINK          =(  31 << 8 ),  /* Too many links */
    POSIX_ERRNO_EPIPE           =(  32 << 8 ),  /* Broken pipe */
    POSIX_ERRNO_EDOM            =(  33 << 8 ),  /* Numerical domain */
    POSIX_ERRNO_ERANGE          =(  34 << 8 ),  /* Numerical range */
    POSIX_ERRNO_EDEADLK         =(  35 << 8 ),  /* Resource deadlock */
    POSIX_ERRNO_ENAMETOOLONG    =(  36 << 8 ),  /* Filename too long */
    POSIX_ERRNO_ENOLOCKS        =(  37 << 8 ),  /* No locks available */
    POSIX_ERRNO_ENOSYS          =(  38 << 8 ),  /* Function not implemented */
    POSIX_ERRNO_ENOTEMPTY       =(  39 << 8 ),  /* Directory not empty */
    POSIX_ERRNO_ELOOP           =(  40 << 8 ),  /* Symbolic link too deep */
    POSIX_ERRNO_ENOMESSAGE      =(  42 << 8 ),  /* No message of this type */
    POSIX_ERRNO_EIDRM           =(  43 << 8 ),  /* Identifier removed */
    POSIX_ERRNO_ECHRNG          =(  44 << 8 ),  /* Channel number range */
    POSIX_ERRNO_ELNRNG          =(  48 << 8 ),  /* Link number range */
    POSIX_ERRNO_EUNATCH         =(  49 << 8 ),  /* Protocol driver not attached */
    POSIX_ERRNO_EBADE           =(  52 << 8 ),  /* Invalid exchange */
    POSIX_ERRNO_EBADR           =(  53 << 8 ),  /* Invalid request descriptor */
    POSIX_ERRNO_EXFULL          =(  54 << 8 ),  /* Exchange full */
    POSIX_ERRNO_EBADRQC         =(  56 << 8 ),  /* Invalid request code */
    POSIX_ERRNO_ENOSTR          =(  60 << 8 ),  /* Not a stream device */
    POSIX_ERRNO_ENODATA         =(  61 << 8 ),  /* No data available */
    POSIX_ERRNO_ETIME           =(  62 << 8 ),  /* Timer expired */
    POSIX_ERRNO_ENONET          =(  64 << 8 ),  /* No network */
    POSIX_ERRNO_ECOMM           =(  70 << 8 ),  /* Communication error */
    POSIX_ERRNO_EPROTO          =(  71 << 8 ),  /* Protocol error */
    POSIX_ERRNO_EBADMSG         =(  74 << 8 ),  /* Bad message */
    POSIX_ERRNO_EOVERFLOW       =(  75 << 8 ),  /* Value too large for data type */
    POSIX_ERRNO_EBADFD          =(  77 << 8 ),  /* Bad file descriptor state */
    POSIX_ERRNO_EDESTADDRREQ    =(  89 << 8 ),  /* Destination address required */
    POSIX_ERRNO_EMSGSIZE        =(  90 << 8 ),  /* Message too long */
    POSIX_ERRNO_EPROTOTYPE      =(  91 << 8 ),  /* Wrong protocol */
    POSIX_ERRNO_EPROTONOOPT     =(  92 << 8 ),  /* Protocol Option not supported */
    POSIX_ERRNO_EPROTONOSUPP    =(  93 << 8 ),  /* Protocol not supported */
    POSIX_ERRNO_EOPNOTSUPP      =(  95 << 8 ),  /* Operation not supported on transport endpoint */
    POSIX_ERRNO_EADDRINUSE      =(  98 << 8 ),  /* Address already in use */
    POSIX_ERRNO_EADDRNOTVAIL    =(  99 << 8 ),  /* Address not available */
    POSIX_ERRNO_ENETDOWN        =( 100 << 8 ),  /* Network is down */
    POSIX_ERRNO_ENETUNREACH     =( 101 << 8 ),  /* Network is unreachable */
    POSIX_ERRNO_ENETRESET       =( 102 << 8 ),  /* Network is reset */
    POSIX_ERRNO_ECONNABORT      =( 103 << 8 ),  /* Network connection aborted */
    POSIX_ERRNO_ECONNRESET      =( 104 << 8 ),  /* Network connection reset by peer */
    POSIX_ERRNO_ENOBUFS         =( 105 << 8 ),  /* No buffer space available */
    POSIX_ERRNO_EISCONN         =( 106 << 8 ),  /* Endpoint already connected */
    POSIX_ERRNO_ENOTCONN        =( 107 << 8 ),  /* Endpoint is not connected */
    POSIX_ERRNO_ETIMEDOUT       =( 110 << 8 ),  /* Connection timed out */
    POSIX_ERRNO_ECONNREFUSED    =( 111 << 8 ),  /* Remote connection refused */
    POSIX_ERRNO_EHOSTDOWN       =( 112 << 8 ),  /* Remote host down */
    POSIX_ERRNO_EALREADY        =( 114 << 8 ),  /* operation in progress */
    POSIX_ERRNO_EINPROGRESS     =( 115 << 8 ),  /* Connection already in progress */
    POSIX_ERRNO_ESTALE          =( 116 << 8 ),  /* Stale file handle */
    POSIX_ERRNO_EREMOTEIO       =( 121 << 8 ),  /* Remote I/O error */
    POSIX_ERRNO_EDQUOT          =( 122 << 8 ),  /* Disc quota exceeded */
    POSIX_ERRNO_ENOMEDIUM       =( 123 << 8 ),  /* No medium inserted */
    POSIX_ERRNO_EMEDIUMTYPE     =( 124 << 8 ),  /* Wrong medium type */
    POSIX_ERRNO_ECANCELED       =( 125 << 8 ),  /* Operation cancelled */
    POSIX_ERRNO_ENOTRECOVERABLE =( 131 << 8 ),  /* State not recoverable */

    POSIX_ERRNO_END

} POSIX_ERRNO;


typedef enum _dev_mode
{
    /* generic modes */
    DEV_MODE_UNBUFFERED        =( 0x0 << 0 ),  // blocking mode, calling task may block
    DEV_MODE_BUFFERED          =( 0x1 << 0 ),  // non-blocking mode, calling task will not block
    DEV_MODE_BUFFERED_MASK     =( 0x1 << 0 ),

    /* GPIO modes (as per Zilog PS015317 table 6) */
    DEV_MODE_GPIO_OUT          =( 0x1 << 1 ),  // Mode 1 - standard digital output
    DEV_MODE_GPIO_IN           =( 0x2 << 1 ),  // Mode 2 - standard digital input
    DEV_MODE_GPIO_DIO          =( 0x3 << 1 ),  // Mode 3 - Open Drain I/O (needs external pullup)
    DEV_MODE_GPIO_SIO          =( 0x4 << 1 ),  // Mode 4 - Open Source I/O (needs external pulldown)
    DEV_MODE_GPIO_INTRDE       =( 0x6 << 1 ),  // Mode 6 - Input Dual-Edge triggered INTR
    DEV_MODE_GPIO_ALTFUNC      =( 0x7 << 1 ),  // Mode 7 - Alternate hardware function
    DEV_MODE_GPIO_INTRLOW      =( 0x8 << 1 ),  // Mode 8 - Input Interrupt active level low
    DEV_MODE_GPIO_INTRHIGH     =( 0x9 << 1 ),  // Mode 8 - Input Interrupt active level high
    DEV_MODE_GPIO_INTRFALL     =( 0xA << 1 ),  // Mode 9 - Input Interrupt active falling edge
    DEV_MODE_GPIO_INTRRISE     =( 0xB << 1 ),  // Mode 9 - Input Interrupt active rising edge
    DEV_MODE_GPIO_MASK         =( 0xF << 1 ),

    /* UART modes */
    DEV_MODE_UART_NO_FLOWCTRL  =( 0x1 << 5 ),  // no hardware flow control (uart= only rx, tx)
    DEV_MODE_UART_HW_FLOWCTRL  =( 0x2 << 5 ),  // hardware flow control (uart += rts,cts)
    DEV_MODE_UART_MOD_FLOWCTRL =( 0x3 << 5 ),  // modem hardware flow control (uart += dsr,dtr,ri,dcd)
    DEV_MODE_UART_MASK         =( 0x3 << 5 ),

    /* I2C modes */
    DEV_MODE_I2C_FREQ_DEFAULT  =( 0x1 << 7 ),  // I2C frequency 57600 bps
    DEV_MODE_I2C_FREQ_57600    =( 0x1 << 7 ),  // I2C frequency 57600 bps
    DEV_MODE_I2C_FREQ_115200   =( 0x2 << 7 ),  // I2C frequency 115200 bps
    DEV_MODE_I2C_FREQ_230400   =( 0x3 << 7 ),  // I2C frequency 230400 bps
    DEV_MODE_I2C_MASK          =( 0x3 << 7 ),

} DEV_MODE;


/*  GPIO Connector pinout
      Pin numbers follow Agon Light2; and Agon Origins. 
      (Different pin numbers are used on Console8.)
      Unenumerated pins 1..5 are power
                   pins 6..12 are ESP32 GPIO (not yet accessible through VDP protocol
                   pins 27..32 are non-gpio functions
                   pins 33..34 are power
*/
typedef enum _gpio_pin_num
{
    /* Pin Name      Pin#     Function      Ez80 pin name
    --------------   ----     -----------   -------------*/
    GPIO_13        = 13,   // GPIO pin 13   (PD4)
    GPIO_14        = 14,   // GPIO pin 14   (PD5)
    GPIO_15        = 15,   // GPIO pin 15   (PD6)
    GPIO_16        = 16,   // GPIO pin 16   (PD7)
    GPIO_17        = 17,   // GPIO pin 17   (PC0)
    GPIO_18        = 18,   // GPIO pin 18   (PC1)
    GPIO_19        = 19,   // GPIO pin 19   (PC2)
    GPIO_20        = 20,   // GPIO pin 20   (PC3)
    GPIO_21        = 21,   // GPIO pin 21   (PC4)
    GPIO_22        = 22,   // GPIO pin 22   (PC5)
    GPIO_23        = 23,   // GPIO pin 23   (PC6)
    GPIO_24        = 24,   // GPIO pin 24   (PC7)
    GPIO_25        = 25,   // GPIO pin 25   (PB2)
    GPIO_26        = 26    // GPIO pin 26   (PB5)

} GPIO_PIN_NUM;


/*----- Type Definitions ----------------------------------------------------*/


/*----- Function Declarations -----------------------------------------------*/
    /* DEV_API: uart_open
       Open UART1 for i/o
       Defined in devapi.c */
POSIX_ERRNO uart_open( 
                DEV_MODE const mode 
            );


    /* DEV_API: uart_close
       Close previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_close( 
                void 
            );


    /* DEV_API: uart_read
       Read data from previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_read(
                void * const buffer,
                size_t const num_bytes_to_read
            );


    /* DEV_API: uart_write
       Write data to a previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_write(
                void * const buffer,
                size_t const num_bytes_to_write,
                size_t * num_bytes_buffered
            );


    /* DEV_API: uart_poll
       Poll previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_poll(
                size_t * num_bytes_to_read,    // content of uart input buffer
                size_t * num_bytes_to_write    // free space in uart output buffer
            );


    /* DEV_API: i2c_open
       Open I2C for i/o
	   Frequency is one of 
	     DEV_MODE_I2C_FREQ_57600, 
		 DEV_MODE_I2C_FREQ_115200,
         DEV_MODE_I2C_FREQ_230400.
       Defined in devapi.c */
POSIX_ERRNO i2c_open( 
                DEV_MODE const frequency 
            );


    /* DEV_API: i2c_close
       Close previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_close( 
                void 
            );


    /* DEV_API: i2c_read
       Read data from previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_read(
                void * const buffer,
                size_t const num_bytes_to_read
            );


    /* DEV_API: i2c_write
       Write data to a previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_write(
                void * const buffer,
                size_t const num_bytes_to_write
            );


    /* DEV_API: spi_open
       Open SPI for i/o
       Defined in devapi.c */
POSIX_ERRNO spi_open( 
                DEV_MODE const mode
            );


    /* DEV_API: spi_close
       Close previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_close( 
                void 
            );


    /* DEV_API: spi_read
       Read data from previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_read(
                void * const buffer,
                size_t const num_bytes_to_read
            );


    /* DEV_API: spi_write
       Write data to a previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_write(
                void * const buffer,
                size_t const num_bytes_to_write
            );


    /* DEV_API: gpio_open
       Open GPIO for i/o
       Defined in devapi.c */
POSIX_ERRNO gpio_open( 
                GPIO_PIN_NUM const pin, 
                DEV_MODE const mode,
                unsigned char const init
            );


    /* DEV_API: gpio_close
       Close previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_close( 
                GPIO_PIN_NUM const pin
            );


    /* DEV_API: gpio_read
       Read data from previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_read(
                GPIO_PIN_NUM const pin,
                unsigned char * const val
            );


    /* DEV_API: gpio_write
       Write data to a previously opened GPIO
       Defined in devapi.c */
POSIX_ERRNO gpio_write(
                GPIO_PIN_NUM const pin,
                unsigned char const val
            );


#endif /* DEVAPI_H */
