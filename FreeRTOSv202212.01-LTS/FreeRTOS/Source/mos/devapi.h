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
    DEV_MODE_UNBUFFERED           =( 0x0 << 0 ),  // un-buffered semantics
    DEV_MODE_BUFFERED             =( 0x1 << 0 ),  // buffered semantics
    DEV_MODE_BUFFERED_MASK        =( 0x1 << 0 ),

    /* GPIO modes (refer to Zilog PS015317 table 6) */
    DEV_MODE_GPIO_OUT             =( 0x1 << 1 ),  // Mode 1 - standard digital output
    DEV_MODE_GPIO_IN              =( 0x2 << 1 ),  // Mode 2 - standard digital input
    DEV_MODE_GPIO_DIO             =( 0x3 << 1 ),  // Mode 3 - Open Drain I/O (needs external pullup)
    DEV_MODE_GPIO_SIO             =( 0x4 << 1 ),  // Mode 4 - Open Source I/O (needs external pulldown)
    DEV_MODE_GPIO_INTRDE          =( 0x6 << 1 ),  // Mode 6 - Input Dual-Edge triggered INTR
    DEV_MODE_GPIO_ALTFUNC         =( 0x7 << 1 ),  // Mode 7 - Alternate hardware function
    DEV_MODE_GPIO_INTRLOW         =( 0x8 << 1 ),  // Mode 8 - Input Interrupt active level low
    DEV_MODE_GPIO_INTRHIGH        =( 0x9 << 1 ),  // Mode 8 - Input Interrupt active level high
    DEV_MODE_GPIO_INTRFALL        =( 0xA << 1 ),  // Mode 9 - Input Interrupt active falling edge
    DEV_MODE_GPIO_INTRRISE        =( 0xB << 1 ),  // Mode 9 - Input Interrupt active rising edge
    DEV_MODE_GPIO_MASK            =( 0xF << 1 ),

    /* UART modes */                              // DTE=computer, DCE=modem
    DEV_MODE_UART_NO_FLOWCONTROL  =( 0x01 << 5 ), // DTE<->DTE no flow control
    DEV_MODE_UART_SW_FLOWCONTROL  =( 0x02 << 5 ), // DTE<->DTE Xon / Xoff sw flow control
    DEV_MODE_UART_HALF_MODEM      =( 0x04 << 5 ), // DTE<->DCE RTS/CTS hw flow control, straight-through wiring
    DEV_MODE_UART_FULL_MODEM      =( 0x08 << 5 ), // DTE<->DCE RTS/CTS DTR/DSR hw flow control, straight-through
    DEV_MODE_UART_HALF_NULL_MODEM =( 0x14 << 5 ), // DTE<->DTE RTS/CTS hw flow control, cross-over wiring
    DEV_MODE_UART_FULL_NULL_MODEM =( 0x18 << 5 ), // DTE<->DTE RTS/CTS DTR/DSR hw flow control, cross-over
    DEV_MODE_UART_NULL_MODEM_MASK =( 0x10 << 5 ),
    DEV_MODE_UART_LOOPBACK        =( 0x20 << 5 ), // DTE<> loopback, no flow control
    DEV_MODE_UART_MASK            =( 0x2f << 5 ),

    /* I2C modes */
    DEV_MODE_I2C_MASTER           =( 0x1 << 11 ), // Single-mastering
    DEV_MODE_I2C_MASTER_SLAVE     =( 0x2 << 11 ), // Multi-mastering
    DEV_MODE_I2C_MASK             =( 0x3 << 11 ),

    /* SPI modes */
    DEV_MODE_SPI_DEFAULT          =( 0x1 << 13 ),
    DEV_MODE_SPI_MASK             =( 0x1 << 13 ),

} DEV_MODE;


/*  GPIO Connector pinout
      Pin numbers follow Agon Light2; and Agon Origins. 
      (Different pin numbers are used on Console8.)
      Unenumerated pins 1..5 are power
                   pins 6..12 are ESP32 GPIO (not yet accessible through VDP protocol)
                   pins 27..32 are non-gpio functions (I2C, SPI, Clock)
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

#define NUM_PINS_GPIO ( GPIO_26 - GPIO_13 + 1 )


typedef enum _dev_ioctl
{
    DEV_IOCTL_NULL = 0,
    
    DEV_IOCTL_UART_EXEC_RX_FLOW_CONTROL        = 1,
    DEV_IOCTL_UART_WRITE_MODEM                 = 2,
    DEV_IOCTL_UART_SET_DTR                     = 3,
    DEV_IOCTL_UART_GET_DSR                     = 4,
    DEV_IOCTL_UART_GET_DCD                     = 5,
    DEV_IOCTL_UART_GET_RI                      = 6,
    
    DEV_IOCTL_I2C_SET_FREQUENCY                = 10,
    DEV_IOCTL_I2C_SET_SLAVE_ADDRESS            = 11,
    DEV_IOCTL_I2C_ENABLE_GENERAL_CALL_ADDRESS  = 12,
    DEV_IOCTL_I2C_DISABLE_GENERAL_CALL_ADDRESS = 13

} DEV_IOCTL;


typedef enum _uart_baud_rate   // BRG = 18,432,000 /( 16 * BAUD )
{                              //                    -- refer to Zilog PS015317 pp:109
    UART_BAUD_150     =  0,    // BRG=7,680   0x1E00
    UART_BAUD_300     =  1,    // BRG=3,840   0x0F00
    UART_BAUD_1200    =  2,    // BRG=960     0x03C0
    UART_BAUD_2400    =  3,    // BRG=480     0x01E0
    UART_BAUD_4800    =  4,    // BRG=240     0x00F0
    UART_BAUD_9600    =  5,    // BRG=120     0x0078
    UART_BAUD_11520   =  6,    // BRG=100     0x0064
    UART_BAUD_14400   =  7,    // BRG=80      0x0050
    UART_BAUD_19200   =  8,    // BRG=60      0x003C
    UART_BAUD_28800   =  9,    // BRG=40      0x0028
    UART_BAUD_38400   = 10,    // BRG=30      0x001E
    UART_BAUD_57600   = 11,    // BRG=20      0x0014
    UART_BAUD_115200  = 12,    // BRG=10      0x000A
    UART_BAUD_144000  = 13,    // BRG=8       0x0008
    UART_BAUD_192000  = 14,    // BRG=6       0x0006
    UART_BAUD_288000  = 15,    // BRG=4       0x0004
    UART_BAUD_384000  = 16,    // BRG=3       0x0003
    UART_BAUD_576000  = 17,    // BRG=2       0x0002 -- system reset default
    UART_BAUD_1152000 = 18,    // BRG=1       0x0001 -- minimum possible

    NUM_UART_BAUD = 19

} UART_BAUD_RATE;


typedef enum _uart_databits
{                      // values map directly to bits in the eZ80 UART Line Control Register
    UART_DATABITS_5  =( 0x0 << 0 ),
    UART_DATABITS_6  =( 0x1 << 0 ),
    UART_DATABITS_7  =( 0x2 << 0 ),
    UART_DATABITS_8  =( 0x3 << 0 )

} UART_DATABITS;


typedef enum _uart_stopbits
{                      // values map directly to bits in the eZ80 UART Line Control Register
    UART_STOPBITS_1  =( 0x0 << 2 ),
    UART_STOPBITS_2  =( 0x1 << 2 )

} UART_STOPBITS;


typedef enum _uart_paritybit
{                       // values map directly to bits in the eZ80 UART Line Control Register
    UART_PARITY_NONE  =( 0x0 << 3 ),   // PEN=0
    UART_PARITY_ODD   =( 0x1 << 3 ),   // PEN=1 + EPS=0
    UART_PARITY_EVEN  =( 0x3 << 3 )    // PEN=1 + EPS=1

} UART_PARITY_BIT;


typedef enum _uart_event
{
    UART_EVENT_NONE = 0,
    UART_EVENT_RX_READY,
    UART_EVENT_RX_FULL,
    UART_EVENT_RX_ERROR,
    UART_EVENT_TX_DONE,
    UART_EVENT_TX_ERROR,
    UART_EVENT_TX_UNEXPECTED,
    
} UART_EVENT;


typedef enum _spi_baud_rate   // SPICLK = 3Mhz ( sysclk=18,432,000 /( 2 * brg=3 ))
{                             //                 -- refer to Zilog PS015317 pp:134
    SPI_BAUD_115200  =  0,    // BRG=80      0x0050
    SPI_BAUD_153600  =  1,    // BRG=60      0x003C
    SPI_BAUD_184320  =  2,    // BRG=50      0x0032
    SPI_BAUD_230400  =  3,    // BRG=40      0x0028
    SPI_BAUD_307200  =  4,    // BRG=30      0x001E
    SPI_BAUD_460800  =  5,    // BRG=20      0x0014
    SPI_BAUD_921600  =  6,    // BRG=10      0x000A
    SPI_BAUD_1152000 =  7,    // BRG=8       0x0008
    SPI_BAUD_1536000 =  8,    // BRG=6       0x0006
    SPI_BAUD_1843200 =  9,    // BRG=5       0x0005
    SPI_BAUD_2304000 = 10,    // BRG=4       0x0004
    SPI_BAUD_3072000 = 11,    // BRG=3       0x0003  // 3=SPI Master lowest value

    NUM_SPI_BAUD     = 12

} SPI_BAUD_RATE;


typedef enum _spi_clockpolarity
{
    SPI_CPOL_LOW  = 0,      // SPI clock idles low
    SPI_CPOL_HIGH = 1       // SPI clock idles high
    
} SPI_CLOCK_POLARITY;


typedef enum _spi_clockphase
{
    SPI_CPHA_LEADING = 0,   // SPI sample on clock leading edge (transmit on trailing)
    SPI_CPHA_TRAILING = 1   // SPI sample on clock trailing edge (transmit on leading)

} SPI_CLOCK_PHASE;


typedef enum _i2c_scl_frequency
{
    I2C_SCL_FREQ_DEFAULT = 0x0,  // default to 57600, or as set in ioctl
    I2C_SCL_FREQ_57600   = 0x0,  // 57600 bps
    I2C_SCL_FREQ_115200  = 0x1,  // 115200 bps
    I2C_SCL_FREQ_230400  = 0x2,  // 230400 bps

    NUM_I2C_SCL_FREQ     = 3
    
} I2C_SCL_FREQ;


typedef enum _i2c_event
{
    I2C_EVENT_NONE = 0,
    I2C_EVENT_RX_READY,
    I2C_EVENT_RX_FULL,       // slave receive configDRV_I2C_BUFFER_NUM too small
    I2C_EVENT_RX_OVERFLOW,   // slave receive configDRV_I2C_BUFFER_SZ too small
    I2C_EVENT_RX_ERROR,
    I2C_EVENT_RX_INTERRUPTED,    
    I2C_EVENT_TX_DONE,
    I2C_EVENT_TX_ERROR,
    I2C_EVENT_TX_INTERRUPTED,    
    I2C_EVENT_TX_STRANS_OK,  // successful responded to a Slave Transmit
    I2C_EVENT_TX_STRANS_INTERRUPTED, // interrupted responded to a Slave Transmit
    
} I2C_EVENT;


typedef enum _dev_num_major
{
    DEV_NUM_GPIO = 0,
    DEV_NUM_UART = 1,
    DEV_NUM_SPI = 2,
    DEV_NUM_I2C = 3,
    
    NUM_DEV_MAJOR = 4

} DEV_NUM_MAJOR;

typedef GPIO_PIN_NUM DEV_NUM_MINOR;


/*----- Type Definitions ----------------------------------------------------*/
typedef void ( *INTERRUPT_HANDLER )( DEV_NUM_MAJOR const, DEV_NUM_MINOR const );
typedef void ( *FAST_INTERRUPT_HANDLER )( void );
typedef void ( *I2C_HANDLER )( void );


typedef struct _uart_params      // UART descriptor
{
    UART_BAUD_RATE   baudRate;
    UART_DATABITS    dataBits;   // number of databits per character
    UART_STOPBITS    stopBits;   // number of stopbits per character
    UART_PARITY_BIT  parity;     // parity bit per character

} UART_PARAMS;


typedef struct _uart_mode_status // Modem (DCE) state descriptor
{                                //      Agon = DTE
    unsigned char ri : 1;        // DCE->Agon Ring Indication (signal high)
    unsigned char dcd: 1;        // DCE->Agon Data Carrier Detect (Remote connected)
    unsigned char dsr: 1;        // DCE->Agon Data Set Ready (Modem ready)
    unsigned char dtr: 1;        // DCE<-Agon Data Terminal Ready (Agon ready)
    unsigned char cts: 1;        // DCE->Agon Clear To Send (Remote ready)
    unsigned char rts: 1;        // DCE<-Agon Request To Send (Agon write request)
    unsigned char rxd: 1;        // DCE->Agon Receive Data stream (remote to Agon)
    unsigned char txd: 1;        // DCE<-Agon Transmit Data stream (Agon to remote)
    
} UART_MODEM_STATUS;


typedef struct _spi_params         // SPI descriptor
{
    SPI_BAUD_RATE      baudRate;   // Clock bit rate
    SPI_CLOCK_POLARITY cPolarity;  // Clock Polarity
    SPI_CLOCK_PHASE    cPhase;     // Clock Phase

} SPI_PARAMS;


/* The I2C Address is either a 7-bit item or a 10-bit item.
   Set sz to 0 for a 7-bit address, or to 1 for a 10-bit address.
   The bitmask for the 7-bit item is [0]=0xXnnn nnnn 
    such that the bottom 7 bits are the required address in range 8..119
    Addresses in the range 0..7 and 120.127 are reserved.
   The bitmasks for the 10-bit item are [0]=0xX111 10nn [1]=0xnnnn nnnn 
    such that the bottom 2 bits of [0] are the MSBs and the 8-bits of [1] are
    the LSBs of the 10-bit address. 
   Refer to NXP UM10204 section 3.1.12 Table 4.
   For example, to initialise a 7-bit address 0x34 in your application:
    I2C_ADDR mySlave ={ 0, 0x34 };
   And to initialise a 10-bit address 0x34 in your application:
    I2C_ADDR mySlave ={ 1, 0x78, 0x34 };   */
typedef struct _i2c_addr
{
    unsigned char sz:1;           // 0 = 7-bit, 1 = 10-bit
    union
    {
        unsigned char  _7bit[ 1 ]; // bits 6:0 = 7-bit addr
        unsigned char _10bit[ 2 ]; // bits 1:0+7:0 = 10-bit addr
    } form;

} I2C_ADDR;

  /* I2C_MSG is used with Master Transmit i2c_writem and Master Receive 
     i2c_readm. The common DEV API read/write functions take buffer and size 
     parameters. So we need to pack the destination slave address into a 
     buffer struct. */
typedef struct _i2c_msg
{
    I2C_ADDR addr;
    unsigned char buf[ configDRV_I2C_BUFFER_SZ ];

} I2C_MSG;


/*----- Function Declarations -----------------------------------------------*/
    /* DEV_API: gpio_open
       Open GPIO for i/o
       Additional parameters:
         For mode in { DEV_MODE_GPIO_OUT } supply int initial value
         For mode in { DEV_MODE_GPIO_INTRDE, DEV_MODE_GPIO_INTRLOW, 
                       DEV_MODE_GPIO_INTRHIGH, DEV_MODE_GPIO_INTRFALL, 
                       DEV_MODE_GPIO_INTRRISE } supply INTERRUPT_HANDLER
         For mode in { DEV_MODE_GPIO_IN, DEV_MODE_GPIO_DIO,
                       DEV_MODE_GPIO_SIO DEV_MODE_GPIO_ALTFUNC } none
        Defined in devapi.c */
POSIX_ERRNO gpio_open( 
                GPIO_PIN_NUM const pin, 
                DEV_MODE const mode,
                ...  // additional parameters depending on mode
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


    /* DEV_API: i2c_open
       Open an I2C channel for i/o
       Defined in devapi.c */
POSIX_ERRNO i2c_open( 
                DEV_MODE const mode,
                ...
            );


    /* DEV_API: i2c_close
       Close previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_close( 
                void
            );


    /* DEV_API: i2c_readm (Master Receive)
       Inititate a Read from a target slave, through previously opened I2C.
       The target slave address is embedded in the I2C_MSG parameter
       Defined in devapi.c */
POSIX_ERRNO i2c_readm(
                I2C_MSG const * message,
                size_t const msgLen,
                size_t * num_bytes_buffered,
                POSIX_ERRNO * result
            );


    /* DEV_API: i2c_writem (Master Transmit)
       Initiate a Write to a target slave through previously opened I2C
       The target slave address is embedded in the I2C_MSG parameter
       Defined in devapi.c */
POSIX_ERRNO i2c_writem(
                I2C_MSG const * const message,
                size_t const msgLen,
                size_t * num_bytes_sent,
                POSIX_ERRNO * result
            );


    /* DEV_API: i2c_reads (Slave Receive)
       Application calls i2c_reads to collect data received from remote 
       master messages via the Slave Receive role.
       Result codes:
         POSIX_ERRNO_ENONE      data returned in buffer, and buffer emptied 
         POSIX_ERRNO_ENOTEMPTY  data returned from buffer, buffer not empty
         POSIX_ERRNO_ENODATA    no data available
       Defined in devapi.c */
POSIX_ERRNO i2c_reads(
                unsigned char const * const buffer,
                size_t const num_bytes_to_read,
                size_t * num_bytes_buffered,
                POSIX_ERRNO * result
            );


    /* DEV_API: i2c_writes (Slave Transmit)
       ISR Handler Task (i2cHandler) calls i2c_writes to respond to a Slave
       Transmit request from a remote bus-master.
       This function would not normally be invoked from other tasks.
       Defined in devapi.c */
POSIX_ERRNO i2c_writes(
                unsigned char const * const buffer,
                size_t const num_bytes_to_write,
                size_t * num_bytes_sent,
                POSIX_ERRNO * result
            );


/* DEV_API: i2c_ioctl
       Perform an IO Control function on previously opened I2C
       Defined in devapi.c */
POSIX_ERRNO i2c_ioctl(
                DEV_IOCTL const cmd,
                void * param
            );


    /* DEV_API: spi_open
       Open SPI for i/o
       Defined in devapi.c */
POSIX_ERRNO spi_open( 
                GPIO_PIN_NUM const slaveSelect, 
                DEV_MODE const mode,
                SPI_PARAMS const * params
            );


    /* DEV_API: spi_close
       Close previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_close( 
                GPIO_PIN_NUM const slaveSelect
            );


    /* DEV_API: spi_read
       Read data from previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_read(
                GPIO_PIN_NUM const slaveSelect,
                unsigned char * const tx_buffer,
                unsigned char * rx_buffer,
                size_t const num_bytes_to_transceive
            );


    /* DEV_API: spi_write
       Write data to a previously opened SPI
       Defined in devapi.c */
POSIX_ERRNO spi_write(
                GPIO_PIN_NUM const slaveSelect, 
                void * const buffer,
                size_t const num_bytes_to_write
            );


    /* DEV_API: uart_open
       Open UART1 for i/o
       Defined in devapi.c */
POSIX_ERRNO uart_open( 
                DEV_MODE const mode,
                UART_PARAMS const * params
            );


    /* DEV_API: uart_close
       Close previously opened UART1
       Defined in devapi.c */
POSIX_ERRNO uart_close( 
                void 
            );


    /* DEV_API: uart_read
       Read data from a previously opened UART1
       Calling task may block until either the read is complete or an error
         transpires.
       Defined in devapi.c */
POSIX_ERRNO uart_read(
                void * const buffer,
                size_t const num_bytes_to_read
            );


    /* DEV_API: uart_read_buffered
       Read data from previously opened UART1
       If the uart is opened with DEV_MODE_BUFFERED in the mode parameters,
         the calling task will not block, but return immediately with any data
         that has been received at the time of calling. It is like a poll that
         receives data immediately.
       If the uart is opened without DEV_MODE_BUFFERED in the mode parameters,
         then the behaviour is like uart_read and the calling task may block. 
       Defined in devapi.c */
POSIX_ERRNO uart_read_buffered(
                void * const buffer,
                size_t const num_bytes_to_read,
                size_t * num_bytes_read,
                POSIX_ERRNO * result
            );


    /* DEV_API: uart_getch
       Read a single byte from the previously opened UART1
       Calling task may block. 
       On error 0xff is returned, but this may also be a valid value.
       Defined in devapi.c */
char uart_getch( 
         void
     );


    /* DEV_API: uart_write
       Write data to a previously opened UART1
       Calling task will block until either the write is complete or an error
         happens.
       Defined in devapi.c */
POSIX_ERRNO uart_write(
                void * const buffer,
                size_t const num_bytes_to_write
            );


    /* DEV_API: uart_write_buffered
       Write data to a previously opened UART1
       If the uart is opened with DEV_MODE_BUFFERED in the mode parameters,
         the calling task will not block, but return immediately.
       If the uart is opened without DEV_MODE_BUFFERED in the mode parameters,
         then the behaviour is like uart_write and the calling task may block. 
       Defined in devapi.c */
POSIX_ERRNO uart_write_buffered(
                void * const buffer,
                size_t const num_bytes_to_write,
                size_t * num_bytes_written,
                POSIX_ERRNO * result
            );


    /* DEV_API: uart_putch
       Write a single byte to the previously opened UART1
       Calling task will return immediately; user must check return value
       Defined in devapi.c */
POSIX_ERRNO uart_putch(
                char const ch
            );


    /* DEV_API: uart_poll
       Poll previously opened UART1.
       Any of the parameters can be a buffer of designated type or NULL.
       Defined in devapi.c */
POSIX_ERRNO uart_poll(
                size_t * num_bytes_to_read,     // input buffer ready
                size_t * num_bytes_to_write,    // output buffer free space
                UART_MODEM_STATUS *modem_status // modem activity
            );


    /* DEV_API: uart_ioctl
       Perform an IO Control function on previously opened UART1
       I/O operations can be one of three types:
         Executive - in which param is generally NULL
         Set - in which param is an input
         Get - in which param is an output
       Defined in devapi.c */
POSIX_ERRNO uart_ioctl( 
                DEV_IOCTL const cmd,
                void * const param
            );


#endif /* DEVAPI_H */
