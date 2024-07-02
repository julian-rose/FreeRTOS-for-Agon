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
 * devapil.h for Agon Light
 *
 * DEV API low-level for FreeRTOS for Agon Light by Julian Rose, 17.Jun.2024, 
 * with copyright assigned to Amazon Web Services as for FreeRTOS version 10.5.
 *
 * DEV API low-level specific definitions
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * Definitions for DEV low-level functions for FreeRTOS for Agon Light 
 * (and compatibles).
 * Created Jun.2024 by Julian Rose for Agon Light port
*/

#ifndef DEVAPIL_H
#define DEVAPIL_H


/*----- Constants -----------------------------------------------------------*/
#define configUSE_DEV_DEVICE_DRIVERS                   \
            ( configUSE_DRV_UART | configUSE_DRV_SPI | \
              configUSE_DRV_I2C | configUSE_DRV_GPIO )

#define NUM_PINS         ( PIN_NUM_END - PIN_NUM_START )
#define NUM_DEV_MINOR    NUM_PINS


/* eZ80F92 Hardware Port Definitions:
   Use definitions in .../ZDSII5.3.5/include/zilog/eZ80F92.h
   GPIO: Port A is not available on F92
         Port B:
        #define PB_DR           (*(volatile unsigned char __INTIO *)0x9A)
        #define PB_DDR          (*(volatile unsigned char __INTIO *)0x9B)
        #define PB_ALT1         (*(volatile unsigned char __INTIO *)0x9C)
        #define PB_ALT2         (*(volatile unsigned char __INTIO *)0x9D)
         and similar for Port C and D
   SPI:
        #define SPI_BRG_L       (*(volatile unsigned char __INTIO *)0xB8)
        #define SPI_BRG_H       (*(volatile unsigned char __INTIO *)0xB9)
        #define SPI_CTL         (*(volatile unsigned char __INTIO *)0xBA)
        #define SPI_SR          (*(volatile unsigned char __INTIO *)0xBB)
        #define SPI_TSR         (*(volatile unsigned char __INTIO *)0xBC)
        #define SPI_RBR         (*(volatile unsigned char __INTIO *)0xBC)
   I2C:
        #define I2C_SAR         (*(volatile unsigned char __INTIO *)0xC8)
        #define I2C_XSAR        (*(volatile unsigned char __INTIO *)0xC9)
        #define I2C_DR          (*(volatile unsigned char __INTIO *)0xCA)
        #define I2C_CTL         (*(volatile unsigned char __INTIO *)0xCB)
        #define I2C_SR          (*(volatile unsigned char __INTIO *)0xCC)
        #define I2C_CCR         (*(volatile unsigned char __INTIO *)0xCC)
        #define I2C_SRR         (*(volatile unsigned char __INTIO *)0xCD)
   UART1:
        #define UART1_RBR       (*(volatile unsigned char __INTIO *)0xD0)
        #define UART1_THR       (*(volatile unsigned char __INTIO *)0xD0)
        #define UART1_BRG_L     (*(volatile unsigned char __INTIO *)0xD0)
        #define UART1_IER       (*(volatile unsigned char __INTIO *)0xD1)
        #define UART1_BRG_H     (*(volatile unsigned char __INTIO *)0xD1)
        #define UART1_IIR       (*(volatile unsigned char __INTIO *)0xD2)
        #define UART1_FCTL      (*(volatile unsigned char __INTIO *)0xD2)
        #define UART1_LCTL      (*(volatile unsigned char __INTIO *)0xD3)
        #define UART1_MCTL      (*(volatile unsigned char __INTIO *)0xD4)
        #define UART1_LSR       (*(volatile unsigned char __INTIO *)0xD5)
        #define UART1_MSR       (*(volatile unsigned char __INTIO *)0xD6)
        #define UART1_SPR       (*(volatile unsigned char __INTIO *)0xD7)

   eZ80F92 Hardware Interrupt Vectors
   Use definitions in .../ZDSII5.3.5/include/zilog/eZ80F92.h
   GPIO:
        #define PORTB5_IVECT    0x3A
        #define PORTC0_IVECT    0x40
        #define PORTC1_IVECT    0x42
        #define PORTC2_IVECT    0x44
        #define PORTC3_IVECT    0x46
        #define PORTC4_IVECT    0x48
        #define PORTC5_IVECT    0x4A
        #define PORTC6_IVECT    0x4C
        #define PORTC7_IVECT    0x4E
        #define PORTD4_IVECT    0x58
        #define PORTD5_IVECT    0x5A
        #define PORTD6_IVECT    0x5C
        #define PORTD7_IVECT    0x5E
   SPI:
        #define SPI_IVECT       0x1E
   I2C:
        #define I2C_IVECT       0x1C
   UART1:
        #define UART1_IVECT        0x1A
*/
#define CLR_MASK( bit )\
            ( unsigned char )( 0xff -( 1 <<( bit )))
#define CLEAR_PORT_MASK( prt, clrmsk ) \
            ( prt )&=( unsigned char )( clrmsk )
#define SET_MASK( bit )\
            ( unsigned char )( 1 <<( bit ))
#define SET_PORT_MASK( prt, setmsk ) \
            ( prt )|=( unsigned char )( setmsk )
#define TST_MASK( bit )\
            ( unsigned char )( 1 <<( bit ))
#define TEST_PORT_MASK( prt, tstmsk ) \
            ( prt )&( unsigned char )( tstmsk )


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
    PIN_NUM_START  = 13,

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


typedef enum _pin_state
{
    PIN_STATE_FREE = 0,
    PIN_STATE_INUSE

} PIN_STATE;


typedef enum _port
{
    PORT_A = 0,    // unused
    PORT_B = 1,
    PORT_C = 2,
    PORT_D = 3
    
} PORT;


/* Zilog UART error codes
 * Error codes consist of both the errors reported by the UART device
 * (through status registers), and the errors that occur in the UART driver
 * software.
 * The domain of UART error codes overlays that of MOS error codes
 */
typedef enum _uart_errno
{
    UART_ERRNO_NONE               = 0x00, // success.
    UART_ERRNO_KBHIT              = 0x01,    // keyboard hit.            
    UART_ERRNO_FRAMINGERR         = 0x02,    // Framing error occurs in the character received.        
    UART_ERRNO_PARITYERR          = 0x03, // Parity error occurs in the character received.            
    UART_ERRNO_OVERRUNERR         = 0x04, // Overrun error occurs in the receive buffer register.        
    UART_ERRNO_BREAKINDICATIONERR = 0x05, // Break Indication Error occurs.        
    UART_ERRNO_CHARTIMEOUT        = 0x06, // a character time-out occurs while receiving.            
    UART_ERRNO_INVBAUDRATE        = 0x07, // baud rate specified is invalid.            
    UART_ERRNO_INVPARITY          = 0x08,    // parity option specified is invalid.            
    UART_ERRNO_INVSTOPBITS        = 0x09, // stop bits specified is invalid.            
    UART_ERRNO_INVDATABITS        = 0x0A, // data bits per character specified is invalid.            
    UART_ERRNO_INVTRIGGERLEVEL    = 0x0B, // receive FIFO trigger level specified is invalid.            
    UART_ERRNO_FIFOBUFFERFULL     = 0x0C, // transmit FIFO buffer is full.        
    UART_ERRNO_FIFOBUFFEREMPTY    = 0x0D, // receive FIFO buffer is empty.            
    UART_ERRNO_RECEIVEFIFOFULL    = 0x0E, // software receive FIFO buffer is full.            
    UART_ERRNO_RECEIVEFIFOEMPTY   = 0x0F, // software receive FIFO buffer is empty.            
    UART_ERRNO_PEEKINPOLL         = 0x10, // invalid 'peek a character' while in polling mode
    
    UART_ERRNO_USRBASE            = 0x11, // The error code base value for user applications.

    UART_ERRNO_RX_DATA_READY      = 0x11, // LSR.DR indicated
    UART_ERRNO_CTS_LOST           = 0x12, // MSR.CTS indicated
    UART_ERRNO_CTS_FOUND          = 0x13,
    UART_ERRNO_DSR_LOST           = 0x14, // MSR.DSR indicated
    UART_ERRNO_DSR_FOUND          = 0x15,
    UART_ERRNO_DCD_LOST           = 0x16, // MSR.DCD indicated
    UART_ERRNO_DCD_FOUND          = 0x17,
    UART_ERRNO_RI_HANGUP          = 0x18, // MSR.RI indicated
    UART_ERRNO_RI_CALL            = 0x19

} UART_ERRNO;


/*----- Type Definitions ----------------------------------------------------*/
typedef struct _port_bitmap
{
    PORT port;
    unsigned char bit;
} PORT_BITMAP;


/*----- Global Names --------------------------------------------------------*/
extern PIN_NUM const assigned_pins[ NUM_DEV_MAJOR ]
                                  [ NUM_DEV_MINOR ];
extern PORT_BITMAP const portmap[ NUM_DEV_MINOR ];


/*---- Global Variables -----------------------------------------------------*/
/* Changing any global variable needs to be done within
    portENTER_CRITICAL( );
    {
    }
    portEXIT_CRITICAL( );
*/
extern PIN_STATE pinstate[ NUM_DEV_MINOR ];
extern DEV_MODE pinmode[ NUM_DEV_MINOR ];
extern unsigned char gpio_initial_output_value[ NUM_DEV_MINOR ];


/*------ Generic Pin allocation and free functions --------------------------*/
extern POSIX_ERRNO pins_alloc(
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       DEV_MODE const mode 
                   );

extern POSIX_ERRNO pins_free(
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor
                   );


/*------ GPIO low-level functions -------------------------------------------*/
/* gpio_dev_open
   Device-specific gpio open function for minor device configuration.
   1. Configure pin mode
   2. Attach any interrupt handler */
extern POSIX_ERRNO gpio_dev_open(
                       DEV_NUM_MINOR const minor,
                       DEV_MODE const mode,
                       ...
                   );

/* gpio_dev_close
   Device-specific gpio close function for minor device re-configuration.
   Zilog PS015317 GPIO Operation; After a RESET event, all GPIO port pins
   are configured as standard digital inputs, with interrupts disabled */
extern void gpio_dev_close(
                DEV_NUM_MINOR const minor
            );

/* gpio_dev_read
   Device-specific gpio read function
   Without a signal reference clock, it only makes sense to sample the current input
   and not record a stream of samples.
   Implemented in devgpio.c */
extern POSIX_ERRNO gpio_dev_read(
                       DEV_NUM_MINOR const minor,
                       unsigned char * const sample
                   );

/* gpio_dev_write
   Device-specific gpio write function
   Without a signal reference clock, it only makes sense to output one value
   and not write a stream of values */
extern POSIX_ERRNO gpio_dev_write(
                       DEV_NUM_MINOR const minor,
                       unsigned char const value
                   );


/*------ I2C low-level functions --------------------------------------------*/
/* i2c_dev_open
   Device-specific i2c open function for minor device configuration */
extern POSIX_ERRNO i2c_dev_open(
                       DEV_MODE const mode
                   );

/* i2c_dev_close
   Device-specific i2c close function for device shutdown */
extern void i2c_dev_close(
                void
            );

/* i2c_dev_read
   Device-specific i2c read function */
extern POSIX_ERRNO i2c_dev_read(
                       void * const buffer,              // IN
                       size_t const num_bytes_to_read,   // IN
                       size_t * num_bytes_read,          // OUT
                       POSIX_ERRNO *result               // OUT
                   );

/* i2c_dev_write
   Device-specific I2C write function */
extern POSIX_ERRNO i2c_dev_write(
                       void * const buffer,              // IN
                       size_t const num_bytes_to_write,  // IN
                       size_t * num_bytes_written,       // OUT
                       POSIX_ERRNO *result               // OUT
                   );


/*------ SPI low-level functions --------------------------------------------*/
/* spi_dev_open
   Device-specific spi open function for minor device configuration */
extern POSIX_ERRNO spi_dev_open(
                       DEV_MODE const mode
                   );

/* spi_dev_close
   Device-specific SPI close function for device shutdown */
extern void spi_dev_close(
                void
            );

/* spi_dev_read
   Device-specific spi read function */
extern POSIX_ERRNO spi_dev_read(
                       void * const buffer,              // IN
                       size_t const num_bytes_to_read,   // IN
                       size_t * num_bytes_read,          // OUT
                       POSIX_ERRNO *result               // OUT
                   );

/* spi_dev_write
   Device-specific SPI write function */
extern POSIX_ERRNO spi_dev_write(
                       void * const buffer,              // IN
                       size_t const num_bytes_to_write,  // IN
                       size_t * num_bytes_written,       // OUT
                       POSIX_ERRNO *result               // OUT
                   );


/*------ UART low-level functions -------------------------------------------*/
/* uart_dev_open
   Device-specific uart1 open function device configuration */
extern POSIX_ERRNO uart_dev_open(
                       DEV_MODE const mode,
                       UART_PARAMS const * params
                   );

/* uart_dev_close
   Device-specific UART close function device shutdown */
extern void uart_dev_close(
                void
            );

/* uart_dev_read
   Device-specific UART1 read function */
extern POSIX_ERRNO uart_dev_read(
                       void * const buffer,              // IN
                       size_t const num_bytes_to_read,   // IN
                       size_t * num_bytes_read,          // OUT
                       POSIX_ERRNO *result               // OUT
                   );

/* uart_dev_write
   Device-specific UART1 write function */
extern POSIX_ERRNO uart_dev_write(
                       void * const buffer,              // IN
                       size_t const num_bytes_to_write,  // IN
                       size_t * num_bytes_written,       // OUT
                       POSIX_ERRNO *result               // OUT
                   );


/*------ Generic Driver Functions -------------------------------------------*/
/* dev_open
   Generic function to Open a device for i/o
   Checks the major and mionor device numbers range
   Checks that the required minor device pins are free
   Invokes the device-specific open function for minor device configuration */
extern POSIX_ERRNO dev_open( 
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       DEV_MODE const mode,
                       ...
                   );

/* dev_close
   Generic function to Close a previously opened device
   Checks the major and minor device numbers range
   Checks that the required minor device pins are free
   Invokes the device-specific close function for minor device closure */
extern void dev_close( 
                DEV_NUM_MAJOR const major,
                DEV_NUM_MINOR const minor
            );

/* dev_read
   Read data from a previously opened device
   Calling task may block if opened in UART_MODE_UNBUFFERED */
extern POSIX_ERRNO dev_read( 
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       void * const buffer,
                       size_t const num_bytes_to_read,
                       size_t * num_bytes_read,
                       POSIX_ERRNO *result
                   );

/* dev_write
   Write data to a previously opened device
   Calling task may block if opened in UART_MODE_UNBUFFERED */
extern POSIX_ERRNO dev_write( 
                       DEV_NUM_MAJOR const major,
                       DEV_NUM_MINOR const minor,
                       void * const buffer,
                       size_t const num_bytes_to_write,
                       size_t * const num_bytes_written,
                       POSIX_ERRNO *result
                   );

/* dev_poll
   Retrieve buffers state in previously opened device */
extern POSIX_ERRNO dev_poll( 
                       DEV_NUM_MAJOR const major,
                       unsigned short const minor,
                       size_t * num_bytes_buffered,    // content of uart input buffer
                       size_t * num_bytes_free         // free space in uart output buffer
                   );


#endif /* DEVAPIL_H */
