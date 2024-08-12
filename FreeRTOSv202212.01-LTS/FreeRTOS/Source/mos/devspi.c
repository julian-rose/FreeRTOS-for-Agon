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
 * devspi.c for Agon Light
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The definitions in this file support the DEV API (SPI low-level)
 * for Agon Light (and comptaibles)
 * Created 01.August.2024 by Julian Rose for Agon Light port
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <eZ80F92.h>

#include "FreeRTOS.h"
#include "task.h"
#include "devConfig.h"
#include "devapi.h"
#include "devapil.h"


#if( 1 == configUSE_DRV_SPI )

/*---- Notes ----------------------------------------------------------------*/
/* NOTE 1: No DEV UART like transactions to maximise concurrency potential.
 *
 * The eZ80 SPI device has only a single-place Transmit Holding Register (THR);
 *  there is no FIFO.
 * Unlike the UART, only the eZ80 can bus-master the SPI device for any and all
 *  transmit and receive operations. 
 * If we were to use interrupts, we would spend more time context switching 
 *  than performing useful work.
 * For these reasons, there is no point supporting interrupts, rather we byte
 *  bash.
 * The application can devide how many bytes to transmit / receive per DEV SPI
 *  call.
*/


/*---- Constants ------------------------------------------------------------*/
static unsigned short int const brg[ NUM_SPI_BAUD ]=
{           // Constants for BRG divisor computed at compile time
    0x0050, // SPI_BAUD_115200   // BRG=0x0050
    0x003C, // SPI_BAUD_153600   // BRG=0x003C
    0x0032, // SPI_BAUD_184320   // BRG=0x0032
    0x0028, // SPI_BAUD_230400   // BRG=0x0028
    0x001E, // SPI_BAUD_307200   // BRG=0x001E
    0x0014, // SPI_BAUD_460800   // BRG=0x0014
    0x000A, // SPI_BAUD_921600   // BRG=0x000A
    0x0008, // SPI_BAUD_1152000  // BRG=0x0008
    0x0006, // SPI_BAUD_1536000  // BRG=0x0006
    0x0005, // SPI_BAUD_1843200  // BRG=0x0005
    0x0004, // SPI_BAUD_2304000  // BRG=0x0004
    0x0003  // SPI_BAUD_3072000  // BRG=0x0003  // lowest value for SPI Master
};


/*---- Enumeration Types ----------------------------------------------------*/
typedef enum _spi_status_reg
{                                     // refer to Zilog PS015317 table 75
    SPI_STATUS_REG_BIT_SPIF    = 7,   // 1 = SPI transfer Finish
    SPI_STATUS_REG_BIT_WCOL    = 6,   // 1 = write collision detected
    SPI_STATUS_REG_BIT_MODF    = 4,   // 1 = mode fault detected

    SPI_STATUS_REG_SPIF        =( 1 << SPI_STATUS_REG_BIT_SPIF ),
    SPI_STATUS_REG_WCOL        =( 1 << SPI_STATUS_REG_BIT_WCOL ),
    SPI_STATUS_REG_MODF        =( 1 << SPI_STATUS_REG_BIT_MODF ),

} SPI_STATUS_REG;


typedef enum _spi_ctl_reg
{                                     // refer to Zilog PS015317 table 74
    SPI_CTL_REG_BIT_ENABLE    = 5,    // 1=SPI device enable
    SPI_CTL_REG_BIT_MASTER_EN = 4,    // 1=Master Mode enable
    SPI_CTL_REG_BIT_CPOL      = 3,    // Clock Polarity
    SPI_CTL_REG_BIT_CPHA      = 2,    // Clock Phase

    SPI_CTL_REG_ENABLE        =( 1 << SPI_CTL_REG_BIT_ENABLE ),
    SPI_CTL_REG_MASTER_EN     =( 1 << SPI_CTL_REG_BIT_MASTER_EN ),
    SPI_CTL_REG_CPOL          =( 1 << SPI_CTL_REG_BIT_CPOL ),
    SPI_CTL_REG_CPHA          =( 1 << SPI_CTL_REG_BIT_CPHA )

} SPI_CTL_REG;


/*---- Global Variables -----------------------------------------------------*/
#define defaultParam    { SPI_BAUD_115200,  SPI_CPOL_LOW, SPI_CPHA_LEADING }
static SPI_PARAMS const defaultParams = defaultParam;
static SPI_PARAMS const mosParams =
    { SPI_BAUD_3072000, SPI_CPOL_LOW, SPI_CPHA_LEADING };

/* Changing any global variable needs to be done within
    portENTER_CRITICAL( );
    {
    }
    portEXIT_CRITICAL( );
*/
static SPI_PARAMS workingParams[ NUM_DEV_MINOR ]=
{ 
    defaultParam, defaultParam, defaultParam, defaultParam,
    defaultParam, defaultParam, defaultParam, defaultParam,
    defaultParam, defaultParam, defaultParam, defaultParam,
    defaultParam, defaultParam, defaultParam, defaultParam,
    defaultParam, defaultParam, defaultParam, defaultParam
};

static SPI_PARAMS *inuseParams[ NUM_DEV_MINOR ]={ NULL };


/*------ Local functions ----------------------------------------------------*/
static void program_spi_params( SPI_PARAMS const * params )
{
    // SPI should be disabled before changing CPOL or CPHA
    SPI_CTL &=( unsigned char )( 0xff - SPI_CTL_REG_ENABLE );
    SPI_BRG_H = 0;
    SPI_BRG_L = brg[ params->baudRate ];
    SPI_CTL |=( unsigned char )
                  (( params->cPolarity << SPI_CTL_REG_BIT_CPOL )|
                   ( params->cPhase << SPI_CTL_REG_BIT_CPHA ));
    SPI_CTL |=( unsigned char )
                  ( SPI_CTL_REG_ENABLE | SPI_CTL_REG_MASTER_EN );
}


/*------ SPI low-level functions --------------------------------------------*/
/* These routines normally called from devapi.c                              */

/* spi_dev_open
   Device-specific spi open function for minor device configuration */
POSIX_ERRNO spi_dev_open(
                GPIO_PIN_NUM const slaveSelect,
                DEV_MODE const mode,
                SPI_PARAMS const * params
            )
{
    unsigned int const emuSSel =( slaveSelect - PIN_NUM_START );
    POSIX_ERRNO ret;
    unsigned char setmask;
    unsigned char clrmask;

#   if defined( _DEBUG )&&0
    {
        ( void )printf( "%s : %d : slaveSelect = %d : ", 
                        "devspi.c", __LINE__, slaveSelect );
        ( void )printf( "mode = 0x%x : mask = 0x%x\r\n", 
                        mode, DEV_MODE_SPI_MASK & mode );
    }
#   endif

    /* 1. Allocate and De-assert Emulated /SlaveSelect (GPIO) pin */
    ret = gpio_open( slaveSelect, DEV_MODE_GPIO_OUT, 1 );
    if( POSIX_ERRNO_ENONE == ret )
    {
#       if( 0 )
        {
            // the following is done by MOS and left as is

            /* 2. set the eZ80 port pin modes; refer to Zilog PS15317 table 6 */
            /* 2.1 PB2 /SS pin 25 is set to output a constant '1' value, asserting 
               SPI Master */
            setmask = 0x00;
            clrmask = 0xff;
            setmask |= SET_MASK( portmap[ SPI_SS - PIN_NUM_START ].bit );
            clrmask &= CLR_MASK( portmap[ SPI_SS - PIN_NUM_START ].bit );
            SET_PORT_MASK( PB_DR, setmask );
            CLEAR_PORT_MASK( PB_DDR, clrmask );
            CLEAR_PORT_MASK( PB_ALT1, clrmask );
            CLEAR_PORT_MASK( PB_ALT2, clrmask );
            /* 2.2 PB6 MISO pin 27, PB7 MOSI pin 32 and PB3 SCK pin 31 are set to 
               alternate function (SPI) */
            setmask = 0x00;
            clrmask = 0xff;
            setmask |= SET_MASK( portmap[ SPI_MISO - PIN_NUM_START ].bit );
            clrmask &= CLR_MASK( portmap[ SPI_MISO - PIN_NUM_START ].bit );
            setmask |= SET_MASK( portmap[ SPI_CLK - PIN_NUM_START ].bit );
            clrmask &= CLR_MASK( portmap[ SPI_CLK - PIN_NUM_START ].bit );
            setmask |= SET_MASK( portmap[ SPI_MOSI - PIN_NUM_START ].bit );
            clrmask &= CLR_MASK( portmap[ SPI_MOSI - PIN_NUM_START ].bit );
            CLEAR_PORT_MASK( PB_DR, clrmask );    // not in MOS, CLEAR or SET is ok
            SET_PORT_MASK( PB_DDR, setmask );     // MOS init_params_f92.asm::__init
            CLEAR_PORT_MASK( PB_ALT1, clrmask );  // MOS spi.asm::_init_spi
            SET_PORT_MASK( PB_ALT2, setmask );    // MOS spi.asm::_init_spi
        }
#       endif
 
        /* 3. store working params */
        if( NULL == params )
        {            
            /*  leave the params as per MOS in spi.asm::_init_spi:
                    CPHA = 0x0
                    CPOL = 0x0
                    SPICLK = 3Mhz ( sysclk=18,432,000 /( 2 * brg=3 )) */
            inuseParams[ emuSSel ]= &mosParams;
        }
        else
        {
            portENTER_CRITICAL( );
            {
                /* safeguard updates to all globals */
                workingParams[ emuSSel ]= *params;
                inuseParams[ emuSSel ]= &workingParams[ emuSSel ];

#               if defined( _DEBUG )
                    ( void )printf( "Baudrate = %d : Polarity = %d : "
                                    "Phase = %d\r\n",
                                    workingParams[ emuSSel ].baudRate,
                                    workingParams[ emuSSel ].cPolarity,
                                    workingParams[ emuSSel ].cPhase );
#               endif
            }
            portEXIT_CRITICAL( );
        }
    }

    return( ret );
}


/* spi_dev_close
   Device-specific SPI close function for device shutdown */
void spi_dev_close(
         GPIO_PIN_NUM const slaveSelect
         )
{
    unsigned int const emuSSel =( slaveSelect - PIN_NUM_START );

    /* 1. De-Allocate Emulated SlaveSelect (GPIO) pin */
    gpio_close( slaveSelect );

    portENTER_CRITICAL( );
    {
        /* safeguard updates to all globals */
        inuseParams[ emuSSel ]= &mosParams;
        workingParams[ emuSSel ]= defaultParams;
    }
    portEXIT_CRITICAL( );
}


/* spi_dev_read
   Device-specific spi read function.
   The eZ80 SPI is activated only by writing; reading is synchronous.
   The application task shall send a register address in tx byte n, 
      so that the corresponding data is read in the next rx byte n+1 */
POSIX_ERRNO spi_dev_read(
                GPIO_PIN_NUM const slaveSelect,
                unsigned char * const tx_buffer,
                unsigned char * rx_buffer,
                size_t const num_bytes_to_transceive
            )
{
    unsigned int const emuSSel =( slaveSelect - PIN_NUM_START );
    int i;

    /* Guard against concurrent entry into MOS, because a SPI SD-card operation
       must not conflict with an Expansion Interface SPI operation */
    portEnterMOS( );
    {
//_putchf('r');
        if( &mosParams != inuseParams[ emuSSel ])
        {
//_putchf('1');
            program_spi_params( &workingParams[ emuSSel ]);
        }

        /* Guard against interrupts to enable the SPI transaction to complete. 
           Technically it can pause between bytes, but we cannot know how this 
           affects an external SPI device in general. */
        portENTER_CRITICAL( );
        {
            /* Start of Frame Assert Emulated /SlaveSelect pin */
            gpio_write( slaveSelect, 0 );  // CSB=0
            {
                for( i = 0; num_bytes_to_transceive > i; i++ )
                {
//_putchf('2');
                    SPI_TSR = tx_buffer[ i ];
                    while( 0 ==( SPI_SR & SPI_STATUS_REG_SPIF ))
                        ;
//_putchf('3');
                    rx_buffer[ i ]= SPI_RBR;
                }
            }
            gpio_write( slaveSelect, 1 );  //CSB=1
            /* End of Frame De-assert Emulated /SlaveSelect pin */
        }
        portEXIT_CRITICAL( );

        if( &mosParams != inuseParams[ emuSSel ])
        {
//_putchf('4');
            program_spi_params( &mosParams );  // restore SD-card params
        }
    }
    portExitMOS( );

    return( POSIX_ERRNO_ENONE );
}


/* spi_dev_write
   Device-specific SPI write function */
POSIX_ERRNO spi_dev_write(
                GPIO_PIN_NUM const slaveSelect,
                unsigned char * const tx_buffer,
                size_t const num_bytes_to_write
            )
{
    unsigned int const emuSSel =( slaveSelect - PIN_NUM_START );
    int i;

    /* Guard against concurrent entry into MOS, because a SPI SD-card operation
       must not conflict with an Expansion Interface SPI operation */
    portEnterMOS( );
    {
//_putchf('w');
        if( &mosParams != inuseParams[ emuSSel ])
        {
            //int j;
//_putchf('1');
            program_spi_params( &workingParams[ emuSSel ]);
            // delay 50mS after setting SPI_CTL_REG_ENABLE, like MOS _init_spi
            //   for( j=0; 10000 > j; j++ ) ;
            // doesn't have any effect. Why did MOS do it? Maybe an SD-card 
            // thing; no comment in MOS to share the reason. 
            
            /* vTaskDelay( 1 );
               called within portEnterMOS (a semaphore) the task never returns;
               called outside of portEnterMOS is not a problem.
               I stepped through with the debugger, and the cause lies within 
               the FreeRTOS task code. There is some kind of (race) condition
               such that if a task holds a semaphore and goes onto the (time) 
               delay list (at the call to prvAddCurrentTaskToDelayedList within
               vTaskDelay), then it will not be woken. Breakpoints or single 
               stepping can change this behaviour, such that the task continues
               running if it never gets added. 
               This looks like a bug in the FreeRTOS task wake-up code, inter-
               play between semaphores and delays, but may already be documented 
               as a known behaviour or restriction.
            */
//_putchf('5');
        }

        /* Guard against interrupts to enable the SPI transaction to complete. 
           Technically it can pause between bytes, but we cannot know how this 
           affects an external SPI device in general. */
        portENTER_CRITICAL( );
        {
            /* Start of Frame Assert Emulated /SlaveSelect pin */
            gpio_write( slaveSelect, 0 );  // CSB=0
            {
                for( i = 0; num_bytes_to_write > i; i++ )
                {
//_putchf('2');
                    SPI_TSR = tx_buffer[ i ];
                    while( 0 ==( SPI_SR & SPI_STATUS_REG_SPIF ))
                        ;
//_putchf('3');
                }
            }
            gpio_write( slaveSelect, 1 );  // CSB=1
            /* End of Frame De-assert Emulated /SlaveSelect pin */
        }
        portEXIT_CRITICAL( );

        if( &mosParams != inuseParams[ emuSSel ])
        {
//_putchf('4');
            program_spi_params( &mosParams );  // restore SD-card params
        }
    }
    portExitMOS( );
    
    return( POSIX_ERRNO_ENONE );
}

#endif  /* 1 == configUSE_DRV_SPI */
