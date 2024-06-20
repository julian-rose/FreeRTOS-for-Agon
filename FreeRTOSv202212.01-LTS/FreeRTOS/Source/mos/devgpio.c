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
 * devgpio.c for Agon Light
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The definitions in this file support the DEV API (GPIO low-level)
 * for Agon Light (and comptaibles) and the ZDSII compiler
 * Zilog eZ80 ANSI C Compiler Version 3.4 (19101101).
 * Created 17.Jun.2024 by Julian Rose for Agon Light port
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


#if( 1 == configUSE_DRV_GPIO )

/*---- Global Variables -----------------------------------------------------*/
/* Changing any global variable needs to be done within
    portENTER_CRITICAL( );
    {
    }
    portEXIT_CRITICAL( );
*/
unsigned char gpio_initial_output_value[ NUM_DEV_MINOR ]={ 0 };


/*----- Private functions ---------------------------------------------------*/
/*------ GPIO I/O value functions -------------------------------------------*/
/* Refer to Zilog PS015317 Table 6 */
/* gpio_get_input 
   used in modes DEV_MODE_GPIO_IN Digital input 
                 DEV_MODE_GPIO_DIO Open Drain I/O
                 DEV_MODE_GPIO_SIO Open Source I/O
*/
static unsigned char gpio_get_input( 
                         PORT const p, 
                         unsigned char const bit
                     )
{
    unsigned char const tstmask = TST_MASK( bit );
    unsigned char ch;

    // 1. Get value from the port
    switch( p )
	{
        case PORT_B :
		{
            ch = TEST_PORT_MASK( PB_DR, tstmask );
        }
		break;

        case PORT_C :
		{
            ch = TEST_PORT_MASK( PC_DR, tstmask );
        }
		break;

        case PORT_D :
		{
            ch = TEST_PORT_MASK( PD_DR, tstmask );
        }
		break;

        default:
		{
            ch = 0;
        }
        break;
	}

    // 2. shift value into bit position 0
    ch >>= bit;

    return( ch );
}


/* gpio_set_output 
   used in modes DEV_MODE_GPIO_OUT Digital output 
                 DEV_MODE_GPIO_DIO Open Drain I/O
                 DEV_MODE_GPIO_SIO Open Source I/O
*/
static void gpio_set_output( 
                PORT const p, 
                unsigned char const bit, 
                unsigned char const value   // 0 = clear, else set
            )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            if( value )
			{
                SET_PORT_MASK( PB_DR, setmask );
            }
            else
			{
                CLEAR_PORT_MASK( PB_DR, clrmask );
            }
        }
		break;

        case PORT_C :
		{
            if( value )
			{
                SET_PORT_MASK( PC_DR, setmask );
            }
            else
			{
                CLEAR_PORT_MASK( PC_DR, clrmask );
            }
        }
		break;

        case PORT_D :
		{
            if( value )
			{
                SET_PORT_MASK( PD_DR, setmask );
            }
            else
			{
                CLEAR_PORT_MASK( PD_DR, clrmask );
            }
        }
		break;

        default:
        break;
	}
}


/*------ GPIO mode setting functions ----------------------------------------*/
/* Refer to Zilog PS015317 Table 6 */
/* gpio_set_mode_output 
   set Mode 1 - standard digital output */
static void gpio_set_mode_output( PORT const p, unsigned char const bit )
{
    unsigned char const clrmask = CLR_MASK( bit );

#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", __FILE__, __LINE__ );
        ( void )printf( "port = %d : bit = %d : clrmask = 0x%x\r\n", p, bit, clrmask );
#   endif

    switch( p )
	{
        case PORT_B :
		{
            CLEAR_PORT_MASK( PB_DDR, clrmask );
            CLEAR_PORT_MASK( PB_ALT1, clrmask );
            CLEAR_PORT_MASK( PB_ALT2, clrmask );
        }
		break;

        case PORT_C :
		{
            CLEAR_PORT_MASK( PC_DDR, clrmask );
            CLEAR_PORT_MASK( PC_ALT1, clrmask );
            CLEAR_PORT_MASK( PC_ALT2, clrmask );
        }
		break;

        case PORT_D :
		{
            CLEAR_PORT_MASK( PD_DDR, clrmask );
            CLEAR_PORT_MASK( PD_ALT1, clrmask );
            CLEAR_PORT_MASK( PD_ALT2, clrmask );
        }
		break;

        default:
        break;
	}
}


/* gpio_set_mode_input 
   set Mode 2 - standard digital input */
static void gpio_set_mode_input( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            SET_PORT_MASK( PB_DDR, setmask );
            CLEAR_PORT_MASK( PB_ALT1, clrmask );
            CLEAR_PORT_MASK( PB_ALT2, clrmask );
        }
		break;

        case PORT_C :
		{
            SET_PORT_MASK( PC_DDR, setmask );
            CLEAR_PORT_MASK( PC_ALT1, clrmask );
            CLEAR_PORT_MASK( PC_ALT2, clrmask );
        }
		break;

        case PORT_D :
		{
            SET_PORT_MASK( PD_DDR, setmask );
            CLEAR_PORT_MASK( PD_ALT1, clrmask );
            CLEAR_PORT_MASK( PD_ALT2, clrmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_dio 
   set Mode 3 - Open Drain I/O (needs external pullup) */
static void gpio_set_mode_dio( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            CLEAR_PORT_MASK( PB_DDR, clrmask );
            SET_PORT_MASK( PB_ALT1, setmask );
            CLEAR_PORT_MASK( PB_ALT2, clrmask );
        }
		break;

        case PORT_C :
		{
            CLEAR_PORT_MASK( PC_DDR, clrmask );
            SET_PORT_MASK( PC_ALT1, setmask );
            CLEAR_PORT_MASK( PC_ALT2, clrmask );
        }
		break;

        case PORT_D :
		{
            CLEAR_PORT_MASK( PD_DDR, clrmask );
            SET_PORT_MASK( PD_ALT1, setmask );
            CLEAR_PORT_MASK( PD_ALT2, clrmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_sio 
   set Mode 4 - Open Source I/O (needs external pulldown) */
static void gpio_set_mode_sio( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            SET_PORT_MASK( PB_DDR, setmask );
            SET_PORT_MASK( PB_ALT1, setmask );
            CLEAR_PORT_MASK( PB_ALT2, clrmask );
        }
		break;

        case PORT_C :
		{
            SET_PORT_MASK( PC_DDR, setmask );
            SET_PORT_MASK( PC_ALT1, setmask );
            CLEAR_PORT_MASK( PC_ALT2, clrmask );
        }
		break;

        case PORT_D :
		{
            SET_PORT_MASK( PD_DDR, setmask );
            SET_PORT_MASK( PD_ALT1, setmask );
            CLEAR_PORT_MASK( PD_ALT2, clrmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_intrde 
   set Mode 6 - Input Dual-Edge triggered INTR */
static void gpio_set_mode_intrde( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            SET_PORT_MASK( PB_DR, setmask );
            CLEAR_PORT_MASK( PB_DDR, clrmask );
            CLEAR_PORT_MASK( PB_ALT1, clrmask );
            SET_PORT_MASK( PB_ALT2, setmask );
        }
		break;

        case PORT_C :
		{
            SET_PORT_MASK( PC_DR, setmask );
            CLEAR_PORT_MASK( PC_DDR, clrmask );
            CLEAR_PORT_MASK( PC_ALT1, clrmask );
            SET_PORT_MASK( PC_ALT2, setmask );
        }
		break;

        case PORT_D :
		{
            SET_PORT_MASK( PD_DR, setmask );
            CLEAR_PORT_MASK( PD_DDR, clrmask );
            CLEAR_PORT_MASK( PD_ALT1, clrmask );
            SET_PORT_MASK( PD_ALT2, setmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_altfunc
   set Mode 7 - Alternate hardware function */
static void gpio_set_mode_altfunc( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            SET_PORT_MASK( PB_DDR, setmask );
            CLEAR_PORT_MASK( PB_ALT1, clrmask );
            SET_PORT_MASK( PB_ALT2, setmask );
        }
		break;

        case PORT_C :
		{
            SET_PORT_MASK( PC_DDR, setmask );
            CLEAR_PORT_MASK( PC_ALT1, clrmask );
            SET_PORT_MASK( PC_ALT2, setmask );
        }
		break;

        case PORT_D :
		{
            SET_PORT_MASK( PD_DDR, setmask );
            CLEAR_PORT_MASK( PD_ALT1, clrmask );
            SET_PORT_MASK( PD_ALT2, setmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_intrlow 
   set Mode 8 - Input Interrupt active level low */
static void gpio_set_mode_intrlow( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            CLEAR_PORT_MASK( PB_DR, clrmask );
            CLEAR_PORT_MASK( PB_DDR, clrmask );
            SET_PORT_MASK( PB_ALT1, setmask );
            SET_PORT_MASK( PB_ALT2, setmask );
        }
		break;

        case PORT_C :
		{
            CLEAR_PORT_MASK( PC_DR, clrmask );
            CLEAR_PORT_MASK( PC_DDR, clrmask );
            SET_PORT_MASK( PC_ALT1, setmask );
            SET_PORT_MASK( PC_ALT2, setmask );
        }
		break;

        case PORT_D :
		{
            CLEAR_PORT_MASK( PD_DR, clrmask );
            CLEAR_PORT_MASK( PD_DDR, clrmask );
            SET_PORT_MASK( PD_ALT1, setmask );
            SET_PORT_MASK( PD_ALT2, setmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_intrhigh 
   set Mode 8 - Input Interrupt active level high */
static void gpio_set_mode_intrhigh( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            SET_PORT_MASK( PB_DR, setmask );
            CLEAR_PORT_MASK( PB_DDR, clrmask );
            SET_PORT_MASK( PB_ALT1, setmask );
            SET_PORT_MASK( PB_ALT2, setmask );
        }
		break;

        case PORT_C :
		{
            SET_PORT_MASK( PC_DR, setmask );
            CLEAR_PORT_MASK( PC_DDR, clrmask );
            SET_PORT_MASK( PC_ALT1, setmask );
            SET_PORT_MASK( PC_ALT2, setmask );
        }
		break;

        case PORT_D :
		{
            SET_PORT_MASK( PD_DR, setmask );
            CLEAR_PORT_MASK( PD_DDR, clrmask );
            SET_PORT_MASK( PD_ALT1, setmask );
            SET_PORT_MASK( PD_ALT2, setmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_intrfall 
   set Mode 9 - Input Interrupt active falling edge */
static void gpio_set_mode_intrfall( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );
    unsigned char const clrmask = CLR_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            CLEAR_PORT_MASK( PB_DR, clrmask );
            SET_PORT_MASK( PB_DDR, setmask );
            SET_PORT_MASK( PB_ALT1, setmask );
            SET_PORT_MASK( PB_ALT2, setmask );
        }
		break;

        case PORT_C :
		{
            CLEAR_PORT_MASK( PC_DR, clrmask );
            SET_PORT_MASK( PC_DDR, setmask );
            SET_PORT_MASK( PC_ALT1, setmask );
            SET_PORT_MASK( PC_ALT2, setmask );
        }
		break;

        case PORT_D :
		{
            CLEAR_PORT_MASK( PD_DR, clrmask );
            SET_PORT_MASK( PD_DDR, setmask );
            SET_PORT_MASK( PD_ALT1, setmask );
            SET_PORT_MASK( PD_ALT2, setmask );
        }
		break;

        default:
        break;
	}
}

/* gpio_set_mode_intrrise 
   set Mode 9 - Input Interrupt active rising edge */
static void gpio_set_mode_intrrise( PORT const p, unsigned char const bit )
{
    unsigned char const setmask = SET_MASK( bit );

    switch( p )
	{
        case PORT_B :
		{
            SET_PORT_MASK( PB_DR, setmask );
            SET_PORT_MASK( PB_DDR, setmask );
            SET_PORT_MASK( PB_ALT1, setmask );
            SET_PORT_MASK( PB_ALT2, setmask );
        }
		break;

        case PORT_C :
		{
            SET_PORT_MASK( PC_DR, setmask );
            SET_PORT_MASK( PC_DDR, setmask );
            SET_PORT_MASK( PC_ALT1, setmask );
            SET_PORT_MASK( PC_ALT2, setmask );
        }
		break;

        case PORT_D :
		{
            SET_PORT_MASK( PD_DR, setmask );
            SET_PORT_MASK( PD_DDR, setmask );
            SET_PORT_MASK( PD_ALT1, setmask );
            SET_PORT_MASK( PD_ALT2, setmask );
        }
		break;

        default:
        break;
	}
}


/*------ GPIO low-level functions -------------------------------------------*/
/* gpio_dev_open
   Device-specific gpio open function for minor device configuration.
   1. Configure pin mode
   2. Attach any interrupt handler */
POSIX_ERRNO gpio_dev_open(
                       DEV_NUM_MINOR const minor,
                       DEV_MODE const mode
                   )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    unsigned int const mnr =( minor - PIN_NUM_START );

#   if defined( _DEBUG )&& 0
	{
        ( void )printf( "%s : %d : mnr = %d : ", __FILE__, __LINE__, mnr );
        ( void )printf( "mode = 0x%x : mask = 0x%x\r\n", mode, DEV_MODE_GPIO_MASK & mode );
    }
#   endif

    // 1. Configure Pin Mode	
    switch( DEV_MODE_GPIO_MASK & mode )
	{
        case DEV_MODE_GPIO_OUT:      // Mode 1 - standard digital output
        {
            gpio_set_output(             // initial value = 0 TBD user settable
                portmap[ mnr ].port,
                portmap[ mnr ].bit,
                gpio_initial_output_value[ mnr ]);
            gpio_set_mode_output( 
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_IN:       // Mode 2 - standard digital input
        {
            gpio_set_mode_input(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_DIO:      // Mode 3 - Open Drain I/O (needs external pullup)
        {
            gpio_set_output(             // initial value = High Impedance
                portmap[ mnr ].port,
                portmap[ mnr ].bit,
                1 );
            gpio_set_mode_dio(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_SIO:      // Mode 4 - Open Source I/O (needs external pulldown)
        {
            gpio_set_output(             // initial value = High Impedance
                portmap[ mnr ].port,
                portmap[ mnr ].bit,
                0 );
            gpio_set_mode_sio(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_INTRDE:    // Mode 6 - Input Dual-Edge triggered INTR
        {
            gpio_set_mode_intrde(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_ALTFUNC:  // Mode 7 - Alternate hardware function
        {
            gpio_set_mode_altfunc(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_INTRLOW:   // Mode 8 - Input Interrupt active level low
        {
            gpio_set_mode_intrlow(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_INTRHIGH:  // Mode 8 - Input Interrupt active level high
        {
            gpio_set_mode_intrhigh(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_INTRFALL:  // Mode 9 - Input Interrupt active falling edge
        {
            gpio_set_mode_intrfall(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        case DEV_MODE_GPIO_INTRRISE:  // Mode 9 - Input Interrupt active rising edge
        {
            gpio_set_mode_intrrise(
                portmap[ mnr ].port,
                portmap[ mnr ].bit );
        }
        break;

        default:
        {
            ret = POSIX_ERRNO_ENODEV;
        }
        break;
    }

    // 2. Attach any interrupt handler

    return( ret );
}


/* gpio_dev_close
   Device-specific gpio close function for minor device re-configuration.
   Zilog PS015317 GPIO Operation; After a RESET event, all GPIO port pins
   are configured as standard digital inputs, with interrupts disabled */
void gpio_dev_close(
                DEV_NUM_MINOR const minor
            )
{
    unsigned int const mnr =( minor - PIN_NUM_START );
	
    // 1. Detach any interrupt handler

    // 2. set DEV_MODE_GPIO_IN: Mode 2 - standard digital input
    gpio_set_mode_input( portmap[ mnr ].port, portmap[ mnr ].bit );
}


/* gpio_dev_read
   Device-specific gpio read function
   Without a signal reference clock, it only makes sense to sample the current input
   and not record a stream of samples. */
POSIX_ERRNO gpio_dev_read(
                       DEV_NUM_MINOR const minor,
                       unsigned char * const buffer
                   )
{
    unsigned int const mnr =( minor - PIN_NUM_START );	

    *buffer = gpio_get_input( portmap[ mnr ].port, portmap[ mnr ].bit );
	
    return( POSIX_ERRNO_ENONE );
}


/* gpio_dev_write
   Device-specific gpio write function
   Device-specific gpio write function
   Without a signal reference clock, it only makes sense to output one value */
POSIX_ERRNO gpio_dev_write(
                       DEV_NUM_MINOR const minor,
                       unsigned char const val
                   )
{
    unsigned int const mnr =( minor - PIN_NUM_START );	

    gpio_set_output( portmap[ mnr ].port, portmap[ mnr ].bit, val );

    return( POSIX_ERRNO_ENONE );
}


#endif  /* 1 == configUSE_DRV_GPIO */
