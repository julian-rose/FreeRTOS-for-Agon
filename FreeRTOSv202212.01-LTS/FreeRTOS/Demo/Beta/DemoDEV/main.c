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
 *  Demo application for EZ80 Agon Light FreeRTOS "Beta" capabilities (DEV API)
*/

/***** Memory Regions *****/
#pragma asm "\tDEFINE TASKS, SPACE = RAM, ALIGN = 10000h"

    /* We can separate user task code from kernel code by placing it in a
       different memory segment. The user heap is already in its own segment.
       The default code segment is named "CODE". The compiler inserts a 
       linker directive "SECTION CODE" before the start of each C function, 
       We can override the default by placing
           "#pragma asm "\tSEGMENT TASKS"
       immediately before the function definition to locate it in the TASKS 
       segment. See example below. */


/*----- Includes ------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <eZ80F92.h>

#include "FreeRTOS.h"
#include "task.h"
#include "devapi.h"
#include "mosapi.h"


/*----- Global Names --------------------------------------------------------*/
extern unsigned int _heaptop;  // defined in the linker directive file
extern unsigned int _heapbot;  //   "


/*----- Local Types ---------------------------------------------------------*/
typedef struct _mosftest
{
    char menukey;
    char const * menudesc;
    void( *testf )( void );
} DEVFTEST;


/*----- Private Variables ---------------------------------------------------*/
static unsigned int idlecnt = 0;
static volatile unsigned int gpiocnt = 0;


/*----- Local Declarations --------------------------------------------------*/
void Task1( void *pvParameters );
void vApplicationIdleHook( void );
void /* INTERRUPT_HANDLER */ myGpioHndlr( DEV_NUM_MAJOR const, DEV_NUM_MINOR const );


static void * menu( void );

    /* Device tests */
static void doGPIOWriteTest( void );
static void doGPIOReadWriteTest( void );
static void doGpioCallback( void );


/*----- Function Definitions ------------------------------------------------*/
int main( int const argc, char * const * const argv )
{
    BaseType_t r;

#   if defined( _DEBUG )
    {
        ( void )printf( "&_heaptop = 0x%p\r\n", &_heaptop );
        ( void )printf( "&_heapbot = 0x%p\r\n", &_heapbot );
        ( void )printf( "Heap size 0x%x\r\n", configTOTAL_HEAP_SIZE );
    }
#   endif

    /* Create the tasks */
    r = xTaskCreate( 
            Task1, 
            "DEV test", 
            configMINIMAL_STACK_SIZE, 
            NULL, 
            tskIDLE_PRIORITY + 1, 
            NULL );
    if( pdPASS != r )
    {
        ( void )printf( "Failed to allocate Task1: %d\r\n", r );
    }
    else
    {
        /* Start FreeRTOS */
        ( void )printf( "\r\nEntering scheduler\r\n" );
        vTaskStartScheduler( );
    }
        
    /*** should never get here ***/
    ( void )printf( "Back from scheduler\r\n" );
    
    return( 0 );
}


/* menu
 *   Display a simple menu of tests and execute choice.
 *   Return NULL (choice 'q') to end the tests; any non-NULL to continue
 *   with tests.
*/
static void * menu( void )
{
    DEVFTEST const tests[ ]=
    {
        { '0', "Test DEV GPIO write", doGPIOWriteTest },
        { '1', "Test DEV GPIO read write", doGPIOReadWriteTest },
        { '2', "Test DEV GPIO ISR", doGpioCallback },
        { 'q', "End tests", NULL }
    };
    void ( *ret )( void )=( void* )-1;  /* any non-NULL value */
    char ch;
    int i;
    
    ( void )printf( "\r\n\r\nEnter required test number:\r\n" );
    ( void )printf( "\r\n" );
    for( i = 0; ( sizeof( tests )/sizeof( tests[ 0 ]))> i; i++ )
    {
        ( void )printf( "Press %c to %s\r\n", tests[ i ].menukey, tests[ i ].menudesc );
    }

    ( void )printf( "\r\n>" );
    do
    {
        ch = getchar( );
    }
    while( ESC == ch );

    ( void )printf( "%c\r\n\r\n", ch );
    for( i = 0;( sizeof( tests )/sizeof( tests[ 0 ]))> i; i++ )
    {
        if( tests[ i ].menukey == ch )
        {
            if( ret = tests[ i ].testf )
            {
                ret( );
                break;
            }
        }
    }
    
    if(( void* )-1 == ret )  /* ret has not been assigned to */
    {
        ( void )printf( "\r\nSorry, I do not recognise menu option '%c'\r\n", ch );
    }

    return( ret );
}


/* doGPIOWriteTest
 *   Try out DEV API GPIO functions
 *   Set an output that can be seen with a LED or a multimeter probe
 *   To test this I used a breadboard to wire up a red LED with a 220ohm resistor 
 *    in series between Pin 13 and GND (Pin 3). 
 *    V=IR:  3.3v = 220ohm * 15mA in the circuit
 *    A red LED can drop 2v, such that the resistor will drop the remaining 1.3v
 *    Measure the voltage across the LED or across the resistor with a multimeter.
 *    Measure the current by placing the multimeter in series with the circuit
*/
static void doGPIOWriteTest( void )
{
    POSIX_ERRNO res;
    KEYMAP *kbmap;
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    int i;
    
    ( void )printf( "\r\n\r\nRunning GPIO write test.\r\n" );

    // open GPIO:13 as an output, initial value 0
    res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    ( void )printf( "gpio_open returns : %d\r\n", res );


    ( void )printf( "Toggling GPIO pin 13\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );
    kbmap = mos_getkbmap( );  // only need do this once at startup

    // toggle the output until user-halted
    for( i = 1; ; i = 1 - i )
    {
        res = gpio_write( GPIO_13, i );
        ( void )printf( "gpio_write %d returns : %d\r\n", i, res );

        vTaskDelay( 5 );  // delay 0.5s

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "gpio_close\r\n" );
    gpio_close( GPIO_13 );
}


/* doGPIOReadWriteTest
 *   Try out DEV API GPIO functions
 *   Set an output that can be seen with a LED or a multimeter probe
 *   To test this I used a breadboard as per doGPIOWriteTest; and 
 *    in addition Pin 16 is wired to Pin 13 on the breadboard.
 *    With Pin 13 high, V=IR:  3.3v = 220ohm * 15mA in the circuit
*/
static void doGPIOReadWriteTest( void )
{
    POSIX_ERRNO res;
    KEYMAP *kbmap;
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    int i;
    unsigned char ch;
    
    ( void )printf( "\r\n\r\nRunning GPIO write test.\r\n" );

    // open GPIO:13 as an output, initial value 0
    res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    ( void )printf( "gpio_open(13) returns : %d\r\n", res );

    // open GPIO:16 as an input
    res = gpio_open( GPIO_16, DEV_MODE_GPIO_IN );
    ( void )printf( "gpio_open(16) returns : %d\r\n", res );

    ( void )printf( "Toggling GPIO pin 13\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );
    kbmap = mos_getkbmap( );  // only need do this once at startup

    // toggle the output until user-halted
    for( i = 1; ; i = 1 - i )
    {
        res = gpio_write( GPIO_13, i );
        ( void )printf( "gpio_write(13) %d\r\n", i );

        res = gpio_read( GPIO_16, &ch );
        ( void )printf( "gpio_read(16) = 0x%x\r\n", ch );

        vTaskDelay( 5 );  // delay 0.5s

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "gpio_close\r\n" );
    gpio_close( GPIO_13 );
    gpio_close( GPIO_16 );
}


/*
 * myGpioHndlr is an INTERRUPT_HANDLER
 * It will be invoked in context of a DEV API GPIO ISR
 * It must not call any blocking functions, and avoid calls to MOS;
 * It must not disable interrupts, but may itself be interrupted
 *  version for 0 == configUSE_FAST_INTERRUPTS
 */
void myGpioHndlr( DEV_NUM_MAJOR const major, DEV_NUM_MINOR const minor )
{
    gpiocnt++;

#   if defined( _DEBUG )
    {
        putchar( '*' );
    }
#   endif

#   if( 1 == configUSE_FAST_INTERRUPTS )
    {
        /* This conditional block is provided as a template for writing your 
           own FAST ISR. It is not required for the simpler Interrupt Handler,
           configured with 0 == configUSE_FAST_INTERRUPTS, when similar is done
           in devgpio::gpioisr for you. */

        ( void )major;  // not accessible
        ( void )minor;  // not accessible

        /* clear down interrupt */
        // user TODO

        /* return from intrrupt */
        asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
        asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
        asm( "               ;   __default_mi_handler" );
        asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );
        asm( "\t reti.l      ; need reti.L as per UM007715 table 25 for IM 2 ADL=1 MADL=1");
    }
#   endif /*( 1 == configUSE_FAST_INTERRUPTS )*/
}

/* doGpioCallback
 *   Try out DEV API GPIO functions with interrupt
 *   Set an output that can be seen with a LED or a multimeter probe
 *   To test this I used a breadboard as per doGPIOWriteTest; and 
 *    in addition Pin 16 is wired to Pin 13 on the breadboard.
 *    GPIO 16 is opened using DEV_MODE_GPIO_INTRDE mode supplying the handler
 *    All LED state changes should result in the handler being run
*/
static void doGpioCallback( void )
{
    POSIX_ERRNO res;
    KEYMAP *kbmap;
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    int i;

    ( void )printf( "\r\n\r\nRunning gpio callback test\r\n" );
#   if defined( _DEBUG )&& 0
    {
        ( void )printf( "myGpioHndlr = %p\r\n", &myGpioHndlr );    
    }
#   endif

    // open GPIO:13 as an output, initial value 0
    res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    ( void )printf( "gpio_open(13) output returns : %d\r\n", res );

    // open GPIO:18 as an interrupt input
    res = gpio_open( GPIO_18, DEV_MODE_GPIO_INTRDE, myGpioHndlr );
    ( void )printf( "gpio_open(18) interrupt returns : %d\r\n", res );

    ( void )printf( "Toggling GPIO pin 13\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );
    kbmap = mos_getkbmap( );  // only need do this once at startup

    // toggle the output until user-halted
    for( i = 1; ; i = 1 - i )
    {
        ( void )printf( "gpio_write(13) %d\r\n", i );
        res = gpio_write( GPIO_13, i );

        vTaskDelay( 5 );  // delay 0.5s

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "gpio_close\r\n" );
    gpio_close( GPIO_13 );
    gpio_close( GPIO_18 );

    // finish up
    ( void )printf( "gpiocnt = %d\r\n", gpiocnt );
}


/*----- Task Definitions ----------------------------------------------------*/
#pragma asm "\tSEGMENT TASKS"
void Task1( void *pvParameters )
{
    ( void )printf( "\r\nStarting %s\r\n", pcTaskGetName( NULL ));

    for( ;; )
    {
        if( NULL == menu( ))
        {
            break;
        }
    }
    ( void )printf( "\r\nTests complete\r\n" );
    ( void )printf( "Waiting 3 seconds before resetting Agon\r\n");
    vTaskDelay( 30 );  // wait 3 seconds at 10 ticks/sec

    asm( "\tRST.LIS 00h      ; invoke MOS reset");
    
    ( void )pvParameters;
}


/* vApplicationIdleHook
 *   Runs in context of the idle task.
 *   DO NOT call a BLOCKING function from within the IDLE task;
 *   IDLE must always be in either the READY or the RUN state, and no other. 
 *   Typically used to do a heartbeat LED, or perform heap garbage collection.
*/
#pragma asm "\tSEGMENT TASKS"
void vApplicationIdleHook( void )
{
    //Machen mit ein blinken light would be excellent

    idlecnt++;
}
