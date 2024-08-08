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

extern BaseType_t __higherPriorityTaskWoken;  // set in ISR to context-switch


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
static volatile unsigned int yieldcnt = 0;
static TaskHandle_t isrTestTaskHandle = NULL;


/*----- Local Declarations --------------------------------------------------*/
void Task1( void *pvParameters );
void vApplicationIdleHook( void );
void /* INTERRUPT_HANDLER */ myGpioHndlr( DEV_NUM_MAJOR const, DEV_NUM_MINOR const );


static void * menu( void );

    /* Device tests */
static void doGPIOWriteTest( void );
static void doGPIOReadWriteTest( void );
static void doGPIOCallbackTest( void );
static void doYieldFromISRTest( void );
static void doUARTRxTxTest( void );     /* DEV_MODE_UART_NO_FLOWCONTROL */
static void doUARTXonXoffTest( void );  /* DEV_MODE_UART_SW_FLOWCONTROL */
static void doUARTRtsCtsTest( void );   /* DEV_MODE_UART_HALF_NULL_MODEM */
static void doUARTFullNullTest( void ); /* DEV_MODE_UART_FULL_NULL_MODEM */
static void doUARTloopbackTest( void ); /* DEV_MODE_UART_LOOPBACK */
static void doSPIBMP280Test( void );
static void doSPIMFRC522Test( void );


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
        { '2', "Test DEV GPIO ISR", doGPIOCallbackTest },
        { '3', "Test ISR Yield", doYieldFromISRTest },
        { '4', "Test DEV UART Rx/Tx (no flow control)", doUARTRxTxTest },
        { '5', "Test DEV UART Xon/Xoff software flow control", doUARTXonXoffTest },
        { '6', "Test DEV UART RTS/CTS hardware flow control", doUARTRtsCtsTest },
        { '7', "Test DEV UART Full Null hardware flow control", doUARTFullNullTest },
        { '8', "Test DEV UART Loopback", doUARTloopbackTest },
        { '9', "Test DEV SPI BMP280 Barometer", doSPIBMP280Test },
        { 'a', "Test DEV SPI MFRC522 RFID", doSPIMFRC522Test },
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
    ch = getchar( );  // getchar blocks within MOS (no Idle task time)

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
#if( 1 == configUSE_DRV_GPIO )
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
#endif /*( 1 == configUSE_DRV_GPIO )*/
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
#if( 1 == configUSE_DRV_GPIO )
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
#endif /*( 1 == configUSE_DRV_GPIO )*/
}

#if( 1 == configUSE_DRV_GPIO )
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
           own FAST ISR, configured with 1 == configUSE_FAST_INTERRUPTS. 
           It is not required for the simpler Interrupt Handler, configured 
           with 0 == configUSE_FAST_INTERRUPTS, which wraps up low-level ISR
           in devgpio::gpioisr to enable the simpler user Interrupt Handler. */

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
#endif /*( 1 == configUSE_DRV_GPIO )*/

/* doGPIOCallbackTest
 *   Try out DEV API GPIO functions with interrupt
 *   Set an output that can be seen with a LED or a multimeter probe
 *   To test this I used a breadboard as per doGPIOWriteTest; and 
 *    in addition Pin 18 is wired to Pin 13 on the breadboard.
 *    Refer to Source/docs/DEV API Extension Interface and MOS issues why we
 *    cannot use GPIOs in ports D and B for rising edge interrupts.
 *    GPIO 18 is opened using DEV_MODE_GPIO_INTRDE mode supplying the handler
 *    All LED state changes should result in the handler being run
*/
static void doGPIOCallbackTest( void )
{
#if( 1 == configUSE_DRV_GPIO )
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
    ( void )printf( "If you don't see '*' in the output, "
                    "check pins 13 and 18 are wired together\r\n" );
    ( void )printf( "Toggling GPIO pin 13 periodically\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );
    kbmap = mos_getkbmap( );  // only need do this once at startup

    // open GPIO:13 as an output, initial value 0
    res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    ( void )printf( "gpio_open(13) output returns : %d\r\n", res );

    // open GPIO:18 as an interrupt input
    res = gpio_open( GPIO_18, DEV_MODE_GPIO_INTRDE, myGpioHndlr );
    ( void )printf( "gpio_open(18) interrupt returns : %d\r\n", res );

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
#endif /*( 1 == configUSE_DRV_GPIO )*/
}


/* IsrTestTask
     High-priority task.
     Will run immediately on exit of myYieldHndlr;
       iff the current task is not blocking inside MOS */
static void IsrTestTask( void * params )
{
    unsigned long notification;

    for( ;; )
    {
        notification = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        if( notification )
        {
            putchar( '+' );
        }
    }
}
/*
 * myYieldHndlr is an INTERRUPT_HANDLER
 * It will be invoked in context of a DEV API GPIO ISR
 * It must not call any blocking functions, and avoid calls to MOS;
 * It must not disable interrupts, but may itself be interrupted
 *
 * This test handler will send a notification to a waiting highest priority task. 
 * On exit from the ISR, that highest priority task should run immediately.
 */
void myYieldHndlr( DEV_NUM_MAJOR const major, DEV_NUM_MINOR const minor )
{
    yieldcnt++;
    putchar( '*' );

    /* run in context of high-priority interrupt handler gpioisr */
    vTaskNotifyGiveFromISR( isrTestTaskHandle, &__higherPriorityTaskWoken );
    
    /* __higherPriorityTaskWoken will be tested in ISR */
}

/* doYieldFromISRTest
 *   Try out vPortYieldFromISR
 *   Set an output that can be seen with a LED or a multimeter probe
 *   To test this I used a breadboard as per doGPIOWriteTest; and 
 *    in addition Pin 18 is wired to Pin 13 on the breadboard.
 *    Refer to Source/docs/DEV API Extension Interface and MOS issues why we
 *    cannot use GPIOs in ports D and B for rising edge interrupts.
 *    GPIO 18 is opened using DEV_MODE_GPIO_INTRDE mode supplying the handler
 *    All LED state changes should result in the handler being run
*/
static void doYieldFromISRTest( void )
{
#if( 1 == configUSE_DRV_GPIO )
    POSIX_ERRNO res;
    BaseType_t r;
    KEYMAP *kbmap;
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    int i;

    ( void )printf( "\r\n\r\nRunning Yield from ISR test\r\n" );
#   if defined( _DEBUG )&& 0
    {
        ( void )printf( "myYieldHndlr = %p\r\n", &myYieldHndlr );    
    }
#   endif
    
    r = xTaskCreate( 
            IsrTestTask, 
            "ISR Test Task", 
            configMINIMAL_STACK_SIZE, 
            NULL,                      // no params
            configMAX_PRIORITIES,      // highest possible priority
            &isrTestTaskHandle );
    if( pdPASS != r )
    {
#       if defined( _DEBUG )
        {
            ( void )printf( "Failed to create IsrTestTask: %d\r\n", r );
        }
#       endif
    }
    else
    {
#       if defined( _DEBUG )
        {
            ( void )printf( "Created IsrTestTask\r\n" );
        }
#       endif
    }

    ( void )printf( "If you don't see '*+' in the output, "
                    "check pins 13 and 18 are wired together\r\n" );
    ( void )printf( "Toggling GPIO pin 13 periodically\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );
    kbmap = mos_getkbmap( );  // only need do this once at startup

    // open GPIO:13 as an output, initial value 0
    res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    ( void )printf( "gpio_open(13) output returns : %d\r\n", res );

    // open GPIO:18 as an interrupt input
    res = gpio_open( GPIO_18, DEV_MODE_GPIO_INTRDE, myYieldHndlr );
    ( void )printf( "gpio_open(18) interrupt returns : %d\r\n", res );

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
    ( void )printf( "yieldcnt = %d\r\n", yieldcnt );
    
    ( void )printf( "Deleting IsrTestTask\r\n\r\n" );
    vTaskDelete( isrTestTaskHandle );
#endif /*( 1 == configUSE_DRV_GPIO )*/
}


/*-------- UART Tests -------------------------------------------------------*/
/* doUARTRxTxTest
 *   Try out DEV API UART functions
 *   Connect UART1 tx pin 17 to UART1 rx pin 18 for loopback
 *    or tx pin 17 to remote (CP2102) Rx and rx pin 18 to remote Tx
 *   Send a string (and Echo it back in loopback)
 *    or receive remote end data (and display it) in remote test
*/
static void doUARTRxTxTest( void )
{
#if( 1 == configUSE_DRV_UART )
    UART_PARAMS const uparm =
        { UART_BAUD_9600, UART_DATABITS_8, UART_STOPBITS_1, UART_PARITY_NONE };
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    KEYMAP * const kbmap = mos_getkbmap( );  // only need do this once at startup
    POSIX_ERRNO res;
    char b[ 32 ];
    int i, j;

    ( void )printf( "\r\n\r\nRunning UART echo test\r\n" );
    ( void )printf( "Either wire Agon UART1 TxD pin 17 to UART1 RxD pin 18 (loopback)\r\n" );
    ( void )printf( "Or crosswire UART1 TxD pin 17 to remote RxD, "
                     "and UART1 RxD pin 18 to remote TxD\r\n" );
    ( void )printf( "Wire Agon GPIO pin 13 (task activity) "
                    "-> red LED+220ohm resistor -> GND\r\n" );
    ( void )printf( "Wire Agon GPIO pin 26 (idle activity) "
                    "-> green LED+150ohm resistor -> GND\r\n" );

    ( void )printf( "Press 'ESC' key to exit test\r\n" );

#   if( 1 == configUSE_DRV_GPIO )
    {
        // open GPIO:13 as an output, initial value 0
        res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
        ( void )printf( "gpio_open(13) output returns : %d\r\n", res );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

    // no flow control (simple rx.tx connection)
    res = uart_open( DEV_MODE_UART_NO_FLOWCONTROL, &uparm );
    ( void )printf( "uart_open returns : %d\r\n", res );

    for( i = 1; ; i = 1 - i )
    {
#       if( 1 == configUSE_DRV_GPIO )
        {
            ( void )gpio_write( GPIO_13, i );  // toggle pin 13 (LED)
        }
#       endif /*( 1 == configUSE_DRV_GPIO )*/

        ( void )printf( "uart_write( \"Hello\") [[" );
        res = uart_write( "Hello", 5 );
        ( void )printf( "]] res = 0x%x\r\n", res );

        vTaskDelay( 5 );  // delay 0.5s

        portENTER_CRITICAL( );
        {
            // always safeguard use of Zilog library calls
            memset( b, 0, sizeof( b ));
        }
        portEXIT_CRITICAL( );
        ( void )printf( "uart_read( ) [[" );
        res = uart_read( &b, 16 );
        ( void )printf( "]] res = 0x%x\r\n", res );
        ( void )printf( "buf = (" );
        for( j=0; 16 > j; j++ ) printf( "0x%x ",( unsigned char )( b[ j ]));
        ( void )printf( ")\r\n" );

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "uart_close\r\n" );
    uart_close( );
#   if( 1 == configUSE_DRV_GPIO )
    {
        ( void )printf( "gpio_close\r\n" );
        gpio_close( GPIO_13 );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

#endif /*( 1 == configUSE_DRV_UART )*/
}


/* doUARTXonXoffTest
 *   Try out DEV API UART software flow control Xon / Xoff
 *   Connect UART1 Tx pin 17 to remote (CP2102) Rx, 
 *     and Rx pin 18 to remote Tx
 *   Open UART1 using DEV_MODE_UART_SW_FLOWCONTROL
 *   Agon to receive a file larger than configDRV_UART_BUFFER_SZ
*/
static void doUARTXonXoffTest( void )
{
#if( 1 == configUSE_DRV_UART )
    UART_PARAMS const uparm =
        { UART_BAUD_9600, UART_DATABITS_8, UART_STOPBITS_1, UART_PARITY_NONE };
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    KEYMAP * const kbmap = mos_getkbmap( );  // only need do this once at startup
    POSIX_ERRNO res;
    char b[ 32 ];
    int i, j;

    ( void )printf( "\r\n\r\nRunning UART Xon / Xoff test\r\n" );
    ( void )printf( "Crosswire Agon UART1 TxD pin 17 to remote RxD, "
                    "and RxD pin 18 to remote TxD\r\n" );
    ( void )printf( "Wire Agon GPIO pin 13 (task activity) "
                    "-> red LED+220ohm resistor -> GND\r\n" );
    ( void )printf( "Wire Agon GPIO pin 26 (idle activity) "
                    "-> green LED+150ohm resistor -> GND\r\n" );

    ( void )printf( "Press 'ESC' key to exit test\r\n" );

#   if( 1 == configUSE_DRV_GPIO )
    {
        // open GPIO:13 as an output, initial value 0
        ( void )gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

    // Xon/Xoff software flow control
    res = uart_open( DEV_MODE_UART_SW_FLOWCONTROL, &uparm );
    ( void )printf( "uart_open returns : %d\r\n", res );

    // read input until a terminator character is read (^-Z)
    for( i = 1; ; i = 1 - i )
    {
#       if( 1 == configUSE_DRV_GPIO )
        {
            ( void )gpio_write( GPIO_13, i );  // toggle pin 13 (LED)
        }
#       endif /*( 1 == configUSE_DRV_GPIO )*/

        portENTER_CRITICAL( );
        {
            // always safeguard use of Zilog library calls
            memset( b, 0, sizeof( b ));
        }
        portEXIT_CRITICAL( );
        ( void )printf( "uart_read( ) [[" );
        res = uart_read( &b, 16 );
        ( void )printf( "]] res = 0x%x\r\n", res );
        ( void )printf( "buf = (" );
        for( j=0; 16 > j; j++ ) printf( "0x%x ",( unsigned char )( b[ j ]));
        ( void )printf( ")\r\n" );

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "uart_close\r\n" );
    uart_close( );
#   if( 1 == configUSE_DRV_GPIO )
    {
        ( void )printf( "gpio_close\r\n" );
        gpio_close( GPIO_13 );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

#endif /*( 1 == configUSE_DRV_UART )*/
}


/* doUARTRtsCtsTest
 *   Try out DEV API UART hardware flow control RTS / CTS
 *   The CP2102 USB-UART bridge is wired (to present the host PC) as a DTE.
 *     So NULL modem wiring between the CP2102 and the Agon (a DTE):
 *       Connect Agon UART1 Tx pin 17 to CP2102 Rx, 
 *       Connect Agon UART1 Rx pin 18 to CP2102 Tx
 *       Connect Agon UART1 RTS pin 19 to CP2102 CTS
 *       Connect Agon UART1 CTS pin 17 to CP2102 RTS
 *   Open UART1 using DEV_MODE_UART_HALF_NULL_MODEM.
 *   Agon to receive a file larger than configDRV_UART_BUFFER_SZ
 *   Send a string or receive remote end data (and display it)
*/
static void doUARTRtsCtsTest( void )
{
#if( 1 == configUSE_DRV_UART )
    UART_PARAMS const uparm =
        { UART_BAUD_9600, UART_DATABITS_8, UART_STOPBITS_1, UART_PARITY_NONE };
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    KEYMAP * const kbmap = mos_getkbmap( );  // only need do this once at startup
    POSIX_ERRNO res;
    char b[ 32 ];
    int i, j;

    ( void )printf( "\r\n\r\nRunning UART Half Null Modem RTS / CTS test\r\n" );

    ( void )printf( "Crosswire Agon UART1 TxD pin 17 to remote RxD, "
                     "and RxD pin 18 to remote TxD\r\n" );
    ( void )printf( "Crosswire Agon RTS pin 19 to remote CTS, "
                     "and CTS pin 20 to remote RTS\r\n" );
    ( void )printf( "Wire Agon GPIO pin 13 (task activity) "
                    "-> red LED+220ohm resistor -> GND\r\n" );
    ( void )printf( "Wire Agon GPIO pin 26 (idle activity) "
                    "-> green LED+150ohm resistor -> GND\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );

#   if( 1 == configUSE_DRV_GPIO )
    {
        // open GPIO:13 as an output, initial value 0
        res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
        ( void )printf( "gpio_open(13) output returns : %d\r\n", res );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

    // hardware flow control (cross-wired NULL Modem RTS-CTS)
    res = uart_open( DEV_MODE_UART_HALF_NULL_MODEM, &uparm );
    ( void )printf( "uart_open( ) returns : %d\r\n", res );

    for( i = 1; ; i = 1 - i )
    {
#       if( 1 == configUSE_DRV_GPIO )
        {
            ( void )gpio_write( GPIO_13, i );  // toggle pin 13 (LED)
        }
#       endif /*( 1 == configUSE_DRV_GPIO )*/

#if 1
        if( i )
        {
            ( void )printf( "uart_write( \"Hello\") [[" );
            res = uart_write( "Hello ", 6 );
        }
        else
        {    
            ( void )printf( "uart_write( \"Agon\") [[" );
            res = uart_write( "Agon ", 5 );
        }
        ( void )printf( "]] res = 0x%x\r\n", res );
#endif
        
        for( j=0; sizeof( b )> j; j++ ) b[ j ]= 0;  // empty b
        ( void )printf( "uart_read( ) [[" );
        res = uart_read( &b, 16 );
        ( void )printf( "]] res = 0x%x\r\n", res );
        ( void )printf( "buf = (" );
        for( j=0; 16 > j; j++ ) printf( "0x%x ",( unsigned char )( b[ j ]));
        ( void )printf( ")\r\n" );

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "uart_close\r\n" );
    uart_close( );
#   if( 1 == configUSE_DRV_GPIO )
    {
        ( void )printf( "gpio_close\r\n" );
        gpio_close( GPIO_13 );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

#endif /*( 1 == configUSE_DRV_UART )*/
}


/* doUARTFullNullTest
 *   Try out DEV API UART hardware loopback
 *   The CP2102 USB-UART bridge is wired (to present the host PC) as a DTE.
 *     No wiring between the CP2102 and the Agon (a DTE)
 *   Open UART1 using DEV_MODE_UART_LOOPBACK.
*/
static void doUARTFullNullTest( void )
{
#if( 1 == configUSE_DRV_UART )
    UART_PARAMS const uparm =
        { UART_BAUD_9600, UART_DATABITS_8, UART_STOPBITS_1, UART_PARITY_NONE };
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    KEYMAP * const kbmap = mos_getkbmap( );  // only need do this once at startup
    POSIX_ERRNO res;
    char b[ 32 ];
    int i, j;

    ( void )printf( "\r\n\r\nRunning UART Full Null test\r\n" );

    ( void )printf( "Crosswire Agon UART1 TxD pin 17 to remote RxD, "
                     "and RxD pin 18 to remote TxD\r\n" );
    ( void )printf( "Crosswire Agon RTS pin 19 to remote CTS, "
                     "and CTS pin 20 to remote RTS\r\n" );
    ( void )printf( "Crosswire Agon DTR pin 21 to remote DSR + DCD, "
                     "and DSR pin 22 + DCD pin 23 to remote DTR\r\n" );
    ( void )printf( "Wire Agon GPIO pin 13 (task activity) "
                    "-> red LED+220ohm resistor -> GND\r\n" );
    ( void )printf( "Wire Agon GPIO pin 26 (idle activity) "
                    "-> green LED+150ohm resistor -> GND\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );

#   if( 1 == configUSE_DRV_GPIO )
    {
        // open GPIO:13 as an output, initial value 0
        res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
        ( void )printf( "gpio_open(13) output returns : %d\r\n", res );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

    // hardware flow control (cross-wired NULL Modem RTS-CTS)
    res = uart_open( DEV_MODE_UART_FULL_NULL_MODEM, &uparm );
    ( void )printf( "uart_open( ) returns : %d\r\n", res );

    for( i = 1; ; i = 1 - i )
    {
#       if( 1 == configUSE_DRV_GPIO )
        {
            ( void )gpio_write( GPIO_13, i );  // toggle pin 13 (LED)
        }
#       endif /*( 1 == configUSE_DRV_GPIO )*/

        if( i )
        {
            ( void )printf( "uart_write( \"Hello\") [[" );
            res = uart_write( "Hello ", 6 );
        }
        else
        {    
            ( void )printf( "uart_write( \"Agon\") [[" );
            res = uart_write( "Agon ", 5 );
        }
        ( void )printf( "]] res = 0x%x\r\n", res );
        
        for( j=0; sizeof( b )> j; j++ ) b[ j ]= 0;  // empty b
        ( void )printf( "uart_read( ) [[" );
        res = uart_read( &b, 16 );
        ( void )printf( "]] res = 0x%x\r\n", res );
        ( void )printf( "buf = (" );
        for( j=0; 16 > j; j++ ) printf( "0x%x ",( unsigned char )( b[ j ]));
        ( void )printf( ")\r\n" );

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "uart_close\r\n" );
    uart_close( );
#   if( 1 == configUSE_DRV_GPIO )
    {
        ( void )printf( "gpio_close\r\n" );
        gpio_close( GPIO_13 );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

#endif /*( 1 == configUSE_DRV_UART )*/
}


/* doUARTloopbackTest
 *   Try out DEV API UART hardware loopback
 *   The CP2102 USB-UART bridge is wired (to present the host PC) as a DTE.
 *     No wiring between the CP2102 and the Agon (a DTE)
 *   Open UART1 using DEV_MODE_UART_LOOPBACK.
*/
static void doUARTloopbackTest( void )
{
#if( 1 == configUSE_DRV_UART )
    UART_PARAMS const uparm =
        { UART_BAUD_9600, UART_DATABITS_8, UART_STOPBITS_1, UART_PARITY_NONE };
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    KEYMAP * const kbmap = mos_getkbmap( );  // only need do this once at startup
    POSIX_ERRNO res;
    char b[ 32 ];
    int i, j;

    ( void )printf( "\r\n\r\nRunning UART Loopback test\r\n" );

    ( void )printf( "Wire Agon GPIO pin 13 (task activity) "
                    "-> red LED+220ohm resistor -> GND\r\n" );
    ( void )printf( "Wire Agon GPIO pin 26 (idle activity) "
                    "-> green LED+150ohm resistor -> GND\r\n" );
    ( void )printf( "Unwire all RS232 pins\r\n" );
    ( void )printf( "Press 'ESC' key to exit test\r\n" );

#   if( 1 == configUSE_DRV_GPIO )
    {
        // open GPIO:13 as an output, initial value 0
        res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
        ( void )printf( "gpio_open(13) output returns : %d\r\n", res );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

    // hardware flow control (loopback)
    res = uart_open( DEV_MODE_UART_LOOPBACK, &uparm );
    ( void )printf( "uart_open( ) returns : %d\r\n", res );

    // negotiate call set-up
    i = 1;
    uart_ioctl( DEV_IOCTL_UART_SET_DTR, &i );  // sets both DTR and DCD in loopback
    uart_ioctl( DEV_IOCTL_UART_WRITE_MODEM, &i );
    for( i=0; 0 == i; ) uart_ioctl( DEV_IOCTL_UART_GET_DSR, &i );
    uart_write( "ATDT+1234567890", 15 );  // command 'modem' to call 1234567890
    uart_read( &b, 2 );  // wait for "ok" confirmation
    uart_ioctl( DEV_IOCTL_UART_GET_DCD, &i );
    i = 0;
    uart_ioctl( DEV_IOCTL_UART_WRITE_MODEM, &i );

    // main test loop
    for( i = 1; ; i = 1 - i )
    {
#       if( 1 == configUSE_DRV_GPIO )
        {
            ( void )gpio_write( GPIO_13, i );  // toggle pin 13 (LED)
        }
#       endif /*( 1 == configUSE_DRV_GPIO )*/


        if( i )
        {
            ( void )printf( "uart_write( \"Hello\") [[" );
            res = uart_write( "Hello ", 6 );
        }
        else
        {    
            ( void )printf( "uart_write( \"Agon\") [[" );
            res = uart_write( "Agon ", 5 );
        }
        ( void )printf( "]] res = 0x%x\r\n", res );
        
        for( j=0; sizeof( b )> j; j++ ) b[ j ]= 0;  // empty b
        ( void )printf( "uart_read( ) [[" );
        res = uart_read( &b, 16 );
        ( void )printf( "]] res = 0x%x\r\n", res );
        ( void )printf( "buf = (" );
        for( j=0; 16 > j; j++ ) printf( "0x%x ",( unsigned char )( b[ j ]));
        ( void )printf( ")\r\n" );

        /* scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "uart_close\r\n" );
    uart_close( );
#   if( 1 == configUSE_DRV_GPIO )
    {
        ( void )printf( "gpio_close\r\n" );
        gpio_close( GPIO_13 );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

#endif /*( 1 == configUSE_DRV_UART )*/
}

/*-------- SPI Tests --------------------------------------------------------*/
static unsigned short dig_T1;
static signed short dig_T2, dig_T3;
static unsigned short dig_P1;
static signed short dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static signed long T_fine = 0;
static unsigned char spiReadBuf[ 20 ];

/* Read compensation calibration data from device ROM, refer to 
   BMP280-DS001-26, section 3.11.3 Table 17, and section 3.12 for 
   typical values */
static void bmp280GetCalibrationData( void )
{        
    unsigned char const readCompDataTCmd[ ]=
        { 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0xFF };
    unsigned char const readCompDataPCmd[ ]=
        { 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 
          0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D,
          0x9E, 0x9F, 0xFF };
    POSIX_ERRNO res;

    /* read temperature compensation data */
    res = spi_read( GPIO_14, readCompDataTCmd, spiReadBuf, 7 );
    dig_T1 =( unsigned short )( spiReadBuf[ 2 ]<< 8 )+  // msb  typical=27504
            ( unsigned short )( spiReadBuf[ 1 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_T1=%d ", dig_T1 );
#   endif

    dig_T2 =( signed short )( spiReadBuf[ 4 ]<< 8 )+  // msb    typical=26435
            ( signed short )( spiReadBuf[ 3 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_T2=%d ", dig_T2 );
#   endif

    dig_T3 =( signed short )( spiReadBuf[ 6 ]<< 8 )+  // msb    typical=-1000
            ( signed short )( spiReadBuf[ 5 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_T3=%d\r\n", dig_T3 );
#   endif

    /* read pressure compensation data */
    res = spi_read( GPIO_14, readCompDataPCmd, spiReadBuf, 19 );
    dig_P1 =( unsigned short )( spiReadBuf[ 2 ]<< 8 )+  // msb  typical=36477
            ( unsigned short )( spiReadBuf[ 1 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P1=%d ", dig_P1 );
#   endif

    dig_P2 =( signed short )( spiReadBuf[ 4 ]<< 8 )+  // msb    typical=-10685
            ( signed short )( spiReadBuf[ 3 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P2=%d ", dig_P2 );
#   endif

    dig_P3 =( signed short )( spiReadBuf[ 6 ]<< 8 )+  // msb    typical=3024
            ( signed short )( spiReadBuf[ 5 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P3=%d ", dig_P3 );
#   endif

    dig_P4 =( signed short )( spiReadBuf[ 8 ]<< 8 )+  // msb    typical=2855
            ( signed short )( spiReadBuf[ 7 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P4=%d ", dig_P4 );
#   endif

    dig_P5 =( signed short )( spiReadBuf[ 10 ]<< 8 )+  // msb   typical=140
            ( signed short )( spiReadBuf[ 9 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P5=%d ", dig_P5 );
#   endif

    dig_P6 =( signed short )( spiReadBuf[ 12 ]<< 8 )+  // msb   typical=-7
            ( signed short )( spiReadBuf[ 11 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P6=%d ", dig_P6 );
#   endif

    dig_P7 =( signed short )( spiReadBuf[ 14 ]<< 8 )+  // msb   typical=15500
            ( signed short )( spiReadBuf[ 13 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P7=%d ", dig_P7 );
#   endif

    dig_P8 =( signed short )( spiReadBuf[ 16 ]<< 8 )+  // msb   typical=-14600
            ( signed short )( spiReadBuf[ 15 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P8=%d ", dig_P8 );
#   endif

    dig_P9 =( signed short )( spiReadBuf[ 18 ]<< 8 )+  // msb   typical=6000
            ( signed short )( spiReadBuf[ 17 ]<< 0 );  // lsb
#   if defined( _DEBUG )
    ( void )printf( "dig_P9=%d\r\n", dig_P9 );
#   endif
}


/* Refer to Bosch BMP280-DS001-26. 
   We use section 8 appendix 1 integer 32-bit code, rather than the 64-bit code
   in section 3.11.3 */
static signed long bmp280AdcCalibratedTemp( signed long adc_T )
{
    signed long var1, var2, T;
    
    var1 =(( adc_T >> 3 )-(( signed long )dig_T1 << 1 ));
    var1 =( var1 *( signed long )dig_T2 );
    var1 >>= 11;
    var2 =(( adc_T >> 4 )-(( signed long )dig_T1 ));
    var2 *= var2;
    var2 =(( var2 >> 12 )*( signed long )dig_T3 );
    var2 >>= 14;
    T_fine = var1 + var2;

    T =((( T_fine * 5 )+128 )>> 8 );
    return( T );
}


static unsigned long bmp280AdcCalibratedPressure( signed long adc_P )
{
    signed long var1, var2, var3, var4;
    unsigned long P;
    
    var1 =(((( signed long )T_fine )>> 1 )-( signed long )64000 );
    var2 =(((( var1 >> 2 )*( var1 >> 2 ))>> 11 )*( signed long )dig_P6 );
    var2 +=(( var1 *( signed long )dig_P5 )<< 1 );
    var2 >>= 2;
    var2 +=((( signed long )dig_P4 )<< 16 );
    var3 =((( var1 >> 2 )*( var1 >> 2 ))>> 13 );
    var3 =(( var3 * dig_P3 )>> 3 );
    var4 =((( signed long )dig_P2 * var1 )>> 1 );
    var1 =(( var3 + var4 )>> 18 );
    var1 =((( var1 + 32768 )*(( signed long )dig_P1 ))>> 15 );
    if( 0 == var1 )
    {
        P = 0;
    }
    else
    {
        P =( unsigned long )((( signed long )1048576 )- adc_P );
        P -=( var2 >> 12 );
        P *= 3125;
        if(( unsigned long )0x80000000 > P )
        {
            P =(( P << 1 )/( unsigned long )var1 );
        }
        else
        {
            P =(( P /( unsigned long )var1 )* 2 );
        }
        
        var3 =( signed long )((( P >> 3 )*( P >> 3 ))>> 13 );
        var1 =(((( signed long )dig_P9 )* var3 )>> 12 );
        var2 =(((( signed long )dig_P8 )*(( signed long )( P >> 2 )))>> 13 );
        var4 =(( var1 + var2 + dig_P7 )>> 4 );
        P =( unsigned long )(( signed long )P + var4 );
    }
    
    return( P );
}


/* doSPIBMP280Test
 *   Try out DEV SPI talking to a BMP280 Barometer
 *   Read temperature and pressure data from a BMP280 SPI module; refer to 
  *     Bosch BMP280-DS001-26.
 *   Also refer to Zilog PS015317, SPI section. Note the eZ80 SPI is only 
 *     activated by a write; any read occurs synchronously with the write;
 *     such that data for commanded address n will arrive at n+1. 
 *   Since each SPI-target device differs, initialisation requires 
 *     application-specific programming; we will use Forced Mode such that 
 *     the application controls measurement timing.
 *   BMP280 commands, with ref to BMP280-DS001-26 section 5.3.1 Figure 10:
 *     For reading data, commands consist of an array of BMP280 register 
 *       addresses with the top bit set to 1=read; 
 *       data for commanded address n will arrive at n+1; 
 *       the last address N is a dummy address, to acquire the N-1 read data
 *     For writing, commands consist of an array of <address,data> pairs,
 *       with the top address bit set to 0=write.
*/
static void doSPIBMP280Test( void )
{
#if( 1 == configUSE_DRV_SPI )

    SPI_PARAMS const sparms =
        { SPI_BAUD_115200, SPI_CPOL_HIGH, SPI_CPHA_TRAILING };
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    KEYMAP * const kbmap = mos_getkbmap( );  // only need do this once at startup
    POSIX_ERRNO res;
    signed long adc_P, adc_T;
    unsigned long Pressure;
    signed long Temperature;
    int i, j;

    ( void )printf( "\r\n\r\nRunning SPI BMP280 test\r\n" );

    ( void )printf( "Wire BMP280 SPI module\r\n"
                    "\tSCLK<--pin 31, MOSI<--pin 32, MISO-->pin 27 " );
    ( void )printf( "VDD<--pin 34, GND<--pin 33 "
                    "\tCSB<-- GPIO pin 14\r\n" );

    ( void )printf( "Wire Agon GPIO pin 13 (task activity) "
                    "-> red LED+220ohm resistor -> GND\r\n" );
    ( void )printf( "Wire Agon GPIO pin 26 (idle activity) "
                    "-> green LED+150ohm resistor -> GND\r\n" );

    ( void )printf( "Press 'ESC' key to exit test\r\n" );

    // open GPIO:13 as an output, initial value 0
    res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    ( void )printf( "gpio_open(13) output returns : %d\r\n", res );

    // open the BMP280, using GPIO_14 as Emulated /SS
    res = spi_open( GPIO_14, DEV_MODE_SPI_DEFAULT, &sparms );
    ( void )printf( "spi_open( ) returns : %d\r\n", res );

    for( j=0; sizeof( spiReadBuf )> j; j++ ) spiReadBuf[ j ]= 0x55;
    
#if 0
    /* BMP280-DS001-26 section 5.1, pulling CSB low fixes the BMP280 in SPI 
       mode (and disables I2C until power reset) */
    {
        /* Configure the BMP280, refer to BMP280-DS001-26, section 4.3.5
           Table 23, section 3.3.3 table 6, 3.6.3 table 11
        */
        unsigned char const writeConfigCmd[ ]=
        //write 0xF5 t_sb=0.5ms     filter=off    SPI=4-wire
            { 0x75,( 0x000 << 5 )+( 0x000 << 2 )+( 0x0 << 0 )};

        res = spi_write( GPIO_14, writeConfigCmd, 2 );
    }
#endif

    /* BMP280-DS001-26 section 3.6.1, in sleep mode ChipID and calibration
       data can be read */
    {
         //int j;
        /* Sanity Check, read the device ID register */
        unsigned char const readIdCmd[ ]={ 0xD0, 0xFF };

        res = spi_read( GPIO_14, readIdCmd, spiReadBuf, 2 );
        /* We expect spiReadBuf[ 0 ] to be garbage, spiReadBuf[ 1 ]==0x58 */
        //for( j=0; 2 > j; j++ )( void )printf( "0x%x ", spiReadBuf[ j ]);
        ( void )printf( "Device Id : 0x%x\r\n", spiReadBuf[ 1 ]);
    }

    bmp280GetCalibrationData( );

    {
        /* Configure the BMP280 measurement params, refer to BMP280-DS001-26, 
           section 3.3 Table 7: like "Weather monitoring" (Forced mode) with 
           Standard Resolution
             table 4: osrs_p[2:0]=011 (18bit/0.66Pa/over sampling=1)
             table 5: osrs_t[2:0]=001 (16bit/0.005degC/over sampling=1)
             table 6: IIR=off (1 samples=75% step response)
             table 10: mode[1:0]=01 (Forced mode, one measurement period)
        */
        unsigned char const writeConfigCmd[ ]= // Table 20, 21, 22
                  // osrs_t[7:5] +  osrs_p[4:2] +  mode[1:0]
            { 0x74,( 0x001 << 5 )+( 0x011 << 2 )+( 0x0 << 0 )};

        res = spi_write( GPIO_14, writeConfigCmd, 2 );
    }

    // main test loop
    for( i = 1; ; i = 1 - i )
    {
        /* 1. toggle activity LED */
        ( void )gpio_write( GPIO_13, i );  // toggle pin 13 (LED)

        /* 2. Read the BMP280 (see section 3.6.2, Figure 3 timing diagram) */
        {
            /* 2.1 send Forced mode command */
            unsigned char const writeMeasureCmd[ ]=
                      // osrs_t[7:5] +  osrs_p[4:2] +  mode[1:0]
                { 0x74,( 0x001 << 5 )+( 0x011 << 2 )+( 0x01 << 0 )};

            res = spi_write( GPIO_14, writeMeasureCmd, 2 );
        }
        
           /* 2.2 wait for (at least) the measurement period, table 13
              Standard Resolution takes max 13.3mS */
        vTaskDelay( configTICK_RATE_HZ );  // double-up as the task rate
        
        {
            /* 2.3 get the BMP280 ADC data readout
               The eZ80 requires data writes in order to activate the SPI;
               write control words containing the BMP280 addresses to be read.
               n addressed data value is returned in the n+1 buffer position */
            unsigned char const readDataCmd[ ]=
                { 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFF };
            //    xxxx, psmb, plsb, pxsb, tmsb, tlsb, txsb   spiReadBuf

            res = spi_read( GPIO_14, readDataCmd, spiReadBuf, 7 );

            adc_P =(( spiReadBuf[ 3 ]& 0xf0 )>> 4 )+ /* P xlsb[3:0]  up[3:0]   */
                    ( spiReadBuf[ 2 ]<< 4 )+         /* P  lsb[7:0]  up[11:4]  */
                    ( spiReadBuf[ 1 ]<< 12 );        /* P  msb[7:0]  up[19:12] */

            adc_T =(( spiReadBuf[ 6 ]& 0xf0 )>> 4 )+ /* T xlsb[3:0]  ut[3:0]   */
                    ( spiReadBuf[ 5 ]<< 4 )+         /* T  lsb[7:0]  ut[11:4]  */
                    ( spiReadBuf[ 4 ]<< 12 );        /* T  msb[7:0]  ut[19:12] */
        }

        /* 3. Compute the measurement values from the read adc data and the
              device calibration data; refer to BMP280 section 3.11.3. 
              Get the temperature first as it sets global T_fine ahead of its 
              use in Pressure. 
              As Math library functions are used, need to guard against task 
              re-entrancy. */
        portENTER_CRITICAL( );
        {
            Temperature = bmp280AdcCalibratedTemp( adc_T );
            Pressure = bmp280AdcCalibratedPressure( adc_P );
        }
        portEXIT_CRITICAL( );

        /* 4. Report the Barometric measurements */
        ( void )printf( "Pressure = %d, Temperature = %d\r\n", 
                        Pressure/100,
                        Temperature/100 );

        /* 5. scan keyboard for ESC */
        if((( char* )kbmap )[ escbyte ]&( 1 << escbit ))
        {
            break;
        }
    }

    ( void )printf( "spi_close\r\n" );
    spi_close( GPIO_14 );
    ( void )printf( "gpio_close\r\n" );
    gpio_close( GPIO_13 );

#endif /*( 1 == configUSE_DRV_SPI )*/
}


/* doSPIMFRC522Test
 *   Try out DEV SPI talking to a RC522 RFID device; read any register to prove
 *     the SPI interface
 *   Refer to:
 *     NXP MFRC522DS Rev 3.9 https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
 *     RC522 RFID Development Kit http://www.handsontec.com/dataspecs/RC522.pdf
 *   Read data from an MRFC522 SPI module.
 *   Also refer to Zilog PS015317, SPI section. Note the eZ80 SPI is only 
 *     activated by a write; any read occurs synchronously with the write;
 *     such that data for commanded address n will arrive at n+1. 
 *   Since each SPI-target device differs, initialisation requires 
 *     application-specific programming.
 *   MFRC522 commands, refer to MFRC522DS section 8.1.2
 *     For reading data, section 8.1.2.1, commands consist of an array of 
 *       register addresses, with the top bit set to 1=read;
 *       format => 0x80 + (addr<<1) + 0; eg. address 0x37 => 0xEE = 1110 1110
 *       data for commanded address n will arrive at n+1;
 *       the last address N is a dummy address, to acquire the N-1 read data;
 *     For writing, commands consist of an array of <address,data> pairs,
 *       with the top address bit set to 0=write.
*/
static void doSPIMFRC522Test( void )
{
#if( 1 == configUSE_DRV_SPI )

    SPI_PARAMS const sparms =
        { SPI_BAUD_115200, SPI_CPOL_HIGH, SPI_CPHA_TRAILING };
    int const escbyte =((( 113 - 1 )& 0xF8 )>> 3 );
    int const escbit =  (( 113 - 1 )& 0x07 );
    KEYMAP * const kbmap = mos_getkbmap( );  // only need do this once at startup
    POSIX_ERRNO res;
    int j;

    ( void )printf( "\r\n\r\nRunning SPI RC522 test\r\n" );

    ( void )printf( "Wire RFID RC522 module\r\n"
                    "\tSCK<--pin 31, MOSI<--pin 32, MISO-->pin 27 " );
    ( void )printf( "3.3V<--pin 34, GND<--pin 33 "
                    "\tSDA<-- GPIO pin 14\r\n" );

    ( void )printf( "Wire Agon GPIO pin 13 (task activity) "
                    "-> red LED+220ohm resistor -> GND\r\n" );
    ( void )printf( "Wire Agon GPIO pin 26 (idle activity) "
                    "-> green LED+150ohm resistor -> GND\r\n" );

    ( void )printf( "Press 'ESC' key to exit test\r\n" );

    // open GPIO:13 as an output, initial value 0
    res = gpio_open( GPIO_13, DEV_MODE_GPIO_OUT, 0 );
    ( void )printf( "gpio_open(13) output returns : %d\r\n", res );

    // open the RC522, using GPIO_14 as Emulated /SS
    res = spi_open( GPIO_14, DEV_MODE_SPI_DEFAULT, &sparms );
    ( void )printf( "spi_open( ) returns : %d\r\n", res );

    for( j=0; sizeof( spiReadBuf )> j; j++ ) spiReadBuf[ j ]= 0x55;
    
    {
        int j;
        
        /* Sanity Check, read the device ID register (0x37) */
        // address 0x37 => 0x80 (read) + 0x37<<1 + 0 = 0xEE = 1110 1110
        unsigned char const readIdCmd[ ]={ 0xEE, 0xFF };

        res = spi_read( GPIO_14, readIdCmd, spiReadBuf, 2 );
        /* We expect spiReadBuf[ 0 ] to be garbage, spiReadBuf[ 1 ]==0x92 */
        for( j=0; 2 > j; j++ )( void )printf( "0x%x ", spiReadBuf[ j ]);
        ( void )printf( "\r\nDevice Id : 0x%x\r\n", spiReadBuf[ 1 ]);
    }

    ( void )printf( "spi_close\r\n" );
    spi_close( GPIO_14 );
    ( void )printf( "gpio_close\r\n" );
    gpio_close( GPIO_13 );

#endif /*( 1 == configUSE_DRV_SPI )*/
}


/*----- Task Definitions ----------------------------------------------------*/
#pragma asm "\tSEGMENT TASKS"
void Task1( void *pvParameters )
{
    POSIX_ERRNO res;

    ( void )printf( "\r\nStarting %s\r\n", pcTaskGetName( NULL ));

#   if( 1 == configUSE_DRV_GPIO )
    {
        // open GPIO:26 as an output, initial value 0
        res = gpio_open( GPIO_26, DEV_MODE_GPIO_OUT, 0 );
        if( POSIX_ERRNO_ENONE == res )
        {
            ( void )printf( "Assigned gpio(26) to the idle task\r\n" );
        }
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/


    for( ;; )
    {
        if( NULL == menu( ))
        {
            break;
        }
    }
    
#   if( 1 == configUSE_DRV_GPIO )
    {
        gpio_close( GPIO_26 );
    }
#   endif /*( 1 == configUSE_DRV_GPIO )*/

    ( void )printf( "\r\nTests complete\r\n" );
    ( void )printf( "Waiting 3 seconds before resetting Agon\r\n");
    vTaskDelay( configTICK_RATE_HZ * 3 );  // wait 3 seconds

    asm( "\tRST.LIS 00h      ; invoke MOS reset" );
    
    ( void )pvParameters;
}


/* vApplicationIdleHook
 *   Runs in context of the idle task.
 *   DO NOT call a BLOCKING function from within the IDLE task;
 *    IDLE must always be in either the READY or the RUN state, and no other. 
 *   Typically used to do a heartbeat LED.
*/
#pragma asm "\tSEGMENT TASKS"
void vApplicationIdleHook( void )
{
    unsigned int toggleCnt =
#   if( 1 == configUSE_DRV_UART )
        10000;  
#   else
        30000;  
#   endif
    static unsigned char led = 0;
    static unsigned int toggle = 0;

    idlecnt++;
    
    //Machen mit ein blinken light
    toggle++;
    if( 10000 < toggle ) 
    {
        toggle = 0;
#       if defined( _DEBUG )&& 0
            _putchf( 'I' );
#       endif
        led = 1 - led;
#       if( 1 == configUSE_DRV_GPIO )
        {
            ( void )gpio_write( GPIO_26, led );
        }
#       endif /*( 1 == configUSE_DRV_UART )*/
    }

#   if( 1 == configUSE_DRV_UART )
    {
        /* manage UART Xon/Xoff or RTS/CTS flow control */
        portENTER_CRITICAL( );
        {
            uart_ioctl( DEV_IOCTL_UART_EXEC_RX_FLOW_CONTROL, NULL );
        }
        portEXIT_CRITICAL( );
    }
#   endif /*( 1 == configUSE_DRV_UART )*/
}
