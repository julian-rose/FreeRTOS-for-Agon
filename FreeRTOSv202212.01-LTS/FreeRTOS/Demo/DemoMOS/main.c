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
 *  Demo application for EZ80 Agon Light
*/
#pragma asm "\tDEFINE TASKS, SPACE = RAM, ALIGN = 10000h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "mosapi.h"


extern unsigned int _heaptop;  // defined in the linker directive file
extern unsigned int _heapbot;  //   "


static unsigned int idlecnt = 0;
static volatile char kbbuf[ 128 ]={ 0 };
static volatile unsigned char kbidx = 0;
static volatile unsigned int kbhcnt = 0;

void Task1( void *pvParameters );
void Task2( void *pvParameters );
void vApplicationIdleHook( void );
void kbHndl( void );

static void doKeyboardTest( void );
static void doKeyboardCallback( void );


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
    r = xTaskCreate( Task1, "MOS test", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );
    if( pdPASS != r )
    {
        ( void )printf( "Failed to allocate Task1: %d\r\n", r );
        goto end;
    }

    /* Start FreeRTOS */
    ( void )printf( "\r\nEntering scheduler\r\n" );
            
    vTaskStartScheduler( );
        
end:
    // should never get here
    ( void )printf( "Back from scheduler\r\n" );
    
    return( 0 );
}


#pragma asm "\tSEGMENT TASKS"
void Task1( void *pvParameters )
{
    ( void )printf( "\r\nStarting %s\r\n", pcTaskGetName( NULL ));

    doKeyboardTest( );
    doKeyboardCallback( );

    ( void )printf( "\r\nTests complete\r\n" );
    for( ;; )    
    {
        portYIELD( );
    }
    
    ( void )pvParameters;
}


void vApplicationIdleHook( void )
{
    /* DO NOT call a BLOCKING function from within the IDLE task;
       IDLE must always be in either the READY or the RUN state, and no other. */
    
    //Machen mit ein blinken light would be excellent

    idlecnt++;
}


static void doKeyboardTest( void )
{
    char ch;
    char prevch = 0;

    ( void )printf( "Running keyboard test. "
                    "Type some keys. "
                    "Press ESC to exit test\r\n" );
    while( 1 )
    {
        while( 0 ==( ch = getchar( )))
            ;
        
        if( prevch != ch )  // simple debounce
        {
            putchar( ch );
            prevch = ch;
        }
        else
        {
            prevch = 0;
        }
        if( ch == '\r' )
        {
            putchar( '\n' );
        }
        if( ch == 27 )
        {
            break;
        }
    }
}


/*
 * kbHndlr is a callback within context the MOS VDP keyboard event ISR
 * It must not call any blocking functions, and avoid calls to MOS;
 * It must not disable interrupts, but may itself be interrupted
 */
void kbHndlr( VDP_KB_PACKET *keyboard_packet )
{
    kbhcnt++;

#   if defined( _DEBUG )
    {
        putchar( '*' );
        ( void )printf( "\r\nascii = %d : state = %s\r\n", 
                         keyboard_packet->keyascii,
                         ( 1 == keyboard_packet->keystate )? "down":"up" );
    }
#   endif

    if( keyboard_packet->keystate )
    {
        kbbuf[ kbidx++ ]= keyboard_packet->keyascii;
        if( 128 <= kbidx ) kbidx--;
    }
}


#if defined( _DEBUG )&& 0
  extern volatile int _mosapi_kbvect;
#endif

static void doKeyboardCallback( void )
{
    int i;
    int x = 0;

#   if defined( _DEBUG )&& 0
	{
        ( void )printf( "kbHndlr = %p\r\n", &kbHndlr );    
	}
#   endif

    ( void )printf( "Running keyboard callback test. "
                    "Type some keys. "
                    "Press CRET to exit test\r\n" );

#   if defined( _DEBUG )&& 0
	{
        ( void )printf( "before _mosapi_kbvect = %p\r\n", _mosapi_kbvect );
	}
#   endif

    mos_setkbvector( &kbHndlr );

#   if defined( _DEBUG )&& 0
	{
        ( void )printf( "after _mosapi_kbvect = %p\r\n", _mosapi_kbvect );
	}
#   endif

    for( ;; )
    {
        unsigned char ch = kbbuf[( 0 < kbidx )?( kbidx - 1 ): 0 ];

asm("di");  // BUG: we need to ensure no interrupts over the if statement
        if( 13 == ch )
        {
asm("ei");  // ENDBUG: otherwise we can enter here false positive, because the flags register is wrong
            putchar( '-' );
            break;
        }
asm("ei");  // ENDBUG: when we expect to get here

asm("di");  // BUG: we need to ensure no interrupts over the if statement
        if( kbhcnt > 25 )
        {
asm("ei");  // ENDBUG: otherwise we can enter here false positive, because the flags register is wrong
            putchar( '/' );
            break;
        }
asm("ei");  // ENDBUG: when we expect to get here

asm("di");  // BUG: we need to ensure no interrupts over the if statement
        if( 1000 < x++ )
        {
asm("ei");  // ENDBUG: otherwise we can enter here false positive, because the flags register is wrong
            putchar( '.' );
            x = 0;
        }
asm("ei");  // ENDBUG: when we expect to get here

    }

#   if defined( _DEBUG )&& 0
	{
        ( void )printf( "\r\nbefore 2nd _mosapi_kbvect = %p\r\n", _mosapi_kbvect );
    }
#   endif
	
    mos_setkbvector( NULL );

#   if defined( _DEBUG )&& 0
	{
        ( void )printf( "after 2nd _mosapi_kbvect = %p\r\n", _mosapi_kbvect );
    }
#   endif

    ( void )printf( "kbhcnt = %d\r\n", kbhcnt );
	if( 0 < kbidx )
	{
        ( void )printf( "Dumping callback buffer len:%d : ", kbidx );
        for( i=0; kbidx > i; i++ )
        {
            putchar( kbbuf[ i ]);
			if( 13 == kbbuf[ i ]) putchar( 10 );
            else putchar( ' ' );
        }
        printf( "\r\n" );
    }
}
