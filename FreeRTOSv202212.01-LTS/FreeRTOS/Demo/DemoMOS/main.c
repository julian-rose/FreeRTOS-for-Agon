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

#include "FreeRTOS.h"
#include "task.h"
#include "mosapi.h"
#include "ffsapi.h"


/*----- Global Names --------------------------------------------------------*/
extern unsigned int _heaptop;  // defined in the linker directive file
extern unsigned int _heapbot;  //   "


/*----- Local Types ---------------------------------------------------------*/
typedef struct _mosftest
{
    char menukey;
    char const * menudesc;
    void( *testf )( void );
} MOSFTEST;


/*----- Private Variables ---------------------------------------------------*/
static unsigned int idlecnt = 0;
#define KBBUF_SIZ 128
static volatile char kbbuf[ KBBUF_SIZ ]={ 0 };
static volatile unsigned char kbidx = 0;
static volatile unsigned int kbhcnt = 0;


/*----- Local Declarations --------------------------------------------------*/
void Task1( void *pvParameters );
void vApplicationIdleHook( void );
void kbHndl( void );

static void * menu( void );
static char const * const getName( void );
static void doKeyboardTest( void );
static void doKeyboardCallback( void );
static void doLoadFile( void );
static void doSaveFile( void );
static void doChangeDirectory( void );
static void doDeleteFile( void );
static void doRenameFile( void );
static void doMakeDirectory( void );
static void doCopyFile( void );
static void doSysVars( void );
static void doFopsC( void );
static void doFopsB( void );
static void doFopsS( void );
static void doGetError( void );
static void doGetFil( void );
static void doFFopsR( void );
static void doFFopsS( void );
static void doFFopsW( void );


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
            "MOS test", 
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
    MOSFTEST const tests[ ]=
    {
        { '0', "Test MOS function 00h \"get_key\"", doKeyboardTest },
        { '1', "Test MOS function 1Dh \"set_kbvector\"", doKeyboardCallback },
        { '2', "Test MOS function 01h \"mos_load\"", doLoadFile },
        { '3', "Test MOS function 02h \"mos_save\"", doSaveFile },
        { '4', "Test MOS function 03h \"mos_cd\"", doChangeDirectory },
        { '5', "Test MOS function 05h \"mos_del\"", doDeleteFile },
        { '6', "Test MOS function 06h \"mos_ren\"", doRenameFile },
        { '7', "Test MOS function 07h \"mos_mkdir\"", doMakeDirectory },
        { '8', "Test MOS function 11h \"mos_copy\"", doCopyFile },
        { '9', "Test MOS function 08h \"mos_sysvar\"", doSysVars },
        { 'a', "Test MOS function 0Ah..0Eh \"mos_fgetc, fputc fileops\"", doFopsC },
        { 'b', "Test MOS function 1Ah..1Bh \"mos_fread, mos_fwrite fileops\"", doFopsB },
        { 'c', "Test MOS function 1Ch \"mos_flseek\"", doFopsS },
        { 'd', "Test MOS function 0Fh \"mos_getError\"", doGetError },
        { 'e', "Test MOS function 19h \"mos_getFil\"", doGetFil },
        { 'f', "Test MOS function 80h..82h \"ffs_read\"", doFFopsR },
        { 'g', "Test MOS function 83h \"ffs_fwrite\"", doFFopsW },
        { 'h', "Test MOS function 84h \"ffs_flseek\"", doFFopsS },
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
    ch = getchar( );
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


/* doKeyboardTest
 *   Try out MOS function 0 "getkey".
*/
static void doKeyboardTest( void )
{
    char ch;
    char prevch = 0;

    ( void )printf( "\r\n\r\nRunning keyboard test. "
                    "Type some keys. Press ESC to exit test.\r\n" );
    while( 1 )
    {
        while( 0 ==( ch = mos_getkey( )))
            ;

        if( ESC == ch )
        {
            break;
        }
        putchar( ch );
        if( ch == '\r' )
        {
            putchar( '\n' );
        }
    }
}


/*
 * kbHndlr is a callback within context of the MOS VDP keyboard event ISR.
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
        if( KBBUF_SIZ <= kbidx ) kbidx--;
    }
}


/* doKeyboardCallback
 *   Try out MOS function 1Dh "setkbvector".
 *   An alternative to MOS fucntion 0, bypass the MOS key input buffer and 
 *   attach directly to the VDP ISR in MOS to buffer our own input. This API 
 *   design is well-suited to real-time systems as it treats the keyboard 
 *   like a device. 
*/
static void doKeyboardCallback( void )
{
    extern volatile int _mosapi_kbvect;
    int i;
    int x = 0;

#   if defined( _DEBUG )&& 0
    {
        ( void )printf( "kbHndlr = %p\r\n", &kbHndlr );    
    }
#   endif

    ( void )printf( "\r\n\r\nRunning keyboard callback test. "
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

        if( CRET == ch )
        {
            putchar( '-' );
            break;
        }

        if( KBBUF_SIZ <= kbhcnt )
        {
            putchar( '/' );
            break;
        }

        if( 1000 < x++ )
        {
            putchar( '.' );
            x = 0;
        }
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


/* getName
 *   Enter a file or path name
 *     Common routine for load_file, save_file, cd, ....
*/
static char const * const getName( void )
{
    static char fn[ configMAX_TASK_NAME_LEN ];
    int i;
    unsigned char ch;

    ( void )memset( fn, 0, configMAX_TASK_NAME_LEN );   // fill buffer with 0s

    for( i = 0; configMAX_TASK_NAME_LEN > i; i++ )
    {
        while( 0 ==( ch = mos_getkey( )))
            ;  /*busy wait for a key press */

        if( CRET == ch )
           {
            fn[ i ]= 0;   // string terminator
            break;
        }
           if( BS == ch )
        {
            fn[ --i ]= 0;
            putchar( BS );
            putchar( ' ' );
            putchar( BS );
            continue;
           }

        fn[ i ]= ch;
        putchar( ch );
    }
    
    return( fn );
}


/* doLoadFile
 *   Try out MOS function 01h "mos_load".
 *   User to enter filename
 *   Routine will mallocate some memory 
 *   then call mos_load to copy the file from SD-card to the mallocated address
 *    or return a meaningful error if the file cannot be copied
*/
#define MAX_FILE_SIZE 1024

static void doLoadFile( void )
{
    char *fn;
    int i;
    void * buf;
    MOS_ERRNO err;

#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif

    ( void )printf( "\r\n\r\nRunning file load test.\r\n" );
    
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s
        
        ( void )printf( "Enter a filename (without any path) eg. \"test.dat\".\r\n"
                        "Press <enter> to finish.\r\n" );
        fn = getName( );    
    
        ( void )printf( 
                  "\r\nLoading up to %d bytes from '%s' on SD-card into memory\r\n",
                  MAX_FILE_SIZE, 
                  fn );

        err = mos_load( fn, buf, MAX_FILE_SIZE );
        configASSERT(( MOS_ERR_OK <= err )&&( MOS_ERR_END > err ));
        switch( err )
        {
            default :
            {
                ( void )printf( "\r\nmos_load returned error %d : %s\r\n",
                                    err,
                                    mos_errnos[ err ].errstr );
            }
            break;

               case MOS_ERR_OK :
            {
                // assume the file type is text
                ( void )printf( "\r\nDumping first few bytes of file:\r\n" );
                for( i = 0; MAX_FILE_SIZE > i; i++ )
                {
                    if(( EOF ==(( char* )buf )[ i ])||
                       ( ESC ==(( char* )buf )[ i ])||
                       ( 0 ==(( char* )buf )[ i ]))
                    {
                        break;
                    }
                    putchar((( char* )buf )[ i ]);
                    if(( CRET ==(( char* )buf )[ i ])&&
                       ( LF !=(( char* )buf )[ i ] ))
                    {
                        putchar( LF );
                    }
                }
            }
            break;
           }

#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}


/* doSaveFile
 *   Try out MOS function 02h "mos_save".
 *   User to enter filename
 *   Routine will mallocate some memory and fill it with test data
 *   then call mos_save to copy the test data to the file on SD-card
 *    return a meaningful error if the file cannot be saved
*/
static void doSaveFile( void )
{
    char *fn;
    void * buf;
    MOS_ERRNO err;
    unsigned int len;

#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif

    ( void )printf( "\r\n\r\nRunning file save test.\r\n" );
    
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        // create text file content in buf
        portENTER_CRITICAL( );
        {
            ( void )sprintf( buf, 
                         "This text is auto-generated by the mos_save test function.\r\n"
                         "Hello from FreeRTOS / MOS for Agon!\r\n" );
            len = strlen( buf );
        }
        portEXIT_CRITICAL( );

        ( void )printf( "Enter a filename (without any path) eg. \"test.dat\".\r\n"
                        "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( 
                  "\r\nSaving %d bytes to file '%s' on SD-card\r\n", len, fn );

        err = mos_save( fn, buf,( len + 1 ));
        configASSERT(( MOS_ERR_OK <= err )&&( MOS_ERR_END > err ));
        switch( err )
        {
            default :
            {
                ( void )printf( "\r\nmos_save returned error %d : %s\r\n",
                                    err,
                                    mos_errnos[ err ].errstr );
            }
            break;

               case MOS_ERR_OK :
            {
                ( void )printf( "\r\nSuccess\r\n" );
            }
            break;
           }

#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}

/* doChangeDirectory
 *   Try out MOS function 03h "mos_cd".
 *   User to enter pathname
 *   Routine will call mos_cd to set the CWD on SD-card
 *    return a meaningful error if the directory cannot be located
*/
static void doChangeDirectory( void )
{
    char *pn;
    MOS_ERRNO err;

    ( void )printf( "\r\n\r\nRunning change directory test.\r\n" );
    
    ( void )printf( "Enter a pathname eg. \"/a/b\".\r\n"
                    "Press <enter> to finish.\r\n" );
    pn = getName( );

    ( void )printf( "\r\nChanging directory to '%s' on SD-card\r\n", pn );

    err = mos_cd( pn );
    configASSERT(( MOS_ERR_OK <= err )&&( MOS_ERR_END > err ));
    switch( err )
    {
        default :
        {
            ( void )printf( "\r\nmos_cd returned error %d : %s\r\n",
                                err,
                                mos_errnos[ err ].errstr );
        }
        break;

        case MOS_ERR_OK :
           {
            ( void )printf( "\r\nSuccess\r\n" );
        }
        break;
    }
}


/* doDeleteFile
 *   Try out MOS function 05h "mos_del".
 *   User to enter a file or path name
 *   Routine will call mos_del to delete the file or the directory from the SD-card
 *    return a meaningful error if the file or path name cannot be located
*/
static void doDeleteFile( void )
{
    char *pn;
    MOS_ERRNO err;

    ( void )printf( "\r\n\r\nRunning delete file test.\r\n" );
    
    ( void )printf( "Enter a pathname eg. \"/a/b\".\r\n"
                    "Press <enter> to finish.\r\n" );
    pn = getName( );

    ( void )printf( "\r\Deleting file '%s' from SD-card\r\n", pn );

    err = mos_del( pn );
    configASSERT(( MOS_ERR_OK <= err )&&( MOS_ERR_END > err ));
    switch( err )
    {
        default :
        {
            ( void )printf( "\r\nmos_del returned error %d : %s\r\n",
                                err,
                                mos_errnos[ err ].errstr );
        }
        break;

        case MOS_ERR_OK :
           {
            ( void )printf( "\r\nSuccess\r\n" );
        }
        break;
    }
}


/* doRenameFile
 *   Try out MOS function 06h "mos_ren".
 *   User to enter a current filename followed by a new filename
 *   Routine will call mos_ren to change the file on the SD-card
 *    return a meaningful error if the rename cannot be done
*/
static void doRenameFile( void )
{
    char orgname[ configMAX_TASK_NAME_LEN ];
    char *pn;
    MOS_ERRNO err;

    ( void )printf( "\r\n\r\nRunning rename file test.\r\n" );
    
    ( void )printf( "Enter the orignal filename eg. \"a.txt\".\r\n"
                    "Press <enter> to finish.\r\n" );
    pn = getName( );
    portENTER_CRITICAL( );
    {
        /* remember to guard Zilog library calls from task re-entrancy */
        ( void )memcpy( orgname, pn, configMAX_TASK_NAME_LEN );
    }
    portEXIT_CRITICAL( );

    ( void )printf( "\r\nEnter the new filename eg. \"b.txt\".\r\n"
                    "Press <enter> to finish.\r\n" );
    pn = getName( );

    ( void )printf( "\r\Renaming file '%s' to '%s' on SD-card\r\n", 
                    orgname, pn );

    err = mos_ren( orgname, pn );
    configASSERT(( MOS_ERR_OK <= err )&&( MOS_ERR_END > err ));
    switch( err )
    {
        default :
        {
            ( void )printf( "\r\nmos_ren returned error %d : %s\r\n",
                                err,
                                mos_errnos[ err ].errstr );
        }
        break;

        case MOS_ERR_OK :
           {
            ( void )printf( "\r\nSuccess\r\n" );
        }
        break;
    }
}


/* doMakeDirectory
 *   Try out MOS function 07h "mos_mkdir".
 *   User to enter pathname
 *   Routine will call mos_mkdir to create the directory on SD-card
 *    return a meaningful error if the directory cannot be made
*/
static void doMakeDirectory( void )
{
    char *pn;
    MOS_ERRNO err;

    ( void )printf( "\r\n\r\nRunning make directory test.\r\n" );
    
    ( void )printf( "Enter a pathname to make eg. \"/a/b\".\r\n"
                    "Press <enter> to finish.\r\n" );
    pn = getName( );

    ( void )printf( "\r\nMaking directory '%s' on SD-card\r\n", pn );

    err = mos_mkdir( pn );
    configASSERT(( MOS_ERR_OK <= err )&&( MOS_ERR_END > err ));
    switch( err )
    {
        default :
        {
            ( void )printf( "\r\nmos_mkdir returned error %d : %s\r\n",
                                err,
                                mos_errnos[ err ].errstr );
        }
        break;

        case MOS_ERR_OK :
           {
            ( void )printf( "\r\nSuccess\r\n" );
        }
        break;
    }
}


/* doCopyFile
 *   Try out MOS function 11h "mos_copy".
 *   User to enter a current filename followed by a new filename
 *   Routine will call mos_copy to copy current file to new file on the SD-card
 *    return a meaningful error if the copy cannot be done
*/
static void doCopyFile( void )
{
    char src[ configMAX_TASK_NAME_LEN ];
    char *dest;
    MOS_ERRNO err;

    ( void )printf( "\r\n\r\nRunning copy file test.\r\n" );
    
    ( void )printf( "Enter the source filename eg. \"a.txt\".\r\n"
                    "Press <enter> to finish.\r\n" );
    dest = getName( );
    portENTER_CRITICAL( );
    {
        /* remember to guard Zilog library calls from task re-entrancy */
        ( void )memcpy( src, dest, configMAX_TASK_NAME_LEN );
    }
    portEXIT_CRITICAL( );

    ( void )printf( "\r\nEnter the destination filename eg. \"b.txt\".\r\n"
                    "Press <enter> to finish.\r\n" );
    dest = getName( );

    ( void )printf( "\r\Copying file '%s' to '%s' on SD-card\r\n", src, dest );

    err = mos_copy( src, dest );
    configASSERT(( MOS_ERR_OK <= err )&&( MOS_ERR_END > err ));
    switch( err )
    {
        default :
        {
            ( void )printf( "\r\nmos_copy returned error %d : %s\r\n",
                                err,
                                mos_errnos[ err ].errstr );
        }
        break;

        case MOS_ERR_OK :
           {
            ( void )printf( "\r\nSuccess\r\n" );
        }
        break;
    }
}


/* doSysVars
 *   Try out MOS function 08h "mos_sysvars".
 *   Routine will call mos_getsysvars to retrieve system variables pointer
 *   Routine will look up certain variables (RTC, screen width) using offset and size
*/
static void doSysVars( void )
{
    MOS_SYSVARS_DESC * svard;
    
    ( void )printf( "\r\n\r\nRunning sysvars test.\r\n" );
    
    svard =(( MOS_SYSVARS_DESC * )mos_getsysvars( ));
    ( void )printf( "\r\nsysvars = %p\r\n", svard );

    /* RTC results aren't that useful; not updating by the clock */
    ( void )printf( "\r\Retrieving RTC data from sysvars\r\n" );
    ( void )printf( 
                "\r\nYear = %d : Month = %d : Day = %d" 
                " : Hour = %d : Minute = %d : Second = %d\r\n",
                svard->MOS_SYSVAR_RTC[ 0 ]+ 1980, 
                svard->MOS_SYSVAR_RTC[ 1 ], 
                svard->MOS_SYSVAR_RTC[ 2 ],
                svard->MOS_SYSVAR_RTC[ 3 ], 
                svard->MOS_SYSVAR_RTC[ 4 ], 
                svard->MOS_SYSVAR_RTC[ 5 ]);    

    ( void )printf( "Screen width = %d\r\n", svard->MOS_SYSVAR_SCR_WIDTH );
    ( void )printf( "Screen height = %d\r\n", svard->MOS_SYSVAR_SCR_HEIGHT );

}


/* doGetError
 *   Try out MOS function 0Fh "mos_getError".
 *   Routine will call mos_getError to retrieve a range of system error messages
*/
static void doGetError( void )
{
    void *buf;
    int i;
    
    ( void )printf( "\r\n\r\nRunning Get Error test.\r\n" );

#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( 128 );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ 128 ];
        buf = buff;
    }
#   endif

    if( NULL != buf )
    {
        for( i=0; MOS_ERR_END > i; i++ )
        {
            ( void )memset( buf, 0, 128 );
            mos_geterror( i, buf, 128 );
            ( void )printf( "\r\nErr : %d : %s", i, buf );
        }

        ( void )printf( "\rnSuccess\rn" );

#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}


/* doFopsC
 *   Try out MOS function 0Ah "mos_fopen"
 *                        0Bh "mos_fclose"
 *                        0Ch "mos_fgetc"
 *                        0Dh "mos_fputc"
 *                        0Eh "mos_feof"
 * 1.User to enter filename
 *   Routine will call mos_fopen to open an existing file for read on the SD-card
 *    return a meaningful error if the file cannot be located or read
 *
 * 2.User to enter filename
 *   Routine shall call mos_fopen to create a new file for writing on the SD-card
 *    return a meaningful error if the file cannot be created or written to
*/
static void doFopsC( void )
{
    char *fn;
    int fh;
    char ch;
    char *buf;
    int len;
    int i;


    /***** 1. Read test *****/
    ( void )printf( "\r\n\r\nRunning fops test 1 - read file.\r\n" );

    ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                    "Press <enter> to finish.\r\n" );
    fn = getName( );

    ( void )printf( "\r\nOpening file '%s' for reading on SD-card\r\n", fn );

    fh = mos_fopen( fn, MOS_FAM_READ );
    if( fh )
    {
        ( void )printf( "Success\r\n" );

        ( void )printf( "\r\nReading file '%s' from SD-card\r\n", fn );
        while( ! mos_feof( fh ))
        {
            ch = mos_fgetc( fh );
            putchar( ch );
        }

        ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
        ( void )mos_fclose( fh );
    }
    else
    {
        ( void )printf( "Failed to open '%s'\r\n", fn );
        /* MOS 1.04 does not return the error code back from mos_fopen,
           although mos_fopen captures the error code; nor does MOS maintain 
           a system _errno together with an api like mos_errno() to retrieve 
           it. */
    }


    /***** 2. Write test *****/
    ( void )printf( "\r\n\r\nRunning fops test 2 - write file.\r\n" );

#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        // create text file content in buf
        portENTER_CRITICAL( );
        {
            ( void )sprintf( buf, 
                         "This text is auto-generated by the mos_fops test function.\r\n"
                         "Hello from FreeRTOS / MOS for Agon!\r\n" );
            len = strlen( buf );
        }
        portEXIT_CRITICAL( );

        ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                        "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( "\r\nOpening file '%s' for writing on SD-card\r\n", fn );

        fh = mos_fopen( fn, MOS_FAM_CREATE | MOS_FAM_WRITE );
        if( fh )
        {
            ( void )printf( "Success\r\n" );

            ( void )printf( "\r\nWriting file '%s' from SD-card\r\n", fn );
            for( i=0; len > i; i++ )
            {
                mos_fputc( fh, buf[ i ]);
                putchar( buf[ i ]);
            }

            ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
            ( void )mos_fclose( fh );
        }
        else
        {
            ( void )printf( "Failed to open '%s'\r\n", fn );
        }

#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}


/* doFopsB
 *   Try out MOS function 0Ah "mos_fopen"
 *                        0Bh "mos_fclose"
 *                        1Ah "mos_fread"
 *                        1Bh "mos_fwrite"
 * 1.User to enter filename
 *   Routine will call mos_fopen to open an existing file for fread on the SD-card
 *    return a meaningful error if the file cannot be located or read
 *
 * 2.User to enter filename
 *   Routine shall call mos_fopen to create a new file for fwrite on the SD-card
 *    return a meaningful error if the file cannot be created or written to
*/
static void doFopsB( void )
{
    char *fn;
    int fh;
    char ch;
    char *buf;
    int len;
    int i;


#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        /***** 3. Read test *****/
        ( void )printf( "\r\n\r\nRunning fops test 3 - fread file.\r\n" );

        ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                    "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( "\r\nOpening file '%s' for fread on SD-card\r\n", fn );

        fh = mos_fopen( fn, MOS_FAM_READ );
        if( fh )
        {
            ( void )printf( "Success\r\n" );

            ( void )printf( "\r\nReading file '%s' from SD-card\r\n", fn );
            len = mos_fread( fh, buf, MAX_FILE_SIZE );
            for( i=0; len > i; i++ )
            {
                putchar( buf[ i ]);
            }

            ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
            ( void )mos_fclose( fh );
        }
        else
        {
            ( void )printf( "Failed to open '%s'\r\n", fn );
            /* MOS 1.04 does not return the error code back from mos_fopen,
               although mos_fopen captures the error code; nor does MOS maintain 
               a system _errno together with an api like mos_errno() to retrieve 
               it. */
        }


        /***** 4. Write test *****/
        ( void )printf( "\r\n\r\nRunning fops test 4 - fwrite file.\r\n" );
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        // create text file content in buf
        portENTER_CRITICAL( );
        {
            ( void )sprintf( buf, 
                         "Auto-generated by the mos_fops test function.\r\n"
                         "Hello from FreeRTOS / MOS for Agon!\r\n" );
            len = strlen( buf );
        }
        portEXIT_CRITICAL( );

        ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                        "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( "\r\nOpening file '%s' for writing on SD-card\r\n", fn );

        fh = mos_fopen( fn, MOS_FAM_CREATE | MOS_FAM_WRITE );
        if( fh )
        {
            ( void )printf( "Success\r\n" );

            ( void )printf( "\r\nWriting file '%s' from SD-card\r\n", fn );
            len = mos_fwrite( fh, buf, len );

            ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
            ( void )mos_fclose( fh );
        }
        else
        {
            ( void )printf( "Failed to open '%s'\r\n", fn );
        }

#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}


/* doFopsS
 *   Try out MOS function 1Ch "mos_flseek"
 *
 *   User to enter filename
 *   Routine will call mos_fopen to open an existing file on the SD-card and do an flseek
 *    return a meaningful error if the file cannot be located or seeked
*/
static void doFopsS( void )
{
    char *fn;
    int fh;
    char *buf;
    int i;
    int len;
    int err;


#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        ( void )printf( "\r\n\r\nRunning fops test 5 - flseek file.\r\n" );

        ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                    "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( "\r\nOpening file '%s' for fread on SD-card\r\n", fn );

        fh = mos_fopen( fn, MOS_FAM_READ );
        if( fh )
        {
            ( void )printf( "Success\r\n" );

            ( void )printf( "\r\nSeeking 10 bytes in to file\r\n", fn );
            err = mos_flseek( fh, 10 );
            ( void )printf( "\r\nResult = %d\r\n", err );

            ( void )printf( "\r\nReading file '%s' from SD-card\r\n", fn );
            len = mos_fread( fh, buf, MAX_FILE_SIZE );
            for( i=0; len > i; i++ )
            {
                putchar( buf[ i ]);
            }

            ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
            ( void )mos_fclose( fh );
        }
        else
        {
            ( void )printf( "Failed to open '%s'\r\n", fn );
            /* MOS 1.04 does not return the error code back from mos_fopen,
               although mos_fopen captures the error code; nor does MOS maintain 
               a system _errno together with an api like mos_errno() to retrieve 
               it. */
        }


#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}


/* doGetFil
 *   Try out MOS function 19h "mos_getfil".
 *   Routine will call mos_getfil to retrieve a FIL structure for an fopen'ed file
*/
static void doGetFil( void )
{
    MOS_FIL *fil;
    char *fn;
    int fh;
    
    ( void )printf( "\r\n\r\nRunning Get Fil test.\r\n" );

    ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                    "Press <enter> to finish.\r\n" );
    fn = getName( );

    ( void )printf( "\r\nRetrieving FIL for file '%s' on SD-card\r\n", fn );
    fh = mos_fopen( fn, MOS_FAM_OPEN_ALWAYS );
    if( fh )
    {
        fil = mos_getfil( fh );

        ( void )printf( "\r\nFIL data:\r\n" );
        ( void )printf( 
                "\r\n\tFlag = 0x%x\r\n\tErr = 0x%x\r\n\tFptr = 0x%x" 
                "\r\n\tClust = %d\r\n\tSect = %d\r\n\tDirSect = %d"
                "\r\n\tDirPtr = 0x%p\r\n",
                fil->flag, fil->err, fil->fptr,
                fil->clust, fil->sect, fil->dir_sect, fil->dir_ptr );
             // Flag = mos_fopen mode arg (MOS_FAM_OPEN_ALWAYS)

        ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
        ( void )mos_fclose( fh );
    }
    else
    {
        ( void )printf( "Failed to open '%s'\r\n", fn );
        /* MOS 1.04 does not return the error code back from mos_fopen,
           although mos_fopen captures the error code; nor does MOS maintain 
           a system _errno together with an api like mos_errno() to retrieve 
           it. */
    }
}


/* doFFopsR
 *   Try out FFS function 80h "ffs_fopen"
 *                        81h "ffs_fclose"
 *                        82h "ffs_fread"
 *                        8Eh "ffs_feof"
 *   User to enter filename
 *   Routine will call ffs_fopen to open an existing file for ffs_fread on the SD-card
 *    return a meaningful error if the file cannot be located or read
*/
static void doFFopsR( void )
{
    char *fn;
    MOS_FIL fil;
    MOS_ERRNO err;
    char *buf;
    unsigned int len;
    unsigned int cnt;
    int i;


    ( void )printf( "\r\n\r\nRunning ffsops test 1 - ffs_fread file.\r\n" );

#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                    "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( "\r\nOpening file '%s' for ffs_fread on SD-card\r\n", fn );

        err = ffs_fopen( &fil, fn, MOS_FAM_READ );
        if( MOS_ERR_OK == err )
        {
            ( void )printf( "\r\nReading file '%s' from SD-card\r\n", fn );
            for( cnt = 0;
                 ( !ffs_feof( &fil ))&&( MAX_FILE_SIZE > cnt );
                 cnt += len )
            {
                err = ffs_fread( &fil, &buf[ cnt ], MAX_FILE_SIZE - cnt, &len );
                if( MOS_ERR_OK == err )
                {
                    ( void )printf( "\r\nRead '%d' bytes\r\n", len );
            
                    for( i=cnt;( cnt + len )> i; i++ )
                    {
                        putchar( buf[ i ]);
                    }
                }
                else
                {
                    ( void )printf( 
                                "Failed to read '%s' : %d : %s\r\n", 
                                fn, err, mos_errnos[ err ].errstr );
                    break;
                }
            }

            ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
            ( void )ffs_fclose( &fil );
        }
        else
        {
            ( void )printf( 
                        "Failed to open '%s' : %d : %s\r\n", 
                        fn, err, mos_errnos[ err ].errstr );
        }


#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}


/* doFFopsW
 *   Try out FFS function 83h "ffs_fwrite"
 *   User to enter filename
 *   Routine will call ffs_fopen to open an existing file for ffs_fwrite on the SD-card
 *    return a meaningful error if the file cannot be located or written
*/
static void doFFopsW( void )
{
    char *fn;
    MOS_FIL fil;
    MOS_ERRNO err;
    char *buf;
    unsigned int len;


    ( void )printf( "\r\n\r\nRunning ffsops test 2 - ffs_fwrite file.\r\n" );

#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                    "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( "\r\nOpening file '%s' for ffs_fwrite on SD-card\r\n", fn );

        err = ffs_fopen( &fil, fn, MOS_FAM_WRITE | MOS_FAM_CREATE );
        if( MOS_ERR_OK == err )
        {
            // create text file content in buf
            portENTER_CRITICAL( );    // safeguard sprintf and strlen zilog library routines
            {
                ( void )sprintf( buf, 
                         "Auto-generated by the ffs_fops test function.\r\n"
                         "Hello from FreeRTOS / MOS for Agon!\r\n" );
                len = strlen( buf );
            }
            portEXIT_CRITICAL( );

            ( void )printf( 
                        "\r\n\tFlag = 0x%x\r\n\tErr = 0x%x\r\n\tFptr = 0x%x" 
                        "\r\n\tClust = %d\r\n\tSect = %d\r\n\tDirSect = %d"
                        "\r\n\tDirPtr = 0x%p\r\n",
                        fil.flag, fil.err, fil.fptr,
                        fil.clust, fil.sect, fil.dir_sect, fil.dir_ptr );

            ( void )printf( "Writing %d bytes\r\n", len );
            err = ffs_fwrite( &fil, buf, len, &len );
            if( MOS_ERR_OK == err )
            {
                ( void )printf( "Wrote %d bytes\r\n", len );

                ( void )printf( 
                            "\r\n\tFlag = 0x%x\r\n\tErr = 0x%x\r\n\tFptr = 0x%x" 
                            "\r\n\tClust = %d\r\n\tSect = %d\r\n\tDirSect = %d"
                            "\r\n\tDirPtr = 0x%p\r\n",
                            fil.flag, fil.err, fil.fptr,
                            fil.clust, fil.sect, fil.dir_sect, fil.dir_ptr );
            }
            else
            {
                ( void )printf( 
                            "Failed to write '%s' : %d : %s\r\n", 
                            fn, err, mos_errnos[ err ].errstr );
            }

            
            ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
            ( void )ffs_fclose( &fil );
        }
        else
        {
            ( void )printf( 
                        "Failed to open '%s' : %d : %s\r\n", 
                        fn, err, mos_errnos[ err ].errstr );
        }


#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
}


/* doFFopsS
 *   Try out FFS function 80h "ffs_fopen"
 *                        81h "ffs_fclose"
 *                        82h "ffs_fread"
 *                        84h "ffs_fseek"
 *   User to enter filename
 *   Routine will call ffs_fopen to open an existing file for ffs_lseek on the SD-card
 *    return a meaningful error if the file cannot be located or seeked
*/
static void doFFopsS( void )
{
    char *fn;
    MOS_FIL fil;
    MOS_ERRNO err;
    char *buf;
    unsigned int len;
    int i;


    ( void )printf( "\r\n\r\nRunning ffsops test 3 - ffs_flseek file.\r\n" );

#   if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
    {
        /* remember when we call Zilog library functions 
           to safeguard from concurrent access */
        portENTER_CRITICAL( );
        {
            buf = malloc( MAX_FILE_SIZE );     
        }
        portEXIT_CRITICAL( );
    }
#   else
    {
        static unsigned char buff[ MAX_FILE_SIZE ];
        buf = buff;
    }
#   endif
    if( NULL != buf )
    {
        ( void )memset( buf, 0, MAX_FILE_SIZE );   // fill buffer with 0s

        ( void )printf( "Enter a filename eg. \"test.dat\".\r\n"
                    "Press <enter> to finish.\r\n" );
        fn = getName( );

        ( void )printf( "\r\nOpening file '%s' for ffs_flseek on SD-card\r\n", fn );

        err = ffs_fopen( &fil, fn, MOS_FAM_READ );
        if( MOS_ERR_OK == err )
        {
            ( void )printf( "\r\nSeeking file '%s' to offset 20\r\n", fn );
            err = ffs_flseek( &fil, 20 );
            if( MOS_ERR_OK == err )
            {
                err = ffs_fread( &fil, buf, MAX_FILE_SIZE, &len );
                if( MOS_ERR_OK == err )
                {
                    ( void )printf( "\r\nRead '%d' bytes\r\n", len );
            
                    for( i=0; len > i; i++ )
                    {
                        putchar( buf[ i ]);
                    }
                }
                else
                {
                    ( void )printf( 
                                "Failed to read '%s' : %d : %s\r\n", 
                                fn, err, mos_errnos[ err ].errstr );
                }
            }
            else
            {
                ( void )printf( 
                            "Failed to seek '%s' : %d : %s\r\n", 
                            fn, err, mos_errnos[ err ].errstr );
            }

            ( void )printf( "\r\nClosing file '%s' on SD-card\r\n", fn );
            ( void )ffs_fclose( &fil );
        }
        else
        {
            ( void )printf( 
                        "Failed to open '%s' : %d : %s\r\n", 
                        fn, err, mos_errnos[ err ].errstr );
        }


#       if( 1 == configSUPPORT_DYNAMIC_ALLOCATION )
        {        
            portENTER_CRITICAL( );
            {
                free( buf );  // return mallocated memory to the heap
            }
            portEXIT_CRITICAL( );
        }
#       endif
    }
    else
    {
        ( void )printf( "\r\nFailed to allocate heap memory\r\n" );
    }
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

    for( ;; )    
    {
        portYIELD( );
    }
    
    ( void )pvParameters;
}


/* vApplicationIdleHook
 *   Runs in context of the idle task.
 *   DO NOT call a BLOCKING function from within the IDLE task;
 *   IDLE must always be in either the READY or the RUN state, and no other. 
 *   Typically used to do a heartbeat LED, or perform heap garbage collection.
*/
void vApplicationIdleHook( void )
{
    //Machen mit ein blinken light would be excellent

    idlecnt++;
}
