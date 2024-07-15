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
 * devuart.c for Agon Light
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The definitions in this file support the DEV API (UART low-level)
 * for Agon Light (and comptaibles)
 * Created 20.Jun.2024 by Julian Rose for Agon Light port
 *
 * Refer to Zilog PS015317 "eZ80F92/F93 Product Specification" UART section
 * And to the Zilog ZDSII_eZ80Acclaim!_5.3.5\src\uart\ for INTERRUPT mode, 
 * on which this is in-part derived
 * (The MOS UART code is also derived from the Zilog code, POLL mode)
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include <stdio.h>

#include <eZ80F92.h>
#include <uartdefs.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "devConfig.h"
#include "devapi.h"
#include "devapil.h"
#include "mosapi.h"


#if( 1 == configUSE_DRV_UART )


/*---- Notes ----------------------------------------------------------------*/
/* NOTE 1: DTE - MODEM wiring
 *
 * Refer to https://en.wikipedia.org/wiki/RS-232#3-wire_and_5-wire_RS-232 for
 *   DTE<->DCE "modem" wiring
 *     DTE is Data Terminal Equipment, "computer" (Agon), or target device
 *     DCE is Data Communication Equipment, "modem" or "wifi router"
 * And http://www.nullmodem.com/NullModem.htm for DTE<->DTE "NULL modem" wiring.
 *     Also https://en.wikipedia.org/wiki/Null_modem#Wiring_diagrams
 *
 * DEV API can be opened any one of five modes, for a combination of:
 *    Non-blocking / Blocking with Straight-through / Cross-Over wiring
 *
 * Bitor DEV_MODE_UNBUFFERED for Blocking mode, or DEV_MODE_BUFFERED for 
 * Non-blocking mode, with one of the Straight-through / Cross-over modes 
 * as follows:
 *
 *    DEV_MODE_UART_NO_MODEM
 *      Use:
 *        'No Modem' direct DTE<->DTE signalling; low capacitance (short wires)
 *      Wiring: 
 *        DTE1.Tx  --> DTE2.Rx    cross-over
 *        DTE1.Rx  <-- DTE2.Tx
 *      Protocol:
 *        UART1_MCTL is fixed at 0x00, with /RTS and /DTR de-asserted (high)
 *        Transmission and Reception are not controlled by hardware status.
 *        There should be no UART_IIR_MODEMSTAT events.
 *
 *    DEV_MODE_UART_HALF_MODEM
 *      Use:
 *        DTE<->DCE half-duplex signalling
 *      Wiring: 
 *        DTE1.Tx  --> DCE1.Tx
 *        DTE1.Rx  <-- DCE1.Rx
 *        DTE1.RTS --> DCE1.RTS   straight-through
 *        DTE1.CTS <-- DCE1.CTS
 *      Protocol:
 *        Transmission and Reception are mutually exclusive (half-duplex). 
 *        Reception is possible while /RTS is de-asserted (high).
 *        /RTS (Request To Send) shall be asserted (low) when the DTE is ready
 *          to TRANSMIT.
 *        Prior to transmission, /RTS must be asserted (low) and /CTS assertion 
 *          tested (low). 
 *        If during transmission /CTS becomes de-asserted (high), then 
 *          transmission must pause, and LCTL.SB set to indicate a break. 
 *          Once /CTS is re-asserted (low) SB shall be cleared and transmission 
 *          resumed.
 *          With ( 1 == configUSE_PREEMPTION ) configured in FreeRTOSConfig.h, 
 *          we could set a timeout on /CTS re-assertion, on which we abandon 
 *          the transmission.
 *        On transmission completion or abandon, /RTS must be de-asserted (high)
 *
 *    DEV_MODE_UART_HALF_NULL_MODEM 
 *      Use:
 *        DTE<->DTE half-duplex signalling
 *      Wiring: 
 *        DTE1.Tx  --> DTE2.Rx 
 *        DTE1.Rx  <-- DTE2.Tx
 *        DTE1.RTS --> DTE2.CTS   cross-over
 *        DTE1.CTS <-- DTE2.RTS
 *      Protocol:
 *        Transmission and Reception are mutually exclusive (half-duplex). 
 *        With crossover wiring /RTS shall be asserted (low) when the DTE is 
 *          Ready to RECEIVE. Think of it as Remote To Send..
 *        /RTS (Remote To Send) shall be de-asserted (high) when the DTE is 
 *          ready to TRANSMIT.
 *        Prior to transmission, /RTS must be de-asserted (high) and /CTS 
 *          assertion tested (low). 
 *        If during transmission /CTS becomes de-asserted (high), then 
 *          transmission must pause, and LCTL.SB set to indicate a break. 
 *          Once /CTS is re-asserted (low) SB shall be cleared and transmission 
 *          resumed.
 *        On transmission completion or abandon, /RTS must be asserted (low).
 *
 *    DEV_MODE_UART_FULL_MODEM 
 *      Use:
 *        DTE<->DCE full-duplex signalling
 *      Wiring: 
 *        DTE1.Tx  --> DCE1.Tx
 *        DTE1.Rx  <-- DCE1.Rx
 *        DTE1.RTS --> DCE1.RTS   straight-through
 *        DTE1.CTS <-- DCE1.CTS
 *        DTE1.DTR --> DCE1.DTR
 *        DTE1.DSR <-- DCE1.DSR
 *        DTE1.DCD <-- DCE1.DCD
 *        DTE1.RI  <-- DCE1.RI
 *      Protocol:
 *        Transmission and Reception may be simultaneous (full-duplex). 
 *        /DTR shall be asserted (low) when the DTE is opened, and de-asserted
 *          when the DTE is closed
 *        Reception is possible with /RTS asserted (low) or de-asserted (high).
 *        /RTS (Request To Send) shall be asserted (low) when the DTE is ready
 *          to TRANSMIT.
 *        Prior to transmission, /RTS must be asserted (low) and /CTS assertion 
 *          tested (low). 
 *        If during transmission /CTS becomes de-asserted (high), then 
 *          transmission must pause, and LCTL.SB set to indicate a break. 
 *          Once /CTS is re-asserted (low) SB shall be cleared and transmission 
 *            resumed.
 *          With ( 1 == configUSE_PREEMPTION ) configured in FreeRTOSConfig.h, 
 *          we could set a timeout on /CTS re-assertion, on which we abandon 
 *          the transmission.
 *        If during transmission either /DCD or /DSR become de-asserted (high), 
 *          then transmission must be abandoned.
 *        On transmission completion or abandon, /RTS must be de-asserted (high).
 *
 *    DEV_MODE_UART_FULL_NULL_MODEM
 *      Use:
 *        DTE<->DTE full-duplex signalling
 *      Wiring: 
 *        DTE1.Tx  --> DTE2.Rx
 *        DTE1.Rx  <-- DTE2.Tx
 *        DTE1.RTS --> DTE2.CTS   cross-over
 *        DTE1.CTS <-- DTE2.RTS
 *        DTE1.DTR --> DTE2.DCD
 *                 --> DTE2.DSR
 *        DTE1.DSR <-- DTE2.DTR
 *        DTE1.DCD <--
 *      Protocol:
 *        Transmission and Reception may be simultaneous (full-duplex). 
 *        /DTR shall be asserted (low) when the DEV UART is opened, and de-
 *          asserted (high) when DEV UART is closed
 *        With crossover wiring /RTS shall be asserted (low) when the DTE is 
 *          Ready to RECEIVE. Think of it as Remote To Send..
 *        Prior to transmission, /RTS must be de-asserted (high) and /CTS 
 *          assertion tested (low). 
 *        If during transmission /CTS becomes de-asserted (high), then 
 *          transmission must pause, and LCTL.SB set to indicate a break. 
 *          Once /CTS is re-asserted (low) SB shall be cleared and transmission 
 *          resumed.
 *          With ( 1 == configUSE_PREEMPTION ) configured in FreeRTOSConfig.h, 
 *          we could set a timeout on /CTS re-assertion, on which we abandon 
 *          the transmission.
 *        If during transmission either /DCD or /DSR become de-asserted (high), 
 *          then transmission must be abandoned.
 *        On transmission completion or abandon, /RTS must be asserted (low)
 *
 *-----------------------------------------------------------------------------
 * NOTE 2: eZ80 IIR handling
 *
 * If you mask out IER then the corresponding bits in IIR will be cleared
 *     at the same time. So, sadly, masking is not possible:
 *         ier_mask = UART1_IER;
 *         UART1_IER = 0x0;  // mask off IER until IIR is processed
 *     And if you exit an ISR with bits still set in IIR, then the ISR will
 *     immediately re-enter. So, it is not possible to notify a highest-priority
 *     task to run as an Interrupt Handler. Instead, the handling must be done 
 *     within the ISR context, which means other interrupts are disabled for the
 *     duration of executing uartisr_1. Sigh.* /
 *
 *
 * #if( 1 == configUSE_PREEMPTION )
 *   static TaskHandle_t UartIsrHandlerTaskHandle = NULL;
 *   static void UartIsrHandlerTask( void * params );
 *   static unsigned int UartIsrHandlerTaskCnt = 0;
 * #endif
 * static unsigned char ier_mask = 0;
 *
 * #if( 1 == configUSE_PREEMPTION )
 * static POSIX_ERRNO createUartIsrHandlerTask( void )
 * {
 *     POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
 *     BaseType_t r;
 * 
 *     if( NULL == UartIsrHandlerTaskHandle )
 *     {
 *         r = xTaskCreate( 
 *                 UartIsrHandlerTask, 
 *                 "UART ISR Handler", 
 *                 configMINIMAL_STACK_SIZE, 
 *                 NULL,                      // no params
 *                 configMAX_PRIORITIES,      // highest possible priority
 *                 &UartIsrHandlerTaskHandle );
 *         if( pdPASS != r )
 *         {
 * #           if defined( _DEBUG )
 *             {
 *                 ( void )printf( "Failed to allocate UartIsrHandlerTask: %d\r\n", r );
 *             }
 * #           endif
 * 
 *             ret = POSIX_ERRNO_ENSRCH;
 *         }
 *         else
 *         {
 * #           if defined( _DEBUG )
 *             {
 *                 ( void )printf( "Created UartIsrHandlerTask\r\n" );
 *             }
 * #           endif
 *         }
 *     }
 * 
 *     return( ret );
 * }
 * #endif
 * 
 * #if( 1 == configUSE_PREEMPTION )
 * /* UartIsrHandlerTask
 *      High-priority Uart interrupt handler task.
 *      Will run immediately on exit of _uartisr_0;
 *        iff the current task is not blocking inside MOS * /
 * static void UartIsrHandlerTask( void * params )
 * {
 *     unsigned long notification;
 * 
 *     for( ;; )
 *     {
 *         notification = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
 *         if( notification )
 *         {
 *             uartisr_1( );
 * 
 *             UartIsrHandlerTaskCnt++;
 * 
 *             UART1_IER = ier_mask;   // re-enable masked interrupts
 *         }
 *     }
 * }
 * #endif
 *
 *
 * uartisr_0 - address written into the interrupt vector table
 *      Standard ISR reti epilogue * /
 * static void uartisr_0( void )
 * {
 *     BaseType_t mosHigherPriorityTaskWoken;
 *     
 * #   if defined( _DEBUG )
 *         _putchf( 'U' );
 * #   endif
 * 
 * 
 * #   if( 1 == configUSE_PREEMPTION )
 *     {
 *         mosHigherPriorityTaskWoken = pdFALSE;
 * 
 *         /* run in context of high-priority interrupt handler UartIsrHandlerTask.
 *              Other hardware interrupts may be raised concurrently.
 *              If the uart isr handling isn't served quickly enough, you can
 *               - either increase the uart handler task priority, 
 *               - or change ( 1 == configUSE_PREEMPTION ) to 0 in order to call 
 *                 uartisr_1 directly from uartisr_0 * /
 *         _putchf( 'V' );
 *         vTaskNotifyGiveFromISR( UartIsrHandlerTaskHandle, &mosHigherPriorityTaskWoken );
 * 
 *         if( pdTRUE == mosHigherPriorityTaskWoken ) _putchf( 'Y' );
 *     
 *     }
 * #   else
 *     {
 *         /* run in context of this ISR.
 *              Other hardware interrupts are blocked out for the duration. * /
 *         uartisr_1( );
 *         UART1_IER = ier_mask;   // re-enable masked interrupts
 *     }
 * #   endif
 * 
 *     _putchf( 'W' );
 * 
 *     /* If mosHigherPriorityTaskWoken is now set to pdTRUE then a context 
 *        switch should be performed to ensure the interrupt returns directly 
 *        to the highest priority task. 
 *        This must be the last function before reti. Refer to port.c::timer_isr * /
 *     vPortYieldFromISR( mosHigherPriorityTaskWoken );
 *     _putchf( 'Z' );
 * 
 *     asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
 *     asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
 *     asm( "               ;   __default_mi_handler" );
 *     asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );
 *     asm( "\t reti.l      ; need reti.L as per UM007715 table 25 for IM 2 ADL=1 MADL=1");
 * 
 *     /* following compiler-inserted epilogue is not executed * /
 * }
 *
 *-----------------------------------------------------------------------------
 * NOTE 3: DEV UART initial State design
 *
 *   In UART_STATE_INITIAL we are not connected to the uart.
 *     On all interrupts:
 *       clear them down; 
 *
 *   In UART_STATE_READY we are connected to the uart.
 *     On receive interrupts: 
 *       clear them down; 
 *       copy FIFO-received bytes from RBR into rxPrivBuf[ rxPos++ ]; 
 *       if event queue last != rx event
 *         record rx ready event in event queue.
 *     On rx error interrupts
 *       clear them down; 
 *       reset read FIFO
 *       if event queue last != rx error event
 *         record rx error in event queue;
 *     On tx interrupts: 
 *       clear them down;
 *       record spurious tx in event queue.
 *     On tx error interrupts:
 *       clear them down; 
 *       reset write FIFO.
 *
 *   In UART_STATE_READ an rx Transaction is running
 *     On rx interrupts: 
 *       clear them down; 
 *       if rxPos < rxLen 
 *         copy bytes from RHR to rxPrivBuf[rxPos++];
 *       if rxPos == rxLen 
 *         if DEV_MODE_UNBUFFERED
 *           give uartSemaphore;
 *         else 
 *           if event queue last != rx ready event
 *             record rx ready event in event queue;
 *           else 
 *             record rx overflow event in event queue.;
 *         set uartState = UART_STATE_READY.
 *     On rx error interrupts
 *       clear them down; 
 *       reset read FIFO;
 *       if event queue last != erroneous rx event
 *         record erroneous rx in event queue;
 *       if DEV_MODE_UNBUFFERED
 *         give uartSemaphore;
 *       set uartState = UART_STATE_READY.
 *     On tx interrupts: 
 *       clear them down; 
 *       reset write FIFO;
 *       if event queue last != spurious tx event
 *         record spurious tx event in event queue.
 *     On tx error interrupts
 *       clear them down; 
 *       reset write FIFO.
 *
 *   In UART_STATE_WRITE a tx Transaction is running
 *     On tx interrupts: 
 *       clear them down; 
 *       if txPos < txLen 
 *         copy bytes from txPrivBuf[txPos++] to THR;
 *       if txPos == txLen 
 *         if DEV_MODE_UNBUFFERED
 *           give uartSemaphore;
 *           record tx done event in event queue;
 *         set uartState = UART_STATE_READY.
 *     On tx error interrupts
 *       clear them down; 
 *       reset write FIFO;
 *       if event queue last != erroneous tx event
 *         record erroneous tx in event queue;
 *       if DEV_MODE_UNBUFFERED
 *         give uartSemaphore;
 *       set uartState = UART_STATE_READY.
 *     On receive interrupts: 
 *       clear them down; 
 *       copy FIFO-received bytes from RBR into rxPrivBuf[ rxPos++ ]; 
 *       if event queue last != rx event
 *         record rx ready event in event queue.
 *     On rx error interrupts
 *       clear them down; 
 *       reset read FIFO;
 *       if event queue last != erroneous rx event
 *         record erroneous rx in event queue;
*/


/*---- Constants ------------------------------------------------------------*/
#define THR_FIFO_MAX     ( 16 )
#define RHR_FIFO_MAX     ( 16 )

#define MINOR_NUM        ( UART_1_TXD - PIN_NUM_START )


#define CREATE_UART_TRANSACTION( _t, _tMode, _buf, _tSz, _bufSz, _bufResult )\
    ( _t ).mode =( _tMode ), \
    ( _t ).buf =( _buf ), \
    ( _t ).len =( _tSz ), \
    ( _t ).bufTRx =( _bufSz ), \
    ( _t ).bufRes =( _bufResult ), \
    ( _t ).lenTRx =0    /* last in a comma-sequence is the return value */


#define CLEAR_UART_TRANSACTION( _t ) \
    ( _t ).mode = 0, \
    ( _t ).buf = NULL, \
    ( _t ).len = 0, \
    ( _t ).bufTRx = NULL, \
    ( _t ).bufRes = NULL, \
    ( _t ).lenTRx = 0   /* last in a comma-sequence is the return value */


#if defined( _DEBUG )
  // printf is okay when run in context of a calling task
# define PRINT_UART_TRANSACTION( _t )\
    ( void )printf( "%s: mode=0x%x; buf=0x%p; len=%d; bufTRx=%p; bufRes=0x%p; lenTRX=%d\r\n", \
        ( &( _t )== &txTransact )? "txTransact" : "rxTransact", \
        ( _t ).mode, \
        ( _t ).buf, \
        ( _t ).len, \
        ( _t ).bufTRx, \
        ( _t ).bufRes, \
        ( _t ).lenTRx );
#endif


static unsigned short int const brg[ NUM_UART_BAUD ]=
{           // Constants for BRG divisor computed at compile time
    0x1E00, // UART_BAUD_150
    0x0F00, // UART_BAUD_300
    0x03C0, // UART_BAUD_1200
    0x01E0, // UART_BAUD_2400
    0x00F0, // UART_BAUD_4800
    0x0078, // UART_BAUD_9600
    0x0064, // UART_BAUD_11520
    0x0050, // UART_BAUD_14400
    0x003C, // UART_BAUD_19200
    0x0028, // UART_BAUD_28800
    0x001E, // UART_BAUD_38400
    0x0014, // UART_BAUD_57600
    0x000A, // UART_BAUD_115200
    0x0008, // UART_BAUD_144000
    0x0006, // UART_BAUD_192000
    0x0004, // UART_BAUD_288000
    0x0003, // UART_BAUD_384000
    0x0002, // UART_BAUD_576000
    0x0001  // UART_BAUD_1152000
};


/*----- Enum Type Definitions -----------------------------------------------*/
typedef enum _uart_fifo_trigger_level
{                        // values as per Zilog PS015317 table 59
    UART_FIFO_TRIGLVL_1   = 0x00,
    UART_FIFO_TRIGLVL_4   = 0x01,
    UART_FIFO_TRIGLVL_8   = 0x02,
    UART_FIFO_TRIGLVL_14  = 0x03

} UART_FIFO_TRIGLVL;


typedef enum _uart_state
{                        // UART connection and transaction states
    UART_STATE_INITIAL =( 0<<0 ),
    UART_STATE_READY   =( 1<<0 ),
    UART_STATE_READ    =( 1<<1 ),
    UART_STATE_WRITE   =( 1<<2 )

} UART_STATE;


typedef enum _uart_fifo_operation
{
    UART_FIFO_DISABLE = 0,
    UART_FIFO_ENABLE

} UART_FIFO_OPERATION;


/*----- Type Definitions ----------------------------------------------------*/
/* We use a common buffer and transaction definition for both Rx and Tx.
     However, Tx is operated as a linear buffer, with at most one active 
     transaction (governated by UART_STATE). Whereas Rx is operated as a 
     circular buffer, because we can receive bytes from the (ungovernated) 
     far end-connection at any time; these are buffered while waiting for 
     our application to call uart_read. */
#if( configDRV_UART_BUFFER_SZ < 16 )
#   error "configDRV_UART_BUFFER_SZ too small"
#elif( configDRV_UART_BUFFER_SZ > 1024 )
#   error "configDRV_UART_BUFFER_SZ too large"
#endif


#define RX_CIRCULAR_BUFFER_NUM_OCCUPIED( r )        \
        { unsigned char const i = rxPrivBuf.Index;  \
          unsigned char const o = rxPrivBuf.Outdex; \
                                                    \
          ( r )=( i < o )                           \
            ?(( configDRV_UART_BUFFER_SZ - o )+ i ) \
            :( i - o );                             \
        }

#define RX_CIRCULAR_BUFFER_NUM_FREE( r ) \
        { unsigned char const i = rxPrivBuf.Index;  \
          unsigned char const o = rxPrivBuf.Outdex; \
                                                    \
          ( r )=( i < o )                           \
            ?( o - i )                              \
            :(( configDRV_UART_BUFFER_SZ - i )+ o );\
        }


typedef struct _uart_buffer
{
    unsigned short Index;                           // next input position
    unsigned short Outdex;                          // next output position
    unsigned char  _[ configDRV_UART_BUFFER_SZ ];   // storage

} UART_BUFFER;


typedef struct _uart_transaction
{
    unsigned int mode;        // DEV_MODE_BUFFERED or DEV_MODE_UNBUFFERED  IN
    unsigned char *buf;       // Dest or Src buffer                    IN/OUT
    size_t len;               // number of bytes to     read/write         IN
    size_t lenTRx;            // local tracker of bytes transceived      TEMP
    size_t *bufTRx;           // destination buffer for lenTRx            OUT
    POSIX_ERRNO *bufRes;      // destination buffer for result            OUT
    
} UART_TRANSACTION;


/*---- Global Variables -----------------------------------------------------*/
static SemaphoreHandle_t uartRxSemaphore = NULL; // for non-buffered blocking-mode user task
static SemaphoreHandle_t uartTxSemaphore = NULL; // for non-buffered blocking-mode user task

static UART_STATE uartState = UART_STATE_INITIAL;

static UART_BUFFER txPrivBuf ={{ 0 }, 0, 0 };   // a linear buffer, tx from application
static UART_BUFFER rxPrivBuf ={{ 0 }, 0, 0 };   // a circular buffer, rx as it arrives

static UART_TRANSACTION txTransact ={ 0, NULL, 0, 0, NULL, NULL };
static UART_TRANSACTION rxTransact ={ 0, NULL, 0, 0, NULL, NULL };

static void( *prevUartISR )( void )= NULL;

static unsigned short uartRxEvent = 0;
static unsigned short uartTxEvent = 0;

extern BaseType_t mosHigherPriorityTaskWoken;  // set in ISR to context-switch


/*----- Private init functions ----------------------------------------------*/
static POSIX_ERRNO initialiseUartSemaphore( SemaphoreHandle_t * const s )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    if( NULL == *s )
    {
        // one-time creation
        *s = xSemaphoreCreateBinary( );
    }

    if( NULL == *s )
    {
        // critical error, failed to create uart semaphore
#       if defined( _DEBUG )
        {
            ( void )printf( "%s : %d : Failed to allocate UART semaphore\r\n", 
                            "devuart", __LINE__ );
        }
#       endif
        ret = POSIX_ERRNO_ENOLOCKS;
    }
    else
    {
        /* We may call uart_dev_open successive times; ensure the sempahore is 
           in the blocking state, after any previous use. 
           For a binary semaphore uxSemaphoreGetCount returns 1 if it is 
           available, and 0 if it is not available. */
        UBaseType_t cnt = uxSemaphoreGetCount( *s );
        if( cnt )
        {
            xSemaphoreTake( *s, 0 );
        }
    }

    return( ret );
}


/*------ Private Modem Functions --------------------------------------------*/
/* assertRTSandWaitCTS
     For straight-through (Modem) wiring, we assert /RTS (low) to signal 
     "Request To Send" prior to transmission. For cross-over (NULL modem) 
     wiring, we de-assert /RTS (high) to signal "Remote To Send" not clear
     to send. 
*/
static POSIX_ERRNO assertRTSandWaitCTS( void )
{
    TickType_t const ms100 = configTICK_RATE_HZ / 10;
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    unsigned char msr;

    if( 0 ==( DEV_MODE_UART_NO_MODEM & pinmode[ MINOR_NUM ]))
    {
        if(( DEV_MODE_UART_HALF_MODEM & pinmode[ MINOR_NUM ])||
           ( DEV_MODE_UART_FULL_MODEM & pinmode[ MINOR_NUM ]))
        {
            UART1_MCTL |= UART_MCTL_RTS;  // straight-through wiring, assert /RTS
        }
        else
        if(( DEV_MODE_UART_HALF_NULL_MODEM & pinmode[ MINOR_NUM ])||
           ( DEV_MODE_UART_FULL_NULL_MODEM & pinmode[ MINOR_NUM ]))
        {                                 // crossover wiring, de-assert /RTS
            UART1_MCTL &=( unsigned char )( 0xff - UART_MCTL_RTS );
        }

        msr = UART1_MSR;
        if( 0 ==( UART_MSR_CTS & msr ))
        {
            vTaskDelay( ms100 );       // wait 100mS for /CTS to be asserted
        }
        msr = UART1_MSR;
        if( 0 ==( UART_MSR_CTS & msr ))
        {
            ret = POSIX_ERRNO_EPROTO;  // no /CTS from the remote end
        }
    }

    return( ret );
}


/* clearRTS
     On completion or abandonment of transmission, for straight-through (Modem) 
       wiring, we de-assert /RTS (low) to clear "Request To Send". 
       For cross-over (NULL modem) wiring, we assert /RTS (high) to signal 
       "Remote To Send" clear to send. 
*/
static void clearRTS( void )
{
    if(( DEV_MODE_UART_HALF_MODEM & pinmode[ MINOR_NUM ])||
       ( DEV_MODE_UART_FULL_MODEM & pinmode[ MINOR_NUM ]))
    {                               // straight-through wiring, de-assert /RTS
        UART1_MCTL &=( unsigned char )( 0xff - UART_MCTL_RTS );
    }
    else
    if(( DEV_MODE_UART_HALF_NULL_MODEM & pinmode[ MINOR_NUM ])||
       ( DEV_MODE_UART_FULL_NULL_MODEM & pinmode[ MINOR_NUM ]))
    {                               // crossover wiring, assert /RTS
        UART1_MCTL |=( unsigned char )UART_MCTL_RTS;
    }
}


/*------  Private UART Setup functions --------------------------------------*/
/* uartResetFIFO
     Initialise UART FIFO (FCNTL register).
     Parameters: 
       mask is a bitor: UART_FCTL_CLRTxF to reset the Tx FIFO bitor
                        UART_FCTL_CLRRxF to reset the Rx FIFO.
       op is either UART_FIFO_DISABLE or UART_FIFO_ENABLE the fifos
     While this appears to initialise fine, refer to Zilog UP004909 Errata,
     in case of Issue 7 - continuous Rx interrupts */
static void uartResetFIFO( 
                unsigned char const mask, 
                UART_FIFO_OPERATION const op )
{
    /* Flush UART1 FIFOs and leave FIFOs enabled 
       Errata UP004909 Issue 7: looks like we should only clear the FIFOs by
       setting the UART_FCTL_CLRTxF or UART_FCTL_CLRRxF bits while 
       UART_FCTL_FIFOEN is cleared */
    UART1_FCTL = 0x0;
    UART1_FCTL =( unsigned char )( mask );             // command Flush tx and/or rx hardware FIFO.
    UART1_FCTL =( unsigned char )( UART_FCTL_FIFOEN ); // execute Flush command and enable the FIFO

    /* Initialise Rx Fifo Trigger Level, leaving FIFOs enabled.
         Maximise use of the FIFO hardware, to maximise task concurrency.
           'Receiver Data Ready' triggered when RxFIFO depth >= this value.
           'Receiver Timeout' triggered when RxFIFO depth < this
            value AND 4 consecutive byte times pass with no further data.
         Given Errata UP004909 Issue 7: it follows we ought to set the FIFO
           trigger level while the FIFO is disabled. */
    UART1_FCTL =( unsigned char )  0x0;                          // disable the FIFO
    UART1_FCTL =( unsigned char )( UART_FIFO_TRIGLVL_14 << 6 );  // command the required rx trigger level
    UART1_FCTL =( unsigned char )( UART_FCTL_FIFOEN );           // execute the trigger level command and enable the FIFO

    if( UART_FIFO_DISABLE == op )
    {
        UART1_FCTL =( unsigned char )0x00;  // disable the FIFO
    }
}


/* uartEnableTxInterrupts
     enable interrupts for a tx transaction 
     Only do this after the Tx FIFO is loaded, otherwise infinite "empty"
     events result */
static void uartEnableTxInterrupts( void )
{
    unsigned char const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier;
    
    ier =( unsigned char )( UART_IER_TRANSMITINT | UART_IER_TRANSCOMPLETEINT );

    if(( DEV_MODE_UART_HALF_MODEM & wiring )     ||
       ( DEV_MODE_UART_HALF_NULL_MODEM & wiring )||
       ( DEV_MODE_UART_FULL_MODEM & wiring )     ||
       ( DEV_MODE_UART_FULL_NULL_MODEM & wiring ))
    {
        ier |=( unsigned char )UART_IER_MODEMINT;
    }

    UART1_IER |= ier;
}


/* uartDisableTxInterrupts
     disable interrupts after a tx transaction completes */
static void uartDisableTxInterrupts( unsigned char iirEvent )
{
    unsigned char const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier =( unsigned char )UART_IER_TRANSMITINT;

    if( UART_IIR_TRANSCOMPLETE == iirEvent )
    {
        ier |=( unsigned char )UART_IER_TRANSCOMPLETEINT;

        if(( DEV_MODE_UART_HALF_MODEM & wiring )||
           ( DEV_MODE_UART_HALF_NULL_MODEM & wiring ))
        {
            ier |=( unsigned char )UART_IER_MODEMINT;
        }
        else
        if( 0 == rxTransact.mode )
        {
            if(( DEV_MODE_UART_FULL_MODEM & wiring )||
               ( DEV_MODE_UART_FULL_NULL_MODEM & wiring ))
            {
                ier |=( unsigned char )UART_IER_MODEMINT;
            }
        }
    }

    UART1_IER &=( unsigned char )( 0xff - ier );
}


/* uartEnableRxInterrupts
     enable interrupts for an rx transaction
     Only do this after the Rx FIFO is loaded, otherwise infinite "empty"
     events result */
static void uartEnableRxInterrupts( void )
{
    unsigned char const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier;

    ier =   ( unsigned char )( UART_IER_RECEIVEINT | 
                               UART_IER_LINESTATUSINT );

    if(( DEV_MODE_UART_HALF_MODEM & wiring )||
       ( DEV_MODE_UART_HALF_NULL_MODEM & wiring )||
       ( DEV_MODE_UART_FULL_MODEM & wiring )||
       ( DEV_MODE_UART_FULL_NULL_MODEM & wiring ))
    {
        ier |=( unsigned char )UART_IER_MODEMINT;
    }

    UART1_IER |= ier;
}


/* uartDisableRxInterrupts
     disable interrupts after a tx transaction completes */
static void uartDisableRxInterrupts( void )
{
    unsigned char const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier;
    
    ier =     ( unsigned char )UART_IER_RECEIVEINT;

    if(( DEV_MODE_UART_HALF_MODEM & wiring )||
       ( DEV_MODE_UART_HALF_NULL_MODEM & wiring ))
    {
        ier |=( unsigned char )UART_IER_MODEMINT;
    }
    else
    if( 0 == txTransact.mode )
    {
        if(( DEV_MODE_UART_FULL_MODEM & wiring )||
           ( DEV_MODE_UART_FULL_NULL_MODEM & wiring ))
        {
            ier |=( unsigned char )UART_IER_MODEMINT;
        }
    }

    UART1_IER &=( unsigned char )( 0xff - ier );
}


/*------ Private Rx and Tx Buffer functions ---------------------------------*/
/* loadTxBuffer
     txPrivBuf is operated as a linear buffer
     to be run in context of the calling task */
static void loadTxBuffer( unsigned char *buffer, size_t const sz )
{
    int l;

    for( l=0; sz > l; l++ )
    {
        txPrivBuf._[ l ]=( buffer )[ l ];
    }
    txPrivBuf.Index = sz;   // next input position
    txPrivBuf.Outdex = 0;   // next output position
}


/* storeRxBuffer
     rxPrivBuf is operated as a circular buffer
     memcpy may be in two parts, over the circular boundary */
static unsigned short storeRxBuffer( void )
{
    unsigned short cnt = rxTransact.lenTRx;
    unsigned short numStored;
    unsigned short const od = rxPrivBuf.Outdex;  // start point
    unsigned short l;
    unsigned short numLeft;

    RX_CIRCULAR_BUFFER_NUM_OCCUPIED( numStored );
    if( numStored < cnt ) cnt = numStored;

    numLeft = cnt + od;

    //avoid calling MOS functions from within an ISR
    //printf( " <numStored = %d> <cnt = %d> <od = %d> ", numStored, cnt, od );

    if( configDRV_UART_BUFFER_SZ <= numLeft )
    {
        // this case is wrapped around the circular boundary
        for( l = od; configDRV_UART_BUFFER_SZ > l; l++ )
        {
            *( rxTransact.buf )= rxPrivBuf._[ l ];
            ( rxTransact.buf )++;
        }
        numLeft -= configDRV_UART_BUFFER_SZ;
        for( l = 0; numLeft > l; l++ )
        {
            *( rxTransact.buf ) = rxPrivBuf._[ l ];
            ( rxTransact.buf )++;
        }
    }
    else
    {
        // this case has not wrapped around
        for( l = od; numLeft > l; l++ )
        {
            *( rxTransact.buf ) = rxPrivBuf._[ l ];
            ( rxTransact.buf )++;
        }
    }

    // adjust current Outdex    
    rxPrivBuf.Outdex += cnt;
    if( configDRV_UART_BUFFER_SZ < rxPrivBuf.Outdex )
    {
        rxPrivBuf.Outdex -= configDRV_UART_BUFFER_SZ;
    }
    
    return( cnt );
}


/* transmitChar
     a fast version if only transmitting one char at a time
     FIFO is not used */
static POSIX_ERRNO transmitChar( unsigned char const ch )
{
    UART1_THR = ch;
    return( POSIX_ERRNO_ENONE );
}


/* transmitNextBlock
     send the next bytes from the tx transaction
     The FIFO is enabled, so up to 16 bytes can be written at once */
static void transmitNextBlock( unsigned char * numTxBytes )
{
    unsigned char numB = txPrivBuf.Index - txPrivBuf.Outdex;  // not circular
    int i;

    numB =( THR_FIFO_MAX < numB )
        ? THR_FIFO_MAX
        : numB;

    for( i = 0; numB > i; i++ )
    {
        UART1_THR = txPrivBuf._[ txPrivBuf.Outdex++ ];
    }
    ( *numTxBytes )= numB;

    if( 0 < numB )
    {
        /* only enable interrupts AFTER the tx FIFO has been loaded,
           otherwise an empty tx fifo interrupt will immediately fire */
        uartEnableTxInterrupts( );
    }
}


/* finaliseTxTransaction
     On completion or abandonment of transmission, clear down RTS, set the
       calling task callback hooks, and erase the transaction.
*/
static void finaliseTxTransaction( POSIX_ERRNO const ret )
{
    clearRTS( );

    if( txTransact.bufTRx )
    {
        *( txTransact.bufTRx )= txTransact.lenTRx;
    }
    
    if( txTransact.bufRes )
    {
        *( txTransact.bufRes )= ret;
    }
}


/* receiveNextBlock
     recover the next bytes from the rx fifo
     The FIFO is enabled, so up to 16 bytes can be read at once */
static UART_ERRNO receiveNextBlock( unsigned char * numRxBytes )
{
    UART_ERRNO res = UART_ERRNO_NONE;
    int i;
    unsigned char numB;
    unsigned char rbr;

    RX_CIRCULAR_BUFFER_NUM_FREE( numB );
    numB =( RHR_FIFO_MAX < numB )
        ? RHR_FIFO_MAX
        : numB;

    *numRxBytes = 0;

    for( i = 0; numB > i; i++ )
    {
_putchf( 'g' );
        if( UART1_LSR & UART_LSR_DR )
        {
_putchf( '1' );
            rbr = UART1_RBR;  // read byte from UART
            
            if( UART_STATE_READY <= uartState )  // _READY, _READ and _WRITE
            {
_putchf( '2' );
                rxPrivBuf._[ rxPrivBuf.Index++ ]= rbr;
                ( *numRxBytes )++;

                // check circular buffer index
                if( configDRV_UART_BUFFER_SZ == rxPrivBuf.Index )
                {
                    rxPrivBuf.Index = 0;
                }
            }
        }
        else
            break;
    }

    /* if there are more rx bytes in the FIFO then rxPrivBuf is full,
        raise the error condition */
    if( UART1_LSR & UART_LSR_DR )
    {
        res = UART_ERRNO_RECEIVEFIFOFULL;
        
_putchf( '3' );
        // Flush the RxFIFO, which contains excess data
        uartResetFIFO( UART_FCTL_CLRRxF, UART_FIFO_ENABLE );
    }
    else
    if( 0 == *numRxBytes )
    {
_putchf( '4' );
        // TRIGGER LEVEL interrupt fired with nothing to read
        // Reset the RxFIFO then enable the FIFOs
        uartResetFIFO( UART_FCTL_CLRRxF, UART_FIFO_ENABLE );
    }

    return( res );
}


/* finaliseRxTransaction
     On completion or abandonment of reception, set the
       calling task callback hooks, and erase the transaction.
*/
static void finaliseRxTransaction( POSIX_ERRNO const ret )
{
    if( rxTransact.bufTRx )
    {
        *( rxTransact.bufTRx )= rxTransact.lenTRx;
    }
    
    if( rxTransact.bufRes )
    {
        *( rxTransact.bufRes )= ret;
    }
}


/*------- Private Interrupt Handling functions ------------------------------*/
/* clearDownLineStatus
     Refer to Zilog PS015317 UART Line Status Register table 64 */
static UART_ERRNO clearDownLineStatus( void )
{
    UART_ERRNO temp = UART_ERRNO_NONE;
    unsigned char lsr;

    lsr = UART1_LSR;
    
    if( UART_LSR_FIFOERR & lsr )   // bit 7 set
    {
        if( UART_LSR_BREAKINDICATIONERR & lsr )
        {
            /* break indication infers the remote end DTE still has data to 
               send, but wasn't able to prepare it in time for its serial tx
               hardware. We should expect more rx data to follow shortly. */
            temp = UART_ERRNO_BREAKINDICATIONERR;
              // we might start a timer, after expiry we abandon the rx
              // otherwise we just rely on the uartSemaphore-take to timeout
        }
        else 
        if( UART_LSR_FRAMINGERR & lsr )
        {
            /* The Rx data will be corrupted */
            temp = UART_ERRNO_FRAMINGERR;
        }
        else
        if( UART_LSR_PARITYERR & lsr )
        {
            /* The Rx data will be corrupted */
            temp = UART_ERRNO_PARITYERR;
        }
    }
    else  // bit 7 clear
    {
        if( UART_LSR_OVERRRUNERR & lsr )
        {
            temp = UART_ERRNO_OVERRUNERR;   // Rx FIFO full
              // we treat this as an error as we have lost rx data
        }
    }

    if(( UART_ERRNO_NONE != temp )&&
       ( UART_ERRNO_BREAKINDICATIONERR != temp ))
    {
        // Flush the RxFIFO, which contains corrupted data
        uartResetFIFO( UART_FCTL_CLRRxF, UART_FIFO_ENABLE );
    }
    else
    if( UART_LSR_DATA_READY & lsr )
    {
        // Rx Data has been transferred from the RBR to the RxFIFO
              // recall Zilog UP0049 errata note 7 on continuous rx interrupts
              // though that configuration combination doesn't apply here
        temp = UART_ERRNO_RX_DATA_READY;
    }

    return( temp );
}


/* clearDownModemStatus
     Refer to Zilog PS015317 UART Modem Status Register table 65
*/
static UART_ERRNO clearDownModemStatus( void )
{
#   define DEASSERTED 0
    UART_ERRNO temp = UART_ERRNO_NONE;
    unsigned char msr;

    msr = UART1_MSR;

    /* Tests are arranged into priority order */

    if(( DEV_MODE_UART_FULL_MODEM & pinmode[ MINOR_NUM ])||
       ( DEV_MODE_UART_FULL_NULL_MODEM & pinmode[ MINOR_NUM ]))
    {
        if( UART_MSR_DDSR & msr )
        {
            if( DEASSERTED ==( UART_MSR_DSR & msr ))  // remote end gone away
                temp = UART_ERRNO_DSR_LOST;
            else
                temp = UART_ERRNO_DSR_FOUND;
        }
        else
        if( UART_MSR_DDCD & msr )
        {
            if( DEASSERTED ==( UART_MSR_DCD & msr ))  // remote end signal lost
                temp = UART_ERRNO_DCD_LOST;
            else
                temp = UART_ERRNO_DCD_FOUND;
        }
    }

    if(( UART_ERRNO_NONE == temp )&&
       (( DEV_MODE_UART_HALF_MODEM & pinmode[ MINOR_NUM ])||
        ( DEV_MODE_UART_HALF_NULL_MODEM & pinmode[ MINOR_NUM ])))
    {
        if( UART_MSR_DCTS & msr )
        {
            if( DEASSERTED ==( UART_MSR_CTS & msr ))  // remote end rx buffer full
            {
                temp = UART_ERRNO_CTS_LOST;
                if( UART_STATE_WRITE & uartState )
                {
                    UART1_LCTL |= UART_LCTL_SB;       // break in transmission
                }
            }
            else
            {
                temp = UART_ERRNO_CTS_FOUND;
                if( UART_STATE_WRITE & uartState )
                {                                     // resume transmission
                    UART1_LCTL &=( unsigned char )( 0xff - UART_LCTL_SB );
                }
            }
        }
    }

    if(( UART_ERRNO_NONE == temp )&&
       (( DEV_MODE_UART_FULL_MODEM & pinmode[ MINOR_NUM ])||
        ( DEV_MODE_UART_FULL_NULL_MODEM & pinmode[ MINOR_NUM ])))
    {
        if( UART_MSR_TERI & msr )
        {
            if( DEASSERTED ==( UART_MSR_RI & msr ))
                temp = UART_ERRNO_RI_HANGUP;          // remote end hung up
            else
                temp = UART_ERRNO_RI_CALL;            // remote end calling
        }
    }

    /*
    if( DEV_MODE_UART_NO_MODEM & pinmode[ MINOR_NUM ])
    {
        // nothing to do; how did we get here, nobody knows :-?
    }
    */

    return( temp );
}


/*------ UART ISR -----------------------------------------------------------*/
/* uartisr_1
 *   decode cause(s) of interrupt and act on them
 *   Refer to Zilog PS015317 section UART registers UIIR tables 57 & 58
 */
static void uartisr_1( void )
{
    unsigned char iir;
    unsigned char numTRxBytes;
    UART_ERRNO _uart_errno;

    
    /* Read UART Interrupt Identification Register (IRR, table 57)
         Each interrupt is reported only after higher priority ones (table 58) 
         have been cleared down. */
    for( iir = UART1_IIR; 0 ==( iir & UART_IIR_INTBIT ); iir = UART1_IIR )
    {
        if( UART_IIR_LINESTATUS ==( iir & UART_IIR_ISCMASK ))
        {
            _uart_errno = clearDownLineStatus( );
_putchf('a');
            if(( UART_ERRNO_RX_DATA_READY == _uart_errno )||
               ( UART_ERRNO_BREAKINDICATIONERR == _uart_errno ))
            {
                /* Don't need to copy data as the FIFO is enabled,
                   wait for UART_IIR_DATAREADY_TRIGLVL
                   if however the FIFO is not enabled, then we must
                   call receiveNextBlock here */
            }
            else
            if( UART_ERRNO_NONE != _uart_errno )
            {
_putchf('1');
                if( UART_STATE_READY <= uartState )  // _READY, _READ and _WRITE
                {
                    uartRxEvent |=( 1 << UART_EVENT_RX_ERROR );
                }

                if( UART_STATE_READ & uartState )
                {
                    uartDisableRxInterrupts( );

                    if( DEV_MODE_UNBUFFERED == rxTransact.mode )
                    {
                        xSemaphoreGiveFromISR( uartRxSemaphore, NULL );
                    }
                    else
                    {
                        storeRxBuffer( );
                        finaliseRxTransaction( _uart_errno | uartRxEvent );
                        CLEAR_UART_TRANSACTION( rxTransact );
                        
                        uartRxEvent = 0;
                        uartState &= ~UART_STATE_READ;
                    }
                }
            }
        }

        /* A receiver time-out interrupt (UART_IIR_CHARTIMEOUT) is generated 
           when there are fewer data bytes in the receiver FIFO than the 
           trigger level and there are no reads and writes to or from the 
           receiver FIFO for four consecutive byte times (32-bit times) - 
           meaning there is a gap in reception. This will always occur if the
           rx data stream is less than a multiple of the trigger level; we 
           treat it like a FIFO trigger (UART_IIR_DATAREADY_TRIGLVL).It is 
           cleared only after emptying the receive FIFO. */
        if(( UART_IIR_DATAREADY_TRIGLVL ==( iir & UART_IIR_ISCMASK ))||
           ( UART_IIR_CHARTIMEOUT ==( iir & UART_IIR_ISCMASK )))
        {
_putchf('b');
if( UART_IIR_DATAREADY_TRIGLVL ==( iir & UART_IIR_ISCMASK )) _putchf('1');
if( UART_IIR_CHARTIMEOUT ==( iir & UART_IIR_ISCMASK )) _putchf('2');
            _uart_errno = receiveNextBlock( &numTRxBytes );

            if( UART_ERRNO_RECEIVEFIFOFULL == _uart_errno )
            {
_putchf('3');
                uartRxEvent |=( 1 << UART_EVENT_RX_FULL );
            }

            if( UART_STATE_READ & uartState )
            {
_putchf('4');
                rxTransact.lenTRx += numTRxBytes;
                if( rxTransact.lenTRx >= rxTransact.len )
                {
                    uartRxEvent |=( 1 << UART_EVENT_RX_READY );
                    uartDisableRxInterrupts( );

                    if( DEV_MODE_UNBUFFERED == rxTransact.mode )
                    {
_putchf('5');
                        xSemaphoreGiveFromISR( uartRxSemaphore, NULL );
                    }
                    else
                    {
_putchf('6');
                        storeRxBuffer( );
                        finaliseRxTransaction( _uart_errno | uartRxEvent );
                        CLEAR_UART_TRANSACTION( rxTransact );
                        
                        uartRxEvent = 0;
                        uartState &= ~UART_STATE_READ;
                    }
                }
            }
            else
            if(( UART_STATE_READY & uartState )||
               ( UART_STATE_WRITE & uartState ))
            {
_putchf('7');
                uartRxEvent |=( 1 << UART_EVENT_RX_READY );
            }
        }

        if( UART_IIR_TRANSCOMPLETE ==( iir & UART_IIR_ISCMASK ))
        {
            // Transmit Complete Interrupt (TCI) is generated when both 
            // the serial device Transmit Shift Register and the Transmit Hold 
            // Register are empty. Continuously - need to disable the interrupt.
            // This should signify we have no more data to send
_putchf('c');
            if( UART_STATE_WRITE & uartState )
            {
_putchf('1');
                uartDisableTxInterrupts( UART_IIR_TRANSCOMPLETE );
                // reset Tx FIFO and leave Rx Fifo enabled
                uartResetFIFO( UART_FCTL_CLRTxF, UART_FIFO_ENABLE );

                if( txTransact.lenTRx < rxTransact.len )
                {
_putchf('2');
                    uartTxEvent |=( 1 << UART_EVENT_TX_ERROR );
                }
                else
                {
_putchf('3');
                    uartTxEvent |=( 1 << UART_EVENT_TX_DONE );
                }

                if( DEV_MODE_BUFFERED == txTransact.mode )
                {
_putchf('4');
                    finaliseTxTransaction( POSIX_ERRNO_ENONE | uartTxEvent );
                    CLEAR_UART_TRANSACTION( txTransact );
                        
                    uartTxEvent = 0;
                    uartState &= ~UART_STATE_WRITE;
                }
                else
                {
_putchf('5');
                    xSemaphoreGiveFromISR( 
                        uartTxSemaphore, &mosHigherPriorityTaskWoken );
                }
            }
            else
            if(( UART_STATE_READY & uartState )||
               ( UART_STATE_READ & uartState ))
            {
_putchf('6');
                uartTxEvent |=( 1 << UART_EVENT_TX_UNEXPECTED );
                uartDisableTxInterrupts( UART_IIR_TRANSCOMPLETE );
                // reset Tx FIFO and leave Rx Fifo enabled
                uartResetFIFO( UART_FCTL_CLRTxF, UART_FIFO_ENABLE );
            }
        }

        if( UART_IIR_TRANSBUFFEREMPTY ==( iir & UART_IIR_ISCMASK ))
        {
            // Transmit Buffer Empty (TCIE) is generated when the Transmit Hold
            // Register is empty.
_putchf('d');
            if( UART_STATE_WRITE & uartState )
            {
_putchf('1');
                transmitNextBlock( &numTRxBytes );
                txTransact.lenTRx += numTRxBytes;
                // if there's nothing more to send, disable UART_IIR_TRANSBUFFEREMPTY
                if( 0 == numTRxBytes )
                {
_putchf('2');
                    // wait for UART_IIR_TRANSCOMPLETE after last byte shifted out
                    uartDisableTxInterrupts( UART_IIR_TRANSBUFFEREMPTY );
                }
            }
            else
            if(( UART_STATE_READY & uartState )||
               ( UART_STATE_READ & uartState ))
            {
_putchf('3');
                uartTxEvent |=( 1 << UART_EVENT_TX_UNEXPECTED );
                uartDisableTxInterrupts( UART_IIR_TRANSCOMPLETE );
                // reset Tx FIFO and leave Rx Fifo enabled
                uartResetFIFO( UART_FCTL_CLRTxF, UART_FIFO_ENABLE );
            }
        }

        if( UART_IIR_MODEMSTAT ==( iir & UART_IIR_ISCMASK ))
        {
            _uart_errno = clearDownModemStatus( );
_putchf('e');
            if(( UART_ERRNO_DSR_LOST == _uart_errno )||  // remote end gone away
               ( UART_ERRNO_DCD_LOST == _uart_errno ))   // remote end signal lost
            {
                if( UART_STATE_WRITE & uartState )
                {
                    uartTxEvent |=( 1 << UART_EVENT_TX_ERROR );
                }
                else
                if( UART_STATE_READ & uartState )
                {
                    uartRxEvent |=( 1 << UART_EVENT_RX_ERROR );
                }

                uartDisableTxInterrupts( UART_IIR_TRANSCOMPLETE );
                uartDisableRxInterrupts( );
                uartResetFIFO( 
                    UART_FCTL_CLRTxF | UART_FCTL_CLRRxF, UART_FIFO_DISABLE );

                if( UART_STATE_READ & uartState )
                {
                    if( DEV_MODE_BUFFERED == rxTransact.mode )
                    {
                        finaliseRxTransaction( POSIX_ERRNO_ENONE | uartTxEvent );
                        CLEAR_UART_TRANSACTION( rxTransact );

                        uartRxEvent = 0;
                        uartState &= ~UART_STATE_READ;
                    }
                    else
                    {
                        xSemaphoreGiveFromISR( 
                            uartRxSemaphore, &mosHigherPriorityTaskWoken );
                    }
                }

                if( UART_STATE_WRITE & uartState )
                {
                    if( DEV_MODE_BUFFERED == txTransact.mode )
                    {
                        finaliseTxTransaction( POSIX_ERRNO_ENONE | uartTxEvent );
                        CLEAR_UART_TRANSACTION( txTransact );

                        uartTxEvent = 0;
                        uartState &= ~UART_STATE_WRITE;
                    }
                    else
                    {
                        xSemaphoreGiveFromISR( 
                            uartTxSemaphore, &mosHigherPriorityTaskWoken );
                    }
                }
            }
        }
    }

    _putchf('f');
}


/* uartisr_0 - address written into the interrupt vector table
     Standard ISR reti.L epilogue */
static void uartisr_0( void )
{
    /* The current task context SHALL be saved first on entry to an ISR */
    portSAVE_CONTEXT( );

#   if defined( _DEBUG )
        _putchf( 'U' );
#   endif

    mosHigherPriorityTaskWoken = pdFALSE;

    /* run in context of this ISR.
         Other hardware interrupts are blocked out for the duration. */
    uartisr_1( );

    /* If mosHigherPriorityTaskWoken is now set to pdTRUE then a 
       context switch should be performed to ensure the 
       interrupt returns directly to the highest priority task */
    if( pdTRUE == mosHigherPriorityTaskWoken )
    {
#       if defined( _DEBUG )
            _putchf( 'V' );
#       endif

        asm( "\txref _vPortYieldFromISR_2   ; reti from vPortYieldFromISR" );
        asm( "\tJP _vPortYieldFromISR_2     ;   with saved context" );
    }
    else
    {
        asm( "\t                            ; reti from here" );
#       if defined( _DEBUG )
            _putchf( 'W' );
#       endif

        /* RESTORE CONTEXT SHALL be the last action prior to pop ix; reti.L */
        portRESTORE_CONTEXT( );

        asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
        asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
        asm( "               ;   __default_mi_handler" );
        asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );  // *2
        asm( "\t reti.L      ; Long ret (ADL_CALL_MODE_LIL + 3 byte PC)" ); // *3
        asm( "               ; as per UM007715 table 25 for IM 2 ADL=1 MADL=1" );
        asm( "               ; compiler-inserted epilogue below will not be executed" );
    }
}


/*------ UART low-level API functions ---------------------------------------*/
/* These routines normally called from devapi.c                              */

/* uart_dev_open
   Device-specific uart1 open function device configuration.
   Starts from one of the initial conditions:
     by hardware reset, 
     or as programmed by MOS uart.c::init_UART1,
     or as left after gpio_dev_close,
     with the pin functions assigned as Mode 2 (High-Impedance inputs).
   Plus with 1==configUSE_DEV_SAFEGUARDS, we know the mode-required pins are 
     free. We assume them free with 0==configUSE_DEV_SAFEGUARDS.

   Refer to ZDSII/src/common/openuart1.c::::open_UART1
                             controluart1.c::control_UART1
   Also refer to agon-mos-main/src/
      main.c::main invokes uart.c::init_UART1
      uart.c::init_UART1 programs PortC DR, DDR, ALT, ALT2 as high-impedance 
          inputs (Mode 2), which is the hardware reset default
      uart.c::open_UART1 compare how it programs the UART registers as per 
          PS015317  "UART Recommended Usage".
*/
POSIX_ERRNO uart_dev_open(
                DEV_MODE const mode,
                UART_PARAMS const * params
            )
{
    POSIX_ERRNO ret;
    unsigned int endpin;
    unsigned char setmask;
    unsigned char clrmask;
    unsigned char ier;
    int i;


    /* -2. Create the high-priority interrupt handler task */
#   if 0 //( 1 == configUSE_PREEMPTION )
    {
        ret = createUartIsrHandlerTask( );
    }
#   endif

    /* -1. Reset the UART semaphores on each uart_dev_open() */
    ret = initialiseUartSemaphore( &uartRxSemaphore );
    if( POSIX_ERRNO_ENONE != ret )
    {
        goto uart_dev_open_end;
    }
    ret = initialiseUartSemaphore( &uartTxSemaphore );
    if( POSIX_ERRNO_ENONE != ret )
    {
        goto uart_dev_open_end;
    }

#   if( 0 == configUSE_DEV_SAFEGUARDS )
    {
        /* 0. remember the pinmode for hardware flow control in the ISR */
        pinmode[ MINOR_NUM ]= mode;
    }
#   endif

    /* 1. set the eZ80 port pin modes for Alternate Function number 0 */
    if(( DEV_MODE_UART_FULL_MODEM & mode )||
       ( DEV_MODE_UART_FULL_NULL_MODEM & mode ))
    {
        endpin = UART_1_RI;  // last pin used in full handshaking
    }
    else
    if(( DEV_MODE_UART_HALF_MODEM & mode )||
       ( DEV_MODE_UART_HALF_NULL_MODEM & mode ))
    {
        endpin = UART_1_CTS; // last pin used in half-handshaking
    }
    else
    {
        endpin = UART_1_RXD; //last pin used without handshaking
    }

    for( i = UART_1_TXD, setmask = 0x00, clrmask = 0xff; endpin >= i; i++ )
    {
        unsigned int const mnri =( i - PIN_NUM_START );

        setmask |= SET_MASK( portmap[ mnri ].bit );
        clrmask &= CLR_MASK( portmap[ mnri ].bit );
    }

    CLEAR_PORT_MASK( PC_DR, clrmask );
    SET_PORT_MASK( PC_DDR, setmask );
    CLEAR_PORT_MASK( PC_ALT1, clrmask );
    SET_PORT_MASK( PC_ALT2, setmask );

    /* 2. program the uart registers for passed in params */
    /* 2.1 Program the BRG
     *     Refer to ZDSII/src/common/openuart1.c::::open_UART1
     *                               controluart1.c::control_UART1
     *     refer to Zilog ps015317 Recommended Usage of the Baud Rate Generator */
    UART1_LCTL |=( unsigned char )UART_LCTL_DLAB;  // enable baud rate generator modify
    UART1_BRG_L =( unsigned char )( brg[ params->baudRate ]& 0x00ff );
    UART1_BRG_H =( unsigned char )( brg[ params->baudRate ]& 0xff00 )>> 8;
    UART1_LCTL &=( unsigned char )( 0xff - UART_LCTL_DLAB );  // disable baud rate generator modify

    /* 2.2 Clear the Modem Control register
           This sets pin /DTR high, indicating UART1 is not ready */
    UART1_MCTL =( unsigned char )0x00;

    /* 2.3 Set the Line Control Register */
    UART1_LCTL =
        ( unsigned char )( params->dataBits | params->stopBits | params->parity );

    /* 2.4 Initialise the software FIFO and State for Interrupt Mode operation,
             including storing the mode so the ISR can manage RI/DCD/DSR/CTS 
             and set DTR/RTS */
    portENTER_CRITICAL( );
    {
        for( i=0; sizeof( UART_BUFFER )> i; i++ )
        {
            (( unsigned char* )&txPrivBuf )[ i ]= 0;
            (( unsigned char* )&rxPrivBuf )[ i ]= 0;
        }
        CLEAR_UART_TRANSACTION( txTransact );
        CLEAR_UART_TRANSACTION( rxTransact );

        uartState = UART_STATE_READY;
    }
    portEXIT_CRITICAL( );

    /* 2.5 Attach the ISR */
    prevUartISR =  // remember old vector to restore on uart_close
        mos_setintvector( UART1_IVECT, uartisr_0 );

    /* 2.6 Initialise the UART device FIFOs */
    uartResetFIFO( UART_FCTL_CLRRxF | UART_FCTL_CLRTxF, UART_FIFO_ENABLE );
 
    /* 2.7 Initialise Interrupt Enable Register
           We only initialise Rx interrupts when we're ready to receive,
           And tx interrupts when we're ready to transmit;
           Otherwise "empty" interrupts fire evermore */
    UART1_IER = /*ier_mask =*/ 0x00;

    /* 2.8 Assert /DTR to signal we are ready for transceiving */
    if(( DEV_MODE_UART_FULL_MODEM & mode )||
       ( DEV_MODE_UART_FULL_NULL_MODEM & mode ))
    {
        UART1_MCTL |= UART_MCTL_DTR;
    }
    
uart_dev_open_end:
#if defined( _DEBUG )&& 0
    ( void )printf( "devuart.c : %d\r\n", __LINE__ );
#endif

    return( ret );
}


/* uart_dev_close
   Device-specific UART close function device shutdown 
   Starts from used condition
     with the pin functions assigned as Mode 7 (Alternative function 0).
     
   Refer to ZDSII/src/common/closeuart1.c::::close_UART1
   Also refer to agon-mos-main/src/
      uart.c::close_UART1 compare how it programs the UART registers versus 
          PS015317  "UART Recommended Usage".
*/
void uart_dev_close(
         void
     )
{
    unsigned int endpin;
    unsigned char setmask;
    unsigned char clrmask;
    int i;

    // 1. Disable UART
    UART1_IER = /*ier_mask =*/ 0x00;  // Disable UART1 interrupts
    UART1_LCTL = 0x00; // Bring line control register to reset value.
    UART1_MCTL = 0x00; // Bring modem control register to reset value.
                       // This clears /DTR to signal we are not ready to transceive

    /* 2 Flush UART1 FIFOs and disable FIFOs
       Errata UP004909 Issue 7: looks like we should only clear the FIFOs by
       setting the UART_FCTL_CLRTxF or UART_FCTL_CLRRxF bits while 
       UART_FCTL_FIFOEN is cleared */
    uartResetFIFO( UART_FCTL_CLRRxF | UART_FCTL_CLRTxF, UART_FIFO_DISABLE );

    // 2.1 Detach from UART
    portENTER_CRITICAL( );
    {
        uartState = UART_STATE_INITIAL;
    }
    portEXIT_CRITICAL( );

    // 2.2 restore previous interrupt vector table entry and unknow it
    if( prevUartISR )
    {
        mos_setintvector( UART1_IVECT, prevUartISR );
    }
    prevUartISR = NULL;

    // 3. set each assigned UART pin back to Mode 2 - standard digital input
    if(( DEV_MODE_UART_FULL_MODEM & pinmode[ MINOR_NUM ])||
       ( DEV_MODE_UART_FULL_NULL_MODEM & pinmode[ MINOR_NUM ]))
    {
        endpin = UART_1_RI;
    }
    else
    if(( DEV_MODE_UART_HALF_MODEM & pinmode[ MINOR_NUM ])||
       ( DEV_MODE_UART_HALF_NULL_MODEM & pinmode[ MINOR_NUM ]))
    {
        endpin = UART_1_CTS;
    }
    else
    {
        endpin = UART_1_RXD;
    }

    for( i = UART_1_TXD, setmask = 0x00, clrmask = 0xff; endpin >= i; i++ )
    {
        unsigned int const mnri =( i - PIN_NUM_START );

        setmask |= SET_MASK( portmap[ mnri ].bit );
        clrmask &= CLR_MASK( portmap[ mnri ].bit );
    }

    CLEAR_PORT_MASK( PC_DR, clrmask );
    SET_PORT_MASK( PC_DDR, setmask );
    CLEAR_PORT_MASK( PC_ALT1, clrmask );
    CLEAR_PORT_MASK( PC_ALT2, clrmask );

#   if( 0 == configUSE_DEV_SAFEGUARDS )
    {
        /* 4. unknow the user mode */
        pinmode[ MINOR_NUM ]= 0;
    }
#   endif
}


/* uart_dev_read
   Device-specific UART1 read function 
     The receive paramaters are used to create a transaction.
     In DEV_MODE_UNBUFFERED mode the calling task will be suspended while the 
     read is in progress; whereas in DEV_MODE_BUFFERED it will return without
     waiting.
     Once num_bytes_to_read have been received, or if an error occurs, the driver 
     will invoke an application callback to return the POSIX_ERRNO result. 
     It is then up to the application to check the result.
   If the application makes a uart_read call while another is in progress,
     then the result POSIX_ERRNO_EBUSY will be immediately returned. 
   Refer to Zilog PS015317 UART Receiver
     And to src/uart/common/readuart1.c
                           /fifoget1.c
*/
POSIX_ERRNO uart_dev_read(
                void * const buffer,              // OUT
                size_t const num_bytes_to_read,   // IN
                size_t * num_bytes_read,          // OUT
                POSIX_ERRNO *res                  // OUT
            )
{
    DEV_MODE const modeB =(( NULL != num_bytes_read )&&( NULL != res ))
                             ? DEV_MODE_BUFFERED
                             : DEV_MODE_UNBUFFERED;
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;


    if( configDRV_UART_BUFFER_SZ >= num_bytes_to_read )
    {
        portENTER_CRITICAL( );
        if( 0 ==( UART_STATE_READ & uartState ))
        {
            uartState |= UART_STATE_READ;
            portEXIT_CRITICAL( );

            CREATE_UART_TRANSACTION( 
                rxTransact, modeB, 
                buffer, num_bytes_to_read, num_bytes_read, res );
                // set rxTransact = count of rx data buffered
            RX_CIRCULAR_BUFFER_NUM_OCCUPIED( rxTransact.lenTRx );
#           if defined( _DEBUG )&& 0
                PRINT_UART_TRANSACTION( rxTransact );
#           endif

            if( rxTransact.lenTRx >= rxTransact.len )
            {
                // enough is already buffered to return immediately
                storeRxBuffer( );
                finaliseRxTransaction( ret );
                CLEAR_UART_TRANSACTION( rxTransact );

                ret = POSIX_ERRNO_EALREADY | uartRxEvent;
                uartRxEvent = 0;
                
                uartState &= ~UART_STATE_READ;
            }
            else
            {
                uartEnableRxInterrupts( );
            
                if( DEV_MODE_BUFFERED == modeB )
                {
                    /* DEV_MODE_BUFFERED mode: the calling task will not block.
                        It will return immediately, and shall periodically test 
                        num_bytes_read and res. */
                }
                else
                {
                    /* In DEV_MODE_UNBUFFERED mode, the calling task is suspended 
                       until the UART receive completes or an error happens, when 
                       the ISR will wake the calling task. Other transactions will
                       be refused until the read completes. */
                    if( pdTRUE == xSemaphoreTake( 
                                      uartRxSemaphore, 
                                      configDRV_UART_UNBUFFERED_DELAY ))
                    {
                        // calling task should check callback result
                        ret = POSIX_ERRNO_EALREADY;  // num_bytes_to_read
                    }
                    else
                    {
                        // calling task should check callback result
                        ret = POSIX_ERRNO_ETIMEDOUT; // less than num_bytes_to_read
                    }

                    ret |= uartRxEvent;
                    uartRxEvent = 0;

                    storeRxBuffer( );
                    finaliseRxTransaction( ret );
                    CLEAR_UART_TRANSACTION( rxTransact );

                    uartState &= ~UART_STATE_READ;
                }
            }
        }
        else
        {
            portEXIT_CRITICAL( );
            ret = POSIX_ERRNO_EBUSY;
        }
    }
    else
    {
        ret = POSIX_ERRNO_EMSGSIZE;
    }

    return( ret );
}


/* uart_dev_write
   Device-specific UART1 write function 
     The bytes to be transmitted are used to create a transaction.
     In DEV_MODE_UNBUFFERED mode the calling task will be suspended while the 
     write is in progress.
     Whereas in DEV_MODE_BUFFERED it will return without waiting; it is then up 
     to the application to poll the result.
     If the application makes a uart_write call while another is in progress,
     then the result POSIX_ERRNO_EBUSY will be immediately returned. 
   Refer to Zilog PS015317 UART Transmit
*/
POSIX_ERRNO uart_dev_write(
                void * const buffer,              // IN
                size_t const num_bytes_to_write,  // IN
                size_t * num_bytes_written,       // OUT
                POSIX_ERRNO *res                  // OUT
            )
{
    POSIX_ERRNO ret;
    DEV_MODE const modeB =(( NULL != num_bytes_written )&&( NULL != res ))
                             ? DEV_MODE_BUFFERED
                             : DEV_MODE_UNBUFFERED;
    unsigned char numTxBytes;

    if( configDRV_UART_BUFFER_SZ >= num_bytes_to_write )
    {
        portENTER_CRITICAL( );
        if( 0 ==( UART_STATE_WRITE & uartState ))  // if not already writing
        {
            uartState |= UART_STATE_WRITE;
            portEXIT_CRITICAL( );

            ret = assertRTSandWaitCTS( );
            if( POSIX_ERRNO_EPROTO == ret )
            {
                uartState &= ~UART_STATE_WRITE;
                goto uart_dev_write_1;
            }

            // Good to go
            if( 1 == num_bytes_to_write )
            {
                // don't use the FIFO for just one byte
                ret = transmitChar((( char * )buffer )[ 0 ]);
                uartState &= ~UART_STATE_WRITE;
                if( DEV_MODE_BUFFERED == modeB )
                {
                    *num_bytes_written = 1;
                    *res = ret;
                }
            }
            else
            {
                // 2 or more bytes to send, use the FIFO
                CREATE_UART_TRANSACTION( 
                    txTransact, modeB, 
                    buffer, num_bytes_to_write, num_bytes_written, res );
#               if defined( _DEBUG )&& 0
                    PRINT_UART_TRANSACTION( txTransact );
#               endif

                loadTxBuffer(( unsigned char * )buffer, num_bytes_to_write );

                    // kick off the write transaction
                transmitNextBlock( &numTxBytes );
                txTransact.lenTRx += numTxBytes;

                if( DEV_MODE_BUFFERED == modeB )
                {
                    /* In DEV_MODE_BUFFERED mode the calling task will run 
                       immediately. The application must poll num_bytes_written
                       and res to test the write result. */
                    ret = POSIX_ERRNO_EINPROGRESS;
                }
                else            
                {
                    /* In DEV_MODE_UNBUFFERED mode the calling task is suspended 
                       until the UART transfer completes or an error happens, 
                       when the calling task will resume.
                        xSemaphoreTake doesn't timeout indefinitely despite
                        what FreeRTOS API says */
UBaseType_t cnt = uxSemaphoreGetCount( uartTxSemaphore );

_putchf( 'P' );
                    if( pdTRUE == xSemaphoreTake( 
                                      uartTxSemaphore, 
                                      configDRV_UART_UNBUFFERED_DELAY ))
                    {
                        ret = POSIX_ERRNO_EALREADY;  //0x72
                    }
                    else
                    {
                        ret = POSIX_ERRNO_ETIMEDOUT;  //0x6E
                    }
_putchf( 'Q' );

                    ret |= uartTxEvent;
                    uartTxEvent = 0;

                    finaliseTxTransaction( ret );
                    CLEAR_UART_TRANSACTION( txTransact );

                    uartState &= ~UART_STATE_WRITE;
_putchf( 'R' );
                }
            }
        }
        else
        {
            portEXIT_CRITICAL( );

            ret = POSIX_ERRNO_EBUSY;   // transaction not accepted
        }
    }
    else
    {
        ret = POSIX_ERRNO_EMSGSIZE;
    }

uart_dev_write_1:

    return( ret );
}


#endif  /* 1 == configUSE_DRV_UART */
