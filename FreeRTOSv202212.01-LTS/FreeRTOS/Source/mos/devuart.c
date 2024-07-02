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
/* Refer to https://en.wikipedia.org/wiki/RS-232#3-wire_and_5-wire_RS-232 for
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
 */


/*---- Constants ------------------------------------------------------------*/
#define THR_FIFO_MAX     ( 16 )
#define RHR_FIFO_MAX     ( 16 )

#define MINOR_NUM        ( UART_1_TXD - PIN_NUM_START )


#define CREATE_UART_TRANSACTION( _t, _tMode, _tSz, _bufSz, _bufResult )\
    ( _t ).mode =( _tMode ), \
    ( _t ).len =( _tSz ), \
    ( _t ).bufTRx =( _bufSz ), \
    ( _t ).bufRes =( _bufResult ), \
    ( _t ).lenTRx = 0      /* last in sequence doubles up as a return value */

#define CLEAR_UART_TRANSACTION( _t ) \
    ( _t ).mode = 0, \
    ( _t ).len = 0, \
    ( _t ).bufTRx = NULL, \
    ( _t ).bufRes = NULL, \
    ( _t ).lenTRx = 0      /* last in sequence doubles up as a return value */


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
    UART_STATE_INITIAL = 0,
    UART_STATE_READY,
    UART_STATE_READ,
    UART_STATE_WRITE,

} UART_STATE;


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


#define RX_CIRCULAR_BUFFER_TAKEN( r ) \
        { unsigned char const i = rxPrivBuf.Index; \
          unsigned char const o = rxPrivBuf.Outdex; \
          \
          ( r )=( i < o ) \
            ?(( configDRV_UART_BUFFER_SZ - o )+ i ); \
            :( i - o ) \
        }

#define RX_CIRCULAR_BUFFER_AVAIL( r ) \
        { unsigned char const i = rxPrivBuf.Index; \
          unsigned char const o = rxPrivBuf.Outdex; \
          \
          ( r )=( i < o ) \
            ?( o - i ) \
            :(( configDRV_UART_BUFFER_SZ - i )+ o ); \
        }


typedef struct _uart_buffer
{
    unsigned short Index;                           // next input position
    unsigned short Outdex;                          // next output position
    unsigned char  _[ configDRV_UART_BUFFER_SZ ];   // storage

} UART_BUFFER;


typedef struct _uart_transaction
{
    unsigned int mode;        // DEV_MODE_BUFFERED or DEV_MODE_UNBUFFERED
    size_t len;               // number of bytes to     read/write
    size_t lenTRx;            // local tracker of bytes transceived
    size_t *bufTRx;           // destination buffer for lenTRx
    POSIX_ERRNO *bufRes;      // destination buffer for result
    
} UART_TRANSACTION;


/*---- Global Variables ----------------------------------------------------*/
/* Changing any global variable needs to be done within
    portENTER_CRITICAL( );
    {
    }
    portEXIT_CRITICAL( );
*/
static SemaphoreHandle_t uartSemaphore = NULL; // for non-buffered blocking-mode user task

#if( 1 == configUSE_PREEMPTION )
    static TaskHandle_t uartIsrTaskHandle = NULL;
    static void UartIsrTask( void * params );
    static unsigned int uartIsrTaskCnt = 0;
#endif

static UART_STATE uartState = UART_STATE_INITIAL;

static UART_BUFFER txPrivBuf ={{ 0 }, 0, 0 };   // a linear buffer, tx from application
static UART_BUFFER rxPrivBuf ={{ 0 }, 0, 0 };   // a circular buffer, rx as it arrives

static UART_TRANSACTION txTransact ={ 0, 0 };
static UART_TRANSACTION rxTransact ={ 0, 0 };

static void( *prevUartISR )( void )= NULL;

static UART_ERRNO _uart_errno;  // a system level errnp

static unsigned short uEvent = 0;


/*----- Private functions ---------------------------------------------------*/
#if( 1 == configUSE_PREEMPTION )
static POSIX_ERRNO createUartIsrTask( void )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    BaseType_t r;

    r = xTaskCreate( 
            UartIsrTask, 
            "UART ISR Handler", 
            configMINIMAL_STACK_SIZE, 
            NULL,                      // no params
            configMAX_PRIORITIES,      // highest possible priority
            &uartIsrTaskHandle );
    if( pdPASS != r )
    {
#       if defined( _DEBUG )
        {
            ( void )printf( "Failed to allocate UartIsrTask: %d\r\n", r );
        }
#       endif

        ret = POSIX_ERRNO_ENSRCH;
    }
    else
    {
#       if defined( _DEBUG )
        {
            ( void )printf( "Created UartIsrTask\r\n" );
        }
#       endif
    }

    return( ret );
}
#endif


static POSIX_ERRNO initialiseUartSemaphore( )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    if( NULL == uartSemaphore )
    {
        // one-time creation
        uartSemaphore = xSemaphoreCreateBinary( );
    }

    if( NULL == uartSemaphore )
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
        /* We may call uart_Dev_open multiple times. Ensure the sempahore is in 
           the blocking state, after any previous use 
           If the semaphore is a binary semaphore then uxSemaphoreGetCount 
           returns 1 if the semaphore is available, and 0 if the semaphore is 
           not available. */
        UBaseType_t cnt = uxSemaphoreGetCount( uartSemaphore );
        if( cnt )
        {
            xSemaphoreTake( uartSemaphore, 0 );
        }
    }

    return( ret );
}


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
    txPrivBuf.Index = sz ;
    txPrivBuf.Outdex = 0;
}


/* storeRxBuffer
     rxPrivBuf is operated as a circular buffer
     memcpy may be in two parts, over the circular boundary */
static unsigned short storeRxBuffer( 
                          unsigned char * dest, 
                          unsigned short cnt )
{
    unsigned short numStored;
    unsigned short const od = rxPrivBuf.Outdex;  // start point
    unsigned short l;
    unsigned short numLeft;

    RX_CIRCULAR_BUFFER_AVAIL( numStored );
    if( numStored < cnt ) cnt = numStored;

    numLeft = cnt + od;
    if( configDRV_UART_BUFFER_SZ <= numLeft )
    {
        // this case is wrapped around the circular boundary
        for( l = od; configDRV_UART_BUFFER_SZ > l; l++ )
        {
            *dest++ = rxPrivBuf._[ l ];
        }
        numLeft -=( configDRV_UART_BUFFER_SZ - od );
        for( l = 0; numLeft > l; l++ )
        {
            *dest++ = rxPrivBuf._[ l ];
        }
    }
    else
    {
        // this case has not wrapped around
        for( l = od; numLeft > l; l++ )
        {
            *dest++ = rxPrivBuf._[ l ];
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


/*------ UART I/O value functions -------------------------------------------*/
/* transmitNextBlock
     send the next bytes from the tx transaction
     The FIFO is enabled, so up to 16 bytes can be written at once */
static unsigned char transmitNextBlock( unsigned char * numTxBytes )
{
    unsigned char numB = txPrivBuf.Index - txPrivBuf.Outdex;  // not circular
    int i;

    *numTxBytes = 0;
    
    numB =( THR_FIFO_MAX < numB )
        ? THR_FIFO_MAX
        : numB;

    for( i = 0; numB > i; i++ )
    {
        UART1_THR = txPrivBuf._[ txPrivBuf.Outdex++ ];
    }

    ( *numTxBytes )= numB;

    return( UART_ERRNO_NONE );
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

    RX_CIRCULAR_BUFFER_AVAIL( numB );
    numB =( RHR_FIFO_MAX < numB )
        ? RHR_FIFO_MAX
        : numB;

    *numRxBytes = 0;

    for( i = 0; numB > i; i++ )
    {
        if( UART1_LSR & UART_LSR_DR )
        {
            rbr = UART1_RBR;
            
            if( UART_STATE_READY <= uartState )  // _READY, _READ and _WRITE
            {
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
        
        // empty the Rx FIFO (lose data as we have nowhere to put it)
        while( UART1_LSR & UART_LSR_DR )
        {
            rbr = UART1_RBR;
        }
    }

    return( res );
}


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
        UART1_FCTL =( unsigned char )( UART_FCTL_CLRRxF | UART_FCTL_FIFOEN ); // Flush receive hardware FIFO
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
                if( UART_STATE_WRITE == uartState )
                {
                    UART1_FCTL |= UART_LCTL_SB;       // break in transmission
                }
            }
            else
            {
                temp = UART_ERRNO_CTS_FOUND;
                if( UART_STATE_WRITE == uartState )
                {                                     // resume transmission
                    UART1_FCTL &=( unsigned char )( 0xff - UART_LCTL_SB );
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


static void finaliseTransmit( POSIX_ERRNO const ret )
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

    CLEAR_UART_TRANSACTION( txTransact );
}


/*------ UART ISR functions -------------------------------------------------*/
/* uartisr_1
 *   decode cause(s) of interrupt and act on them
 *   runs with interrupts disabled
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
 *         set uState = UART_STATE_READY.
 *     On rx error interrupts
 *       clear them down; 
 *       reset read FIFO;
 *       if event queue last != erroneous rx event
 *         record erroneous rx in event queue;
 *       if DEV_MODE_UNBUFFERED
 *         give uartSemaphore;
 *       set uState = UART_STATE_READY.
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
 *         set uState = UART_STATE_READY.
 *     On tx error interrupts
 *       clear them down; 
 *       reset write FIFO;
 *       if event queue last != erroneous tx event
 *         record erroneous tx in event queue;
 *       if DEV_MODE_UNBUFFERED
 *         give uartSemaphore;
 *       set uState = UART_STATE_READY.
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
 *
 *   Refer to Zilog PS015317 section UART registers UIIR tables 57 & 58
 */
static void uartisr_1( void )
{
    unsigned char iir;
    unsigned char numTRxBytes;

    
    /* Read UART Interrupt Identification Register (IRR, table 57)
       Test interrupt sources in order of priority (table 58)
         Each one appears only after higher priority ones have been 
         cleared down */
    for( iir = UART1_IIR; 0 ==( iir & UART_IIR_INTBIT ); iir = UART1_IIR )
    {
        if( UART_IIR_LINESTATUS ==( iir & UART_IIR_ISCMASK ))
        {
            _uart_errno = clearDownLineStatus( );
            if(( UART_ERRNO_RX_DATA_READY == _uart_errno )||
               ( UART_ERRNO_BREAKINDICATIONERR == _uart_errno ))
            {
                // don't need to copy data as the FIFO is enabled,
                // wait for UART_IIR_DATAREADY_TRIGLVL
                // if however the FIFO is not enabled, then we must
                // call receiveNextBlock here
            }
            else
            if( UART_ERRNO_NONE != _uart_errno )
            {
                if( UART_STATE_READY <= uartState )  // _READY, _READ and _WRITE
                {
                    uEvent |=( 1 << UART_EVENT_RX_ERROR );
                }

                if( UART_STATE_READ == uartState )
                {
                    if( DEV_MODE_UNBUFFERED == rxTransact.mode )
                    {
                        xSemaphoreGiveFromISR( uartSemaphore, NULL );
                    }
                    else
                    {
                        /* no need to finalise rx transaction; 
                           no such thing as DEV_MODE_BUFFERED rxTransact */
                    }
                }
            }
        }

            // A receiver time-out interrupt is generated when there are fewer 
            // data bytes in the receiver FIFO than the trigger level and there
            // are no reads and writes to or from the receiver FIFO for four 
            // consecutive byte times - meaning there is a gap in reception. 
            // This will always occur if the rx data stream is less than a 
            // multiple of the trigger level; we treat it like a FIFO trigger.
            // It is cleared only after emptying the receive FIFO.
        if(( UART_IIR_DATAREADY_TRIGLVL ==( iir & UART_IIR_ISCMASK ))||
           ( UART_IIR_CHARTIMEOUT ==( iir & UART_IIR_ISCMASK )))
        {
            _uart_errno = receiveNextBlock( &numTRxBytes );

            if( UART_ERRNO_RECEIVEFIFOFULL == _uart_errno )
            {
                uEvent |=( 1 << UART_EVENT_RX_FULL );
            }

            if( UART_STATE_READ == uartState )
            {
                rxTransact.lenTRx += numTRxBytes;
                if( rxTransact.lenTRx >= rxTransact.len )
                {
                    uEvent |=( 1 << UART_EVENT_RX_READY );

                    if( DEV_MODE_UNBUFFERED == rxTransact.mode )
                    {
                        xSemaphoreGiveFromISR( uartSemaphore, NULL );
                    }
                }
            }
            else
            if(( UART_STATE_READY == uartState )||
               ( UART_STATE_WRITE == uartState ))
            {
                uEvent |=( 1 << UART_EVENT_RX_READY );
            }
        }

        if( UART_IIR_TRANSCOMPLETE ==( iir & UART_IIR_ISCMASK ))
        {
            // Transmit Complete Interrupt (TCI) is generated when both 
            // the serial device Transmit Shift Register and the Transmit Hold 
            // Register are empty.
            // This should signify we have no more data to send
            if( UART_STATE_WRITE == uartState )
            {
                if( txTransact.lenTRx < rxTransact.len )
                {
                    uEvent |=( 1 << UART_EVENT_TX_ERROR );
                }
                else
                {
                    uEvent |=( 1 << UART_EVENT_TX_DONE );
                }

                if( DEV_MODE_BUFFERED == txTransact.mode )
                {
                    finaliseTransmit( POSIX_ERRNO_ENONE | uEvent );
                    uEvent = 0;
                }
                else
                {
                    xSemaphoreGiveFromISR( uartSemaphore, NULL );
                }
            }
            else
            if(( UART_STATE_READY == uartState )||
               ( UART_STATE_READ == uartState ))
            {
                uEvent |=( 1 << UART_EVENT_TX_UNEXPECTED );
            }
        }

        if( UART_IIR_TRANSBUFFEREMPTY ==( iir & UART_IIR_ISCMASK ))
        {
            // Transmit Buffer Empty (TCIE) is generated when the Transmit Hold
            // Register is empty.
            if( UART_STATE_WRITE == uartState )
            {
                _uart_errno = transmitNextBlock( &numTRxBytes );
                txTransact.lenTRx += numTRxBytes;
            }
            else
            if(( UART_STATE_READY == uartState )||
               ( UART_STATE_READ == uartState ))
            {
                uEvent |=( 1 << UART_EVENT_TX_UNEXPECTED );
            }
        }

        if( UART_IIR_MODEMSTAT ==( iir & UART_IIR_ISCMASK ))
        {
            _uart_errno = clearDownModemStatus( );
            if(( UART_ERRNO_DSR_LOST == _uart_errno )||  // remote end gone away
               ( UART_ERRNO_DCD_LOST == _uart_errno ))   // remote end signal lost
            {
                if( UART_STATE_WRITE == uartState )
                {
                    uEvent |=( 1 << UART_EVENT_TX_ERROR );
                }
                else
                if( UART_STATE_READ == uartState )
                {
                    uEvent |=( 1 << UART_EVENT_RX_ERROR );
                }

                if( DEV_MODE_UNBUFFERED == txTransact.mode )
                {
                    xSemaphoreGiveFromISR( uartSemaphore, NULL );
                }
            }
        }
    }
}


#if( 1 == configUSE_PREEMPTION )
/* UartIsrTask
     High-priority Uart interrupt handler task.
     Will run immediately on exit of _uartisr_0;
       iff the current task is not blocking inside MOS */
static void UartIsrTask( void * params )
{
    unsigned long notification;

    for( ;; )
    {
        notification = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
_putchf( 'V' );
        if( notification )
        {
            uartisr_1( );

            uartIsrTaskCnt++;
        }
    }
}
#endif


/* uartisr_0 - address written into the interrupt vector table
     Standard ISR reti epilogue */
static void uartisr_0( void )
{
#   if( 1 == configUSE_PREEMPTION )
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

_putchf( 'U' );
        /* run in context of high-priority interrupt handler uartIsrTask.
             Other hardware interrupts may be raised concurrently.
             If the uart isr handling isn't served quickly enough, you can
             increase the uart handler task priority, or revert back to calling
             uartisr_1 directly from uartisr_0 */
        vTaskNotifyGiveFromISR( uartIsrTaskHandle, &xHigherPriorityTaskWoken );
        
        /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context 
           switch should be performed to ensure the interrupt returns directly 
           to the highest priority task. */
        vPortYieldFromISR( xHigherPriorityTaskWoken );
    }
#   else
    {
        /* run in context of this ISR.
             Other hardware interrupts are blocked out for the duration. */
        uartisr_1( );
    }
#   endif

    asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
    asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
    asm( "               ;   __default_mi_handler" );
    asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );
    asm( "\t reti.l      ; need reti.L as per UM007715 table 25 for IM 2 ADL=1 MADL=1");

    /* following compiler-inserted epilogue is not executed */
}


/*------ UART low-level functions -------------------------------------------*/
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

#if defined( _DEBUG )
    ( void )printf( "devuart.c : %d\r\n", __LINE__ );
#endif
    
    /* -1. Reset the UART semaphores on each uart_dev_open() */
    ret = initialiseUartSemaphore( );
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

    /* 2.6 Flush UART1 FIFOs and leave FIFOs enabled */
    UART1_FCTL =( unsigned char )( UART_FCTL_FIFOEN );                    // First enable the FIFO
    UART1_FCTL =( unsigned char )( UART_FCTL_CLRTxF | UART_FCTL_FIFOEN ); // Flush transmit hardware FIFO.
    UART1_FCTL =( unsigned char )( UART_FCTL_CLRRxF | UART_FCTL_FIFOEN ); // Flush receive hardware FIFO

#if defined( _DEBUG )
    ( void )printf( "devuart.c : %d\r\n", __LINE__ );
#endif
    /* 2.7 Initialise Fifo Trigger Level, leaving FIFOs enabled.
           Maximise use of the FIFO hardware, to maximise task concurrency.
             Interrupt 'Receiver Data Ready' triggered when RxFIFO depth >= 
             this value.
             Interrupt 'Receiver Timeout' triggered when RxFIFO depth < this
             value AND 4 consecutive byte times pass with no further data */
    UART1_FCTL =
        ( unsigned char )(( UART_FIFO_TRIGLVL_14 << 6 )| UART_FCTL_FIFOEN );

    /* 2.8 Initialise Interrupt Enable Register */
        ier =( unsigned char )( UART_IER_RECEIVEINT | 
                                UART_IER_TRANSMITINT |
                                UART_IER_LINESTATUSINT |
                                UART_IER_TRANSCOMPLETEINT );
    if(( DEV_MODE_UART_HALF_MODEM & mode )||
       ( DEV_MODE_UART_HALF_NULL_MODEM & mode )||
       ( DEV_MODE_UART_FULL_MODEM & mode )||
       ( DEV_MODE_UART_FULL_NULL_MODEM & mode ))
    {
        ier |=( unsigned char ) UART_IER_MODEMINT;
    }
#if defined( _DEBUG )
    ( void )printf( "devuart.c : %d\r\n", __LINE__ );
#endif
    UART1_IER = ier;
#if defined( _DEBUG )
    ( void )printf( "devuart.c : %d\r\n", __LINE__ );
#endif

    /* 2.9 Assert /DTR to signal we are ready for transceiving */
    if(( DEV_MODE_UART_FULL_MODEM & mode )||
       ( DEV_MODE_UART_FULL_NULL_MODEM & mode ))
    {
        UART1_MCTL |= UART_MCTL_DTR;
    }
    
uart_dev_open_end:
#if defined( _DEBUG )
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
    UART1_IER = 0x00;  // Disable UART1 interrupts
    UART1_LCTL = 0x00; // Bring line control register to reset value.
    UART1_MCTL = 0x00; // Bring modem control register to reset value.
                       // This clears /DTR to signal we are not ready to transceive

    /* 2 Flush UART1 FIFOs and disable FIFOs */
    UART1_FCTL =( unsigned char )( UART_FCTL_FIFOEN );                    // First enable the FIFO
    UART1_FCTL =( unsigned char )( UART_FCTL_CLRTxF | UART_FCTL_FIFOEN ); // Flush transmit hardware FIFO.
    UART1_FCTL =( unsigned char )( UART_FCTL_CLRRxF | UART_FCTL_FIFOEN ); // Flush receive hardware FIFO
    UART1_FCTL =( unsigned char )0x00; // Bring FIFO control register to reset value.    

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
                void * const buffer,              // IN
                size_t const num_bytes_to_read,   // IN
                size_t * num_bytes_read,          // OUT
                POSIX_ERRNO *res                  // OUT
            )
{
    TickType_t const timeout =( configTICK_RATE_HZ * 10 );  // arbitarily chose 10s delay
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    size_t stored;

    if( configDRV_UART_BUFFER_SZ >= num_bytes_to_read )
    {
        portENTER_CRITICAL( );

        if( UART_STATE_READY == uartState )
        {
            if(( NULL != num_bytes_read )&&( NULL != res ))
            {
                /* DEV_MODE_BUFFERED mode: the calling task will not block.
                    It will return immediately any content from rxPrivBuf */
                *num_bytes_read = storeRxBuffer( buffer, num_bytes_to_read );
                *res = POSIX_ERRNO_ENONE | uEvent;
                uEvent = 0;

                portEXIT_CRITICAL( );
            }
            else            
            {
                /* In DEV_MODE_UNBUFFERED mode, the calling task is suspended 
                   until the UART transfer completes or an error happens, when 
                   the ISR will wake the calling task. Other transactions will
                   be refused until the read completes. */
                uartState = UART_STATE_READ;
                portEXIT_CRITICAL( );

                CREATE_UART_TRANSACTION( 
                    rxTransact, DEV_MODE_UNBUFFERED, 
                    num_bytes_to_read, num_bytes_read, res );

                if( pdTRUE == xSemaphoreTake( uartSemaphore, timeout ))
                {
                    // calling task should check callback result
                    ret = POSIX_ERRNO_EALREADY;
                }
                else
                {
                    ret = POSIX_ERRNO_ETIMEDOUT;
                }

                ret |= uEvent;

                stored = storeRxBuffer( buffer, rxTransact.len );
                CLEAR_UART_TRANSACTION( rxTransact );

                if( num_bytes_read )
                {
                    *num_bytes_read = stored;
                }
                if( res )
                {
                    *res = ret;
                }

                uartState = UART_STATE_READY;
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
     write is in progress; whereas in DEV_MODE_BUFFERED it will return without
     waiting.
     It is then up to the application to poll the result.
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
    TickType_t const timeout =( configTICK_RATE_HZ * 10 );
    POSIX_ERRNO ret;
    unsigned char numTxBytes;
    unsigned char lsr;
    unsigned int const modeB =(( NULL != num_bytes_written )&&( NULL != res ))
                               ? DEV_MODE_BUFFERED
                               : DEV_MODE_UNBUFFERED;

    if( configDRV_UART_BUFFER_SZ >= num_bytes_to_write )
    {
        portENTER_CRITICAL( );

        if( UART_STATE_READY == uartState )
        {
            uartState = UART_STATE_WRITE;
            portEXIT_CRITICAL( );

            /* If we're in half duplex mode then test if a read is active;
               if so return EWOULDBLOCK */
            if(( DEV_MODE_UART_HALF_MODEM & pinmode[ MINOR_NUM ])||
               ( DEV_MODE_UART_HALF_NULL_MODEM & pinmode[ MINOR_NUM ]))
            {
                lsr = UART1_LSR;
                if( UART_LSR_DR & lsr )
                {
                    ret = POSIX_ERRNO_EWOULDBLOCK;
                    goto uart_dev_write_1;
                }
            }
            
            ret = assertRTSandWaitCTS( );
            if( POSIX_ERRNO_EPROTO == ret )
            {
                goto uart_dev_write_1;
            }

            CREATE_UART_TRANSACTION( 
                txTransact, modeB, num_bytes_to_write, num_bytes_written, res );

            loadTxBuffer(( unsigned char * )buffer, num_bytes_to_write );

                // kick off the write transaction
            _uart_errno = transmitNextBlock( &numTxBytes );
            txTransact.lenTRx += numTxBytes;

            if( modeB )
            {
                /* In DEV_MODE_BUFFERED mode, the calling task will run immediately.
                   The application must poll the driver to get the write result. */
                ret = POSIX_ERRNO_EINPROGRESS;
            }
            else            
            {
                /* In DEV_MODE_UNBUFFERED mode, the calling task is suspended 
                   until the UART transfer completes or an error happens, when 
                   the calling task will resume. */
                    // xSemaphoreTake cannot timeout indefinitely;
                    // so arbitarily chose 10s delay
                if( pdTRUE == xSemaphoreTake( uartSemaphore, timeout ))
                {
                    // calling task should check callback result
                    ret = POSIX_ERRNO_EALREADY;
                }
                else
                {
                    ret = POSIX_ERRNO_ETIMEDOUT;
                }

                ret |= uEvent;
                finaliseTransmit( ret );
                uEvent = 0;
            }

uart_dev_write_1:
            uartState = UART_STATE_READY;
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

    return( ret );
}


#endif  /* 1 == configUSE_DRV_UART */
