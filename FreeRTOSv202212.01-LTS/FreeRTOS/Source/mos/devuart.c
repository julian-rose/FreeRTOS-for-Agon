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
/* NOTE 1: Wiring Modes and Use Cases
 *
 * Refer to https://en.wikipedia.org/wiki/RS-232#3-wire_and_5-wire_RS-232 for
 *   DTE<->DCE "modem" wiring
 *     DTE is Data Terminal Equipment: Agon, PC, or other end-point device
 *     DCE is Data Communication Equipment, "bridge", "modem", "router", "switch"
 * And http://www.nullmodem.com/NullModem.htm for DTE<->DTE "NULL modem" wiring.
 * Also https://en.wikipedia.org/wiki/Null_modem#Wiring_diagrams
 *
 * DEV API can be opened any one of six modes, for a combination of:
 *    Non-blocking / Blocking with Straight-through / Cross-Over wiring.
 *    Each mode is bit-or'ed either with DEV_MODE_UNBUFFERED for task Blocking
 *    mode, or with DEV_MODE_BUFFERED for task Non-blocking mode, as follows:
 *
 *    DEV_MODE_UART_NO_FLOWCONTROL
 *    DEV_MODE_UART_SW_FLOWCONTROL
 *      Use Case:
 *        'No Modem' direct DTE<->DTE data transfer; low capacitance (short wires)
 *      Wiring: 
 *        DTE1.Tx  --> DTE2.Rx    cross-over
 *        DTE1.Rx  <-- DTE2.Tx
 *      Protocol:
 *        There is no DTE-DTE set-up; rather DTEs are 'always on'. 
 *        Transmission and Reception are not controlled by hardware status, and
 *         are simultaneously possible. 
 *        With DEV_MODE_UART_NO_FLOWCONTROL, there are no end-to-end flow
 *         control command bytes. This incurs zero overhead, but the receive
 *         buffer may overflow and lose data.
 *        With DEV_MODE_UART_SW_FLOWCONTROL, Xon / Xoff flow control command 
 *         bytes can be sent in-stream from either end to the other. These
 *         are used to put the sender on pause to avoid buffer overflow. This
 *         incurs overhead as the receiver has to parse every byte received,
 *         but is less likely to result in data loss.
 *        UART1_MCTL is fixed at 0x00, with /RTS and /DTR de-asserted (high)
 *         There should be no UART_IIR_MODEMSTAT events.
 *
 *    DEV_MODE_UART_HALF_NULL_MODEM 
 *      Use Case:
 *        'No Modem' direct DTE<->DTE signalling; low capacitance (short wires)
 *      Wiring: 
 *        DTE1.Tx  --> DTE2.Rx 
 *        DTE1.Rx  <-- DTE2.Tx
 *        DTE1.RTS --> DTE2.CTS   cross-over
 *        DTE1.CTS <-- DTE2.RTS
 *      Protocol:
 *        There is no DTE-DTE set-up; rather DTEs are 'always on'. 
 *        Reception is always possible; transmission is by /RTS-/CTS handshake.
 *        With DEV_MODE_UART_HALF_MODEM, hardware takes the place of in-stream
 *         Xon / Xoff software command bytes. This exchanges the overhead of in-
 *         stream data byte parsing for managing hardware signalling. The 
 *         liklihood of data loss is comparable to that of Xon / Xoff.
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
 *        On transmission completion or abandon, /RTS must be asserted (low),
 *          so that reception remains possible.
 *
 *    DEV_MODE_UART_FULL_NULL_MODEM
 *      Use Case:
 *        'No Modem' direct DTE<->DTE signalling; low capacitance (short wires)
 *      Wiring: 
 *        DTE1.Tx  --> DTE2.Rx
 *        DTE1.Rx  <-- DTE2.Tx
 *        DTE1.RTS --> DTE2.CTS   cross-over
 *        DTE1.CTS <-- DTE2.RTS
 *        DTE1.DTR --> DTE2.DCD
 *              '----> DTE2.DSR
 *        DTE1.DSR <-- DTE2.DTR
 *        DTE1.DCD <----'
 *      Protocol:
 *        The DTE end-points handshake connection set-up and take-down through  
 *         the additional /DTR, /DSR and /DCD wires; 'switched on'.
 *        Once the connection is 'set-up', reception is always possible and
 *         transmission is by /RTS-/CTS handshake.
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
 *        If during transmission either /DCD or /DSR become de-asserted (high), 
 *          then transmission must be abandoned.
 *        On transmission completion or abandon, /RTS must be asserted (low)
 *
 *
 * NOTE 1a: DEV_MODE_UART_HALF_MODEM and DEV_MODE_UART_FULL_MODEM require
 *          external "modem", "bridge", "router", or "switch" DCE. 
 *          Note DEV UART does not emulate a modem.
 *
 * NOTE 1b: DEV_MODE_UART_HALF_MODEM has not been extensively tested, but 
 *          functions exactly as per DEV_MODE_HALF_NULL_MODEM. 
 *
 *    DEV_MODE_UART_HALF_MODEM
 *      Use Case:
 *        Both DTE<->DCE and DTE<->DTE control flow signalling; long-distance 
 *        wiring.
 *      Wiring: 
 *        DTE1.Tx  --> DCE1.Tx
 *        DTE1.Rx  <-- DCE1.Rx
 *        DTE1.RTS --> DCE1.RTS   straight-through
 *        DTE1.CTS <-- DCE1.CTS
 *      Protocol:
 *        There is no DTE-DTE set-up; rather DTEs are 'always on'. (DTE knows
 *         nothing about the connection state of the DCE 'modem', which may
 *         be 'always on' or 'woken up'.)
 *        Reception is always possible; transmission is by /RTS-/CTS handshake.
 *        In addition to DTE endpoint /RTS - /CTS control-flow management, the 
 *         'modems' or 'bridges' that sit in-between the DTE end-points may 
 *         manage the control flow (by de-asserting /CTS) depending on signal 
 *         quality, further reducing chance of data loss. 
 *        /RTS (Request To Send) shall be asserted (low) when the DTE is ready
 *          to TRANSMIT.
 *        Prior to transmission, /RTS must be asserted (low) and /CTS assertion 
 *          tested (low). 
 *        If during transmission /CTS becomes de-asserted (high), then 
 *          transmission must pause, and LCTL.SB set to indicate a break. 
 *          Once /CTS is re-asserted (low) SB shall be cleared and transmission 
 *          resumed.
 *        On transmission completion or abandon, /RTS must be de-asserted (high)
 *
 *
 * NOTE 1c: DEV_MODE_UART_FULL_MODEM has not been tested. This is the only
 *          mode that requires application task involvement in UART signalling:
 *          for connection set-up in a switched network (such as ATDT dialling).
 *
 *    DEV_MODE_UART_FULL_MODEM 
 *      Use Case:
 *        DTE<->DCE switching; and DTE<->DTE control flow signalling; long-
 *        distance wiring.
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
 *        The DTE handshakes connection set-up and take-down with its local DCE 
 *         'modem' through the additional /DTR, /DSR, /DCD and /RI wires. This 
 *         differs to the fixed point-to-point connection models of other modes
 *         by allowing switched networks with multiple end-points. 
 *        /DTR shall be asserted (low) when the DTE wishes to write or when an 
 *         incoming /RI is indicated; either of which initiate 'call set-up'. 
 *         /DTR shall be de-asserted on transmission end, or when one of /DSR 
 *         or /DCD is de-asserted which initiate 'call take-down'.
 *        Once a connection is set-up, reception is always possible; 
 *         transmission is by handshake.
 *        Reception is possible with /RTS asserted (low) or de-asserted (high).
 *        /RTS (Request To Send) shall be asserted (low) when the DTE is ready
 *          to TRANSMIT.
 *        Prior to transmission, /RTS must be asserted (low) and /CTS assertion 
 *          tested (low). 
 *        If during transmission /CTS becomes de-asserted (high), then 
 *          transmission must pause, and LCTL.SB set to indicate a break. 
 *          Once /CTS is re-asserted (low) SB shall be cleared and transmission 
 *            resumed.
 *        If during transmission either /DCD or /DSR become de-asserted (high), 
 *          then transmission must be abandoned.
 *        On transmission completion or abandon, /RTS must be de-asserted (high).
 *
 * NOTE 1d: Connection set-up
 *    Each 'Modem' defines its own connection setup language and protocol (such
 *      as Hayes "AT" commands, or TCP/IP packets), so it is not practical to 
 *      embed in DEV UART. Rather, DEV UART supports connection setup as 
 *      follows:
 *        1. uart_open in DEV_MODE_UART_FULL_MODEM (or DEV_MODE_UART_LOOPBACK)
 *        2i. Initiate a connection:
 *           a. uart_ioctl DEV_IOCTL_UART_WRITE_MODEM param=1
 *           b. uart_ioctl DEV_IOCTL_UART_SET_DTR param=1
 *           c. uart_ioctl DEV_IOCTL_UART_GET_DSR until param = 1
 *           d. uart_write "connect request data" (iterate)
 *           e. uart_read "modem confirmation data" (iterate)
 *           f. uart_ioctl DEV_IOCTL_UART_GET_DCD until param = 1
 *           g. uart_ioctl DEV_IOCTL_UART_WRITE_MODEM param=0
 *        3. uart_read and uart_write the remote end-point
 *        4. [optional] Drop the connection:
 *           a. uart_ioctl DEV_IOCTL_UART_WRITE_MODEM param=1
 *           b. uart_write "disconnect request data" (iterate)
 *           c. uart_read "modem confirmation" (iterate)
 *           d. uart_ioctl DEV_IOCTL_UART_WRITE_MODEM param=0
 *        5. uart_close, which performs the following:
 *           a. uart_ioctl DEV_IOCTL_UART_SET_DTR param=0
 *           b. uart_ioctl DEV_IOCTL_UART_GET_DCD until param = 0
 *           c. uart_ioctl DEV_IOCTL_UART_GET_DSR until param = 0
 * 
 *        2ii. Respond to a call:
 *           a. uart_ioctl DEV_IOCTL_UART_GET_RI if param = 1
 *           b. uart_ioctl DEV_IOCTL_UART_WRITE_MODEM param=1
 *           c. uart_ioctl DEV_IOCTL_UART_SET_DTR param=1
 *           d. uart_ioctl DEV_IOCTL_UART_GET_DSR until param = 1
 *           e. uart_read "modem indication data" (iterate)
 *           f. uart_write "connect response data" (iterate)
 *           g. uart_ioctl DEV_IOCTL_UART_GET_DCD until param = 1
 *           h. uart_ioctl DEV_IOCTL_UART_WRITE_MODEM param=0
 *
 *-----------------------------------------------------------------------------
 * NOTE 2: eZ80 IIR handling
 *
 * If eZ80 IER is masked out then the corresponding bits in IIR are cleared at
 *     the same time. So, sadly, masking such as:
 *         ier_mask = UART1_IER;
 *         UART1_IER = 0x0;  // mask off IER until IIR is processed
 *     is not possible. And if you exit an ISR with bits still set in IIR, then 
 *     the ISR will immediately re-enter. So, it is not possible to notify a 
 *     highest-priority task to run as an Interrupt Handler (such as follows). 
 *     Instead, all interrupt handling must be done within the ISR context, 
 *     which means other interrupts are disabled for the duration of executing 
 *     uartisr_1. Sigh.
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
 * /* uartisr_0 - address written into the interrupt vector table
 *      Standard ISR reti epilogue * /
 * static void uartisr_0( void )
 * {
 *     BaseType_t __higherPriorityTaskWokenUART;
 *     
 * #   if( 1 == configUSE_PREEMPTION )
 *     {
 *         __higherPriorityTaskWokenUART = pdFALSE;
 * 
 *         /* run in context of high-priority interrupt handler UartIsrHandlerTask.
 *              Other hardware interrupts may be raised concurrently.
 *              If the uart isr handling isn't served quickly enough, you can
 *               - either increase the uart handler task priority, 
 *               - or change ( 1 == configUSE_PREEMPTION ) to 0 in order to call 
 *                 uartisr_1 directly from uartisr_0 * /
 *         _putchf( 'V' );
 *         vTaskNotifyGiveFromISR( UartIsrHandlerTaskHandle, &__higherPriorityTaskWokenUART );
 * 
 *         if( pdTRUE == __higherPriorityTaskWokenUART ) _putchf( 'Y' );
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
 *     /* If __higherPriorityTaskWokenUART is now set to pdTRUE then a context 
 *        switch should be performed to ensure the interrupt returns directly 
 *        to the highest priority task. 
 *        This must be the last function before reti. Refer to port.c::timer_isr * /
 *     vPortYieldFromISR( __higherPriorityTaskWokenUART );
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
 * NOTE 3: initial DEV UART State design (details changed during implementation)
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
        ( _t ).lenTRx )
#endif


#define MODEM_TXD_OFF( )    modemStatus.txd = 0
#define MODEM_TXD_ON( )     modemStatus.txd = 1
#define MODEM_RXD_OFF( )    modemStatus.rxd = 0
#define MODEM_RXD_ON( )     modemStatus.rxd = 1
#define MODEM_RTS_OFF( )    modemStatus.rts = 0
#define MODEM_RTS_ON( )     modemStatus.rts = 1
#define MODEM_CTS_OFF( )    modemStatus.cts = 0
#define MODEM_CTS_ON( )     modemStatus.cts = 1
#define MODEM_DTR_OFF( )    modemStatus.dtr = 0
#define MODEM_DTR_ON( )     modemStatus.dtr = 1
#define MODEM_DSR_OFF( )    modemStatus.dsr = 0
#define MODEM_DSR_ON( )     modemStatus.dsr = 1
#define MODEM_DCD_OFF( )    modemStatus.dcd = 0
#define MODEM_DCD_ON( )     modemStatus.dcd = 1
#define MODEM_RI_OFF( )     modemStatus.ri = 0
#define MODEM_RI_ON( )      modemStatus.ri = 1

#define testCTS( ) \
            ( UART1_MSR & UART_MSR_CTS )

#define testDSR( ) \
            ( UART1_MSR & UART_MSR_DCD )

#define testDCD( ) \
            ( UART1_MSR & UART_MSR_DCD )

#define testRI( ) \
            ( UART1_MSR & UART_MSR_RI )


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
    UART_STATE_INITIAL   =( 0<<0 ),  // before uart_open or after uart_close
    UART_STATE_READY     =( 1<<0 ),  // after uart_open and before uart_close
    UART_STATE_READ      =( 1<<1 ),  // read controls rx data byte storage
    UART_STATE_READLOCK  =( 1<<2 ),  // readlock ensures at most one reader
    UART_STATE_WRITE     =( 1<<3 ),  // write controls tx data operation
    UART_STATE_WRITELOCK =( 1<<4 ),  // writelock ensures at most one writer
    UART_STATE_PAUSE     =( 1<<5 )   // In Xoff state with software handshaking

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

#define RX_CIRCULAR_BUFFER_NEARLY_FULL     /* few spaces free */  \
        ( configDRV_UART_BUFFER_SZ >> 3 )

#define RX_CIRCULAR_BUFFER_NEARLY_EMPTY    /* many spaces free */ \
        ( configDRV_UART_BUFFER_SZ - RX_CIRCULAR_BUFFER_NEARLY_FULL )


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
    size_t len;               // number of bytes to read/write             IN
    size_t lenTRx;            // local tracker of bytes transceived      TEMP
    size_t *bufTRx;           // destination buffer for lenTRx            OUT
    POSIX_ERRNO *bufRes;      // destination buffer for result            OUT
    
} UART_TRANSACTION;


/*---- Global Variables -----------------------------------------------------*/
static SemaphoreHandle_t uartRxSemaphore = NULL; // for non-buffered blocking-mode user task
static SemaphoreHandle_t uartTxSemaphore = NULL; // for non-buffered blocking-mode user task

static UART_STATE uartState = UART_STATE_INITIAL;
static UART_MODEM_STATUS modemStatus ={ 0 };
static _Bool connection_establishment = false;

static UART_BUFFER txPrivBuf ={ 0, 0,{ 0 }};     // a linear buffer, tx to remote
static UART_BUFFER rxPrivBuf ={ 0, 0,{ 0 }};     // a circular buffer, rx from remote
static _Bool txPending = false;                  // Xon buffer tx side
static _Bool rxPaused = false;                   // Xon buffer rx side

static UART_TRANSACTION txTransact ={ 0, NULL, 0, 0, NULL, NULL };
static UART_TRANSACTION rxTransact ={ 0, NULL, 0, 0, NULL, NULL };

static void( *prevUartISR )( void )= NULL;       // MOS ISR for re-attach on uart_close

static unsigned short uartRxEvent = 0;           // buffer for rx function results
static unsigned short uartTxEvent = 0;           // buffer for tx function results

static BaseType_t __higherPriorityTaskWokenUART; // set in ISR to context-switch


/*---- Private Function Declarations ----------------------------------------*/
static void uart_rxFlowControl( void );


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
/* DTRready
     Set /DTR output to its 'asserted' value, that is value 0.
     Only used in FULL (or FULL_NULL, or Loopback) wiring.
*/
static void DTRready( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =
        ( UART_MCTL_LOOP | UART_MCTL_OUT1 | UART_MCTL_OUT2 );
    unsigned char v;

#   if defined( configTRACE )
        _putchf( 'h' );
#   endif

    if(( DEV_MODE_UART_FULL_MODEM & wiring )||  // FULL or FULL_NULL MODEM
       ( DEV_MODE_UART_LOOPBACK == wiring ))
    {
#       if defined( configTRACE )
            _putchf( '1' );
#       endif

        v = UART1_MCTL &( mlo | UART_MCTL_RTS );   // leave /RTS intact
        UART1_MCTL =( v | UART_MCTL_DTR );         // set /DTR
        MODEM_DTR_ON( );
    }
}


/* DTRnotReady
     Set /DTR output to its 'de-asserted' value, that is value 1.
     Only used in FULL (or FULL_NULL, or Loopback) wiring
*/
static void DTRnotReady( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =
        ( UART_MCTL_LOOP | UART_MCTL_OUT1 | UART_MCTL_OUT2 );
    unsigned char v;

#   if defined( configTRACE )
        _putchf( 'i' );
#   endif

    if(( DEV_MODE_UART_FULL_MODEM & wiring )||  // FULL or FULL_NULL MODEM
       ( DEV_MODE_UART_LOOPBACK == wiring ))
    {
#       if defined( configTRACE )
            _putchf( '1' );
#       endif

        v = UART1_MCTL &( mlo | UART_MCTL_RTS );   // leave /RTS intact
        UART1_MCTL = v;                            // clear /DTR
        MODEM_DTR_OFF( );
    }
}


/* waitDSR
     Data Set Ready is an input, set by the modem (or by mctl.DTR in 
       loopback wiring.)
     Can only be called within a task context, not ISR: use of vTaskDelay
*/
static POSIX_ERRNO waitDSR( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    MODEM_DSR_OFF( );
    if(( DEV_MODE_UART_FULL_MODEM & wiring )||  // FULL or FULL_NULL MODEM
       ( DEV_MODE_UART_LOOPBACK == wiring ))
    {
#       if defined( configTRACE )
            _putchf( 'n' );
#       endif

        if( 0 == testDSR( ))
        {
            TickType_t const ms100 = configTICK_RATE_HZ / 10;

#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            /* The logic here is we are prepared to wait a short period for CTS
               to be asserted following our RTS assertion. We chose 100mS
               arbitarily - perhaps we could have a devConfig setting? It would
               also be better to start a timer and poll MSR, giving up on 
               timerout. */
            vTaskDelay( ms100 );       // wait 100mS for /DSR to be asserted
            if( 0 == testDSR( ))
            {
#               if defined( configTRACE )
                    _putchf( '2' );
#               endif

                ret = POSIX_ERRNO_EPROTO;  // no /CTS from the remote end
            }
        }

        if( POSIX_ERRNO_ENONE == ret )
        {
            MODEM_DSR_ON( );
        }
    }

    return( ret );
}


/* RTSforSend
     For straight-through (Modem) wiring, we set /RTS (low) to signal 
       "Request To Send" prior to transmission. 
     For cross-over (NULL modem) wiring, we set /RTS (high) to clear
       "Remote To Send" (clears remote Clear To Send) 
*/
static void RTSforSend( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =
        ( UART_MCTL_LOOP | UART_MCTL_OUT1 | UART_MCTL_OUT2 );
    unsigned char v;

    // check if we're using software or hardware flow control
#   if defined( configTRACE )
        _putchf( 'j' );
#   endif

    if( 0 ==(( DEV_MODE_UART_NO_FLOWCONTROL | DEV_MODE_UART_SW_FLOWCONTROL )&
              wiring ))
    {
        if(( DEV_MODE_UART_HALF_MODEM == wiring )||
           ( DEV_MODE_UART_FULL_MODEM == wiring ))
        {                                 // straight-through wiring
#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            v = UART1_MCTL &( mlo | UART_MCTL_DTR );   // leave /DTR intact
            UART1_MCTL =( v | UART_MCTL_RTS );         // set /RTS
            MODEM_RTS_ON( );
        }
        else
        if(( DEV_MODE_UART_HALF_NULL_MODEM == wiring )||
           ( DEV_MODE_UART_FULL_NULL_MODEM == wiring ))
        {                                 // crossover wiring
#           if defined( configTRACE )
                _putchf( '2' );
#           endif

            v = UART1_MCTL &( mlo | UART_MCTL_DTR );   // leave /DTR intact
            UART1_MCTL = v;                            // clear /RTS
            MODEM_RTS_OFF( );
        }
    }
}


/* RTSforReceive
     For straight-through (Modem) wiring, we set /RTS (high) to clear
       "Request To Send"
     For cross-over (NULL modem) wiring, we set /RTS (low) to set
       "Remote To Send" (sets remote Clear To Send) 
     Called from within ISR.
*/
static void RTSforReceive( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =
        ( UART_MCTL_LOOP | UART_MCTL_OUT1 | UART_MCTL_OUT2 );
    unsigned char v;

    if( 0 ==(( DEV_MODE_UART_NO_FLOWCONTROL | DEV_MODE_UART_SW_FLOWCONTROL )&
              wiring ))
    {
#       if defined( configTRACE )
            _putchf( 'k' );
#       endif

        if(( DEV_MODE_UART_HALF_MODEM == wiring )||
           ( DEV_MODE_UART_FULL_MODEM == wiring ))
        {                               // straight-through wiring
#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            v = UART1_MCTL &( mlo | UART_MCTL_DTR );   // leave /DTR intact
            UART1_MCTL = v;                            // clear /RTS
            MODEM_RTS_OFF( );
        }
        else
        if(( DEV_MODE_UART_HALF_NULL_MODEM == wiring )||
           ( DEV_MODE_UART_FULL_NULL_MODEM == wiring ))
        {                               // crossover wiring
#           if defined( configTRACE )
                _putchf( '2' );
#           endif

            v = UART1_MCTL &( mlo | UART_MCTL_DTR );   // leave /DTR intact
            UART1_MCTL =( v | UART_MCTL_RTS );         // set /RTS 
            MODEM_RTS_ON( );
        }
    }
}


/* waitCTS
     Clear To Send is an input, set by the remote end (or by /RTS in loopback
       wiring)
     Can only be called within a task context, not ISR: use of vTaskDelay
*/
static POSIX_ERRNO waitCTS( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    MODEM_CTS_OFF( );
    if( 0 ==(( DEV_MODE_UART_NO_FLOWCONTROL | DEV_MODE_UART_SW_FLOWCONTROL )&
              wiring ))
    {
#       if defined( configTRACE )
            _putchf( 'l' );
#       endif

        if( 0 == testCTS( ))
        {
            TickType_t const ms100 = configTICK_RATE_HZ / 10;

#           if defined( configTRACE )
                _putchf( 'l' );
#           endif

            /* The logic here is we are prepared to wait a short period for CTS
               to be asserted following our RTS assertion. We chose 100mS
               arbitarily - perhaps we could have a devConfig setting? It would
               also be better to start a timer and poll MSR, giving up on 
               timerout. */
            vTaskDelay( ms100 );       // wait 100mS for /CTS to be asserted
            if( 0 == testCTS( ))
            {
#               if defined( configTRACE )
                    _putchf( '2' );
#               endif

                ret = POSIX_ERRNO_EPROTO;  // no /CTS from the remote end
            }
        }

        if( POSIX_ERRNO_ENONE == ret )
        {
            MODEM_CTS_ON( );
        }
    }

    return( ret );
}


/* DCDloopReady
     In loopback mode, set /DCD output to its 'true' value, that is value 0
*/
static void DCDloopReady( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =( UART_MCTL_LOOP | UART_MCTL_OUT1 );
    unsigned char v;

    /* check if we're using full modem (null or straight-through) hardware flow 
       control */
#   if defined( configTRACE )
        _putchf( 'l' );
#   endif

    if( DEV_MODE_UART_LOOPBACK & wiring )
    {
#       if defined( configTRACE )
            _putchf( '1' );
#       endif

        v = UART1_MCTL &( mlo | UART_MCTL_RTS | UART_MCTL_DTR );
        UART1_MCTL =( v | UART_MCTL_OUT2 );   // set /DCD via OUT2 in loopback
        MODEM_DCD_ON( );
    }
}


/* DCDloopNotReady
     In loopback mode de-assert /DCD output to its 'false' value, that is value 1
*/
static void DCDloopNotReady( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =( UART_MCTL_LOOP | UART_MCTL_OUT1 );
    unsigned char v;

    /* check if we're using full modem (null or straight-through) hardware flow 
       control */
#   if defined( configTRACE )
        _putchf( 'i' );
#   endif

    if( DEV_MODE_UART_LOOPBACK & wiring )  // FULL or FULL_NULL
    {
#       if defined( configTRACE )
            _putchf( '1' );
#       endif

        v = UART1_MCTL &( mlo | UART_MCTL_RTS | UART_MCTL_DTR );
        UART1_MCTL = v;                      // clear /DCD through out2 in loopback
        MODEM_DCD_OFF( );
    }
}


/* waitDCD
     Data Carrier Detect is an input, set by the modem (or by mctl.out2 in 
       loopback wiring.) It's usual meaning is 'local modem - remote modem 
       connection established'. 
     Can only be called within a task context, not ISR: use of vTaskDelay
*/
static POSIX_ERRNO waitDCD( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    MODEM_DCD_OFF( );
    if(( DEV_MODE_UART_FULL_MODEM & wiring )||  // FULL or FULL_NULL MODEM
       ( DEV_MODE_UART_LOOPBACK == wiring ))
    {
#       if defined( configTRACE )
            _putchf( 'm' );
#       endif

        if( 0 == testDCD( ))
        {
            TickType_t const ms100 = configTICK_RATE_HZ / 10;

#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            /* The logic here is we are prepared to wait a short period for CTS
               to be asserted following our RTS assertion. We chose 100mS
               arbitarily - perhaps we could have a devConfig setting? It would
               also be better to start a timer and poll MSR, giving up on 
               timerout. */
            vTaskDelay( ms100 );       // wait 100mS for /DCD to be asserted
            if( 0 == testDCD( ))
            {
#               if defined( configTRACE )
                    _putchf( '2' );
#               endif

                ret = POSIX_ERRNO_EPROTO;  // no /CTS from the remote end
            }
        }

        if( POSIX_ERRNO_ENONE == ret )
        {
            MODEM_DCD_ON( );
        }
    }

    return( ret );
}


/* RIloopRing
     In loopback mode, set /RI input to its 'true' value, that is value 0
*/
static void RIloopRing( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =( UART_MCTL_LOOP | UART_MCTL_OUT2 );
    unsigned char v;

    /* check if we're using full modem (null or straight-through) hardware flow 
       control */
#   if defined( configTRACE )
        _putchf( 'l' );
#   endif

    if( DEV_MODE_UART_LOOPBACK & wiring )
    {
#       if defined( configTRACE )
            _putchf( '3' );
#       endif

        v = UART1_MCTL &( mlo | UART_MCTL_RTS | UART_MCTL_DTR );
        UART1_MCTL =( v | UART_MCTL_OUT1 );   // set /RI via OUT1 in loopback
        MODEM_RI_ON( );
    }
}


/* RIloopSilent
     In loopback mode de-assert /RI input to its 'false' value, that is value 1
*/
static void RIloopSilent( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char const mlo =( UART_MCTL_LOOP | UART_MCTL_OUT2 );
    unsigned char v;

    /* check if we're using full modem (null or straight-through) hardware flow 
       control */
#   if defined( configTRACE )
        _putchf( 'l' );
#   endif

    if( DEV_MODE_UART_LOOPBACK & wiring )  // FULL or FULL_NULL
    {
#       if defined( configTRACE )
            _putchf( '4' );
#       endif

        v = UART1_MCTL &( mlo | UART_MCTL_RTS | UART_MCTL_DTR );
        UART1_MCTL = v;                      // clear /RI through out1 in loopback
        MODEM_RI_OFF( );
    }
}


/*------  Private UART Setup functions --------------------------------------*/
/* uartResetFIFO
     Initialise UART FIFO (FCNTL register).
     Parameters: 
       mask is a bitor: UART_FCTL_CLRTxF to reset the Tx FIFO bitor
                        UART_FCTL_CLRRxF to reset the Rx FIFO.
         (We assume disabling is distinct from resetting either fifo pipe.)
       op is either UART_FIFO_DISABLE or UART_FIFO_ENABLE both fifos
     (While this appears to initialise fine, refer to Zilog UP004909 Errata,
      in case of Issue 7 - continuous Rx interrupts.) */
static void uartResetFIFO( 
                unsigned char const mask, 
                UART_FIFO_OPERATION const op )
{
    /* A. Flush tx and/or rx UART1 FIFO(s) */
    UART1_FCTL =( unsigned char )0x00;                        // 1. Disable the FIFO
    UART1_FCTL =( unsigned char )(        UART_FCTL_FIFOEN ); // 2. Enable FIFO
    UART1_FCTL =( unsigned char )( mask | UART_FCTL_FIFOEN ); // 3. Masked FIFO reset

    UART1_FCTL =( unsigned char )0x00;                        // 1. disable the FIFO

    /* B. Enable both Rx and Tx FIFOs */
    if( UART_FIFO_ENABLE == op )
    {
        /* Initialise Rx Fifo Trigger Level, leaving both Rx and Tx FIFOs enabled.
            Maximise use of the FIFO hardware, to maximise FreeRTOS concurrency.
            'Receiver Data Ready' triggered when RxFIFO depth >= this value.
            'Receiver Timeout' triggered when RxFIFO depth < this value AND 4 
               consecutive byte times pass with no further data. */
        UART1_FCTL =( unsigned char )(  UART_FCTL_FIFOEN );   // 2. Enable FIFO
        UART1_FCTL =( unsigned char )(( UART_FIFO_TRIGLVL_14 << 6 )|
                                        UART_FCTL_FIFOEN  );  // 3. Set rx trigger level
    }
}


/* uartEnableTxInterrupts
     enable interrupts for a tx transaction 
     Only do this after the Tx FIFO is loaded, otherwise infinite "empty"
     events result */
static void uartEnableTxInterrupts( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier;

    ier =( unsigned char )( UART_IER_TRANSMITINT | UART_IER_TRANSCOMPLETEINT );

    if(( DEV_MODE_UART_HALF_MODEM | DEV_MODE_UART_HALF_NULL_MODEM |
         DEV_MODE_UART_FULL_MODEM | DEV_MODE_UART_FULL_NULL_MODEM )
       &
       wiring )
    {
        ier |=( unsigned char )UART_IER_MODEMINT;
    }

    UART1_IER |= ier;
}


/* uartDisableTxInterrupts
     disable interrupts after a tx transaction completes */
static void uartDisableTxInterrupts( unsigned char iirEvent )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier =( unsigned char )UART_IER_TRANSMITINT;

    if( UART_IIR_TRANSCOMPLETE == iirEvent )
    {
        ier |=( unsigned char )UART_IER_TRANSCOMPLETEINT;

        if( DEV_MODE_UART_HALF_MODEM & wiring )  // HALF or HALF_NULL
        {
            ier |=( unsigned char )UART_IER_MODEMINT;
        }
        else
        if( 0 == rxTransact.mode )  // if there is no active rx transaction
        {
            if( DEV_MODE_UART_FULL_MODEM & wiring )  // FULL or FULL_NULL
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
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier;

    ier =   ( unsigned char )( UART_IER_RECEIVEINT | 
                               UART_IER_LINESTATUSINT );

    if(( DEV_MODE_UART_HALF_MODEM | DEV_MODE_UART_HALF_NULL_MODEM |
         DEV_MODE_UART_FULL_MODEM | DEV_MODE_UART_FULL_NULL_MODEM )
       & wiring )
    {
        ier |=( unsigned char )UART_IER_MODEMINT;
    }

    UART1_IER |= ier;
}


/* uartDisableRxInterrupts
     disable interrupts after an rx transaction completes */
static void uartDisableRxInterrupts( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char ier;
    
    ier =     ( unsigned char )UART_IER_RECEIVEINT;

    if( DEV_MODE_UART_HALF_MODEM & wiring )   // HALF or HALF_NULL
    {
        ier |=( unsigned char )UART_IER_MODEMINT;
    }
    else
    if( 0 == txTransact.mode )  // only if there is no active tx transaction
    {
        if( DEV_MODE_UART_FULL_MODEM & wiring )  // FULL or FULL_NULL
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


/* transmitChar
     if only transmitting one char at a time FIFO is not used */
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

    if( 0 ==( UART_STATE_PAUSE & uartState ))  // in XON state
    {
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
    else
    {
        /* remember to transmit when XOFF is received */
        *numTxBytes = 0;
        txPending = true;
    }
}


/* finaliseTxTransaction
     On completion or abandonment of transmission, clear down RTS, set the
       calling task callback hooks, and erase the transaction.
*/
static void finaliseTxTransaction( POSIX_ERRNO const ret )
{
    if( false == rxPaused )
    {
        RTSforReceive( );
    }

    if( txTransact.bufTRx )
    {
        *( txTransact.bufTRx )= txTransact.lenTRx;
    }
    
    if( txTransact.bufRes )
    {
        *( txTransact.bufRes )= ret;
    }
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


/* storeRxBuffer
     rxPrivBuf is operated as a circular buffer
     memcpy may be in two parts, over the circular boundary
     Can be called from within ISR or from uart_read */
static unsigned short storeRxBuffer( void )
{
    unsigned short cnt;
    unsigned short numStored;
    unsigned short const od = rxPrivBuf.Outdex;  // start point
    unsigned short l;
    unsigned short numLeft;

    // determine how many rx data bytes we can store in user task buffer
    RX_CIRCULAR_BUFFER_NUM_OCCUPIED( numStored );
    cnt =( rxTransact.len > numStored )
          ? numStored
          : rxTransact.len;

    numLeft = cnt + od;

    //avoid calling MOS functions from within an ISR
    //printf( " <numStored = %d> <cnt = %d> <od = %d> ", numStored, cnt, od );

    if( configDRV_UART_BUFFER_SZ <= numLeft )
    {
        /* This case wraps around the circular boundary;
            first do the part at the top of the circular buffer */
        for( l = od; configDRV_UART_BUFFER_SZ > l; l++ )
        {
            *( rxTransact.buf )= rxPrivBuf._[ l ];
            ( rxTransact.buf )++;
        }
        numLeft -= configDRV_UART_BUFFER_SZ;
        /*  second do the part at the bottom of the circular buffer */
        for( l = 0; numLeft > l; l++ )
        {
            *( rxTransact.buf ) = rxPrivBuf._[ l ];
            ( rxTransact.buf )++;
        }
    }
    else
    {
        /* This case has not wrapped around;
            all content is in the middle of the circular buffer */
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


/* receiveNextBlock
     recover the next bytes from the rx fifo
     The FIFO is enabled, so up to 16 bytes can be read at once */
static UART_ERRNO receiveNextBlock( unsigned char * numRxBytes )
{
    _Bool const swFlowCtrl =
        ( DEV_MODE_UART_SW_FLOWCONTROL == pinmode[ MINOR_NUM ])
            ? true
            : false;
    UART_ERRNO res = UART_ERRNO_NONE;
    int i;
    unsigned char numB;
    unsigned char rbr;
    unsigned char numTxBytes;
    _Bool copyByte;


#   if defined( configTRACE )
        _putchf( 'g' );
#   endif

    RX_CIRCULAR_BUFFER_NUM_FREE( numB );
    numB =( RHR_FIFO_MAX < numB )
        ? RHR_FIFO_MAX
        : numB;

    *numRxBytes = 0;

    for( i = 0; numB > i; i++ )
    {
        if( UART1_LSR & UART_LSR_DR )
        {
            rbr = UART1_RBR;  // read byte from UART

            copyByte = true;
            if( true == swFlowCtrl )
            {
                if( UART_SW_FLOWCTRL_XON == rbr )
                {    // remote end buffer space available
#                   if defined( configTRACE )
                        _putchf( '1' );
#                   endif

                    copyByte = false;
                    uartState &=( UART_STATE )( ~UART_STATE_PAUSE );
                    if( true == txPending )
                    {
                        // re-start pending transmit
                        transmitNextBlock( &numTxBytes );
                        txTransact.lenTRx += numTxBytes;
                        txPending = false;
                    }
                }
                else
                if( UART_SW_FLOWCTRL_XOFF == rbr )
                {    // remote end buffer space full
#                   if defined( configTRACE )
                        _putchf( '2' );
#                   endif

                    copyByte = false;
                    uartState |= UART_STATE_PAUSE;
                }
            }
            
            if(( UART_STATE_READY & uartState )&&( true == copyByte ))
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
        raise the error condition.
        We could also get here with Xoff if the remote end transmitted many
        bytes before receiving our Xoff */
    if( UART1_LSR & UART_LSR_DR )
    {
        res = UART_ERRNO_RECEIVEFIFOFULL;
        
#       if defined( configTRACE )
            _putchf( '3' );
#       endif

        // Flush the RxFIFO??
        uartResetFIFO( UART_FCTL_CLRRxF, UART_FIFO_ENABLE );
    }

    uart_rxFlowControl( );  /* Check receive-side Xon/Xoff; 
                               should be called by Idle or Tick for best performance */

    if(( 0 == *numRxBytes )     // we received nothing
       /*( false == swFlowCtrl )*/) // except a sole Xon / Xoff control byte
    {
#       if defined( configTRACE )
            _putchf( '9' );
#       endif

        // TRIGGER LEVEL interrupt fired with nothing to read
        //   Reset the RxFIFO then enable the FIFOs
        uartResetFIFO( UART_FCTL_CLRRxF, UART_FIFO_ENABLE );
    }

#   if defined( configTRACE )
        _putchf( '0' );
#   endif

    return( res );
}


/*------- IO Control functions ----------------------------------------------*/
/* uart_rxFlowControl
     Manage Receiver-side Xon/Xoff software or RTS/CTS hardware flow control.
     Not a formal interrupt routine, but to be called by either (best) the tick 
     ISR, or (second best) the Idle Task.
     Can also be called by receiveNextBlock - but that alone may not be enough */
static void uart_rxFlowControl( void )
{
    if( UART_STATE_READY & uartState )  // is uart device open?
    {
        DEV_MODE const wiring = pinmode[ MINOR_NUM ];
        _Bool const swFlowCtrl =   // software flow control?
            ( DEV_MODE_UART_SW_FLOWCONTROL == wiring )
                ? true
                : false;
        _Bool const hwFlowCtrl =   // null modem hardware RTS/CTS flow control?
            ( DEV_MODE_UART_NULL_MODEM_MASK & wiring )
                ? true
                : false;

        if(( true == swFlowCtrl )||( true == hwFlowCtrl ))
        {
            unsigned char numB;

            RX_CIRCULAR_BUFFER_NUM_FREE( numB );
            /* If rxPrivBuf is nearly full, pause remote end transmission */
            if(( RX_CIRCULAR_BUFFER_NEARLY_FULL > numB )&&( false == rxPaused ))
            {
#               if defined( configTRACE )
                    _putchf( 'q' );
#               endif

                if( true == swFlowCtrl )
                {
                    ( void )transmitChar(( unsigned char )UART_SW_FLOWCTRL_XOFF );
                }
                else
                {
                    RTSforSend( );  // in NULL modem, disable remote CTS
                }
                rxPaused = true;  // remote end paused (assumption)
            }
            else
            /* If rxPrivBuf is nearly empty, resume remote end transmission */
            if(( RX_CIRCULAR_BUFFER_NEARLY_EMPTY < numB )&&( true == rxPaused ))
            {
#               if defined( configTRACE )
                    _putchf( 'r' );
#               endif

                if( true == swFlowCtrl )
                {
                    ( void )transmitChar(( unsigned char )UART_SW_FLOWCTRL_XON );
                }
                else
                {
                    RTSforReceive( );  // in NULL modem, enable remote CTS
                }
                rxPaused = false; // remote end resumed (assumption)
            }
        }
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
              /* we might start a timer, after expiry we abandon the rx;
                 otherwise in non-buffered mode we just rely on the 
                 uartSemaphore to timeout */
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
            /* Rx data will be lost when FIFO is full */
            temp = UART_ERRNO_OVERRUNERR;
              /* we treat this as an error as we have lost rx data;
                 very likely due with too much putchf debug in the isr */
        }
    }

    if(( UART_ERRNO_NONE != temp )&&
       ( UART_ERRNO_BREAKINDICATIONERR != temp ))
    {
        // Flush the RxFIFO, which contains corrupted or lost data
        uartDisableRxInterrupts( );
        uartResetFIFO( UART_FCTL_CLRRxF, UART_FIFO_ENABLE );
        uartEnableRxInterrupts( );
    }
    else
    if( UART_LSR_DATA_READY & lsr )
    {
        // Rx Data has been transferred from the RBR to the RxFIFO
              // recall Zilog UP0049 errata note 7 on continuous rx interrupts
              // though that configuration combination doesn't apply here
        MODEM_RXD_ON( );
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
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    UART_ERRNO temp = UART_ERRNO_NONE;
    unsigned char msr;

    msr = UART1_MSR;  // reading MSR clears delta bits

    if(( DEV_MODE_UART_NO_FLOWCONTROL | DEV_MODE_UART_SW_FLOWCONTROL )& wiring )
    {
        // nothing to do
    }
    else
    {
        if( UART_MSR_DCTS & msr )
        {
            if( DEASSERTED ==( UART_MSR_CTS & msr ))
            {   // remote end rx buffer full
                temp = UART_ERRNO_CTS_LOST;
                MODEM_CTS_OFF( );
                if( UART_STATE_WRITE & uartState )
                {       // break in transmission if we're writing
                    UART1_LCTL |= UART_LCTL_SB;
                }
            }
            else
            {
                temp = UART_ERRNO_CTS_FOUND;
                MODEM_CTS_ON( );
                if( UART_STATE_WRITE & uartState )
                {       // resume transmission if we're writing
                    UART1_LCTL &=( unsigned char )( 0xff - UART_LCTL_SB );
                }
            }
        }

        if( DEV_MODE_UART_FULL_MODEM & wiring )  // FULL or FULL_NULL
        {
            if( UART_MSR_DDSR & msr )
            {           // remote end gone away
                if( DEASSERTED ==( UART_MSR_DSR & msr ))
                {
                    temp = UART_ERRNO_DSR_LOST;
                    MODEM_DSR_OFF( );
                }
                else
                {
                    temp = UART_ERRNO_DSR_FOUND;
                    MODEM_DSR_ON( );
                }
            }
            else
            if( UART_MSR_DDCD & msr )
            {           // remote end signal lost
                if( DEASSERTED ==( UART_MSR_DCD & msr ))
                {
                    temp = UART_ERRNO_DCD_LOST;
                    MODEM_DCD_OFF( );
                }
                else
                {
                    temp = UART_ERRNO_DCD_FOUND;
                    MODEM_DCD_ON( );
                }
            }
            else
            if( UART_MSR_TERI & msr )
            {
                /* Function of RI varies by modem: 
                     UART_ERRNO_RI_CALLING may indicate a state of ringing; 
                     or UART_ERRNO_RI_CALLING .. UART_ERRNO_RI_SILENT cycles 
                      may indicate a specific ring signal encoding, for which
                      we would need to add a device driver layer for decoding. 
                     So state is it.
                   It would be nice to raise a task interrupt; but uart_dev_poll
                     is probably a good enough place to indicate ring and is
                     simpler. */
                if( DEASSERTED ==( UART_MSR_RI & msr ))
                {       // remote end call level low
                    temp = UART_ERRNO_RI_SILENT;
                    MODEM_RI_OFF( );
                }
                else
                {       // remote end call level high
                    temp = UART_ERRNO_RI_CALLING;
                    MODEM_RI_ON( );
                }
            }
        }
    }

    return( temp );
}


/*------ UART ISR -----------------------------------------------------------*/
/* uartisr_1
 *   decode cause(s) of interrupt and act on them
 *   Refer to Zilog PS015317 section UART registers UIIR tables 57 & 58
 */
static void uartisr_1( void )
{
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned char iir;
    unsigned char irrm;
    unsigned char numTRxBytes;
    UART_ERRNO _uart_errno;

    
    /* Read UART Interrupt Identification Register (IRR, table 57)
         Each interrupt is reported only after higher priority ones (table 58) 
         have been cleared down. */
    for( iir = UART1_IIR; 0 ==( iir & UART_IIR_INTBIT ); iir = UART1_IIR )
    {
        irrm =( iir & UART_IIR_ISCMASK );

        if( UART_IIR_LINESTATUS == irrm )
        {
            _uart_errno = clearDownLineStatus( );
#           if defined( configTRACE )
                _putchf( 'a' );
#           endif

            if(( UART_ERRNO_RX_DATA_READY == _uart_errno )||
               ( UART_ERRNO_BREAKINDICATIONERR == _uart_errno ))
            {
                /* Don't need to copy data as the FIFO content remains intact,
                    just wait for UART_IIR_DATAREADY_TRIGLVL.
                   [If however devuart is modified without the FIFO enabled, 
                    then we must call receiveNextBlock here.] */
            }
            else
            if( UART_ERRNO_NONE != _uart_errno )
            {
#               if defined( configTRACE )
                    _putchf( '1' );
#               endif

                /* error conditions are one of UART_LSR_FRAMINGERR,
                   UART_LSR_PARITYERR, UART_LSR_OVERRRUNERR. 
                   clearDownLineStatus will have called uartResetFIFO on
                   the Rx path, taken down and re-enabled Rx interrupts */
                if( UART_STATE_READY & uartState )
                {
                    uartRxEvent |=( 1 << UART_EVENT_RX_ERROR );
                }

                if( UART_STATE_READ & uartState )
                {
                    if( DEV_MODE_UNBUFFERED == rxTransact.mode )
                    {
                        /* Clear UART_STATE_READ here because another interrupt 
                           may follow immediately, before the user task is able 
                           to take uartRxSemaphore. Semantics of UART_STATE_READ 
                           and UART_STATE_READLOCK now diverge.*/
                        uartState &=( UART_STATE )( ~UART_STATE_READ );
                        MODEM_RXD_OFF( );
                        xSemaphoreGiveFromISR( uartRxSemaphore, NULL );
                    }
                    else
                    {
                        storeRxBuffer( );
                        finaliseRxTransaction( _uart_errno | uartRxEvent );
                        CLEAR_UART_TRANSACTION( rxTransact );
                        
                        uartRxEvent = 0;
                        uartState &=
                            ( UART_STATE )~( UART_STATE_READ | UART_STATE_READLOCK );
                        MODEM_RXD_OFF( );
                    }
                }
            }
        }

        /* According to Zilog PS015317 section UART Receiver Interrupts, RDR is 
           generated if the number of bytes in the receiver FIFO is greater 
           than or equal to the trigger level. If the FIFO is not enabled, RDR 
           is generated if the receive buffer contains a data byte.
           A receiver time-out interrupt (UART_IIR_CHARTIMEOUT) is generated 
           when there are fewer data bytes in the receiver FIFO than the 
           trigger level and there are no reads and writes to or from the 
           receiver FIFO for four consecutive byte times (32-bit times) - 
           meaning there is a gap in reception. This will always occur if the
           rx data stream is less than a multiple of the trigger level; we 
           treat it like a FIFO trigger (UART_IIR_DATAREADY_TRIGLVL).It is 
           cleared only after emptying the receive FIFO. */
        if(( UART_IIR_DATAREADY_TRIGLVL == irrm )||
           ( UART_IIR_CHARTIMEOUT == irrm ))
        {
#           if defined( configTRACE )
                _putchf( 'b' );
#           endif

//if( UART_IIR_DATAREADY_TRIGLVL == irrm ) _putchf('1');
//if( UART_IIR_CHARTIMEOUT == irrm ) _putchf('2');
            _uart_errno = receiveNextBlock( &numTRxBytes );

            if( UART_ERRNO_RECEIVEFIFOFULL == _uart_errno )
            {
#               if defined( configTRACE )
                    _putchf( '3' );
#               endif

                uartRxEvent |=( 1 << UART_EVENT_RX_FULL );
            }

            if( UART_STATE_READ & uartState )
            {
#               if defined( configTRACE )
                    _putchf( '4' );
#               endif

                rxTransact.lenTRx += numTRxBytes;
                if( rxTransact.lenTRx >= rxTransact.len )
                {
                    uartRxEvent |=( 1 << UART_EVENT_RX_READY );
                    /*
                    Unlike uartDisableTxInterrupts, which is transaction-based, 
                    uartDisableRxInterrupts is error-based. We only invoke 
                    uartDisableRxInterrupts as an error-recovery method; after 
                    which we will re-start the background read activity. 
                    */

                    if( DEV_MODE_UNBUFFERED == rxTransact.mode )
                    {
#                       if defined( configTRACE )
                            _putchf( '5' );
#                       endif

                        /* Clear UART_STATE_READ here because another interrupt 
                           may follow immediately, before the user task is able 
                           to take uartRxSemaphore. Semantics of UART_STATE_READ 
                           and UART_STATE_READLOCK now diverge.*/
                        uartState &=( UART_STATE )( ~UART_STATE_READ );
                        MODEM_RXD_OFF( );
                        xSemaphoreGiveFromISR( uartRxSemaphore, NULL );
                    }
                    else
                    {
#                       if defined( configTRACE )
                            _putchf( '6' );
#                       endif

                        storeRxBuffer( );
                        finaliseRxTransaction( _uart_errno | uartRxEvent );
                        CLEAR_UART_TRANSACTION( rxTransact );
                        
                        uartRxEvent = 0;
                        uartState &=
                            ( UART_STATE )~( UART_STATE_READ | UART_STATE_READLOCK );
                        MODEM_RXD_OFF( );
                    }
                }
            }
            else
            if( UART_STATE_READY & uartState )
            {
#               if defined( configTRACE )
                    _putchf( '7' );
#               endif

                uartRxEvent |=( 1 << UART_EVENT_RX_READY );
            }
        }

        if( UART_IIR_TRANSCOMPLETE == irrm )
        {
            // Transmit Complete Interrupt (TCI) is generated when both 
            // the serial device Transmit Shift Register and the Transmit Hold 
            // Register are empty. Continuously - need to disable the interrupt.
            // This should signify we have no more data to send
#           if defined( configTRACE )
                _putchf( 'c' );
#           endif

            if( UART_STATE_WRITE & uartState )
            {
#               if defined( configTRACE )
                    _putchf( '1' );
#               endif

                uartDisableTxInterrupts( UART_IIR_TRANSCOMPLETE );
                // reset Tx FIFO and leave Rx Fifo enabled
                uartResetFIFO( UART_FCTL_CLRTxF, UART_FIFO_ENABLE );

                if( txTransact.lenTRx < rxTransact.len )
                {
#                   if defined( configTRACE )
                        _putchf( '2' );
#                   endif

                    uartTxEvent |=( 1 << UART_EVENT_TX_ERROR );
                }
                else
                {
#                   if defined( configTRACE )
                        _putchf( '3' );
#                   endif

                    uartTxEvent |=( 1 << UART_EVENT_TX_DONE );
                }

                if( DEV_MODE_BUFFERED == txTransact.mode )
                {
#                   if defined( configTRACE )
                        _putchf( '4' );
#                   endif

                    finaliseTxTransaction( POSIX_ERRNO_ENONE | uartTxEvent );
                    CLEAR_UART_TRANSACTION( txTransact );
                        
                    uartTxEvent = 0;
                    uartState &=( UART_STATE )
                        ~( UART_STATE_WRITE | UART_STATE_WRITELOCK );
                    MODEM_TXD_OFF( );
                }
                else
                {
#                   if defined( configTRACE )
                        _putchf( '5' );
#                   endif

                    /* Clear UART_STATE_WRITE here because another interrupt 
                       may follow immediately, before the user task is able 
                       to take uartTxSemaphore. Semantics of UART_STATE_WRITE 
                       and UART_STATE_WRITELOCK now diverge.*/
                    uartState &=( UART_STATE )( ~UART_STATE_WRITE );
                    MODEM_TXD_OFF( );
                    xSemaphoreGiveFromISR( 
                        uartTxSemaphore, &__higherPriorityTaskWokenUART );
                }
            }
            else
            if( UART_STATE_READY & uartState )
            {
#               if defined( configTRACE )
                    _putchf( '6' );
#               endif

                uartTxEvent |=( 1 << UART_EVENT_TX_UNEXPECTED );
                uartDisableTxInterrupts( UART_IIR_TRANSCOMPLETE );
                // reset Tx FIFO and leave Rx Fifo enabled
                uartResetFIFO( UART_FCTL_CLRTxF, UART_FIFO_ENABLE );
            }
        }

        if( UART_IIR_TRANSBUFFEREMPTY == irrm )
        {
            // Transmit Buffer Empty (TCIE) is generated when the Transmit Hold
            // Register is empty.
#           if defined( configTRACE )
                _putchf( 'd' );
#           endif

            if( UART_STATE_WRITE & uartState )
            {
#               if defined( configTRACE )
                    _putchf( '1' );
#               endif

                transmitNextBlock( &numTRxBytes );
                txTransact.lenTRx += numTRxBytes;
                // if there's nothing more to send, disable UART_IIR_TRANSBUFFEREMPTY
                if( 0 == numTRxBytes )
                {
#                   if defined( configTRACE )
                        _putchf( '2' );
#                   endif

                    // wait for UART_IIR_TRANSCOMPLETE after last byte shifted out
                    uartDisableTxInterrupts( UART_IIR_TRANSBUFFEREMPTY );
                }
            }
            else
            if( UART_STATE_READY & uartState )
            {
#               if defined( configTRACE )
                    _putchf( '3' );
#               endif

                uartTxEvent |=( 1 << UART_EVENT_TX_UNEXPECTED );
                uartDisableTxInterrupts( UART_IIR_TRANSCOMPLETE );
                // reset Tx FIFO and leave Rx Fifo enabled
                uartResetFIFO( UART_FCTL_CLRTxF, UART_FIFO_ENABLE );
            }
        }

        if( UART_IIR_MODEMSTAT == irrm )
        {
            _uart_errno = clearDownModemStatus( );
#           if defined( configTRACE )
                _putchf( 'e' );
#           endif

            if(( UART_ERRNO_DSR_LOST == _uart_errno )||  // remote end gone away
               ( UART_ERRNO_DCD_LOST == _uart_errno ))   // remote end signal lost
            {
                uartTxEvent |=( 1 << _uart_errno );
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
                        uartState &=
                            ( UART_STATE )~( UART_STATE_READ | UART_STATE_READLOCK );
                        MODEM_RXD_OFF( );
                    }
                    else
                    {
                        /* Clear UART_STATE_READ here because another interrupt 
                           may follow immediately, before the user task is able 
                           to take uartRxSemaphore. Semantics of UART_STATE_READ 
                           and UART_STATE_READLOCK now diverge.*/
                        uartState &=( UART_STATE )( ~UART_STATE_READ );
                        MODEM_RXD_OFF( );
                        xSemaphoreGiveFromISR( 
                            uartRxSemaphore, &__higherPriorityTaskWokenUART );
                    }
                }

                if( UART_STATE_WRITE & uartState )
                {
                    if( DEV_MODE_BUFFERED == txTransact.mode )
                    {
                        finaliseTxTransaction( POSIX_ERRNO_ENONE | uartTxEvent );
                        CLEAR_UART_TRANSACTION( txTransact );

                        uartTxEvent = 0;
                        uartState &=( UART_STATE )
                            ~( UART_STATE_WRITE | UART_STATE_WRITELOCK );
                        MODEM_TXD_OFF( );
                    }
                    else
                    {
                        /* Clear UART_STATE_WRITE here because another interrupt 
                           may follow immediately, before the user task is able 
                           to take uartTxSemaphore. Semantics of UART_STATE_WRITE 
                           and UART_STATE_WRITELOCK now diverge.*/
                        uartState &=( UART_STATE )( ~UART_STATE_WRITE );
                        MODEM_TXD_OFF( );
                        xSemaphoreGiveFromISR( 
                            uartTxSemaphore, &__higherPriorityTaskWokenUART );
                    }
                }
            }
        }
    }

#   if defined( configTRACE )
        _putchf( 'f' );
#   endif
}


/* uartisr_0 - address written into the interrupt vector table
     Standard ISR reti.L epilogue */
static void uartisr_0( void )
{
    /* The current task context SHALL be saved first on entry to an ISR */
    portSAVE_CONTEXT( );

#   if defined( configTRACE )&& 0
        _putchf( 'U' );
#   endif

    __higherPriorityTaskWokenUART = pdFALSE;

    /* run in context of this ISR.
         Other hardware interrupts are blocked out for the duration. */
    uartisr_1( );

    /* If __higherPriorityTaskWokenUART is now set to pdTRUE then a 
       context switch should be performed to ensure the 
       interrupt returns directly to the highest priority task */
    if( pdTRUE == __higherPriorityTaskWokenUART )
    {
#       if defined( configTRACE )&& 0
            _putchf( 'V' );
#       endif

        asm( "\txref _vPortYieldFromISR_2   ; reti from vPortYieldFromISR" );
        asm( "\tJP _vPortYieldFromISR_2     ;   with saved context" );
    }
    else
    {
        asm( "\t                            ; reti from here" );
#       if defined( configTRACE )&& 0
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


    /* -2. Create the high-priority interrupt handler task * /
#   if( 1 == configUSE_PREEMPTION )
    {
        ret = createUartIsrHandlerTask( );
    }
#   endif
    */

    if( UART_STATE_INITIAL != uartState )
    {
        ret = POSIX_ERRNO_EADDRINUSE;
        goto uart_dev_open_end;
    }

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
        /* 0. remember the pinmode for hardware flow control in the ISR
           This is done with SAFEGUARDS enabled, but we must do so explicitly
           otherwise. */
        pinmode[ MINOR_NUM ]= mode;
    }
#   endif

    /* 1. set the eZ80 port pin modes for Alternate Function number 0 
          refer to Zilog PS15317 table 6 */
    if(( DEV_MODE_UART_FULL_MODEM & mode )||  // FULL or FULL_NULL
       ( DEV_MODE_UART_LOOPBACK & mode ))
    {
        endpin = UART_1_RI;  // last pin used in full handshaking
    }
    else
    if( DEV_MODE_UART_HALF_MODEM & mode )  // HALF or HALF_NULL
    {
        endpin = UART_1_CTS; // last pin used in half-handshaking
    }
    else
    {
        endpin = UART_1_RXD; //last pin used without hardware handshaking
    }

    for( i = UART_1_TXD, setmask = 0x00, clrmask = 0xff; endpin >= i; i++ )
    {
        unsigned int const mnri =( i - PIN_NUM_START );

        setmask |= SET_MASK( portmap[ mnri ].bit );
        clrmask &= CLR_MASK( portmap[ mnri ].bit );
    }

    CLEAR_PORT_MASK( PC_DR, clrmask );   // also tried SET_PORT_MASK PC_DR, either work
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

    /* 2.2 Initialise the Modem Control register
           This sets pin /DTR high, indicating UART1 is not ready (for full
           or full-null modem mode); it also sets /RTS high */
    UART1_MCTL =( unsigned char )0x00;
    if( DEV_MODE_UART_LOOPBACK & mode )
    {
        /* 2.2.1 Program MCTL in Loopback mode; refer to PS015317 table 63. 
                 This connects:       rx <- tx [the fifos remain operational];
                               mctl.OUT2 -> msr.DCD,
                               mctl.OUT1 -> msr.RI
                               mctl.RTS  -> msr.CTS
                               mctl.DTR  -> msr.DSR */
        UART1_MCTL |= UART_MCTL_LOOP;
    }

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
           Enable Rx interrupts.
           We only initialise Tx interrupts when we're ready to receive,
           otherwise "empty" interrupts fire evermore */
    UART1_IER = 0x00;
    uartEnableRxInterrupts( );

    /* 2.8 Assert /DTR to signal we are 'up', 
           unless full modem or loopback wiring, in which case "connection
           establishment" will follow. */
    if(( DEV_MODE_UART_FULL_MODEM != mode )&&
       ( DEV_MODE_UART_LOOPBACK != mode ))
    {
        DTRready( );
    }

    /* 2.9 /RTS default behaviour is always ready to receive */
    RTSforReceive( );

uart_dev_open_end:
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
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    unsigned int endpin;
    unsigned char setmask;
    unsigned char clrmask;
    int i;

    if( UART_STATE_READY & uartState )
    {
        // 0. Make sure we release Xoff
        if( true == rxPaused )
        {
            ( void )transmitChar(( unsigned char )UART_SW_FLOWCTRL_XON );
            rxPaused = false;  // remote end resumed (assumption)
        }

        // 1. Disable UART
        UART1_IER = /*ier_mask =*/ 0x00;  // Disable UART1 interrupts
        UART1_LCTL = 0x00; // Reset line control register

        // 1.5 de-assert /DTR to signal we are 'down'
        DTRnotReady( );    // signal end of transmission
        DCDloopNotReady( ); // signal connection dropped
        UART1_MCTL = 0x00; // Reset modem control register (technically redundant)

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
        if( DEV_MODE_UART_FULL_MODEM & wiring )  // FULL or FULL_NULL
        {
            endpin = UART_1_RI;
        }
        else
        if( DEV_MODE_UART_HALF_MODEM & wiring )  // HALF or HALF_NULL
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

#       if( 0 == configUSE_DEV_SAFEGUARDS )
        {
            /* 4. unknow the user mode */
            pinmode[ MINOR_NUM ]= 0;
        }
#       endif
    }
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
     then the result POSIX_ERRNO_EBUSY will be immediately returned. We split
     the semantics of UART_STATE_READ (which controls ISR buffering) into 
      UART_STATE_READLOCK (which guarantees at most one reader task or thread). 
   For DEV_MODE_UART_FULL_MODEM and DEV_MODE_UART_LOOPBACK, the application
     is responsible for establishing a connection using uart_dev_ioctl's prior
     to calling uart_dev_write or uart_dev_read.
   Refer to Zilog PS015317 UART Receiver
     And to src/uart/common/readuart1.c, .../fifoget1.c
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
        if(( UART_STATE_READY & uartState )&&
           ( 0 ==( UART_STATE_READLOCK & uartState )))  // if not already reading
        {
            uartState |=( UART_STATE_READ | UART_STATE_READLOCK );
            MODEM_RXD_ON( );
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
                portENTER_CRITICAL( );
                {
                    ret = POSIX_ERRNO_EALREADY | uartRxEvent;
                    uartRxEvent = 0;

                    storeRxBuffer( );
                    finaliseRxTransaction( ret );
                    CLEAR_UART_TRANSACTION( rxTransact );
                    
                    uartState &=
                        ( UART_STATE )~( UART_STATE_READ | UART_STATE_PAUSE );
                    MODEM_RXD_OFF( );
                }
                portEXIT_CRITICAL( );
            }
            else
            {
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
                        ret = POSIX_ERRNO_EALREADY;  // = num_bytes_to_read
                    }
                    else
                    {
                        // calling task should check callback result
                        ret = POSIX_ERRNO_ETIMEDOUT; // < num_bytes_to_read
                    }

                    portENTER_CRITICAL( );
                    {
                        ret |= uartRxEvent;
                        uartRxEvent = 0;

                        storeRxBuffer( );
                        finaliseRxTransaction( ret );
                        CLEAR_UART_TRANSACTION( rxTransact );

                        /* uartisr0 will clear UART_STATE_READ before giving 
                           the semaphore. We need to clear it here too in case 
                           of no interrupt and POSIX_ERRNO_ETIMEDOUT */
                        uartState &=
                            ( UART_STATE )~( UART_STATE_READ | UART_STATE_PAUSE );
                        MODEM_RXD_OFF( );
                    }
                    portEXIT_CRITICAL( );
                }
            }

            uart_rxFlowControl( );
            uartState &=( UART_STATE )( ~UART_STATE_READLOCK );
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
   For DEV_MODE_UART_FULL_MODEM and DEV_MODE_UART_LOOPBACK, the application
     is responsible for establishing a connection using uart_dev_ioctl's prior
     to calling uart_dev_write or uart_dev_read. But this same routine is used
     to send data directly to the modem, hence uart_dev_write_2.
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
    DEV_MODE const wiring = pinmode[ MINOR_NUM ];
    DEV_MODE const modeB =(( NULL != num_bytes_written )&&( NULL != res ))
                             ? DEV_MODE_BUFFERED
                             : DEV_MODE_UNBUFFERED;
    unsigned char numTxBytes;

    if( configDRV_UART_BUFFER_SZ >= num_bytes_to_write )
    {
        portENTER_CRITICAL( );
        if(( UART_STATE_READY & uartState )&&
           ( 0 ==( UART_STATE_READLOCK & uartState )))  // if not already writing
        {
            uartState |= UART_STATE_WRITELOCK;
            portEXIT_CRITICAL( );

            if(( DEV_MODE_UART_FULL_MODEM & wiring )||  // FULL or FULL_NULL MODEM
               ( DEV_MODE_UART_LOOPBACK == wiring ))
            {
                /* If the application has set DEV_IOCTL_UART_WRITE_MODEM for
                   writing directly to the modem, intended for connection 
                   establishment, then skip the usual end-point handshaking */
                if( true == connection_establishment )
                {
                    goto uart_dev_write_2;
                }
            
                // check the remote-end connection is established
                ret = waitDSR( );     // wait for DCE to become ready
                if( POSIX_ERRNO_EPROTO == ret )
                {
                    uartState &=( UART_STATE )( ~UART_STATE_WRITELOCK );
                    goto uart_dev_write_1;
                }

                ret = waitDCD( );  // wait for connection with our peer
                if( POSIX_ERRNO_EPROTO == ret )
                {
                    uartState &=( UART_STATE )( ~UART_STATE_WRITELOCK );
                    goto uart_dev_write_1;
                }
            }

            RTSforSend( );
            ret = waitCTS( );
            if( POSIX_ERRNO_EPROTO == ret )
            {
                uartState &=( UART_STATE )( ~UART_STATE_WRITELOCK );
                goto uart_dev_write_1;
            }

            // Good to go
            if( 1 == num_bytes_to_write )
            {
                // don't use the FIFO for just one byte (uart_putch)
                ret = transmitChar((( char * )buffer )[ 0 ]);
                if( DEV_MODE_BUFFERED == modeB )
                {
                    *num_bytes_written = 1;
                    *res = ret;
                }
                uartState &=( UART_STATE )( ~UART_STATE_WRITELOCK );
            }
            else
            {
uart_dev_write_2:

                // 2 or more bytes to send, use the FIFO
                CREATE_UART_TRANSACTION( 
                    txTransact, modeB, 
                    buffer, num_bytes_to_write, num_bytes_written, res );
#               if defined( _DEBUG )&& 0
                    PRINT_UART_TRANSACTION( txTransact );
#               endif

                loadTxBuffer(( unsigned char * )buffer, num_bytes_to_write );

                // kick off the write transaction
                uartState |= UART_STATE_WRITE;
                MODEM_TXD_ON( );
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

                    portENTER_CRITICAL( );  // technically but likely unnecessary
                    {
                        ret |= uartTxEvent;
                        uartTxEvent = 0;

                        finaliseTxTransaction( ret );
                        CLEAR_UART_TRANSACTION( txTransact );

                        /* uartisr0 will clear UART_STATE_WRITE before giving 
                           the semaphore. We need to clear it here too in case 
                           of no interrupt and POSIX_ERRNO_ETIMEDOUT */
                        uartState &=( UART_STATE )
                            ~( UART_STATE_WRITE | 
                               UART_STATE_WRITELOCK | 
                               UART_STATE_PAUSE );
                        MODEM_TXD_OFF( );
                    }
                    portEXIT_CRITICAL( );
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


/* uart_dev_poll
   Device-specific UART1 poll function 
   Set num_bytes_free in private transmit buffer, num_bytes_avail in private
    receive buffer, and modem status
*/
POSIX_ERRNO uart_dev_poll( 
                size_t * num_tx_bytes_buffered,   // OUT
                size_t * num_rx_bytes_buffered,   // OUT
                UART_MODEM_STATUS *modem_status   // OUT
            )
{
    if( num_rx_bytes_buffered )
    {
        RX_CIRCULAR_BUFFER_NUM_OCCUPIED( *num_rx_bytes_buffered );
    }

    if( num_tx_bytes_buffered )
    {
        *num_tx_bytes_buffered = txPrivBuf.Index - txPrivBuf.Outdex;
    }

    if( modem_status )
    {
        *modem_status = modemStatus;
    }
    
    return( POSIX_ERRNO_ENONE );
}


/* uart_dev_ioctl
   Device-specific UART1 IO Control function
   IOCtl is introduced mainly to support FULL_MODEM (and Loopback) connection
    establishment. Connection establishment requires a protocol, specific for
    each kind of 'Modem'. Connection establishment is required to achieve:
      1. set DTR (either in response to an incoming RI, or to initiate a call)
      2. wait for DSR
      3. Negotiate call parameters, by 
         i. sending tx data to the Modem (with no RTS/CTS handshake)
         ii. interpreting rx data responses from the Modem
      4. wait for DCD
*/
POSIX_ERRNO uart_dev_ioctl( 
                DEV_IOCTL const cmd,
                void * const param
            )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    
    switch( cmd )
    {
        case DEV_IOCTL_UART_EXEC_RX_FLOW_CONTROL :
        {
            uart_rxFlowControl( );
        }
        break;

        case DEV_IOCTL_UART_WRITE_MODEM :
        {
            if( param )
            {
                if( 0 ==( int* )param )
                {
                    connection_establishment = false;
                }
                else
                {
                    connection_establishment = true;
                }
            }
            else
            {
                ret = POSIX_ERRNO_EINVAL;
            }
        }
        break;
        

        case DEV_IOCTL_UART_SET_DTR :
        {
            if( param )
            {
                if( 0 ==( int* )param )
                {
                    DTRnotReady( );     // signal end of transmission
                    DCDloopNotReady( ); // signal connection dropped in loopback
                }
                else
                {
                    DTRready( );     // /DTR to signal connection wanted
                    DCDloopReady( ); // /DCD to signal connection established
                }
            }
            else
            {
                ret = POSIX_ERRNO_EINVAL;
            }
        }
        break;
        
        case DEV_IOCTL_UART_GET_DSR :
        {
            if( param )
            {
                ( void )waitDSR( );
                *(( unsigned char* )param )= testDSR( );
            }
            else
            {
                ret = POSIX_ERRNO_EINVAL;
            }
        }
        break;
        
        case DEV_IOCTL_UART_GET_DCD :
        {
            if( param )
            {
                ( void )waitDCD( );
                *(( unsigned char* )param )= testDCD( );
            }
            else
            {
                ret = POSIX_ERRNO_EINVAL;
            }
        }
        break;
        
        case DEV_IOCTL_UART_GET_RI :
        {
            if( param )
            {
                *(( unsigned char* )param )= testRI( );
            }
            else
            {
                ret = POSIX_ERRNO_EINVAL;
            }
        }
        break;
        
        default:
        {
            ret = POSIX_ERRNO_EINVAL;
        }
        break;
    }
    
    return( ret );
}


#endif  /* 1 == configUSE_DRV_UART */
