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
 * port.c for Agon Light
 *
 * Evolved from the earlier eZ80 port by Marcos A. Pereira began in May 2005. 
 * This was updated by Jean-Michel Roux prior to 2010. 
 * Ported to Agon Light by Julian Rose in 2024, with copyright made over to 
 * Amazon Web Services above for FreeRTOS version 10.5.
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The functions in this file configure the FreeRTOS portable code
 * for Agon Light (and comptaibles) and the ZDSII compiler
 * Zilog eZ80 ANSI C Compiler Version 3.4 (19101101).
 * Created 09/May/2024 by Julian Rose for Agon Light port
 *
 * These functions should not normally be application-user altered.
*/

/*----- Notes -----------------------------------------------------------------
 1. MOS sets eZ80 mixed mode (MADL=1) in 
    ...\src\Quark\agon-mos-main\src_startup\vectors16.asm::_rst0
    Refer to UM007715 Table 25 ADL Mode ADLbit=1 MADLbit=1
 2. MOS sets the eZ80 into Interrupt Mode 2 (IM 2) in
    ...\src\Quark\agon-mos-main\src_startup\vectors16.asm::_init_default_vectors
*/

#include <ez80.h>

#include <stdlib.h>
#if defined( _DEBUG )
#   include <stdio.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"


/*----- Enumeration Types ---------------------------------------------------*/
/* Calling Modes - refer to Zilog UG07715 Table 49 */
typedef enum _adl_call_mode
{
    ADL_CALL_MODE_SIS = 0x0,   /* call from Z80 (ADL=0) to Z80 (ADL<-0) */
    ADL_CALL_MODE_LIS = 0x1,   /* call from ADL (ADL=1) to Z80 (ADL<-0) */
    ADL_CALL_MODE_SIL = 0x2,   /* call from Z80 (ADL=0) to ADL (ADL<-1) */
    ADL_CALL_MODE_LIL = 0x3    /* call from ADL (ADL=1) to ADL (ADL<-1) */

} ADL_CALL_MODE;


/*----- Global Names --------------------------------------------------------*/
/* Address of the (tasks.c) pxCurrentTCB variable, 
   but don't need to know its type. */
typedef void tskTCB;
extern tskTCB * volatile pxCurrentTCB;
extern void * mos_setpitvector( 
                 unsigned int, 
                 void ( *intr_hndlr )( void )
              );


/*----- Private Variables ---------------------------------------------------*/
/* mutex to implement MOS critical regions */
static SemaphoreHandle_t portMOSMutex = NULL;

/* we keep track of our installed ISR, to remove it in case of exit */
static void( *portPrevprev )( void ) = NULL;
static int portTmr = -1;


/*----- Private Functions ---------------------------------------------------*/
static BaseType_t prvSetupTimerInterrupt( void );
static void portTeardownTimerInterrupt( void );
static void timer_isr( void );

static void portTaskExit( void );

static BaseType_t portCreateMOSMutex( void );

static void vPortYieldFromTick( void );


/*----- Context Construction ------------------------------------------------*/
/* pxPortInitialiseStack
     Initialse the stack for an ADL task. This includes the ADL-mode byte
     after the PC register initial value.
*/
portSTACK_TYPE *pxPortInitialiseStack( 
    portSTACK_TYPE *pxTopOfStack, 
    pdTASK_CODE pxCode, 
    void *pvParameters 
    )
{
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#   endif
    
    /* Place the parameter on the stack in the expected location. */
    *pxTopOfStack-- =( portSTACK_TYPE )pvParameters;

    /* Place the task return address on stack. Not normally used */
    *pxTopOfStack-- =( portSTACK_TYPE )&portTaskExit;

    /* The start of the task code will be popped off the stack last, so place
    it on first. */
    *pxTopOfStack =( portSTACK_TYPE )pxCode;    /* PC [intr or call.LIL] */

    /* All call stacks SHALL include the [ADL byte] */
    pxTopOfStack =( portSTACK_TYPE * )((( unsigned char * )pxTopOfStack )- 1 );
    *(( unsigned char * )pxTopOfStack )=( unsigned char )ADL_CALL_MODE_LIL;
    pxTopOfStack--;

    /* ZDS II Stack Frame saved by the compiler (IX Register). */
    *pxTopOfStack-- =( portSTACK_TYPE )0x111111;  /* IX  [prologue] */

    /* Now load the registers to create a dummy frame
       (refer to portSAVE_CONTEXT). */
    *pxTopOfStack-- =( portSTACK_TYPE )0xAFAFAF;  /* AF  */
    *pxTopOfStack-- =( portSTACK_TYPE )0xBCBCBC;  /* BC  */
    *pxTopOfStack-- =( portSTACK_TYPE )0xDEDEDE;  /* DE  */
    *pxTopOfStack-- =( portSTACK_TYPE )0xEFEFEF;  /* HL  */
    *pxTopOfStack-- =( portSTACK_TYPE )0x222222;  /* IY  */
    *pxTopOfStack-- =( portSTACK_TYPE )0xFAFAFA;  /* AF' */
    *pxTopOfStack-- =( portSTACK_TYPE )0xCBCBCB;  /* BC' */
    *pxTopOfStack-- =( portSTACK_TYPE )0xEDEDED;  /* DE' */
    *pxTopOfStack   =( portSTACK_TYPE )0xFEFEFE;  /* HL' */

    return( pxTopOfStack );
}


/*---- MOS Critical Region --------------------------------------------------*/
/* Create critical region mutex to guard MOS accesses
 * We don't use portENTER_CRITICAL and thereby disable interrupts,
 * because the eZ80 talks to the ESP32 via a serial link and this
 * may use interrupts. Instead we use a semaphore.
 * And this is only necessary if configUSE_PREEMPTION is enabled,
 * otherwise tasks will not swap each other out during MOS calls.
*/
static BaseType_t portCreateMOSMutex( )
{
    BaseType_t r = pdPASS;
    
#   if( 1 == configUSE_PREEMPTION )    
    {
        /* Mutexes (as in xSemaphoreCreateMutex) cannot be used in ISRs;
           so we must use a Binary instead */
        portMOSMutex = xSemaphoreCreateBinary( );
        if( NULL == portMOSMutex )
        {
#           if defined( _DEBUG )
            {
                ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
            }
#           endif
            r = pdFAIL;
        }
        else
        {
            /* make the semaphore initially available */
            xSemaphoreGive( portMOSMutex );
        }
    }
#   endif /* configUSE_PREEMPTION */

    return( r );
}

/* Enter critical region for MOS
 *   We use a dedicated semaphore for MOS critical regions.
 *   Only necessary if configUSE_PREEMPTION is enabled, otherwise access to
 *   MOS will not be interrupted by another task.
 *   Note: Setting timeout to portMAX_DELAY does not suspend the calling task 
 *   as described in https://www.freertos.org/a00122.html in FreeRTOS version 
 *   10.5.1 . So we test the semaphore with time 0 and suspend ourselves if 
 *   we fail to take it.
*/
void portEnterMOS( void )
{
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#   endif

#   if( 1 == configUSE_PREEMPTION )
    {
        if(( NULL != portMOSMutex )&&
           ( taskSCHEDULER_RUNNING == xTaskGetSchedulerState( )))
        {
            /* suspend until semaphore can be taken */
            while( pdTRUE != xSemaphoreTake( portMOSMutex, 0 ))
            {
                /* Some other task has the mutex.
                    If we use vTaskSuspend then the tick ISR will have
                    to call vTaskResume. Easier to use the delayed list.
                    */
                vTaskDelay( 1 );
            }
        }
    }
#   endif /* configUSE_PREEMPTION */

#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#   endif
}


/* Exit critical region for MOS
*/
void portExitMOS( void )
{
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#   endif
    
#   if( 1 == configUSE_PREEMPTION )
    {
        if(( NULL != portMOSMutex )&&
           ( taskSCHEDULER_RUNNING == xTaskGetSchedulerState( )))
        {
            xSemaphoreGive( portMOSMutex );
        }
    }
#   endif /* configUSE_PREEMPTION */
    
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#   endif
}


#if 0
/* Test critical region for MOS
 *    If the semaphore is a binary semaphore then uxSemaphoreGetCount returns 1 
 *    if the semaphore is available, and 0 if the semaphore is not available.
*/
BaseType_t portTestMOS( void )
{
    BaseType_t r = 0;

#   if( 1 == configUSE_PREEMPTION )
    {
        if(( NULL != portMOSMutex )&&
           ( taskSCHEDULER_RUNNING == xTaskGetSchedulerState( )))
        {
            r = uxSemaphoreGetCount( portMOSMutex );
        }
    }
#   endif /* configUSE_PREEMPTION */

    return( r );    
}
#endif


/*----- Scheduler -----------------------------------------------------------*/
/* Start the FreeRTOS scheduler
 *    on success this routine will never return, rather tasks will run
 *    return pdTRUE if we fell out of the scheduler
 *    return pdFALSE if we failed to allocate resources
*/
portBASE_TYPE xPortStartScheduler( void )
{
    portBASE_TYPE ret = pdFALSE;

    /* Setup the hardware to generate the tick. */
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#   endif

    if( pdPASS == prvSetupTimerInterrupt( ))
    {
#       if defined( _DEBUG )&& 0
        {
            ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
        }
#       endif

        if( pdPASS == portCreateMOSMutex( ))
        {
#           if defined( _DEBUG )&& 0
            {
                ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
            }
#           endif

            /* start running the tasks */
            portRESTORE_CONTEXT( );

            asm( "\t pop    ix    ; postlog         " );
            asm( "\t ei           ; enable interrupts to start tick ISR " );
            asm( "\t ret.L        ; ADL mode long ret into Task   " );
            asm( "                ; epilogue below not followed " );
            /* ret from portRESTORE_CONTEXT should be into one of the Tasks */

            /* we shouldn't get here: something has gone wrong */
#           if defined( _DEBUG )
            {
                ( void )printf( "%s : %d : fell out of scheduler\r\n", 
                                "port.c", __LINE__ );
            }
#           endif
            ret = pdTRUE;
        }
        else
        {
#           if defined( _DEBUG )
            {
                ( void )printf( "%s : %d : %s\r\n", 
                                "port.c", 
                                __LINE__,
                                "Failed to allocate MOS semaphore" );
            }
#           endif
        }
    }
    else
    {
#       if defined( _DEBUG )
        {
            ( void )printf( "%s : %d : %s\r\n", 
                            "port.c", 
                            __LINE__,
                            "Failed to allocate timer device" );
        }
#       endif
    }
    
    return( ret );
}

void vPortEndScheduler( void )
{
    /* It is unlikely that the eZ80 port will require this function as there
    is nothing to return to.  If this is required - stop the tick ISR then
    return back to main. */
    portTeardownTimerInterrupt( );
    ( void )printf( "Port End Scheduler\r\n" );
}


/*----- Context Switching ---------------------------------------------------*/
/*
 * Manual context switch.  
 *    MUST be called with a 'call.L' instruction - use portYIELD( ) macro
 *      This is because the saved and restored stacks must be compatible with
 *      MOS interrupt "Yield" versions. These place the additional ADL mode 
 *      byte on the call stack, so that a reti.L is necessary. *3
 *
 *    Actions:
 *      Save the current task context SHALL be done first on entry
 *      Decide which is the new task to run
 *      Restore the new task context SHALL be done last before exit
 *
 *   Entry: With all of the "vPortYield" functions the FIRST thing we MUST do 
 *     (*1) is call portSAVE_CONTEXT. Even in non-preemptive configuration, we 
 *     MUST save the context in order to preserve the AF flags register.
 *   Exit: With all of the "vPortYield" functions we MUST exit in the identical 
 *     sequence FOLLOWING the call to vTaskSwitchContext onwards:
 *            vTaskSwitchContext( );
 *            portRESTORE_CONTEXT( );
 *            asm( "\t pop    ix    ; epilogue" );
 *         *2 asm( "\t ei           ; re-enable interrupts" );
 *         *3 asm( "\t ret.L        ; long ret from here" );
 *            asm( "                ; compiler-inserted postlog below not followed " );
 *   The reason Entry and Exit must be identical is that a task may be Yielded
 *     out by one function, and Yielded back in by another. Between Entry and 
 *     Exit each Yield function can do what it needs. 
 *
 *     *1 We must design the function to use no local stack variables. If the 
 *        compiler allocates space on the stack e.g. push BC then we must undo 
 *        that with 'pop BC' before calling portSAVE_CONTEXT. 
 *        Check this by examining the port.src file output by the compiler.
 *     *2 following di in portSAVE_CONTEXT for the MANUAL Yield.
 *        Interrupt Yields will have "di" on entry
 */
void vPortYield( void )
{
    /* push BC     is inserted by "Zilog eZ80 ANSI C Compiler Version 3.4 (19101101)"
       following the standard prolog. But it is neither popped in the postlog, nor 
       by the caller on return from vPortYield. We don't want it left on the task
       stack. */
    asm( "\t pop    bc   ; undo compiler 'push BC' from the stack" );  // *1

    /* The current task context SHALL be saved first. */
    portSAVE_CONTEXT( );   // pushes the 1-byte ADL mode after the 3-byte PC 

    vTaskSwitchContext( );

    /* RESTORE CONTEXT is shared with vPortYieldFromISR and vPortYieldFromTick,
       so that we must do a long return "reti.L" to the new task context.
       It SHALL be the last action prior to pop ix; ret.L
    */
    portRESTORE_CONTEXT( );

    asm( "\t pop    ix    ; epilogue" );
    asm( "\t ei           ; re-enable interrupts " );  // *2
    asm( "\t ret.L        ; Long ret (ADL_CALL_MODE_LIL + 3 byte PC)" ); // *3
    asm( "                ; compiler-inserted epilogue below will not be executed" );
}


/* vPortYieldFromISR
 *   Context switch entry point used by any ISR 
 *     (except tickISR which has its own alike Yield)
 *     (refer to .../Source/mos/devuart.c::uartisr_0 as an example)
 *
 *    MUST be jumped into from an ISR; the ISR will exit from here.
 *      This is because a ret[i].L SHALL follow immediately after a
 *      portRESTORE_CONTEXT()
 *
 *      The saved and restored stacks MUST be compatible with MOS interrupt 
 *      "Yield" versions. These place the additional ADL mode byte on the call 
 *      stack, so that a reti.L is necessary. *3
 *
 *   Entry: 
 *     A/ ISRs that have not saved a context SHALL jump to _vPortYieldFromISR_1.
 *        As with all of the "vPortYield" functions the FIRST action SHALL be
 *        (*1) to call portSAVE_CONTEXT. Even in non-preemptive configuration,
 *        we MUST save the context in order to preserve the AF flags register.
 *     B/ ISRs that have already executed portSAVE_CONTEXT SHALL jump to
 *        _vPortYieldFromISR_2
 *   Exit: With all of the "vPortYield" functions we MUST exit in the identical 
 *     sequence FOLLOWING the call to vTaskSwitchContext onwards. RESTORE CONTEXT
 *     SHALL be the last action prior to pop ix; reti. 
 *            vTaskSwitchContext( );
 *            portRESTORE_CONTEXT( );
 *            asm( "\t pop    ix    ; postlog         " );
 *         *2 asm( "\t ei           ; re-enable interrupts " ); *2
 *            asm( "\t reti.L       ; ret from here   " );
 *            asm( "                ; compiler-inserted postlog below not followed " );
 *   The reason Entry and Exit must be identical is that a task may be Yielded
 *     out by one function, and Yielded back in by another. Between Entry and 
 *     Exit each Yield function can do what it needs. 
 *
 *     *1 We must design the function to use no local stack variables. If the 
 *        compiler allocates space on the stack e.g. push BC then we must undo 
 *        that with 'pop BC' before calling portSAVE_CONTEXT. 
 *        Check this by examining the port.src file output by the compiler.
 *     *2 following di in portSAVE_CONTEXT for the MANUAL Yield only, NOT in 
 *        ISR Yields (which will do a reti)
*/
void vPortYieldFromISR( void )
{
    asm( "\t xdef _vPortYieldFromISR_1      ; entry point for jump" );
    asm( "\t _vPortYieldFromISR_1:          ;   with no saved context" );
    
    /* The current task context SHALL be saved first; 
       even if 0==configUSE_PREEMPTION. */
    portSAVE_CONTEXT( );
    /* UNLESS the current task context is already saved by the 'calling' ISR */
    asm( "\t xdef _vPortYieldFromISR_2      ; entry point for jump" );
    asm( "\t _vPortYieldFromISR_2:          ;   with already saved context" );

    /* A context switch is required, if not in pre-emptive mode. */
    vTaskMissedYield( );
        
#   if( 1 == configUSE_PREEMPTION )
    {
        /* Check if the current task is in the non-reentrant MOS.
         * If so, then indicate that a context switch is needed.
         */
        if( 1 == uxSemaphoreGetCountFromISR( portMOSMutex ))
        {
#           if defined( _DEBUG )&& 0
                _putchf( 'A' );
#           endif
            /* not blocked on MOS, so do the task yield from ISR */
            asm( "\t push hl        ; in lieu of prologue stack frame" );        
            vTaskSwitchContext( );
            asm( "\t pop hl         ; " );
        } 
        else
        {
#           if defined( _DEBUG )&& 0
                _putchf( 'B' );
#           endif
            /* blocked on MOS; indicate a context switch is required. */
            vTaskMissedYield( );
        }
    }
#   endif /* configUSE_PREEMPTION == 1 */

#   if defined( _DEBUG )&& 0
        _putchf( 'C' );
#   endif

    /* RESTORE CONTEXT is shared with vPortYield and vPortYieldFromTick,
       so that we must do a long return "reti.L" to the ISR and a context
       switched task.
       It SHALL be the last action prior to pop ix; reti.L
    */
    portRESTORE_CONTEXT( );

    asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
    asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
    asm( "               ;   __default_mi_handler" );
    asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );  // *2
    asm( "\t reti.L      ; Long ret (ADL_CALL_MODE_LIL + 3 byte PC)" ); // *3
    asm( "               ; as per UM007715 table 25 for IM 2 ADL=1 MADL=1" );
    asm( "               ; compiler-inserted epilogue below will not be executed" );
}


/* vPortYieldFromTick
 *   Context switch function used by the tick.  
 *
 *   Entry: With all of the "vPortYield" functions the FIRST action SHALL be to
 *     (*1) call portSAVE_CONTEXT. Even in non-preemptive configuration, we 
 *     MUST save the context in order to preserve the AF flags register.
 *   Exit: With all of the "vPortYield" functions we MUST exit in the identical 
 *     sequence FOLLOWING the call to vTaskSwitchContext onwards:
 *            vTaskSwitchContext( );
 *            portRESTORE_CONTEXT( );
 *            asm( "\t pop    ix    ; postlog         " );
 *         *2 asm( "\t ei           ; re-enable interrupts " ); *2
 *            asm( "\t reti.L       ; ret from here   " );
 *            asm( "                ; compiler-inserted postlog below not followed " );
 *   The reason Entry and Exit must be identical is that a task may be Yielded
 *     out by one function, and Yielded back in by another. Between Entry and 
 *     Exit each Yield function can do what it needs. 
 *
 *     *1 We must design the function to use no local stack variables. If the 
 *        compiler allocates space on the stack e.g. push BC then we must undo 
 *        that with 'pop BC' before calling portSAVE_CONTEXT. 
 *        Check this by examining the port.src file output by the compiler.
 *     *2 following di in portSAVE_CONTEXT for the MANUAL Yield only, NOT in 
 *        ISR Yields (which will themselves do a reti)
*/
static void vPortYieldFromTick( void )
{
    /* push BC     is inserted by "Zilog eZ80 ANSI C Compiler Version 3.4 (19101101)"
       following the standard prolog. We don't want it left on the task stack. */
    asm( "\t pop    bc   ; undo compiler 'push BC' from the stack" );  // *1

    asm( "_vPortYieldFromTick_1:      ; entry point for jump" );

    /* The current task context SHALL be saved first; 
       even if 0==configUSE_PREEMPTION. */
    portSAVE_CONTEXT( );

    /* The Tick Count SHALL be incremented; 
       returns pdTRUE if any higher priority task is ready to run */
    if( pdFALSE != xTaskIncrementTick( ))
    {
        /* A context switch is required. */
        vTaskMissedYield( );
    }
    
#   if( 1 == configUSE_PREEMPTION )
    {
        /* Check if the current task is in the non-reentrant MOS.
         * If so, then indicate that a context switch is needed.
         */
        if( 1 == uxSemaphoreGetCountFromISR( portMOSMutex ))
        {
#           if defined( _DEBUG )&& 0
                _putchf( 'E' );
#           endif
            /* not blocked on MOS, so do the task yield from tick 
               xTaskIncrementTick is done in this function */
            asm( "\t push hl        ; in lieu of prologue stack frame" );        
            vTaskSwitchContext( );
            asm( "\t pop hl         ; " );
        } 
        else
        {
#           if defined( _DEBUG )&& 0
                _putchf( 'F' );
#           endif
            /* blocked on MOS; indicate a context switch is required. */
            vTaskMissedYield( );
        }
    }
#   endif /* configUSE_PREEMPTION == 1 */

    /* RESTORE CONTEXT is shared with vPortYield and vPortYieldFromTick,
       so that we must do a long return "reti.L" to the ISR and a context
       switched task. 
       It SHALL be the last action prior to pop ix; reti.L */
    portRESTORE_CONTEXT( );

    asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
    asm( "\t             ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
    asm( "\t             ;   __default_mi_handler" );
    asm( "\t ei          ; re-enable interrupts (after following reti instruction executes)" );  // *2
    asm( "\t reti.L      ; Long ret (ADL_CALL_MODE_LIL + 3 byte PC) de-stacking" ); // *3
    asm( "\t             ;   as per UM007715 tables 25/86 for IM 2 ADL=1 MADL=1" );
    asm( "\t             ; compiler-inserted epilogue below will not be executed" );
}


/*------ Timer ISR ----------------------------------------------------------*/
/* timer_isr is an ISR bound to a PRT countdown expiry event, 
     via the MOS interrupt vector table.
   - MOS runs in mixed mode (MADL=1), so all ISRs begin in ADL mode.
      timer_isr is invoked following an interrupt from the assigned PRT 
      (on-chip vectored timer peripheral) to the eZ80 CPU.
   - Refer to UM007715 Interrupts in Mixed Memory Mode Applications, 
      Table 25 ADL=1 MADL=1 (same as per IM 2 table 24).
   - An ISR will run on the stack of the current FreeRTOS task, 
      and exit from the context-switched task (in _vPortYieldFromTick). */
static void timer_isr( void )
{
    /* We can clear the timer interrupt down ahead of saving the task context,
       provided no general purpose registers are modified. */
    asm( "\t push af           ; preserve AF for SAVE CONTEXT" );
    asm( "\t in0 a,(83h)       ; clear PRT1 (page 0) iflag down" );
    asm( "\t pop af            ; restore AF for SAVE CONTEXT" );

    asm( "\t jp _vPortYieldFromTick_1 " );
}


/*------ Timer attachment ---------------------------------------------------*/
/*
 * Setup timer.
 *   Bind an EZ80F92 TMR as the Periodic Interval Timer function. 
 *   Any of PRT0..PRT3 can be assigned; PRT4..PRT5 cannot be used for PIT.
 *   MOS uses PRT0 to initialise UART0; and at runtime in the i2s driver 
 *   (for read and write), and the SD-card driver.
 *
 *   Algorithm (long): loop over PRT1..PRT3 to find the first believed free 
 *   device: if the previous device and this device are bound to the same 
 *   handler (guess it's the MOS default handler, __default_mi_handler),then 
 *   assign this one; typically PRT2.
 *
 *   Algorithm (short): assign PRT1 to FreeRTOS PIT
 *
 *   Program the selected PRT as a continuous PIT at the configurable Hz.
 *   Refer to eZ80F92 Product Specification (PS015317-0120) section on
 *   Programmable Reload Timers. 
 *   Return: pdPASS if we allocated a PRT, else pdFAIL.
 */
static BaseType_t prvSetupTimerInterrupt( void )
{
    BaseType_t r = pdPASS;
    unsigned char iss;
    unsigned int ctl;
    void( *prev )( void ) = NULL;
    volatile unsigned char __INTIO *tmr_ctl;
    volatile unsigned char __INTIO *tmr_rrh;
    volatile unsigned char __INTIO *tmr_rrl;
    unsigned int reload;
    int i;

    /* Assign PRT1 for FreeRTOS */
    portTmr = 1;
    portPrevprev = mos_setpitvector(( PRT0_IVECT +( portTmr * 2 )), timer_isr );
        
        /* Applications are free to use PRT2 or PRT3 */

#   if defined( _DEBUG )
    {
        ( void )printf( "Assigned Timer %d to FreeRTOS tick\r\n", portTmr );
    }
#   endif
#   if defined( _DEBUG )&& 0
    {
        ( void )printf( "%s : %d : vector 0x%x\r\n", "port.c", 
                        __LINE__, ( PRT0_IVECT +( portTmr * 2 )) );
    }
#   endif

    /* Timer Source Select;
       input for the periodic timer (TMR0_IN) shall be the system clock 
       (divided by the clock divider). Refer to eZ80F92 ProdSpec, section 
       Programmable Reload Timers table 37 */
    iss = TMR_ISS;
    iss &= ~( unsigned char )( PRT_ISS_SYSCLK <<( portTmr * 2 ));
#   if defined( _DEBUG )&& 0
    {
        ( void )printf( "%s : %d : iss %d\r\n", "port.c", __LINE__, iss );
    }
#   endif
    TMR_ISS =( unsigned char )iss;

    /* program Timer i Reload Registers;
       refer to eZ80F92 ProdSpec, section Programmable Reload Timers table 
       35-36 */
    tmr_rrl =( volatile unsigned char __INTIO* )( 0x81 +( portTmr * 3 ));
    tmr_rrh =( volatile unsigned char __INTIO* )( 0x82 +( portTmr * 3 ));
#   if defined( _DEBUG )&& 0
    {
        ( void )printf( "%s : %d : tmr_rrl 0x%x : tmr_rrh 0x%x\r\n", 
                        "port.c", __LINE__, tmr_rrl, tmr_rrh );
    }
#   endif

    /* RRH:RRL are each 8-bit registers nd onr 16-bit register as a pair.
       Refer to eZ80F92 ProdSpec, section Setting Timer Duration.
       TimerPeriod(s) =(( PRT_CLK_PRESCALER * reload )/ configCPU_CLOCK_HZ )
         e.g. 0.1(s) =((       64          * 28800  )/     18432000 )       */
    reload =( configCPU_CLOCK_HZ / PRT_CLK_PRESCALER / configTICK_RATE_HZ );
    configASSERT(( reload & 0xffff )== reload );
    reload &= 0xffff;
    *tmr_rrh =( unsigned char )( reload >> 8 );
    *tmr_rrl =( unsigned char )( reload & 0xFF );

    tmr_ctl =( volatile unsigned char __INTIO* )( 0x80 +( portTmr * 3 ));
    ctl = *tmr_ctl;  /* clear bit 7 PRT_IRQ, by reading CTL */

    /* program TMR0_CTL and enable the timer */
    *tmr_ctl =( PRT_CTL_IRQ_EN |
                PRT_CTL_MODE_CONTINUOUS |
                PRT_CTL_DIV64  |         // cross-relate with PRT_CLK_PRESCALER
                PRT_CTL_RST_EN |
                PRT_CTL_ENABLE );

#   if defined( _DEBUG )&& 0
    {
        ctl = *tmr_ctl;   /* PRT_CTL_RST_EN will be cleared (undocumented but expected) */
        ( void )printf( "%s : %d : ctl = 0x%x\r\n", "port.c", __LINE__, ctl );
    }
#   endif
    
    return( r );
}


/*
 * Teardown timer.
 *   If the application exits (shouldn't ever happen, but anyway...) then we
 *   need to unbind the PRT ISR to prevent the system calling our ISR anymore.
 */
static void portTeardownTimerInterrupt( void )
{
    volatile unsigned char __INTIO *tmr_ctl;
    
    if( 0 <= portTmr )
    {
        tmr_ctl =( volatile unsigned char __INTIO* )( 0x80 +( portTmr * 3 ));

        /* disable TMR_CTL */
        *tmr_ctl =( unsigned char )0;
    }
    
    if( NULL != portPrevprev )
    {
        mos_setpitvector(( PRT0_IVECT +( portTmr * 2 )), portPrevprev );        
    }

    portTmr = -1;
    portPrevprev = NULL;
}


/*----- Utility functions ---------------------------------------------------*/
void portAssert( char *file, unsigned int line )
{
#   if defined( _DEBUG )
    {
        ( void )printf( "Assertion at %s : %d\r\n", file, line );
    }
#   endif
    
    for( ;; );   /* set a BREAKPOINT here */
}


static void portTaskExit( void )
{
#   if defined( _DEBUG )
    {
        ( void )printf( "%s : %d : portTaskExit\r\n", "port.c", __LINE__ );
    }
#   endif
    
    for( ;; );   /* set a BREAKPOINT here */
}
