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
 *
 * These functions should not normally be application-user altered.
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
#include "mosapi.h"


/*----- Global Names --------------------------------------------------------*/

    /* We require the address of the (tasks.c) pxCurrentTCB variable, 
       but don't need to know its type. */
typedef void tskTCB;
extern tskTCB * volatile pxCurrentTCB;


/*----- Private Variables ---------------------------------------------------*/
    /* we use a mutex to implement critical regions */
static SemaphoreHandle_t portMOSMutex = NULL;

/* we keep track of our installed ISR, to remove it in case of exit */
static void( *portPrevprev )( void ) = NULL;
static int portTmr = -1;


/*----- Private Functions ---------------------------------------------------*/
static BaseType_t prvSetupTimerInterrupt( void );
static void portTaskExit( void );
static void portTeardownTimerInterrupt( void );


/*----- Function Definitions ------------------------------------------------*/
/*
 * Macro to save all the general purpose registers, 
 *  then save the stack pointer into the TCB.
 *    [jhr Think about "in 0 a,(80h)" for EZ80L92 from https://www.freertos.org/FreeRTOS_Support_Forum_Archive/March_2007/freertos_Bug_in_the_ez80_1687732.html
 *    Along the lines of EZ80F91 port which did "in0 a,(62h)" to read the timer interrupt bit
 *    Agon reads the timer separately in the ISR]
 */
#define portSAVE_CONTEXT( )                 \
    asm( "\t xref _pxCurrentTCB         " );\
    asm( "\t di                         " );\
    asm( "\t push af                    " );\
    asm( "\t push bc                    " );\
    asm( "\t push de                    " );\
    asm( "\t push hl                    " );\
    asm( "\t push iy                    " );\
    asm( "\t ex   af,   af'             " );\
    asm( "\t exx                        " );\
    asm( "\t push af                    " );\
    asm( "\t push bc                    " );\
    asm( "\t push de                    " );\
    asm( "\t push hl                    " );\
    asm( "\t ld   ix,   0               " );\
    asm( "\t add  ix,   sp              " );\
    asm( "\t ld   hl,   (_pxCurrentTCB) " );\
    asm( "\t ld   (hl), ix              " );

/*
 * Macro to restore the stack pointer from the new TCB,
 *  then restore all the general purpose registers, 
 *  Exact opposite of SAVE CONTEXT.
 */
#define portRESTORE_CONTEXT( )           \
    asm( "\t xref _pxCurrentTCB      " );\
    asm( "\t ld  hl, (_pxCurrentTCB) " );\
    asm( "\t ld  hl, (hl)            " );\
    asm( "\t ld  sp, hl              " );\
    asm( "\t pop hl                  " );\
    asm( "\t pop de                  " );\
    asm( "\t pop bc                  " );\
    asm( "\t pop af                  " );\
    asm( "\t exx                     " );\
    asm( "\t ex  af, af'             " );\
    asm( "\t pop iy                  " );\
    asm( "\t pop hl                  " );\
    asm( "\t pop de                  " );\
    asm( "\t pop bc                  " );\
    asm( "\t pop af                  " );


/* Initialse the stack for a task
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
    *pxTopOfStack-- =( portSTACK_TYPE )pxCode;

    /* ZDS II Stack Frame saved by the compiler (IX Register). */
    *pxTopOfStack-- =( portSTACK_TYPE )0x111111;  /* IX  */

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


/*---------------------------------------------------------------------------*/
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
        portMOSMutex = xSemaphoreCreateMutex( );
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
        if(( NULL != portMOSMutex )&&( taskSCHEDULER_RUNNING == xTaskGetSchedulerState( )))
        {
            /* suspend until semaphore can be taken */
            while( pdTRUE != xSemaphoreTake( portMOSMutex, 0 ))
            {
                /* Some other task has the mutex.
                    If we use vTaskSuspend then the tick ISR will have
                    to call vTaskResume. Easier to use the delayed list.
                    Could be better using notifications?
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
        if(( NULL != portMOSMutex )&&( taskSCHEDULER_RUNNING == xTaskGetSchedulerState( )))
        {
            xSemaphoreGive( portMOSMutex );
        }
    }
#   endif /* configUSE_PREEMPTION */
    
#   if defined( _DEBUG )&& 0
        ( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#   endif
}


/*---------------------------------------------------------------------------*/
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
            asm( "\t ret          ; ret into Task   " );
            asm( "                ; postlog below not followed " );
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


/*---------------------------------------------------------------------------*/
/*
 * Manual context switch.  
 *      Save the current task context
 *      Decide which is the new task to run
 *      Restore the new task context
 */
void vPortYield( void )
{
    /* push BC     is inserted by "Zilog eZ80 ANSI C Compiler Version 3.4 (19101101)"
       following the standard prolog. But it is neither popped in the postlog, nor 
       by the caller on return from vPortYield. We don't want it left on the task
       stack. */
    asm( "\t pop    bc   ; remove pushed BC from the stack" );

    portSAVE_CONTEXT( );
    vTaskSwitchContext( );
    portRESTORE_CONTEXT( );

    asm( "\t pop    ix    ; postlog         " );
    asm( "\t ei           ; re-enable interrupts " );
    asm( "\t ret          ; ret from here   " );
    asm( "                ; compiler-inserted postlog below not followed " );
}


/* vPortYieldFromTick
 *   Context switch function used by the tick.  
 *   This must be identical to vPortYield() from the call to vTaskSwitchContext() 
 *   onwards.  
 *   We have to save the context, even in non-preemptive configuration,
 *   in order to preserve the AF flags register */
static void vPortYieldFromTick( void )
{
    /* The current task context SHALL be saved first; 
       even if 0==configUSE_PREEMPTION. */
    portSAVE_CONTEXT( );

    /* The Tick Count SHALL be incremented; 
       returns pdTRUE if any higher priority task is ready to run */
    if( pdFALSE != xTaskIncrementTick( ))
    {
        /* A context switch is required. */
        asm( "\t push hl        ; in lieu of prologue stack frame" );        
        vTaskMissedYield( );
        asm( "\t pop hl         ; " );
    }
    
#   if( 1 == configUSE_PREEMPTION )
    {
        /* Check if the current task is accessing the non-reentrant MOS.
         * If MOS is in use, then indicate that a context switch is needed.
         */
        if( pdTRUE == xSemaphoreTakeFromISR( portMOSMutex, NULL ))
        {
            ( void )xSemaphoreGiveFromISR( portMOSMutex, NULL );

            /* not blocked on MOS, so do the task yield from tick 
               xTaskIncrementTick is done in this function */
            asm( "\t push hl        ; in lieu of prologue stack frame" );        
            vTaskSwitchContext( );
            asm( "\t pop hl         ; " );
        } 
        else
        {
            /* blocked on MOS; indicate a context switch is required. */
            asm( "\t push hl        ; in lieu of prologue stack frame" );        
            vTaskMissedYield( );
            asm( "\t pop hl         ; " );
        }
    }
#   endif /* configUSE_PREEMPTION == 1 */

    portRESTORE_CONTEXT( );

    /* RESTORE CONTEXT is shared with port yield, so that we must "ret" to a 
       calling task. In this case, the calling tick ISR shall reti */
    asm( "\t pop    ix    ; return from here," );
    asm( "\t ret          ; so that we don't modify the stack pointer" );
    asm( "                ; in the compiler epilog that follows" );
        /* following compiler-inserted epilogue is not executed */
}

/* timer_isr is an ISR bound to a PRT countdown expiry event, 
     via the MOS interrupt vector table.
   - MOS runs in mixed mode (MADL=1), so all ISRs begin in ADL mode.
     timer_isr is invoked following an interrupt from that PRT 
     (on-chip vectored peripheral) to the eZ80 CPU.
   - Refer to UM007715 Interrupts in Mixed Memory Mode Applications, 
     Table 25 ADL=1 MADL=1 (same as per IM 2 table 24).
   - An ISR will run on the stack of the current FreeRTOS task */
static void timer_isr( void )
{
    /* We can clear the timer interrupt down ahead of saving the task context,
       provided no general purpose registers are modified. */
    asm( "\t push af           ; preserve AF for SAVE CONTEXT" );
    asm( "\t in0 a,(83h)       ; clear PRT1 (page 0) iflag down" );
    asm( "\t pop af            ; restore AF for SAVE CONTEXT" );

    vPortYieldFromTick( );

    asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
    asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
    asm( "               ;   __default_mi_handler" );
    asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );
    asm( "\t reti.l      ; need reti.L as per UM007715 table 25 for IM 2 ADL=1 MADL=1");

    /* following compiler-inserted epilogue is not executed */
}


/*---------------------------------------------------------------------------*/
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
    portPrevprev = mos_setintvector(( PRT0_IVECT +( portTmr * 2 )), timer_isr );
        
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
        mos_setintvector(( PRT0_IVECT +( portTmr * 2 )), portPrevprev );        
    }

    portTmr = -1;
    portPrevprev = NULL;
}


/*---------------------------------------------------------------------------*/
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
