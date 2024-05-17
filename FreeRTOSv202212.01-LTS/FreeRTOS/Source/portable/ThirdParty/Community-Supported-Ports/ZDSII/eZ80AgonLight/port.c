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
 * Evolved from the earlier eZ80 port by Marcos A. Pereira around 2008. 
 * This was updated by Jean-Michel Roux prior to 2010. (I think Roux modified 
 * portENTER/EXIT_CRITICAL( ) and portSAVE_CONTEXT( ) to test the PIT state, 
 * specifically for the eZ80F91. That needed removal for the eZ80F92.)
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
/* Assigns a service routine to an interrupt vector */
void *_set_vector_mos( unsigned int, void ( * )( void ));

#include <stdlib.h>
#if defined( _DEBUG )
#	include <stdio.h>
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"


/*-----------------------------------------------------------*/

/* We require the address of the (tasks.c) pxCurrentTCB variable, 
   but don't need to know its type. */
typedef void tskTCB;
extern volatile tskTCB * volatile pxCurrentTCB;

/* we use a mutex to implement critical regions */
static SemaphoreHandle_t portMOSMutex = NULL;

/* we keep track of our installed ISR, to remove it in case of exit */
static void( *portPrevprev )( void ) = NULL;
static int portTmrUsed = -1;


/*-- Private functions --------------------------------------*/
static BaseType_t prvSetupTimerInterrupt( void );
static void portTaskExit( void );
static void portTeardownTimerInterrupt( void );


/*-- Function definitions -----------------------------------*/
/*
 * Macro to save all the general purpose registers, 
 *  then save the stack pointer into the TCB.
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
/*-----------------------------------------------------------*/

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
	
#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif

#	if( 1 == configUSE_PREEMPTION )	
	{
		portMOSMutex = xSemaphoreCreateMutex( );
		if( NULL == portMOSMutex )
		{
#			if defined( _DEBUG )
				( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#			endif
			r = pdFAIL;
		}
		else
		{
			/* make the semaphore initially available */
			xSemaphoreGive( portMOSMutex );
		}
	}
#	endif /* configUSE_PREEMPTION */

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
#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif

#	if( 1 == configUSE_PREEMPTION )
	{
		/* suspend until semaphore can be taken */
		while( pdTRUE != xSemaphoreTake( portMOSMutex, 0 ))
		{
			/* Some other task has the mutex.
			   If we use vTaskSuspend then the tick ISR will have
			   to call vTaskResume. Easier to use the delayed list.
			   Accessing MOS, after all, is not a real-time (fast)
			   operation.
			*/
			vTaskDelay( 1 );
		}
	}
#	endif /* configUSE_PREEMPTION */

#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif
}


/* Exit critical region for MOS
*/
void portExitMOS( void )
{
#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif
	
#	if( 1 == configUSE_PREEMPTION )
	{
		xSemaphoreGive( portMOSMutex );
	}
#	endif /* configUSE_PREEMPTION */
	
#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif
}
/*-----------------------------------------------------------*/

/* Initialse the stack for a task
*/
portSTACK_TYPE *pxPortInitialiseStack( 
	portSTACK_TYPE *pxTopOfStack, 
	pdTASK_CODE pxCode, 
	void *pvParameters 
	)
{
#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif
	
    /* Place the parameter on the stack in the expected location. */
    *pxTopOfStack-- =( portSTACK_TYPE )pvParameters;

    /* Place the task return address on stack. Not used*/
    *pxTopOfStack-- =( portSTACK_TYPE )&portTaskExit;

    /* The start of the task code will be popped off the stack last, so place
    it on first. */
    *pxTopOfStack-- =( portSTACK_TYPE )pxCode;

    /* ZDS II Stack Frame saved by the compiler (IX Register). */
    *pxTopOfStack =( portSTACK_TYPE )pxTopOfStack--;  /* IX  */

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
/*-----------------------------------------------------------*/

/* Start the FreeRTOS scheduler
 *    on success this routine will never return, rather tasks will run
 *    return pdTRUE if we fell out of the scheduler
 *    return pdFALSE if we failed to allocate resources
*/
portBASE_TYPE xPortStartScheduler( void )
{
	portBASE_TYPE ret = pdFALSE;

    /* Setup the hardware to generate the tick. */
#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif

    if( pdPASS == prvSetupTimerInterrupt( ))
	{
#		if defined( _DEBUG )&& 0
			( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#		endif

		if( pdPASS == portCreateMOSMutex( ))
		{
#			if defined( _DEBUG )&& 0
				( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#			endif

			/* start running the tasks */
			portRESTORE_CONTEXT( );

			asm( "\t pop    ix    ; postlog         " );
			asm( "\t ei           ; enable interrupts to start tick ISR " );
			asm( "\t ret          ; ret into Task   " );
			asm( "                ; postlog below not followed " );
			/* exit from portRESTORE_CONTEXT should be into one of the Tasks */

			/* we shouldn't get here: something has gone wrong */
#			if defined( _DEBUG )
				( void )printf( "%s : %d : fell out of scheduler\r\n", 
								"port.c", __LINE__ );
#			endif
			ret = pdTRUE;
		}
		else
		{
#			if defined( _DEBUG )
			{
				( void )printf( "%s : %d : %s\r\n", 
								"port.c", 
								__LINE__,
								"Failed to allocate MOS semaphore" );
			}
#			endif
		}
	}
	else
	{
#		if defined( _DEBUG )
		{
			( void )printf( "%s : %d : %s\r\n", 
							"port.c", 
							__LINE__,
							"Failed to allocate timer device" );
		}
#		endif
	}
	
	return( ret );
}
/*-----------------------------------------------------------*/


void vPortEndScheduler( void )
{
    /* It is unlikely that the eZ80 port will require this function as there
    is nothing to return to.  If this is required - stop the tick ISR then
    return back to main. */
	portTeardownTimerInterrupt( );
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch.  
 *      Save the current task context
 *      Decide which is the new task to run
 *      Restore the new task context
 */
void vPortYield( void )
{
	/* push BC 	is inserted by "Zilog eZ80 ANSI C Compiler Version 3.4 (19101101)"
	   following the standard prolog. But it is neither popped in the postlog, nor 
	   by the caller on return from vPortYield */
	asm( "\t pop    bc   ; remove pushed BC from the stack" );

	portSAVE_CONTEXT( );
    vTaskSwitchContext( );
	portRESTORE_CONTEXT( );

	asm( "\t pop    ix    ; postlog         " );
    asm( "\t ei           ; re-enable interrupts " );
	asm( "\t ret          ; ret from here   " );
	asm( "                ; compiler-inserted postlog below not followed " );
}
/*-----------------------------------------------------------*/

/*
 * Context switch function used by the tick.  This must be identical to
 * vPortYield() from the call to vTaskSwitchContext() onwards.  The only
 * difference from vPortYield() is the tick count is incremented as the
 * call comes from the tick ISR.
 */
void vPortYieldFromTick( void )
{
#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d : vPortYieldFromTick\r\n", "port.c", __LINE__ );
#	endif
	/* push BC 	is inserted by "Zilog eZ80 ANSI C Compiler Version 3.4 (19101101)"
	   following the standard prolog. But it is neither popped in the postlog, nor 
	   by the caller on return from vPortYield */
	asm( "\t pop    bc   ; remove pushed BC from the stack" );

	portSAVE_CONTEXT( );
	if( pdFALSE != xTaskIncrementTick( ))
	{
		/* A context switch is required. */
        vTaskMissedYield( );
	}
    vTaskSwitchContext( );

#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d : back from vTaskSwitchContext\r\n", "port.c", __LINE__ );
#	endif

	portRESTORE_CONTEXT( );

	asm( "\t pop    ix    ; postlog         " );
    asm( "\t ei           ; enable interrupts to start tick ISR " );
	asm( "\t ret          ; ret from here   " );
	asm( "                ; postlog below not followed " );
}
/*-----------------------------------------------------------*/

/* timer_isr is an ISR bound to a PRT countdown expiry event.
   It is invoked following an interrupt from the PRT to the eZ80 CPU;
   when the CPU will save the current IP and set IP (jump) to the address of
   this routine. */
void timer_isr( void )
{
	volatile unsigned char __INTIO *tmr_ctl;
    unsigned char ctl;

	tmr_ctl =( volatile unsigned char __INTIO* )( 0x80 +( portTmrUsed * 3 ));
	ctl = *tmr_ctl;  /* clear bit 7 PRT_IRQ, by reading CTL */
#	if defined( _DEBUG )&& 0
		( void )printf( "\r\n%s : %d : timer_isr ctl = 0x%x\r\n", 
							"port.c", __LINE__, ctl );
		ctl = *tmr_ctl;
		( void )printf( "%s : %d : timer_isr ctl = 0x%x\r\n", 
							"port.c", __LINE__, ctl );
#	endif

    /* now we're done with tmr_ctl and ctl,
	   pop the ix and bc registers from the stack which were pushed by 
	   Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) as part of the prologue. */
    asm ("\t inc sp      ; postlog   SP adjust after prolog");
    asm ("\t pop bc      ; postlog   BC pushed after prolog");
    asm ("\t pop ix      ; postlog   retrieve IX pushed in prolog");

#if configUSE_PREEMPTION == 1
    /*
     * Tick ISR for preemptive scheduler.  
	 * We need to check if the current task is accessing the non-reentrant MOS.
	 * If MOS is in use, then indicate that a context switch might be needed.
	 * The tick count is incremented after the context is saved.
     */
	if( pdTRUE == xSemaphoreTakeFromISR( portMOSMutex, NULL ))
	{
		xSemaphoreGiveFromISR( portMOSMutex, NULL );

		/* not blocked on MOS, so do the usual task yield from tick 
		   xTaskIncrementTick is done in this function */
		vPortYieldFromTick( );
	}
	else
	{
		/* blocked on MOS */
		if( pdFALSE != xTaskIncrementTick( ))
		{
			/* A context switch is required. */
			vTaskMissedYield( );
		}
	}
#else
    /*
     * Tick ISR for the cooperative scheduler.  All this does is increment the
     * tick count.  We don't need to switch context, this can only be done by
     * manual calls to taskYIELD();
     */
    if( pdFALSE != xTaskIncrementTick( ))
	{
		/* A context switch is required. */
		vTaskMissedYield( );
	}
#endif

	asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
	asm( "               ;   __default_mi_handler" );
	asm( "\t ei          ; re-enable interrupts " );
    asm( "\t reti.l      ; need reti.l, not just reti");
}
/*-----------------------------------------------------------*/

/*
 * Setup timer.
 *   Bind an EZ80F92 TMR as the Periodic Interval Timer function. Any of 
 *   PRT0..PRT3 can be assigned; PRT4..PRT5 cannot be used for PIT.
 *   Algorithm: loop over PRT0..PRT3 to find the first believed free device: 
 *   if the previous device and this device are bound to the same handler 
 *   (guess it's the MOS default handler, __default_mi_handler),then assign 
 *   this one; typically PRT1.
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

#	if defined( _DEBUG )&& 0
		( void )printf( "%s : %d\r\n", "port.c", __LINE__ );
#	endif

	/* Search for and assign from the available timers in PRT0..PRT4 */
	for( i = 0; i < 4; i++ )
	{
		/* set Timer interrupt vector (PRT0_IVECT) through MOS */
		prev = _set_vector_mos(( PRT0_IVECT +( i * 2 )), timer_isr );
#		if defined( _DEBUG )&& 0
			( void )printf( "%s : %d : i = %d : prev = %p\r\n", 
							"port.c", __LINE__, i, prev );
#		endif
		if( NULL != prev )
		{
			/* some ISR was already bound to this PRT */
			if( timer_isr == prev )
			{
				/* OMG it's us; we were already installed before, 
				   so if we survived a device reset then continue */
				portTmrUsed = i;
				break;
			}
			else
			{
				/* something else is already bound to this vector */
				if( portPrevprev == prev )
				{
					/* the previous device and this device are both bound to the
					   same vector; guess it's __default_mi_handler, so assign 
					   this device */
					portTmrUsed = i;
					break;
				}
				
				/* don't know if it is the default handler, so restore the
                   previous owner and try the next vector */
				_set_vector_mos(( PRT0_IVECT +( i * 2 )), prev );
				portPrevprev = prev;  /* remember it, it may be a default handler */
				continue;
			}
		}
		else
		{
			/* no previous ISR setup, so use this PRT */
			portTmrUsed = i;
			portPrevprev = NULL;
			break;
		}
	}
	

	if( 0 <= portTmrUsed )
	{
#		if defined( _DEBUG )&& 0
			( void )printf( "%s : %d : vector 0x%x\r\n", "port.c", 
							__LINE__, ( PRT0_IVECT +( portTmrUsed * 2 )) );
#		endif

		/* Timer Source Select;
		   input for the periodic timer (TMR0_IN) shall be the system clock 
		   (divided by the clock divider). Refer to eZ80F92 ProdSpec, section 
		   Programmable Reload Timers table 37 */
		iss = TMR_ISS;
		iss &= ~( unsigned char )( PRT_ISS_SYSCLK <<( portTmrUsed * 2 ));
#		if defined( _DEBUG )&& 0
			( void )printf( "%s : %d : iss %d\r\n", "port.c", __LINE__, iss );
#		endif
		TMR_ISS =( unsigned char )iss;

		/* program Timer i Reload Registers;
		   refer to eZ80F92 ProdSpec, section Programmable Reload Timers table 
		   35-36 */
		tmr_rrl =( volatile unsigned char __INTIO* )( 0x81 +( portTmrUsed * 3 ));
		tmr_rrh =( volatile unsigned char __INTIO* )( 0x82 +( portTmrUsed * 3 ));
#		if defined( _DEBUG )&& 0
			( void )printf( "%s : %d : tmr_rrl 0x%x : tmr_rrh 0x%x\r\n", 
							"port.c", __LINE__, tmr_rrl, tmr_rrh );
#		endif

		/* RRH:RRL are each 8-bit registers nd onr 16-bit register as a pair.
		   Refer to eZ80F92 ProdSpec, section Setting Timer Duration.
		   TimerPeriod(s) =(( PRT_CLK_PRESCALER * reload )/ configCPU_CLOCK_HZ )
		      e.g. 0.1(s) =((       64          * 28800  )/     18432000 )       */
		reload =( configCPU_CLOCK_HZ / PRT_CLK_PRESCALER / configTICK_RATE_HZ );
	    configASSERT(( reload & 0xffff )== reload );
		reload &= 0xffff;
		*tmr_rrh =( unsigned char )( reload >> 8 );
		*tmr_rrl =( unsigned char )( reload & 0xFF );

		tmr_ctl =( volatile unsigned char __INTIO* )( 0x80 +( portTmrUsed * 3 ));
		ctl = *tmr_ctl;  /* clear bit 7 PRT_IRQ, by reading CTL */

		/* program TMR0_CTL and enable the timer */
		*tmr_ctl =(	PRT_CTL_IRQ_EN |
					PRT_CTL_MODE_CONTINUOUS |
					PRT_CTL_DIV64  |           // cross-relate with PRT_CLK_PRESCALER
					PRT_CTL_RST_EN |
					PRT_CTL_ENABLE );

#		if defined( _DEBUG )&& 0
			ctl = *tmr_ctl;   /* PRT_CTL_RST_EN will be cleared (undocumented but expected) */
			( void )printf( "%s : %d : ctl = 0x%x\r\n", "port.c", __LINE__, ctl );
#		endif
	}
	else
	{
		/* No free Timer available */
		r = pdFAIL;
#		if defined( _DEBUG )
			( void )printf( "%s : %d : No timer available\r\n", "port.c", __LINE__ );
#		endif
	}
	
	return( r );
}
/*-----------------------------------------------------------*/

/*
 * Teardown timer.
 *   If the application exits (shouldn't ever happen, but anyway...) then we
 *   need to unbind the PRT ISR to prevent the system calling our ISR anymore.
 */
static void portTeardownTimerInterrupt( void )
{
	volatile unsigned char __INTIO *tmr_ctl;
	
	if( 0 <= portTmrUsed )
	{
		tmr_ctl =( volatile unsigned char __INTIO* )( 0x80 +( portTmrUsed * 3 ));

		/* disable TMR_CTL */
		*tmr_ctl =( unsigned char )0;
	}
	
	if( NULL != portPrevprev )
	{
		_set_vector_mos(( PRT0_IVECT +( portTmrUsed * 2 )), portPrevprev );		
	}

	portTmrUsed = -1;
	portPrevprev = NULL;
}
/*-----------------------------------------------------------*/

void portAssert( char *file, unsigned int line )
{
#	if defined( _DEBUG )
	{
		( void )printf( "Assertion at %s : %d\r\n", file, line );
	}
#	endif
	for( ;; );
}
/*-----------------------------------------------------------*/

static void portTaskExit( void )
{
#	if defined( _DEBUG )
	{
		( void )printf( "%s : %d : portTaskExit\r\n", "port.c", __LINE__ );
	}
#	endif
	for( ;; );
}
