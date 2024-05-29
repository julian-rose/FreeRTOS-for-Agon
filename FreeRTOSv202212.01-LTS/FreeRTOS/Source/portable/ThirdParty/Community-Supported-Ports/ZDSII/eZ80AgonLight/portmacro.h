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
 * portmacro.h for Agon Light
 *
 * Evolved from the earlier eZ80 port by Marcos A. Pereira around 2008. 
 * This was updated by Jean-Michel Roux prior to 2010.
 * Ported to Agon Light by Julian Rose in 2024, with copyright made over
 * to Amazon Web Services as above for FreeRTOS version 10.5.
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The settings in this file configure the FreeRTOS portable code
 * for Agon Light (and compatibles) and the ZDSII compiler
 * Zilog eZ80 ANSI C Compiler Version 3.4 (19101101).
 *
 * These settings should not normally be application-user altered.
*/

#ifndef PORTMACRO_H
#   define PORTMACRO_H


/***--------- Type definitions ---------***/
#define portCHAR        		  char
#define portFLOAT       		  float
#define portDOUBLE      		  double
#define portLONG        		  long
#define portSHORT       		  short
#define portSTACK_TYPE  		  unsigned int
#define portBASE_TYPE   		  int
#define portPOINTER_SIZE_TYPE	  unsigned int

typedef portSTACK_TYPE   		  StackType_t;
typedef portBASE_TYPE    		  BaseType_t;
typedef unsigned portBASE_TYPE    UBaseType_t;


/***-------- Inline functions are not supported by ZDSII --------***/
#define inline


/***--------- Ticks ---------***/
#if ( configUSE_16_BIT_TICKS == 1 )
    typedef unsigned portSHORT 	  portTickType;
#   define portMAX_DELAY          ( portTickType ) 0xffff
#else
   typedef unsigned portLONG 	  portTickType;
   #define portMAX_DELAY 		  ( portTickType ) 0xffffffff
#endif
typedef portTickType 			  TickType_t;


/***--------- Memory Protection Unit ---------***/
/* The eZ80 does not have an MMU */
#define portUSING_MPU_WRAPPERS    0
#define PRIVILEGED_FUNCTION


/***--------- Critical Sections ---------***/
#define	portENTER_CRITICAL( )     asm( "\t di" )
#define portEXIT_CRITICAL( )      asm( "\t ei" )
#define portDISABLE_INTERRUPTS( ) asm( "\t di" )
#define portENABLE_INTERRUPTS( )  asm( "\t ei" )

void portEnterMOS( void );
void portExitMOS( void );


/***--------- Architecture ---------***/
#define portSTACK_GROWTH          ( -1 )
#define portTICK_PERIOD_MS        (( portTickType )1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT        3


/***------ Programmable Timer ------***/
/* We assign any of PRT0..PRT4 for the FreeRTOS periodic interval timer (PIT) 
   function. PRT5..PRT6 cannot be used for PIT.
   Refer to eZ80F92 PS015317-0120 Table 32 for PRT Control Register */
#define PRT_CTL_ENABLE	          ( 1 << 0 )
#define PRT_CTL_RST_EN			  ( 1 << 1 )
#define PRT_CTL_DIV04			  ( 0 << 2 )	/* configTICK_RATE_HZ = 72 is the smallest possible */
#define PRT_CTL_DIV16			  ( 1 << 2 )	/* configTICK_RATE_HZ = 18 is the smallest possible */
#define PRT_CTL_DIV64			  ( 2 << 2 )	/* configTICK_RATE_HZ = 5 is the smallest possible */
#define PRT_CTL_DIV256			  ( 3 << 2 )	/* configTICK_RATE_HZ = 2 is the smallest possible */
#define PRT_CTL_MODE_CONTINUOUS	  ( 1 << 4 )
#define PRT_CTL_IRQ_EN			  ( 1 << 6 )
#define PRT_ISS_SYSCLK			  ( unsigned char )(( 1 << 0 )|( 1 << 1 ))

#define PRT_CLK_PRESCALER		  64UL  		/* one of 16UL, 64UL or 256UL to match PRT_CTL_DIVnn */


/***--------- Kernel ---------***/
extern void vPortYield( void );
#define portYIELD()               vPortYield( )


/***--------- Tasks ---------***/
	/* since we split up tasks into multiple files,
	   we need the tasks variable declarations to be global. */
#define portREMOVE_STATIC_QUALIFIER	1

	/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) \
								  void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) \
								  void vFunction( void *pvParameters )


/***------------ Assert --------------***/
#if defined( configASSERT )
    void portAssert( char *file, unsigned int line );
#endif


#endif /* PORTMACRO_H */
