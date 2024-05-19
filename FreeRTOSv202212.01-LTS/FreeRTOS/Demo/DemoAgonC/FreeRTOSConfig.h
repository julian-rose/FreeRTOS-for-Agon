/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 *  This file is an application-specific configuration for building FreeRTOS
 *  It is application and user-changeable on a per application basis,
 *  to target particular hardware and application requirements.
 *
 *  Application configuration parameters are described in the configuration 
 *  section  of the FreeRTOS API documentation on FreeRTOS.org web site.
 *  See also https://www.freertos.org/a00110.html
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H


/* General kernel configuration */
	// configENABLE_BACKWARD_COMPATIBILITY enables pre-8.0 API (the eZ80 port was originally 5.0)
#define configENABLE_BACKWARD_COMPATIBILITY		1
#define configUSE_PREEMPTION					0
#define configCPU_CLOCK_HZ						(( unsigned portLONG )18432000 )
#define configTICK_RATE_HZ						(( portTickType )10 )
#define configUSE_16_BIT_TICKS					0
#define configUSE_TIME_SLICING					1
#define configIDLE_SHOULD_YIELD					1

/* Stack */
	// configSTACK_DEPTH_TYPE should be a port definition (do not change it from int)
#define configSTACK_DEPTH_TYPE          		int
#define configRECORD_STACK_HIGH_ADDRESS			0
#define configCHECK_FOR_STACK_OVERFLOW			0
#define configMINIMAL_STACK_SIZE				(( portSTACK_TYPE )512 )

/* Memory allocation */
#define configSUPPORT_DYNAMIC_ALLOCATION		1
#define configSUPPORT_STATIC_ALLOCATION			0
	// configure __heaptop and __heapbot in the linker directive (linkcmd) file
#define configTOTAL_HEAP_SIZE					(( size_t )(( unsigned int )&_heaptop -( unsigned int )&_heapbot ))

/* Hooks */
#define configUSE_IDLE_HOOK						1
#define configUSE_TICK_HOOK						0
#define configUSE_MALLOC_FAILED_HOOK			0

/* Software timers */
#define configUSE_TIMERS						0

/* Tasks */
	// 0 is lowest (idle) priority, configMAX_PRIORITIES - 1 is highest priority
#define configMAX_PRIORITIES					5
#define configMAX_TASK_NAME_LEN					16
#define configUSE_TASK_NOTIFICATIONS			1
#define configUSE_COUNTING_SEMAPHORES			0
#define configUSE_QUEUE_SETS					0
#define configUSE_MUTEXES						1

/* Co-routines */
#define configUSE_CO_ROUTINES 					0
#define configMAX_CO_ROUTINE_PRIORITIES 		2

/* Run time and task stats gathering */
#define configGENERATE_RUN_TIME_STATS			0
#if defined( _DEBUG )
#	define configUSE_TRACE_FACILITY				0
#else
#	define configUSE_TRACE_FACILITY				0
#endif

/* Assert */
#if defined( _DEBUG )&&( 0 )
#	define configASSERT( x )					if( 0==( x )) portAssert( __FILE__, __LINE__ )
#endif

/* Extra functions
    set the following definitions to 1 to include the API function, 
                                  or zero to exclude the API function. */
#define INCLUDE_vTaskPrioritySet				1
#define INCLUDE_uxTaskPriorityGet				1
#define INCLUDE_vTaskDelete						1
#define INCLUDE_vTaskCleanUpResources			1
#define INCLUDE_vTaskSuspend					1
#define INCLUDE_vTaskDelayUntil					1
#define INCLUDE_vTaskDelay						1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_uxTaskGetStackHighWaterMark2    1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1

#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          0

#endif /* FREERTOS_CONFIG_H */
