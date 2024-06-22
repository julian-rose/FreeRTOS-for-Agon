<h1>FreeRTOS portable for eZ80-based Agon Light running MOS</h1>

<h2>Description</h2>
FreeRTOS for the Zilog eZ80, ported to the Agon Light (and compatible) boards, 
using the Zilog ZDSII toolset. This port is integrated with FreeRTOS version 
20221201-LTS (10.5.1) and the Zilog eZ80 ANSI C Compiler Version 3.4 (19101101). 
We set out for stability in porting to the newer FreeRTOS kernel, rather than go 
for latest and greatest.

The Agon Light port of FreeRTOS runs in ADL mode, on top of MOS to have access 
to its services, and hence through to the on-board (ESP32) VDP Terminal 
Processor services. 

<h3>Origin</h3>
The origin of the EZ80 port dates from 2010 and is found here: 
https://interactive.freertos.org/hc/en-us/community/posts/210028706-eZ80-using-ZDSII-4-11
Refer also to: https://www.freertos.org/portez80.html but the source code is 
no longer hosted there, and the comment about "#if (...)" is resolved in 20221201-LTS. 
This origin port was integrated with FreeRTOS version 2008xx (5.x).

<h3>Contributors</h3>
The Zilog eZ80 Acclaim! port based on FreeRTOS 5.x was created by Marcos A. 
Pereira from May 2005 (according to https://forums.freertos.org/t/ez80f91-zdsii-port/2864). 
This was updated by Jean-Michel Roux (date unknown, but presumably sometime between 2008-2010). 
The Roux version was added to interactive.freertos.org by Richard Barry in March, 2010. 
The port to Agon Light and FreeRTOS 10.5.1 was made by Julian Rose in May, 2024.

<h3>Porting guidelines</h3>
http://www.realtimeengineers.com/Contributing-files-to-FreeRTOS.html
Although we expect to always be a thrid party contrinution, nonetheless we 
follow the modern convention. This includes assignment of copyright (to 
Amazon Web Services - where Richard Barry now works).

<h3>Locating portmacro.h</h3>
Agon FreeRTOS project settings should locate the PATH to the include file 
./Source/portable/ThirdParty/Community-Supported-Ports/ZDS II/eZ80AgonLight/portmacro.h 
Define ZDSII_EZ80_AGON in Project->settings->C->preprocessor->ProcessorDefinitions.
This brings in portmacro.h through ./Source/include/portable.h/deprecated_definitions.h

<h3>Tasks</h3>
In place of the kernel ./Source/tasks.c it is temporarily necessary to use
./Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/tasks.c  The 
ZDSII compiler generates error "P3: Internal Error(0x83BAF1)" casting away 
global variables. The workaround in the ./Source/portable tasks.c is to 
preprocess out the offending statements at the end of function 
vTaskStartScheduler. A bug report is made to Zilog and will wait on their 
fix to revert back to the proper ./Source/tasks.c 

<h3>Heap Memory</h3>
Each port may choose one from five supplied alternative implementations of heap 
memory management. The Agon port uses heap_3, which relies on the compiler-
provided malloc() and free() implementation. Heap memory space is configured 
between limits in two variables, _heaptop and _heapbot, defined in the linker 
directive file. 
<p>

It was necessary to replace the ZDSII sbrk.asm for FreeRTOS. The Zilog (library)
memory model assumes stack is located above the heap; their malloc routine tests 
the SP register and fails if that condition does not hold. (Refer to the C 
program memory model in UM014425::malloc and around p:315.) However, in FreeRTOS 
we allocate a stack for each task from the heap; such that the Zilog malloc will 
not work after we start the FreeRTOS scheduler. This necessitated a rework of 
the sbrk code, so that it correctly tests allocated memory lies between the 
__heapbot and __heaptop limits. This works for all cases, both before and after
starting the FreeRTOS scheduler.
<p>

(Zilog's routine tests the SP register to check for stack corruption, which is
laudible. However, their implementation of malloc likely predates introduction 
of their linker directives. Zilog should now update their library and tools 
such that runtime stack checking is made a build option, to be performed, say, 
on entry to each library function. We would disable this option and perform our
own stack checking in FreeRTOS builds. Ticket raised to Zilog.)

<h3>Real-Time</h3>
MOS uses the Real-Time Clock capability of the VDP. (eZ80 pin RTC_VDD is tied 
to GND on both Agon Light and Light2 boards.) The RTC time of day clock will 
need to be set on each invocation as it is not battery backed up. 

<h3>Interrupts</h3>
FreeRTOS requires at least a Periodic Interval Timer (PIT) or tick. (Actually,
it doesn't if you prefer to use co-operative task yielding; you may remove 
PRT_CTL_ENABLE from the initialisation in prvSetupTimerInterrupt.) We assign an
EZ80 PRT in port.c to tick. This is done through the MOS interface, so our ISR 
is bound by MOS in its vector table.

<h3>portMOSMutex</h3>
MOS is not re-entrant; which means that each application MOS call has to be 
completed before another can be made. Moreover, MOS (may) use interrupts to 
communicate with the ESP32 Terminal Processor; so we cannot use the traditional 
portENTER_CRITICAL to disable interrupts to guard application MOS calls. 
So, this port provides portMOSMutex through application calls to portEnterMOS 
to guarantee MOS function safety. Initially, applications call portEnterMOS;
in future releases portEnterMOS will be moved within a library of MOS calls, 
hiding it from applications.

<h3>Shared Libraries</h3>
We note here the Zilog ZDSII MAth Library is non-reentrant. If more than one 
task uses the Math library, then access needs to be guarded as per a critical 
region. It is possible other shared libraries need to be likewise access with 
mutual exclusion.

<h3>Startup</h3>
Startup code is located in the file init.asm, based on 
Agon-Projects-main -> C -> Hello. This begins with the MOS standard header 
and C application startup code. This code is located in ./Source/mos/init.asm

<h3>mosvec24</h3>
This started life as the Zilog vectors24.asm file, but is much changed.
It now contains the timer ISR code. This file is located in the 
./Source/portable/ThirdParty/Community-Supported-Ports/ZDS II/eZ80AgonLight/
folder, because it is used by the portable code.
