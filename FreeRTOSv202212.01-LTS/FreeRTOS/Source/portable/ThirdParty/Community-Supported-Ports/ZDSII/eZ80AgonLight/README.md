<h1>FreeRTOS portable for eZ80-based Agon Light running MOS</h1>

<h2>Description</h2>
FreeRTOS for the Zilog eZ80, ported to the Agon Light (and compatible) boards, 
using the Zilog ZDSII toolset. This port is integrated with FreeRTOS version 
20221201-LTS (10.5.1) and the Zilog eZ80 ANSI C Compiler Version 3.4 (19101101). 
We set out for stability in porting to the newer FreeRTOS kernel, rather than go 
for latest and greatest.

The Agon Light port of FreeRTOS runs on top of MOS and willon have access to 
its services, and hence through to the on-board (ESP32) Terminal Processor 
services. In its initial incarnation only putch and MOS function 0x14 
mos_setintvector are supported. Others will follow...

<h3>Origin</h3>
The origin of the EZ80 port dates from 2010 and is found here: 
https://interactive.freertos.org/hc/en-us/community/posts/210028706-eZ80-using-ZDSII-4-11
Refer also to: https://www.freertos.org/portez80.html  But the source code is 
no longer there, and the comment about "#if (...)" is resolved in 20221201-LTS. 
This origin port was integrated with FreeRTOS version 2008xx (5.x).

<h3>Contributors</h3>
The Zilog eZ80 Acclaim! port based on FreeRTOS 5.x was created by Marcos A. 
Pereira around 2008. This was updated by Jean-Michel Roux (date unknown, but 
presumably sometime between 2008-2010). The Roux version was added to 
interactive.freertos.org by Richard Barry in March, 2010. The initial port to 
Agon Light based on FreeRTOS 10.5.1 was made by Julian Rose in May, 2024.

<h3>Porting guidelines</h3>
http://www.realtimeengineers.com/Contributing-files-to-FreeRTOS.html
This includes assignment of copyright (to Amazon Web Services).

<h3>Locating portmacro.h</h3>
Agon FreeRTOS project settings should locate the PATH to the include file 
./Source/portable/ThirdParty/Community-Supported-Ports/ZDS II/eZ80AgonLight/portmacro.h 
Define ZDSII_EZ80_AGON in Project->settings->C->preprocessor->ProcessorDefinitions.
This brings in portmacro.h through ./Source/include/portable.h/deprecated_defifnitions.h

<h3>Tasks</h3>
In place of the kernel ./Source/tasks.c it is temporarily necessary to use
./Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/tasks.c 
The ZDSII compiler generates error "P3: Internal Error(0x83BAF1)" casting away 
global variables. The workaround in the port tasks.c is to preprocess out the 
offending statements at the end of function vTaskStartScheduler. A bug report 
is made to Zilog and will wait on their fix to revert back to the proper 
FreeRTOS tasks.c source.

<h3>Heap Memory</h3>
Each port may choose one from five supplied alternative implementations of heap 
memory management. ZDSII uses heap_3, which is the version that relies on the 
compiler-provided malloc() and free() implementation. 
Heap memory space is configured using two variables, _heaptop and _heapbot,
defined in the linker directive file. 

<h3>Interrupts</h3>
FreeRTOS requires at least a Periodic Interval Timer (PIT) or tick. (Actually,
it doesn't if you prefer to use co-operative task yielding; you may remove 
PRT_CTL_ENABLE from the initialisation in prvSetupTimerInterrupt.) We assign an
EZ80 PRT in port.c to tick. This is done through the MOS interface, so our ISR 
is bound by MOS in its vector table.

<h3>portMOSMutex</h3>
MOS is not re-entrant; which means that each application MOS call has to be 
completed without interruption. Moreover, MOS (may) use interrupts to 
communicate with the ESP32 Terminal Processor; so we cannot use the traditional 
portENTER_CRITICAL to disable interrupts to guard application MOS calls. 
So, this port provides portMOSMutex through application calls to portEnterMOS 
to guarantee MOS function safety. Initially, applications call portEnterMOS;
in future releases portEnterMOS will be moved within a library of MOS calls, 
hiding it from applications.
