<h1>FreeRTOS/MOS for Agon Light</h1>

<h2>Description</h2>
FreeRTOS port for the Zilog eZ80-based Agon Light (and compatibles) running MOS.
The concept is very much FreeRTOS over MOS, reflected in the project name.
FreeRTOS provides concurrency and time, and MOS provides the system services.

<p>
This port integrates FreeRTOS version 20221201-LTS (10.5.1) with the 
Zilog ZDSII eZ80Acclaim! version 5.3.5 (Build 23020901) toolset.
Refer to the README for the detailed description of the Agon port in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/

<p>
The choice of FreeRTOS version 20221201-LTS prioritised stability over latest 
and greatest.

<h3>What is FreeRTOS?</h3>
Put succinctly, FreeRTOS allows an application to be divided into a number of
concurrent tasks. The core of FreeRTOS (and indeed any RTOS) is its kernel; 
refer to https://www.freertos.org/RTOS.html. 
And to https://www.freertos.org/features.html for the API.

<h3>What is Agon?</h3>
Agon Light™ is a fully open-source 8-bit microcomputer and microcontroller in 
one small, low-cost board. Refer to https://github.com/TheByteAttic/AgonLight.
It uses two processor sub-systems: the eZ80 main processor which runs MOS (and
where FreeRTOS application programs will run); the ESP32 terminal processor,
which primarily performs I/O functions.

<h3>What is MOS?</h3>
MOS is the Machine Operating System for Agon Light and compatibles.
Refer to https://agonconsole8.github.io/agon-docs/MOS/.

<h3>Demos</h3>
The FreeRTOS / MOS for Agon port provides the source and binaries to two demo 
programs, which are found in ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/DemoAgonC/ 
and ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/DemoAgonP/. These are MOS application
programs, so that you can download the files .../DemoAgonC/Debug/DemoAgonC.bin 
and .../DemoAgonP/Debug/DemoAgonP.bin, copy them to your SD card, load, and run 
them from the MOS prompt.

<p>
DemoAgonP uses pre-emptive multi-tasking. Two tasks run without knowledge of 
each other, each making MOS calls. The CPU (and MOS) is shared between them. 
Although they have no knowledge of each other, MOS in non-reentrant so that a 
second task will be blocked in making a MOS call until a first completes a MOS 
call. These example tasks also make time delay calls, such that task1 runs once 
per second, and task runs 3.3 times per second, to better demonstrate the
pre-emptive execution model.

<p>
DemoAgonC uses cooperative multi-tasking. Two tasks run without explicit 
knowledge of each other, but in the knowledge other tasks also need CPU time. 
Since they complete a MOS call before yielding, neither will be blocked by the 
other doing similar. 
These example tasks do not make time delay calls, to better demonstrate the 
speed of FreeRTOS / MOS on Agon.

<h2>Build</h2>
To build this project you will need to install the Zilog II eZ80Acclaim! toolkit
on your development machine. In addition to the GUI, compiler, assembler and 
linker tools, this provides header and library files used by FreeRTOS for Agon.

<p>
The port has been developed using using ZDS II - eZ80Acclaim! 5.3.5 (Build 23020901) 
with Compiler Version 3.4 (19101101). Other versions of these tools should be 
okay; though the state of maintenance may result in errors different to those 
encountered and worked around.

<h3>hex2bin</h3>
On compilation success, the output will be a ".hex" file. This needs to be
converted to a ".bin" file using a utility like hex2bin found at
https://sourceforge.net/projects/hex2bin/files/latest/download

<h2>AgDev?</h2>
If you prefer to use the AgDev compiler instead of ZDS, you can Fork FreeRTOS/
MOS for Agon and try building it with AgDev. This will create a new port (each 
port of FreeRTOS is the combination of target hardware / compiler). 

<p>
You will at the least need to provide the heap location in the linker directive 
file, or find another way to locate the heap memory; or, choose another heap
memory solution (there are five options in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/portable/MemMang). 

<p>
A small number of Zilog header files are used, such as ez80.h and eZ80F92.h. 
So it should be possible to copy them over or replace them. Likewise, the Zilog
specific libraries linked are for the C runtime, so that replacing them with 
AgDev ones should be doable. 
No other concern springs to mind immediately...

<h2>ToDo</h2>
This is an alpha port of FreeRTOS / MOS for Agon Light. It is the essential 
port of FreeRTOS, as proof of concept. I have built both Debug and Release
versions, although I only work with the Debug builds at present. 
The alpha port supports putch (printf) [through init.asm in each project] 
and MOS function 14 setIntVector [through mosvec24.asm under portable], needed
to attach the tick ISR.

<p>
A list of ToDos and improvements include providing support for other MOS 
functions; and I daresay a number of bug fixes... to catch those elusive resets
I now need to buy that $100 ZUSBASC0200ZADG debug device, at
https://www.mouser.com/datasheet/2/240/Littelfuse_ZUSBASC0200ZACG_Data_Sheet-3078266.pdf

<p>
Rather than putdown the somewhat buggy Zilog tools, I would like to see them 
rescued from the current maintenance contract and made Open Source. What is 
there is fundamentally good - and would be better off in more caring hands.
