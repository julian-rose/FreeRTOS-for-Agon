<h1>FreeRTOS/MOS for Agon Light</h1>


<h2>Description</h2>
FreeRTOS port for the Zilog eZ80-based Agon Light (and compatibles) running MOS.
The concept is very much FreeRTOS over MOS, reflected in the project name.
FreeRTOS provides concurrency and time, and MOS provides the system services.
<p>

This Agon Light port integrates FreeRTOS version 20221201-LTS (10.5.1) with 
the Zilog ZDSII eZ80Acclaim! version 5.3.5 (Build 23020901) C language toolset. 
The choice of version 20221201-LTS prioritises stability over latest and 
greatest. For a detailed description of the Agon port, refer to the README in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/

<h3>What is FreeRTOS?</h3>
FreeRTOS is a real-time software development kernel and library. It provides an 
application development framework that allows a C language application to be 
arranged into a number of concurrent tasks. The core of FreeRTOS (and indeed 
any RTOS) is its kernel; refer to https://www.freertos.org/RTOS.html. And to 
https://www.freertos.org/features.html for the API.
<p>

FreeRTOS is not an operating system in the sense of MOS or CP/M, which provide
a command console interpreter and long-term storage. Instead, FreeRTOS for Agon 
applications run on MOS to access its services. No re-Flashing is required -
you just build and link a C language application together with the FreeRTOS 
software into a MOS executable. FreeRTOS, while a general framework, is of 
particular interest for using Agon as a micro-controller.

<h4>Why do we care about concurrency and time?</h4>
In a nutshell, because they are in the real world. And a micro-controller 
interacts with the real world, through sensors and actuators. So we need to 
embrace time and the order of events (concurrency) into our software for it 
to function well. 

<h3>What is Agon?</h3>
Agon Light™ is a fully open-source 8-bit microcomputer and microcontroller in 
one small, low-cost board. Refer to https://github.com/TheByteAttic/AgonLight.
It embeds two processor sub-systems: the eZ80-based main processor, which runs 
MOS (and where FreeRTOS programs will run); and the ESP32-based VDP terminal 
processor, which primarily performs I/O functions.

<h3>What is MOS?</h3>
MOS is the Machine Operating System for Agon Light and compatibles. It runs on 
the eZ80 and provides an API to access the eZ80 on-chip peripherals, on-board 
interfaces including the VDP terminal co-processor, and long-term file storage 
through SD-cards. Refer to https://agonconsole8.github.io/agon-docs/MOS/.
<p>

<h4>MOS versions</h4>
FreeRTOS for Agon is built to MOS version 1.04. Version 1.03 may work, but has 
not been tested. Likewise newer Console8 versions of MOS should work, but have 
not been tested. Console8-specific MOS and VDP functions are not yet supported. 
There are a small number of tightly coupled dependencies in parts of the code 
(such as the keyboard read functions) which may fail if differences exist 
between versions of the MOS code.

<h2>Project</h2>
FreeRTOS / MOS for Agon Light is a highly configurable, multi-capability 
project. "Alpha" is the essential capability (and the initial port of FreeRTOS 
as proof of concept running in eZ80 ADL mode). The Alpha capability supports 
all the FreeRTOS functions; and a minimal number of MOS services, namely putch 
(printf) and getch (scanf), and MOS function 14 setIntVector needed to attach 
the tick ISR.

<h3>Capabilities</h3>
<ul>
  <li>alpha:     FreeRTOS API, with a minimal MOS API, in eZ80 ADL mode</li>
  <li>beta:      MOS API (MOS 1.04 subset of the FFS API), plus a DEV API</li>
  <li>gamma:     VDP API (MOS 1.04 subset)</li>
  <li>delta:     Console8 MOS & VDP extended API (only if i buy a second Agon)</li>
  <li>epsilon:   a real-time library inspired by the posix-4 API</li>
  <li>omega:     a safer version with Z80-mode tasks and the ADL-mode kernel</li>
</ul>

Each capability is configurable through user-settable definitions in config 
header files. In this way each application only need include the capabilities 
it requires, such that the FreeRTOS code and the built executable occupy the 
least RAM footprint possible.
<p>

<h4>Alpha</h4>
The "Alpha" capability is already provided, with the FreeRTOS source located in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/ and the eZ80 portable code in
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/.
You can already use "Alpha" to develop full FreeRTOS applications that will run 
on Agon. 
<p>

<h4>Beta</h4>
The "Beta" capability is currently being developed, with the sources located in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/mos/. This capability adds support for
for most of the MOS API defined in
https://agonconsole8.github.io/agon-docs/MOS-API/#the-mos-api
The MOS command line functions (mos_dir, mos_oscli, mos_ediline) are not 
provided; and only the subset of FFS that does not require Console8 MOS 2.2.0 
or greater are provided (that means no ffs_dopen, ffs_dclose, ffs_dread).
<p>

In addition to the MOS API, an extended DEV API is provided for safeguarded 
access to SPI, UART, I2C and GPIO.

<h3>Demos</h3>
The FreeRTOS / MOS for Agon port provides the source and binaries to 'Demo'
programs. These serve both as tests and as a starting point for new user
applications. They are MOS application programs, so that you can download 
them, copy them to your Agon Light SD card, load, and run them from the MOS 
prompt.

<h4>Alpha Demos</h4>
Alpha demos are found in ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/DemoAgonC/ 
and ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/DemoAgonP/. You can open the 
.../DemoAgonC/Debug/DemoAgonC.zdsproj and .../DemoAgonC/Debug/DemoAgonP.zdsproj
files in ZDSII. You can find the pre-built executable files in
.../DemoAgonC/Debug/DemoAgonC.bin and .../DemoAgonP/Debug/DemoAgonP.bin
<p>

DemoAgonP uses pre-emptive multi-tasking. Two tasks run without knowledge of 
each other, each making MOS calls. The CPU (and MOS) is shared between them. 
Although they have no knowledge of each other, MOS in non-reentrant so that a 
second task will be blocked in making a MOS call (such as printf) until a 
first completes a MOS call. These example tasks also make time delay calls, 
such that task1 runs once per second, and task runs 3.3 times per second, to 
better demonstrate the pre-emptive execution model.
<p>

DemoAgonC uses cooperative multi-tasking. Two tasks run without explicit 
knowledge of each other, but in the knowledge other tasks also need CPU time. 
Since they complete a MOS call before yielding, neither will be blocked by the 
other doing similar. The DemoAgonC tasks do not make time delay calls, to 
better demonstrate the speed of FreeRTOS / MOS on Agon.
<p>

<h4>Beta Demos</h4>
A Beta demo is found in ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/DemoMOS/.
DemoMOS uses the MOS API to access MOS services, including files, directory,
and devices including the keyboard, Uart, I2C, SPI and GPIO serial interfaces. 
Beta projects include the configuration file mosConfig.inc, to select which 
APIs should be linked into the executable, mirroring the FreeRTOSConfig.h file;
and the API library files too.

<h3>License</h3>
FreeRTOS / MOS for Agon is released under the MIT license. This is done mainly
for consistency with the existing MOS and core FreeRTOS software. 

<h3>Performance</h3>
Because one FreeRTOS task may block another, and because MOS is non-reentrant
(see below), FreeRTOS / MOS provides a "soft" real-time solution (rather than 
a "hard" one.) A "soft" solution is one in which the response time is not 
guaranteed, and may lag real time by a period (for now we might assume in the 
order of a milli-second, though some tests are needed to provide actual 
measurement data). 


<h2>Build</h2>
To build the demos, and to develop your own application programs, you will need 
to install the Zilog II eZ80Acclaim! toolkit on your PC-host development 
machine. In addition to the GUI, compiler, assembler and linker tools, this 
provides header and library files used by FreeRTOS for Agon.
<p>

The FreeRTOS port has been developed using using ZDS II - eZ80Acclaim! 5.3.5 
(Build 23020901) with Compiler Version 3.4 (19101101). Other recent versions 
of these tools should be okay; though the state of tool maintenance may result 
in errors different to those encountered and worked around.

<h3>hex2bin</h3>
The output from successful compilation will be a ".hex" file. This needs to be
converted to a ".bin" file for loading onto your Agon Light, which is done 
using a utility like hex2bin found at
https://sourceforge.net/projects/hex2bin/files/latest/download


<h2>Debugging</h2>
To catch those elusive resets (refer to UM007715 Illegal Instruction Traps)
in case of stack corruption or other bugs, I now possess a $100 ZUSBASC0200ZADG 
debug device from
https://www.mouser.com/datasheet/2/240/Littelfuse_ZUSBASC0200ZACG_Data_Sheet-3078266.pdf
At the moment it doesn't work on my (Windows 11) laptop :-(  ticket is with
Zilog.
<p>

To use the ZUSBASC0200ZADG Acclaim! smart cable hardware debugger, you will need 
version 5.3.5 of the ZDSII tools. Though, in general, you will not need the 
debugger to build applications, it may speed-up your development time if you do 
use it. FreeRTOS / MOS "alpha" was made running without a debugger, but it gave 
my brain a workout in doing so at times. 

<h3>Compiler Bugs</h3>
There are bugs in the ZDS tools. Rather than dismiss the compiler, I would like 
to see the Zilog tools rescued from their current maintenance contract and made 
Open Source. What is there is fundamentally useful - and would be better off in 
more careful hands.

<h3>Re-entrancy</h3>
I will point out, because you will run into it, that the Zilog libraries are
not re-entrant. C is not in itself a concurrent programming language. Neither
Zilog nor AgDev provide re-entrant libraries. This means that if two FreeRTOS
tasks interleave (make simultaneous) calls to library functions, your program 
may fail or lock up, or cause an Agon reset (through an illegal instruction in 
the case of stack or program counter corruption, or in case of math libraries 
through a divide by zero).
<p>

Your solution is to guard such calls to library functions by use of a semaphore 
or similar method available in FreeRTOS. 
<p>

To give an example, my first demo programs contained two tasks, each of which 
used a loop counter that was modulo tested (<it>if( 0 ==( ctr % 80 ))</it>...). 
Modulo <it>%</it> is implemented in Zilog using the math library. It took
several days until that 'doh' moment, to realise what was causing random
hardware resets. My solution was just to change the loop tests, but I could
have used semaphores around the <it>if</it> statement.
<p>

MOS itself is likewise non-rentrant. The port of FreeRTOS / MOS project takes 
care of this by guarding calls to MOS with a semaphore. This ensures two 
concurrent FreeRTOS tasks will not corrupt MOS or VDP calls. This comes at the 
cost of task-blocking - which means time passes un-usefully for the blocked 
task - but is the correct solution.


<h2>AgDev port</h2>
If you prefer to use the LLVM AgDev compiler instead of ZDS, you can Fork 
FreeRTOS/MOS for Agon and try it. This will create a new port (each port of 
FreeRTOS is the combination of target hardware / compiler). 
<p>

You will at the least need to provide the heap location in the linker directive 
file, or find another way to locate the heap memory; or, choose another heap
memory solution (there are five options in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/portable/MemMang). 
<p>

A small number of Zilog header files are used with the ZDS port, such as ez80.h 
and eZ80F92.h. It should be possible to copy them over or replace them with
equivalent AgDev ones. Similarly, the Zilog specific libraries linked are for 
the C runtime, so that replacing them with AgDev ones should be doable. 
