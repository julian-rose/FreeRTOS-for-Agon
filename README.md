<h1>FreeRTOS/MOS for Agon Light</h1>


<h2>Description</h2>
FreeRTOS port for the Zilog eZ80-based Agon Light (and compatibles) running 
MOS. The concept is very much FreeRTOS over MOS, reflected in the project name. 
MOS provides system services and APIs, which FreeRTOS builds on with multi-
tasking communication and concurrency, real-time, and extended hardware APIs. 
<p>

This Agon Light port integrates FreeRTOS version 20221201-LTS (10.5.1) with 
the Zilog ZDSII eZ80Acclaim! version 5.3.5 (Build 23020901) C language toolset. 
The LTS choice prioritises stability over latest and greatest. For a detailed 
description of the Agon port, refer to the README in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/

<h3>What is FreeRTOS?</h3>
FreeRTOS is a real-time, multi-tasking software development library. It is a 
fully open-source application development framework that allows a C language 
application to be arranged into a number of concurrent tasks. The core of 
FreeRTOS (and indeed any RTOS) is its multi-tasking kernel; refer to the 
overview at https://www.freertos.org/RTOS.html. 
And to https://www.freertos.org/features.html for the developer overview;
and to https://www.freertos.org/a00106.html for the programmers API reference.
<p>

We use FreeRTOS in targetting Agon as a Micro-controller. FreeRTOS is neither a 
standalone nor installable operating system in the sense of MOS or CP/M. Those 
O/S provide command console interpreter (CCP), long-term storage (BDOS) and a 
basic I/O system (BIOS) services. Rather, FreeRTOS applications run on MOS to 
access its services. No re-Flashing is required - you just build and link your 
C language application together with the FreeRTOS software into a MOS 
executable. 

<h4>Why do we care about concurrency and time?</h4>
In a nutshell, because they are in the real world; and a micro-controller 
interacts with the real world, through sensors and actuators, networked
devices and co-processors. So we need to embrace time and event ordering 
(concurrency) into our software for it to function well. We are also
interested in efficiency and response time to real-world inputs and outputs.

<h3>What is Agon?</h3>
Agon Light™ is a fully open-source 8-bit microcomputer and microcontroller in 
one small, low-cost board. Refer to https://github.com/TheByteAttic/AgonLight.
It embeds two processor sub-systems: the eZ80-based main processor, which runs 
MOS (and where FreeRTOS programs will run); and the ESP32-based VDP terminal 
processor, which primarily performs I/O and graphics functions.
<p>

Agon is well-suited to the role of micro-controller through the hardware 
Extensions Interface, providing UART, SPI, I2C and GPIO connectivity. Moreover, 
with its integrated VDP, applications can include a GUI to visualise the 
system under control, or to graph I/O in real-time. 

<h3>What is MOS?</h3>
MOS is the Machine Operating System for Agon Light and compatibles. It runs on 
the eZ80 and provides an API to access eZ80 on-chip peripherals, on-board 
interfaces including the VDP terminal co-processor, and long-term file storage 
through SD-cards. Refer to https://agonconsole8.github.io/agon-docs/MOS/.
<p>

<h4>MOS versions</h4>
FreeRTOS for Agon is built to Quark MOS version 1.04. Version 1.03 may work, 
but has not been tested. FreeRTOS will work with Console8 2.x versions of MOS, 
but has not been tested. Console8-specific MOS and VDP functions are not yet 
supported. 

<h2>Project</h2>
FreeRTOS / MOS for Agon Light is a highly configurable, multi-capability 
project. 

<h3>Capabilities</h3>
<ul>
  <li>alpha:     FreeRTOS API, with a minimal MOS API, in eZ80 ADL mode;
                 the multi-tasking API</li>
  <li>beta:      MOS API (MOS 1.04 subset of the FFS API), plus a DEV API; 
                 the hardware interfaces API</li>
  <li>gamma:     VDP API (MOS 1.04 subset); the graphics API</li>
  <li>delta:     Console8 2.x MOS and VDP extended API; the extended graphics 
                 API (may not happen)</li>
  <li>epsilon:   a real-time library inspired by the posix-4 API; the real-time
                 API</li>
  <li>omega:     a safer version with (protected memory) Z80-mode tasks and the
                 ADL-mode kernel (may not happen)</li>
</ul>

Each capability is configurable through user-settable definitions in 
application-specific config header files. The different capabilities are not 
mutually exclusive; a user-application can be configured to include the exact 
mix of capabilities it requires. In this way the built executable occupies the 
least RAM footprint possible.
<p>

<h4>Alpha</h4>
"Alpha" is the essential capability (and the initial port of FreeRTOS 
as proof of concept running in eZ80 ADL mode). The Alpha capability supports 
all the FreeRTOS functions; the Zilog Standard C library functions; and a 
minimal number of MOS services, namely putch (printf) and getch (scanf).
<p>

The "Alpha" capability is already provided, with the FreeRTOS source located in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/ and the eZ80 portable code in
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/.
You can already use "Alpha" to develop full standard C, FreeRTOS applications 
that will run on Agon. 
<p>

Alpha capabilities are configured in the usual FreeRTOS way, through the
header file FreeRTOSConfig.h, customised for each user application.

<h4>Beta</h4>
The "Beta" capability is currently being developed, with the sources located in 
./FreeRTOSv202212.01-LTS/FreeRTOS/Source/mos/. 
This capability adds support for most of the MOS API defined in
https://agonconsole8.github.io/agon-docs/MOS-API/#the-mos-api (including the
FFS API), plus an additional DEV API. 
<p>

The MOS command line functions (mos_dir, mos_oscli, mos_ediline) are not 
supported; and only the subset of FFS that does not require Console8 MOS 2.2.0 
or greater are provided (that means no ffs_dopen, ffs_dclose, ffs_dread).
<p>

The MOS API is implemented in Assembly language for best performance. In 
addition to FreeRTOSConfig.h used to configure Alpha capabilities, users 
configure MOS API capabilities through an application-specific mosConfig.inc 
header file, to select which MOS APIs should be linked into the executable.
<p>

In addition to the MOS API, Beta capabilties include a new DEV API. This 
extends the Beta capability with access to the Agon Extensions Interface 
directly. It provides a safeguarded API for SPI, UART, I2C and GPIO; and 
through them to connected devices. The DEV API enhances the Uart and I2C 
capability of MOS, using concurrency to provide asynchronous full duplicity 
and multi-mastering (slave) roles respectively, and provides APIs for GPIO 
and SPI that are absent in the MOS API. 
<p>

Users configure the DEV API through an application-specific devConfig.h file, 
to select the extended interfaces for linking into the executable. 

<h3>Demos</h3>
The FreeRTOS / MOS for Agon port provides the source and binaries to 'Demo'
programs. These serve both as tests and as a starting point for new user
applications. They are MOS application programs, so that you can download 
them, copy them to your Agon Light SD card, load, and run them from the MOS 
prompt.

<h4>Alpha Demos</h4>
Alpha demos are found in ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/Alpha/DemoAgonC/ 
and ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/Alpha/DemoAgonP/. You can open the 
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
A Beta demo is found in ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/Beta/DemoMOS/.
DemoMOS uses the MOS API to access MOS services, including files, directory,
and MOS devices including the keyboard, UART, and I2C interfaces. 
<p>

A second Beta demo is found in ./FreeRTOSv202212.01-LTS/FreeRTOS/Demo/Beta/DemoDEV/.
This demonstrates the DEV API capabilities. Currently that is the full GPIO, 
UART and SPI APIS, together with the I2C Single-Master role API. DEV I2C Multi-
Master role remains in development, and requires peer-Agon testing.

<h5>Bosch BMP280 barometer bad batch?</h5>
Note testing DEV SPI revealed a Bosch BMP280 barometer mfg#CFC-KU mounted on 
PCB#GY-BM ME/PM 280 read back all zeros; this could be the BMP280 batch. 
Another with mfg#DHE-KL mounted on PCB#E/P 280 tested fine.

<h3>License</h3>
FreeRTOS / MOS for Agon is released under the MIT license. This is done mainly
for consistency with the existing MOS and core FreeRTOS software. 

<h3>Performance</h3>
Because one FreeRTOS task may block another, and because MOS is non-reentrant
(see below), FreeRTOS / MOS provides a "soft" real-time solution (rather than 
a "hard" one.) A "soft" solution is one in which the response time is not 
guaranteed, and may lag real time by a period (for now we might assume in the 
order of a milli-second, though tests are needed to provide actual measurement 
data). 


<h2>Build</h2>
To build the demos, and to develop your own application programs, you will need 
to install the Zilog II eZ80Acclaim! toolkit on your PC-host development 
machine. In addition to the GUI, compiler, assembler and linker tools, this 
provides header and library files used by FreeRTOS for Agon.
<p>

The FreeRTOS port has been developed using using ZDS II - eZ80Acclaim! 5.3.5 
(Build 23020901) with Compiler Version 3.4 (19101101). Other recent versions 
of these tools should be okay. The FreeRTOS code is mainly written in standard
C with some assembler. But it may be configuration differences manifest with 
other versions of the Zilog tools. 

<h3>hex2bin</h3>
The output from successful compilation will be a ".hex" file. This needs to be
converted to a ".bin" file for loading onto your Agon Light, which is done 
using a utility like hex2bin found at
https://sourceforge.net/projects/hex2bin/files/latest/download


<h2>Debugging</h2>
In general, you will not need a debugger to build FreeRTOS applications. But it 
may speed up your development time and help you to find bugs if you do use one. 
I now possess a $100 ZUSBASC0200ZADG Acclaim! Smart Cable (ASC) debug device, 
bought from
https://www.mouser.com/datasheet/2/240/Littelfuse_ZUSBASC0200ZACG_Data_Sheet-3078266.pdf
To use this hardware debugger, you will need version 5.3.5 of the ZDSII tools.
<p>

It turns out the Zilog Acclaim Smart Cable device driver (found in
ZDSII_eZ80Acclaim!_5.3.5\device drivers\USB\AcclaimSmartCable) is not digitally 
signed. This results in a Windows Device Manager Code 52 exclamation sign. So 
that enabling it requires UEFI Secure Boot to be disabled in the PC BIOS 
settings. This is the official Zilog solution, as per a customer support 
ticket. Refer to:
https://learn.microsoft.com/en-us/windows-hardware/manufacture/desktop/disabling-secure-boot?view=windows-11
<p>

To give an example of its benefit: there was a flaw in the "Alpha" context 
switch code, such that FreeRTOS wasn't exiting interrupts properly. I used the 
debugger to breakpoint the executable and identify the cause. This enabled a 
satisfactory fix, so the context switch now works exactly as it should. 
Moreover, I now understand a bit more about ADL mode call stacks on the eZ80. 

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

To give an example, DemoAgonC contains two tasks, each of which originally used 
a loop counter that was modulo tested (<it>if( 0 ==( ctr % 80 ))</it>...). 
Modulo <it>%</it> is implemented using the Zilog math library. It took several 
days until that 'doh' moment, to realise what was causing random hardware 
resets. This was before I owned the debugger. My solution was just to change 
the loop tests, but I could have used semaphores around the <it>if</it> 
statement.
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

A small number of Zilog header files are used with the ZDS port, such as 
ez80.h, eZ80F92.h in Alpha, and uartdefs.h in Beta. It should be possible to 
copy them over or replace them with equivalent AgDev ones. Similarly, the Zilog 
specific libraries linked are for the C runtime, so that replacing them with 
AgDev ones should be doable. 
