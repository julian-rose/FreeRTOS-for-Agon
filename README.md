<h1>FreeRTOS/MOS for Agon Light</h1>

<h2>Description</h2>
FreeRTOS port for the Zilog eZ80-based Agon Light (and compatibles) running MOS.
This port uses the Zilog ZDSII toolset. Refer to the detailed README.md for the 
Agon port in ./Source/portable/Community-Supported-Ports/ZDSII/eZ80AgonLight/
<p>
The concept is very much FreeRTOS over MOS, reflected in the project name.

<h3>What is FreeRTOS</h3>
Put succinctly, FreeRTOS allows an application to be divided into a number of
concurrent tasks. 
<p>
The core of FreeRTOS (and indeed any RTOS) is its kernel. 
Refer to https://www.freertos.org/RTOS.html.<br>
And to https://www.freertos.org/features.html for the API.<p>

<h3>Demos</h3>
The Agon port provides two demos, which you can find in 
.\FreeRTOSv202212.01-LTS\FreeRTOS\Demo\DemoAgonC and
.\FreeRTOSv202212.01-LTS\FreeRTOS\Demo\DemoAgonP.
These are MOS application programs. You can copy the files DemoAgonC.bin and 
DemoAgonP.bin to your SD card, load, and run them from the MOS prompt.
<p>
DemoAgonP is a demo using pre-emptive multi-tasking. Two tasks run without
knowledge of each other, each making MOS calls. The CPU (and MOS) is shared
between them. Although they have no knowledge of each other, the second task
will be blocked in making a MOS call until the first completes a MOS call.
The tasks also make time delay calls, to better illustrate the execution model.
<p>
DemoAgonC is a demo using cooperative multi-tasking. Two tasks run without
explicit knowledge of each other, but in the knowledge other tasks also need
CPU time. Since they complete a MOS call before yielding, neither will be 
blocked by the other doing similar. These tasks do not delay, to illustrate
the speed of FreeRTOS on Agon / MOS. 

<h2>Build</h2>
To build this project you will need to install the Zilog II eZ80Acclaim! toolkit
on your development machine. In addition to the GUI, compiler, assembler and 
linker tools, this provides header and library files used by FreeRTOS for Agon.
<p>
I built the code using ZDS II - eZ80Acclaim! 5.3.5 (Build 23020901). Other
versions should be okay; though the state of maintenance may result in errors
different to those I encountered and worked around.

<h3>Rescue</h3>
Rather than putdown the Zilog compiler with its various faults, I would like to
see it rescued from its current maintenance contract and made Open Source. 
What is there is fundamentally good - it would be better off in more caring 
hands. 
I could make a similar plea for Zilog to be rescued :-)

<h3>AgDev?</h3>
You can Fork FreeRTOS/MOS for Agon, to try the AgDev tools and create a new
port (each port of FreeRTOS is the target hardware / compiler combination). 
You will at the least need to provide the heap location in a linker directive 
file, or find another way to locate the heap memory. No other concern springs
to mind immediately, but there likely will be...

<h2>ToDo</h2>
This is the alpha port of FreeRTOS / MOS for Agon Light. It is the essential 
port of FreeRTOS, as proof of concept. I have built both Debug and Release
versions, though not the latter for a few days...
The alpha port supports putch (printf) and MOS function 14 setIntVector, needed
to attach the tick ISR.
A long list of ToDos and improvements, including providing support for the
other MOS functions; and I daresay a number of bug fixes...
