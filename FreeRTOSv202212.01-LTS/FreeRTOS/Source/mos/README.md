<h1>FreeRTOS portable for eZ80-based Agon Light running MOS</h1>

<h2>Description</h2>
MOS API definitions
<p>

The MOS API functions provided here for FreeRTOS wrap up calls to the MOS API published in 
https://agonconsole8.github.io/agon-docs/MOS-API/

Decided to create a new ./Source/mos/ folder containing the MOS API files.

<h3>mosapi.h</h3>
C Prototypes for the Agon FreeRTOS / MOS API functions are located in the 
.Source/mos/mosapi.h file.

<h3>mosapi24.asm</h3>
MOS API function definitions are located in the ./Source/mos/mosapi.asm file.

<h3>init.asm</h3>
The file init.asm is based on Agon-Projects-main -> C -> Hello. It contains the MOS standard header (which defines MOSLETs)
and C application startup code.

<h3>mosvec24</h3>
This started life as the Zilog vectors24.asm file, but is much changed.
It now contains the timer ISR code. This file is located in the 
./Source/portable/ThirdParty/Community-Supported-Ports/ZDS II/eZ80AgonLight/
folder, because it is used by the portable code.
