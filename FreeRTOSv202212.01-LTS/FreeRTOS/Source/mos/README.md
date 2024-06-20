<h1>Sources to FreeRTOS for eZ80-based Agon Light running MOS</h1>

<h2>Description</h2>
MOS API definitions including FFS API, MOS devices API, and FreeRTOS-extended 
DEV API
<p>

The MOS API functions provided here for FreeRTOS wrap up calls to the MOS API 
published in https://agonconsole8.github.io/agon-docs/MOS-API/. 
And the FFS API functions to wrap up calls to the FatFS API published in
https://agonconsole8.github.io/agon-docs/MOS-API/#fatfs-commands

These API implementations are provided in a new ./Source/mos/ folder 
containing the MOS API and the FFS API source files.

<h3>init.asm</h3>
The modified file init.asm is based on Agon-Projects-main -> C -> Hello. 
It contains the MOS standard header and C application startup code.

<h3>mos_api.inc</h3>
A lightly modified copy of mos_api.inc is found in .Source/mos/mosapi.h file.

<h3>mosapi.h</h3>
C Prototypes for the Agon FreeRTOS / MOS API functions are located in the 
.Source/mos/mosapi.h file.

<h3>mosapi24.asm</h3>
MOS API function definitions are located in the ./Source/mos/mosapi24.asm file.

<h3>ffsapi.h</h3>
C Prototypes for the Agon FreeRTOS / MOS FFS API functions are located in the 
.Source/mos/ffsapi.h file.

<h3>ffsapi24.asm</h3>
MOS API FFS function definitions are located in the ./Source/mos/ffsapi24.asm file.

<h3>mosdapi24.asm</h3>
MOS API Device function definitions are located in the ./Source/mos/devapi24.asm file.

<h3>devapi.h</h3>
C Prototypes for the Agon FreeRTOS / MOS DEV API functions are located in the 
.Source/mos/devapi.h file.

<h3>devapi.c</h3>
Generic DEV API Device function definitions are located in the ./Source/mos/devapi.c file.

<h3>devapil.h</h3>
C Prototypes for the low-level DEV API functions are located in the .Source/mos/devapil.h file.

<h3>devgpio.c</h3>
Low-level DEV API Device function definitions for GPIO are located in the ./Source/mos/devgpio.c file.

<h3>devi2c.c</h3>
Low-level DEV API Device function definitions for I2C are located in the ./Source/mos/devi2c.c file.

<h3>devspi.c</h3>
Low-level DEV API Device function definitions for SPI are located in the ./Source/mos/devspi.c file.

<h3>devuart.c</h3>
Low-level DEV API Device function definitions for UART are located in the ./Source/mos/devuart.c file.
