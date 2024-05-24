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
 * mosapi.h for Agon Light
 *
 * Ported to Agon Light by Julian Rose in 2024, with copyright made over
 * to Amazon Web Services as above for FreeRTOS version 10.5.
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * Definitions for MOS interface functions for the FreeRTOS portable code
 * for Agon Light (and compatibles).
*/

#ifndef MOSAPI_H
#define MOSAPI_H


/***--------- Type definitions ---------***/


/***--------- MOS API functions ---------***/
	/* MOS_API: mos_getkey:			EQU	00h
	   Get ASCII code of any key pressed, or 0 if no key pressed, non-blocking
	   Defined in mosapi24.asm
	   Usage: while( 0 ==( ch = _mos_getkey( ))); // busy wait until a key is pressed */
unsigned char mos_getkey( void );


	/* MOS_API: mos_load:			EQU	01h
	   Load a file from SD card
	   Defined in mosapi24.asm */
unsigned int mos_load( 
				char * const filename, 
				void * const address, 
				unsigned int const size );


	/* MOS_API: mos_save:			EQU	02h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_cd:				EQU	03h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_dir:			EQU	04h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_del:			EQU	05h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_ren:			EQU	06h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_mkdir:			EQU	07h
	   Load a file from SD card
	   Defined in mosapi24.asm */

	/* MOS_API: mos_sysvars:		EQU	08h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_editline:		EQU	09h    NOT SUPPORTED
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_fopen:			EQU	0Ah
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_fclose:			EQU	0Bh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_fgetc:			EQU	0Ch
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_fputc:			EQU	0Dh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_feof:			EQU	0Eh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_getError:		EQU	0Fh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_oscli:			EQU	10h   NOT SUPPORTED
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_copy:			EQU	11h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_getrtc:			EQU	12h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_setrtc:			EQU	13h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS API: mos_setintvector	EQU	14h
	   Assigns a service routine to an interrupt vector
	   Defined in mosvec24.asm as it is used by the FreeRTOS port to bind the
	   PIT timer */
void *mos_setintvector( unsigned int, void ( * )( void ));


	/* MOS_API: mos_uopen:			EQU	15h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_uclose:			EQU	16h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_ugetc:			EQU	17h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_uputc:			EQU 18h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_getfil:			EQU	19h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_fread:			EQU	1Ah
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_fwrite:			EQU	1Bh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_flseek:			EQU	1Ch
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_setkbvector:	EQU	1Dh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_getkbmap:		EQU	1Eh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_i2c_open:		EQU	1Fh
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_i2c_close:		EQU	20h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_i2c_write:		EQU	21h
	   Load a file from SD card
	   Defined in mosapi24.asm */


	/* MOS_API: mos_i2c_read:		EQU	22h
	   Load a file from SD card
	   Defined in mosapi24.asm */


#endif /* MOSAPI_H */
