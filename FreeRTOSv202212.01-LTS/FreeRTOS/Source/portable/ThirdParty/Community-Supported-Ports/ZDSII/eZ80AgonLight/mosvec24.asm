;* FreeRTOS Kernel V10.5.1
;* Copyright (C) 2024 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
;*
;* SPDX-License-Identifier: MIT
;*
;* Permission is hereby granted, free of charge, to any person obtaining a copy of
;* this software and associated documentation files (the "Software"), to deal in
;* the Software without restriction, including without limitation the rights to
;* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
;* the Software, and to permit persons to whom the Software is furnished to do so,
;* subject to the following conditions:
;*
;* The above copyright notice and this permission notice shall be included in all
;* copies or substantial portions of the Software.
;*
;* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
;* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
;* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
;* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
;* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
;*
;* https://www.FreeRTOS.org
;* https://github.com/FreeRTOS
;*
;*  Please ensure to read the configuration and relevant port sections of the
;*  online documentation.
;*
;*  http://www.FreeRTOS.org - Documentation, latest information, license and
;*  contact details.
;*
;*  mosvec24.asm
;*
;*  derived from vectors24.asm Copyright (c) 2007-2008 Zilog, Inc.
;*  Modified for the Agon Light port
;*    from .\ZDSII_eZ80Acclaim!_5.3.5\src\rtl\common\vectors24.asm
;*    refer to eZ80F92 Product Specification "PS015317-0120"
;*
;*****************************************************************************
include "mos_api.inc"

;*        xref __init                   ; performed by MOS
;*        xdef _reset                   ; performed by MOS
;*        xdef __default_nmi_handler    ; MOS src_startup/vectors16.asm
;*        xdef __default_mi_handler     ; MOS src_startup/vectors16.asm

        xdef __nvectors
;*        xdef _init_default_vectors    ; deprectaed in favour of MOS vectors
;*        xdef __init_default_vectors
;*        xdef __vector_table
		xdef _mos_setintvector


NVECTORS EQU 102      ; number of potential interrupt vectors (00h to 66h)
                      ; refer to "PS015317-0120" Interrupt Controller Table 11


;************************************************************************
;*  Reset and all RST nn's
;*  refer to "PS015317-0120" Interrupt Controller.
;*  The Absolute locations 00h, 08h, 10h, 18h, 20h, 28h, 30h, 38h, and 66h 
;*  are reserved for hardware reset, NMI, and the RST instruction.
;*  These absolute vectors are contained in the Agon MOS software 
;*  defined in MOS src_startup/vectors16.asm


;*		we place interrupt handling code in linked section CODE
        segment CODE
        .assume ADL = 1


;*****************************************************************************
;*  Default Non-Maskable Interrupt handler
;*
;* defined in MOS src_startup/vectors16.asm::__default_nmi_handler


;*****************************************************************************
;*  Default Maskable Interrupt handler
;*
;* defined in MOS src_startup/vectors16.asm::__default_mi_handler


;*****************************************************************************
;*  void _init_default_vectors( void );
;*
;*  initialize all potential interrupt vector locations with a known
;*  default handler.
;*
;*  This would be called from zsldevinit.asm (or a similar init file)
;*
;*  The interrupt vectors are contained in the Agon MOS software 
;*  and we leave them unchanged in FreeRTOS
;*   MOS src_startup/vectors16.asm::_init_default_vectors


;*****************************************************************************
;*  int _set_vector( unsigned short vector, void( *handler )( void ) );
;*
;*  installs a user interrupt handler in the vector table
;*
;*  (IX+9)  handler - address of user interrupt handler
;*  (IX+6)  vector  - vector offset
;*
;* __set_vector:
;* _set_vector:
;*     push ix                    ; Standard prologue
;*     ld ix,0
;*     add ix,sp           
;*     push af
;*     di
;*     
;*     ld hl, 0
;*     ld hl, (ix+6)              ; load vector offset
;*     ld bc, __vector_table      ; load base address for vector table
;*     add hl, bc                 ; calculate vector location
;*     ld bc, (ix+9)              ; handler
;*     ld (hl), bc                ; store new vector address
;*         
;*     ei
;*     pop af                     ; Standard epilogue
;*     ld sp, ix
;*     pop ix
;*     ret


;*****************************************************************************
;* void *mos_setintvector( unsigned short vector, void( *handler )( void ));
;*
;*  (IX+9)  handler - address of user interrupt handler
;*  (IX+6)  vector  - interrupt vector
;*
;* Purpose: install a user interrupt handler in the MOS interrupt vector table
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x14-mos_setintvector
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0x14: mos_setintvector 
;*     Set an interrupt vector (Requires MOS 1.03 or above)
;*     Implemented in MOS:
;*       vectors16.asm::_rst_08_handler -> call mos_api
;*       mos_api.asm::mos_api -> switch-like call to mos_api_setintvector
;*       mos_api.asm::mos_api_setintvector call to _mos_SETINTVECTOR
;*       mos.c::_mos_SETINTVECTOR call to set_vector 
;*       vectors16.asm::set_vector, 
;*         https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*         which installs a user interrupt handler in the 2nd interrupt 
;*         vector jump table
;* 
;* Parameters:
;*     E: Interrupt vector number to set
;*     HLU: Address of new interrupt vector (24-bit pointer)
;* 
;* Returns:
;*     HL(U): Address of the previous interrupt vector (24-bit pointer)
;* 
_mos_setintvector:
    push ix                    ; Standard prologue
    ld ix, 0
    add ix, sp
	push af
     
    ld de, 0                   ; construct parameters for mos_setintvector
    ld de, (ix+6)              ; load vector number, first function call parameter
	ld hl, 0
    ld hl, (ix+9)              ; load pointer to handler
	MOSCALL mos_setintvector   ; function number defined in mos_api.inc
							   ; returns old handler in HLU

    pop af                     ; Standard epilogue
    ld sp, ix                  ;   restore stack pointer
	pop ix
	ret


;*****************************************************************************
;*  .IVECTS segment must be aligned on a 512 byte boundary anywhere in RAM
;*  each entry will be a 3-byte address in a 4-byte space 
;*
;*         define .IVECTS, space = RAM, align = 200h
;*         segment .IVECTS
;* __vector_table:
;*         ds NVECTORS * 4


        segment DATA
;*****************************************************************************
;*  number of vectors supported
;*
__nvectors:
        dw NVECTORS         ; extern unsigned short _num_vectors;

        end
