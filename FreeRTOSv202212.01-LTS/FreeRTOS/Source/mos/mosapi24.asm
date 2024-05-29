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
;* mosapi24.asm
;*   MOS API functions defined at https://agonconsole8.github.io/agon-docs/MOS-API
;*   Created 24/May/2024 by Julian Rose for Agon Light port
;*
;*****************************************************************************
include "mos_api.inc"

    xref _portEnterMOS
    xref _portExitMOS
    xref _putchar

    xdef _mos_getkey
    xdef _mos_setkbvector

    xdef __mosapi_setkbvector   ; local function actually receives kb callback
    xdef __mosapi_kbvect        ; storage for kb vector callback


;******************** Globals ************************************************
;* globals are placed in linked section BSS
    segment BSS

; Userspace hooks
__mosapi_kbvect: DS 3           ; Pointer to keyboard callback


;*****************************************************************************
;*        library code is placed in linked section CODE
    segment CODE
    .assume ADL = 1


;*****************************************************************************
;* unsigned char mos_getkey( void );
;*
;* Purpose: Get ASCII code of last key press, or 0 if none
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x00-mos_getkey
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0x0: _mos_getkey 
;*     Get keycode returned from VDP (Requires MOS 1.03 or above)
;*     Implemented in MOS:
;*       vectors16.asm::_rst_08_handler -> call mos_api
;*       mos_api.asm::mos_api -> switch-like call to mos_api_getkey
;*         https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*         https://github.com/breakintoprogram/agon-mos/blob/main/src/mos.c
;*         if system variable keycount == 0 then return 0
;*         else block while the key is down, return once it is released
;* 
;* Parameters:
;*     None
;*
;* Returns:
;*     A: ASCII value of the pressed key
;*     Refer to AN033301 table 2 for registers containing different return types
;* 
_mos_getkey:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter
    
    MOSCALL mos_getkey          ; function value in mos_api.inc
                                ; returns ASCII byte in A

    push af                     ; MOS critical exit
    call _portExitMOS
    pop af

    ld sp, ix                   ; Standard epilogue
    pop ix
    ret


;*****************************************************************************
;* void mos_setkbvector( void( *cb )( VDP_KB_PACKET* ));
;*
;* Purpose: Store a keyboard event callback
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x1d-mos_setkbvector
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0x1D: mos_setkbvector 
;*   MOS will invoke the callback __mosapi_setkbvector on each VDP keyboard event
;*     Implemented in MOS:
;*       vectors16.asm::_rst_08_handler -> call mos_api
;*       mos_api.asm::mos_api -> switch-like call to mos_api_setkbvector
;*         https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*         https://github.com/breakintoprogram/agon-mos/blob/main/src/mos.c
;*         Store cb in _user_kbvector
;* 
;* Parameters:
;*     void( * )( void )
;*
;* Returns:
;*     None
;*     Refer to AN033301 table 2 for registers containing different return types
; 
;***
;* void __mosapi_setkbvector()
;*
;* Purpose: The actual callback registered with MOS, will chain to the user callback
;*          This is needed because MOS passes the VDP keyboard packet in DE rather
;*          than HL
;*
;*    simulated call from MOS vdp_protocol.asm::vdp_protocol_KEY
;*    DE contains _vdp_protocol_data:VDP_KB_PACKET
;*    
;*    Likewise we simulate a call to the user vector, because the eZ80 instruction
;*      call ( MMmmnn ) in ADL mode does not jump to a 3 byte vector
;*    To simulate a call, first push any params
;*                        then push a return address
;*                        then jump to the function
;*          The 'called' function 'ret' instrcution will pop off the return address
;
__mosapi_setkbvector:

    ;push DE                    ; push keyboard packet address
    ;ld h,0
    ;ld l, 'X'
    ;push HL
    ;call _putchar
    ;pop HL
    ;pop DE                     ; pop keyboard packet address from DE

    ; chain to the actual user callback
    ;call ( __mosapi_kbvect )   ; does not work as documented in UM0077 for ADL mode=1
	                            ; so simulate a call by pushing a return address and jump
    push DE                     ; push C param (right-to-left) on stack
    ld hl, __mosapi_setkbvector1; push return address last
    push HL                     ; will be popped by __mosapi_kbvect ret instruction
    ld hl, ( __mosapi_kbvect )  ; get user callback address to chain
    jp ( hl )                   ; jump to vector
__mosapi_setkbvector1:          ; return address for simulated call
	;pop HL                       done by __mosapi_kbvect ret instruction
	pop DE                      ; clean up params from stack

    ret                         ; return to MOS vdp_protocol_key


_mos_setkbvector:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter
    di                          ; call portENTER_CRITICAL( );  not necessary?

    ld HL, (IX+%6)              ; user callback, passed from C on the stack (1st arg)
    ld ( __mosapi_kbvect ), HL  ; remember user callback for chaining

    ; test whether user callback is NULL
	ld a,0                      ; a = 0
	cp a, (IX+%6)               ; test __mosapi_kbvect == 0
	jp z, __mos_setkbvector1    ; if NULL == __mosapi_kbvect goto _mos_setkbvector1
	
    ld HL, __mosapi_setkbvector  ; NULL != __mosapi_kbvect; set _local_setkbvector
                                ;    which will chain __mosapi_kbvect
__mos_setkbvector1:
    ld C, 0                     ; 0 = 24-bit address (ADL mode), 1 (Z80 mode)
    MOSCALL mos_setkbvector     ; function value in mos_api.inc

    ei                          ; call portEXIT_CRITICAL( );
    call _portExitMOS           ; MOS critical exit

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;*        library data is placed in linked section DATA
    segment DATA

    end
