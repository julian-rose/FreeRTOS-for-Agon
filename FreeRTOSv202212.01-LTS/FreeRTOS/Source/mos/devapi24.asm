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
;* devapi24.asm
;*   MOS API Device functions defined at https://agonconsole8.github.io/agon-docs/MOS-API
;*   Created 24/May/2024 by Julian Rose for Agon Light port
;*
;*****************************************************************************
include "mosConfig.inc"
include "mos_api.inc"

    xref _portEnterMOS
    xref _portExitMOS

    xdef _mos_setintvector

IF( 1 == configUSE_MOS_KEYBOARD_OPS )
    xdef _mos_getkey            ; implement call to MOS API function 00h
    xdef _mos_setkbvector       ; implement call to MOS API function 1Dh
    ;xdef _mos_kbmap            ; implement call to MOS API function 1Dh
ENDIF

IF( 1 == configUSE_MOS_DEVICE_RTC )
    xdef _mos_getrtc            ; implement call to MOS API function 12h
    xdef _mos_setrtc            ; implement call to MOS API function 13h
ENDIF


;******************** Globals ************************************************
;* globals are placed in linked section BSS
    segment BSS

; Userspace hooks
__mosapi_kbvect: DS 3           ; Pointer to keyboard callback


;*****************************************************************************
;*        library code is placed in linked section CODE
    segment CODE
    .assume ADL = 1


;******************** Macros *************************************************
; Macro for setting the upper byte in HLU
; Parameters:
;   uval: The value to set in HL upper byte U
SET_HLU24: MACRO uval
    push AF
    ld A, uval
    push HL            
    ld HL, 2
    add HL, SP
    ld (HL), A
    pop HL
    pop AF
ENDMACRO     


;******************** Functions **********************************************
;*****************************************************************************
;* unsigned char mos_getkey( 
;*                   void 
;*               );
;*
;* Purpose: Get ASCII code of last key press, or 0 if none
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x00-mos_getkey
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0x0: _mos_getkey 
;*     Get keycode returned from VDP (Requires MOS 1.03 or above)
;*   Implemented in MOS:
;*     vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_getkey
;*       https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*       https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.c
;*       if system variable keycount == 0 then return 0
;*       else block while the key is down, return once it is released
;* 
;* Parameters:
;*     n/a
;*
;* Returns:
;*     A reg: ASCII character value of the pressed key
;*     Refer to AN033301 table 2 for registers containing different return types
;*     (Similar to init.asm::_getch, but that returns value in HL)
;* 
IF( 1 == configUSE_MOS_KEYBOARD_OPS )

_mos_getkey:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter
    
    MOSCALL mos_getkey          ; function value in mos_api.inc
                                ; returns ASCII byte in A reg

    push af                     ; preserve A reg over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop af                      ; recover A reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* void *mos_setintvector( unsigned short vector, void( *handler )( void ));
;*
;*  (IX+9)  handler - address of user interrupt handler
;*  (IX+6)  vector  - interrupt vector
;*
;* Purpose: install a user interrupt handler in the MOS interrupt vector table
;*          (similar to mos_setpithandler with _portEnterMOS..._portExitMOS
;*           guards)
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
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter
                                ; construct parameters for mos_setintvector
    ld de, (ix+6)               ; load vector number, first function call parameter
    ld hl, (ix+9)               ; load pointer to handler
    MOSCALL mos_setintvector    ; function number defined in mos_api.inc
                                ; returns old handler in HLU
    call _portExitMOS           ; MOS critical exit
	
    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* void mos_setkbvector( 
;*          void( *cb )( VDP_KB_PACKET* )
;*      );
;*
;* Purpose: Store a VDP keyboard event callback via the MOS API
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x1d-mos_setkbvector
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0x1D: mos_setkbvector 
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     MOS will invoke the callback __mosapi_setkbvector on each VDP keyboard event
;*     mos_api.asm::mos_api -> switch-like call to mos_api_setkbvector
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     Store cb in _user_kbvector
;* 
;* Parameters:
;*     (IX+6) - Address of user callback passed on the stack, into HL
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
;*          than on the stack for C
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
IF( 1 == configUSE_MOS_KEYBOARD_OPS )

__mosapi_setkbvector:

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

    ld HL, (IX+6)               ; user callback, passed from C on the stack (1st arg)
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

ENDIF


;*****************************************************************************
;* unsigned int mos_getrtc( 
;*                MOS_RTC_STRING_R const buffer, 
;*              )
;*
;* Purpose: Get the Real Time Clock data via MOS API from the VDP
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x12-mos_getrtc
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     12h: mos_getrtc
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_getrtc
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/clock.c
;* 
;* Parameters:
;*     (IX+6)  - Pointer to a buffer to copy the MOS_RTC_STRING_R, into HL
;*
;* Returns:
;*     None (actually the pointer to buf; supposed to be strlen(buf), but this is
;*           overwritten in mos_api.asm)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_DEVICE_RTC )

_mos_getrtc:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; buffer, passed from C on the stack (1st arg)
    MOSCALL mos_getrtc          ; function value in mos_api.inc
                                ; returns void
    call _portExitMOS           ; MOS critical exit

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_setrtc( 
;*                MOS_RTC_STRING_W const buffer, 
;*              )
;*
;* Purpose: Set the Real Time Clock data via MOS API from the VDP
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x13-mos_setrtc
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     13h: mos_setrtc
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_setrtc
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/clock.c
;* 
;* Parameters:
;*     (IX+6)  - Pointer to a buffer containing the MOS_RTC_STRING_W, into HL
;*
;* Returns:
;*     None 
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_DEVICE_RTC )

_mos_setrtc:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; buffer, passed from C on the stack (1st arg)
    MOSCALL mos_setrtc          ; function value in mos_api.inc
                                ; returns void
    call _portExitMOS           ; MOS critical exit

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;*        library data is placed in linked section DATA
    segment DATA

    end
