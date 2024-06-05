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

    xdef _mos_getkey            ; implement call to MOS API function 00h
    xdef _mos_load              ; implement call to MOS API function 01h
    xdef _mos_save              ; implement call to MOS API function 02h
    xdef _mos_cd                ; implement call to MOS API function 03h
    xdef _mos_del               ; implement call to MOS API function 05h
    xdef _mos_ren               ; implement call to MOS API function 06h
    xdef _mos_mkdir             ; implement call to MOS API function 07h
    xdef _mos_copy              ; implement call to MOS API function 11h
    xdef _mos_setkbvector       ; implement call to MOS API function 1dh


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
;   uval: The value to set in HLU
SET_AHL24: MACRO uval
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


;*****************************************************************************
;* void mos_setkbvector( 
;*          void( *cb )( VDP_KB_PACKET* )
;*      );
;*
;* Purpose: Store a keyboard event callback
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
;*     (IX+6) - Address of user callback passed on the stack
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


;*****************************************************************************
;* unsigned int mos_load( 
;*                  char const * const filename, 
;*                  void * const address, 
;*                  size_t const size 
;*              );
;*
;* Purpose: Load a file (from SD-card) into memory
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x01-mos_load
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     01h: mos_load
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_load
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     (IX+6)  - Filename (null-terminated) on SD-card to read from
;*     (IX+9)  - Memory address of buffer to load file into
;*     (IX+12) - Size of buffer in bytes
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
_mos_load:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filename, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; address, passed from C on the stack (2nd arg)
    ld BC, (IX+12)              ; buf size, passed from C on the stack (3rd arg)
    MOSCALL mos_load            ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    push AF                     ; get char return value from A into int HL
    SET_AHL24 0                 ; ld 0 into HLU to clear the upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    pop AF                      ; recover mos_load result in A reg
    ld L, A                     ; store mos_load result in HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* unsigned int mos_save( 
;*                  char const * const filename, 
;*                  void * const address, 
;*                  size_t const size 
;*              );
;*
;* Purpose: Load a file (from SD-card) into memory
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x02-mos_save
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     02h: mos_save
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_save
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     (IX+6)  - Filename (null-terminated) on SD-card to write to
;*     (IX+9)  - Memory address of buffer to save file from
;*     (IX+12) - Number of bytes to write from buffer
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
_mos_save:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filename, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; address, passed from C on the stack (2nd arg)
    ld BC, (IX+12)              ; num bytes, passed from C on the stack (3rd arg)
    MOSCALL mos_save            ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    push AF                     ; get char return value from A into int HL
    SET_AHL24 0                 ; ld 0 into HLU to clear the upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    pop AF                      ; recover mos_save result in A reg
    ld L, A                     ; store mos_save result in HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* unsigned int mos_cd( 
;*                char const * const filename, 
;*              )
;*
;* Purpose: Set the current working directory on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x03-mos_cd
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     03h: mos_cd
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_cd
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     (IX+6)  - Pathname (null-terminated) to SD-card directory
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
_mos_cd:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; pathname, passed from C on the stack (1st arg)
    MOSCALL mos_cd              ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    push AF                     ; get char return value from A into int HL
    SET_AHL24 0                 ; ld 0 into HLU to clear the upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    pop AF                      ; recover mos_save result in A reg
    ld L, A                     ; store mos_save result in HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* unsigned int mos_del( 
;*                char const * const filename, 
;*              )
;*
;* Purpose: Delete a file or directory from the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x05-mos_del
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     05h: mos_del
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_del
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     (IX+6)  - Pathname (null-terminated) of SD-card filename or pathname
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
_mos_del:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; pathname, passed from C on the stack (1st arg)
    MOSCALL mos_del             ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    push AF                     ; get char return value from A into int HL
    SET_AHL24 0                 ; ld 0 into HLU to clear the upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    pop AF                      ; recover mos_save result in A reg
    ld L, A                     ; store mos_save result in HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* unsigned int mos_ren( 
;*                char const * const oldname, 
;*                char const * const newname, 
;*              )
;*
;* Purpose: Rename a file or directory on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x06-mos_ren
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     06h: mos_ren
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_ren
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     (IX+6)  - oldname (null-terminated) on SD-card
;*     (IX+9)  - newname (null-terminated) on SD-card
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
_mos_ren:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; oldname, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; newname, passed from C on the stack (2nd arg)
    MOSCALL mos_ren             ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    push AF                     ; get char return value from A into int HL
    SET_AHL24 0                 ; ld 0 into HLU to clear the upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    pop AF                      ; recover mos_save result in A reg
    ld L, A                     ; store mos_save result in HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* unsigned int mos_mkdir( 
;*                char const * const pathname, 
;*              )
;*
;* Purpose: Make a new directory on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x07-mos_mkdir
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     07h: mos_mkdir
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_mkdir
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     (IX+6)  - Pathname (null-terminated) to make on SD-card
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
_mos_mkdir:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; pathname, passed from C on the stack (1st arg)
    MOSCALL mos_mkdir           ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    push AF                     ; get char return value from A into int HL
    SET_AHL24 0                 ; ld 0 into HLU to clear the upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    pop AF                      ; recover mos_save result in A reg
    ld L, A                     ; store mos_save result in HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* unsigned int mos_copy( 
;*                char const * const src, 
;*                char const * const dest, 
;*              )
;*
;* Purpose: Copy the src file to dest on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x11-mos_copy
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     11h: mos_copy
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_copy
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     (IX+6)  - src (null-terminated) on SD-card
;*     (IX+9)  - dest (null-terminated) on SD-card
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
_mos_copy:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; src, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; dest, passed from C on the stack (2nd arg)
    MOSCALL mos_copy            ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    push AF                     ; get char return value from A into int HL
    SET_AHL24 0                 ; ld 0 into HLU to clear the upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    pop AF                      ; recover mos_save result in A reg
    ld L, A                     ; store mos_save result in HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;*        library data is placed in linked section DATA
    segment DATA

    end
