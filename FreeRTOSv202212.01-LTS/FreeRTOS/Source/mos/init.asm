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
;*
;* Agon Light init.asm
;*
;* Title:    MOS Application Initialisation Code
;* Purpose:  Provide MOS header, C startup (in MOS context), putch
;*           modified from Agon-Projects-main -> C -> Hello
;*           Must be first in linked section CODE
;* Author:   Dean Belfield
;*           Julian Rose
;* Created:      22/11/2022
;* Last Updated: 22/05/2024
;*
;* Modinfo:
;* 25/11/2022 db   Added parameter parsing; now accepts CR or NUL as end of string markers
;* 13/05/2024 jhr  Include in FreeRTOS demo
;* 22/05/2024 jhr  Implement getch and re-layout

include "mos_api.inc"


;********** Declarations ******************************************************

    XREF    __low_bss
    XREF    __len_bss
            
    XREF    _main
    xref    _portEnterMOS
    xref    _portExitMOS
    xref    _mos_getkey

    XDEF    _putch
    XDEF    _getch
            
    XDEF    __putch
    XDEF    __putchf        ; no test for MOS rentry, for use in low-leve; debugging
    XDEF    __getch
        

;********** Constants *********************************************************
argv_ptrs_max  EQU  16      ; Maximum number of arguments allowed in argv


;******************** Macros **************************************************
; Macro for setting the upper byte in HLU
; Parameters:
;   uval: The value to set in HLU
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


;********** Code Segment ******************************************************

    SEGMENT CODE


; Start in ADL mode
    .ASSUME ADL = 1    
    JP      _start          ; Entry point


;********** MOS Header ********************************************************
_exec_name: 
    DB "FreeRTOS.BIN", 0    ; The executable name, only used in argv
                            ; part of the MOS header - don't move to DATA section

    ALIGN   64              ; A MOSLET executable header is from byte 64 onwards
            
    DB      "MOS"           ; Flag for MOS - to confirm this is a valid MOS command
    DB      00h             ; MOS header version 0
    DB      01h             ; Flag for run mode (0: Z80, 1: ADL)



;********** Local Functions ***************************************************
;
; And the code follows on immediately after the header
;
_start:
    PUSH    AF              ; Preserve registers to be nice to MOS
    PUSH    BC
    PUSH    DE
    PUSH    IX
    PUSH    IY              ; Need to preserve IY for MOS            

    PUSH    HL              ; Clear the BSS RAM area
    CALL    _clear_bss
    POP     HL

    LD      IX, argv_ptrs   ; The argv array pointer address
    PUSH    IX              ; Parameter 2: argv[0] = IX
    CALL    _parse_params   ; Parse the parameters
    LD      B, 0            ; Clear B from BCU as we just want ARGC
    PUSH    BC              ; Parameter 1: argc
    CALL    _main           ; int main(int argc, char *argv[])
    POP     DE              ; return value from _main
    POP     DE              ; Balance the stack

    POP     IY              ; Restore registers for MOS on exit
    POP     IX
    POP     DE
    POP     BC
    POP     AF
    RET                     ; clean return to MOS


; Clear BSS memory
;
_clear_bss:
    LD      BC, __len_bss   ; Check for non-zero length
    LD      a, __len_bss >> 16
    OR      A, C
    OR      A, B
    RET     Z               ; BSS is zero-length ...
    XOR     A, A
    LD      (__low_bss), A
    SBC     HL, HL          ; HL = 0
    DEC     BC              ; 1st byte's taken care of
    SBC     HL, BC
    RET     Z               ; Just 1 byte ...
    LD      HL, __low_bss   ; Reset HL
    LD      DE, __low_bss + 1    ; [DE] = bss + 1
    LDIR                    ; Clear this section
    RET


; Parse the command line parameter string into a C array
; Parameters
; - HL: Address of parameter string
; - IX: Address for array pointer storage
; Returns:
; -  C: Number of parameters parsed
;
_parse_params:        
    LD      BC, _exec_name
    LD      (IX+0), BC      ; ARGV[0] = the executable name
    INC     IX
    INC     IX
    INC     IX
    CALL    _skip_spaces    ; Skip HL past any leading spaces

    LD      BC, 1           ; C: ARGC = 1 - also clears out top 16 bits of BCU
    LD      B, argv_ptrs_max - 1    ; B: Maximum number of argv_ptrs

_parse_params_1:    
    PUSH    BC              ; Stack ARGC    
    PUSH    HL              ; Stack start address of token
    CALL    _get_token      ; Get the next token
    LD      A, C            ; A: Length of the token in characters
    POP     DE              ; Start address of token (was in HL)
    POP     BC              ; ARGC
    OR      A               ; Check for A=0 (no token found) OR at end of string
    RET     Z

    LD      (IX+0), DE      ; Store the pointer to the token
    PUSH    HL              ; DE=HL
    POP     DE
    CALL    _skip_spaces    ; And skip HL past any spaces onto the next character
    XOR     A
    LD      (DE), A         ; Zero-terminate the token
    INC     IX
    INC     IX
    INC     IX              ; Advance to next pointer position
    INC     C               ; Increment ARGC
    LD      A, C            ; Check for C >= A
    CP      B
    JR      C, _parse_params_1    ; And loop
    RET


; Get the next token
; Parameters:
; - HL: Address of parameter string
; Returns:
; - HL: Address of first character after token
; -  C: Length of token (in characters)
;
_get_token:        
    LD      C, 0            ; Initialise length
$$: LD      A, (HL)         ; Get the character from the parameter string
    OR      A               ; Exit if 0 (end of parameter string in MOS)
    RET     Z
    
    CP      13              ; Exit if CR (end of parameter string in BBC BASIC)
    RET     Z

    CP      ' '             ; Exit if space (end of token)
    RET     Z

    INC     HL              ; Advance to next character
    INC     C               ; Increment length
    JR      $B


; Skip spaces in the parameter string
; Parameters:
; - HL: Address of parameter string
; Returns:
; - HL: Address of next none-space character
;    F: Z if at end of string, otherwise NZ if there are more tokens to be parsed
;
_skip_spaces:        
    LD      A, (HL)         ; Get the character from the parameter string    
    CP      ' '             ; Exit if not space
    RET     NZ

    INC     HL              ; Advance to next character
    JR      _skip_spaces    ; Increment length


; Write a character out to the ESP32
; int putch(int ch)
;
__putch:
_putch:
    PUSH    IY
    LD      IY, 0
    ADD     IY, SP

    push    IY
    call    _portEnterMOS     ; MOS critical enter
    pop     IY
    
    LD      A, (IY+6)
    RST.LIL 10h    
    LD      HL, 0           ; get char result in A to int in HL
    LD      L, A

    push    HL
    push    IY
    call    _portExitMOS
    pop     IY
    pop     HL

    LD      SP, IY
    POP     IY                
    RET

; Write a character out to the ESP32
; int _putchf(int ch)
;
__putchf:
    PUSH    IY
    LD      IY, 0
    ADD     IY, SP

    LD      A, (IY+6)
    RST.LIL 10h    
    LD      HL, 0           ; get char result in A to int in HL
    LD      L, A

    LD      SP, IY
    POP     IY                
    RET


; Read a character in from the ESP32
; int getch(void)
;   return int value in HL
;   (similar to mosapi2.asm::_mos_getkey, which returns result in A)
__getch:
_getch:        
    push    ix              ; Standard prologue
    ld      ix, 0
    add     ix, sp

    push    ix
    call    _portEnterMOS   ; MOS critical enter
    pop     ix
    
    MOSCALL mos_getkey      ; function value in mos_api.inc
    SET_HLU24  0            ; ld 0 (clear) HLU upper byte
    ld      h, 0            ; returns ASCII byte in A
    ld      l, a            ; get char return value in A into int HL

    push    hl              ; preserve HL over call to portExitMOS
    push    ix
    call    _portExitMOS    ; MOS critical exit
    pop     ix
    pop     hl              ; recover preserved HL 

    ld      sp, ix          ; Standard epilogue
    pop     ix
    RET


;********** Data Segment ******************************************************

    SEGMENT DATA

; Storage for the argv array pointers
;
argv_ptrs:
    BLKP    argv_ptrs_max, 0


    END
