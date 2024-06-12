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
;* These functions should not normally be application-user altered.
;*****************************************************************************
include "mosConfig.inc"
include "mos_api.inc"

    xref _portEnterMOS
    xref _portExitMOS

IF( 1 == configUSE_MOS_FILE_OPS )
    xdef _mos_load              ; implement call to MOS API function 01h
    xdef _mos_save              ; implement call to MOS API function 02h
    xdef _mos_getfil            ; implement call to MOS API function 19h
ENDIF

IF( 1 == configUSE_MOS_DIR_OPS )
    xdef _mos_cd                ; implement call to MOS API function 03h
    xdef _mos_del               ; implement call to MOS API function 05h
    xdef _mos_ren               ; implement call to MOS API function 06h
    xdef _mos_mkdir             ; implement call to MOS API function 07h
    xdef _mos_copy              ; implement call to MOS API function 11h
ENDIF

IF( 1 == configUSE_MOS_SYSVARS )
    xdef _mos_getsysvars        ; implement call to MOS API function 08h
ENDIF

IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )
    xdef _mos_fopen             ; implement call to MOS API function 0Ah
    xdef _mos_fclose            ; implement call to MOS API function 0Bh
    xdef _mos_fgetc             ; implement call to MOS API function 0Ch
    xdef _mos_fputc             ; implement call to MOS API function 0Dh
    xdef _mos_feof              ; implement call to MOS API function 0Eh
    xdef _mos_fread             ; implement call to MOS API function 1Ah
    xdef _mos_fwrite            ; implement call to MOS API function 1Bh
    xdef _mos_flseek            ; implement call to MOS API function 1Ch
ENDIF

    xdef _mos_geterror          ; implement call to MOS API function 0Fh


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
;*     (IX+6)  - Filename (null-terminated) on SD-card to read from, into HL
;*     (IX+9)  - Memory address of buffer to load file into, into DE
;*     (IX+12) - Size of buffer in bytes, into BC
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_FILE_OPS )

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
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_save( 
;*                  char const * const filename, 
;*                  void * const address, 
;*                  size_t const size 
;*              );
;*
;* Purpose: Save a file (to SD-card) from memory
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
;*     (IX+6)  - Filename (null-terminated) on SD-card to write to, into HL
;*     (IX+9)  - Memory address of buffer to save file from, into DE
;*     (IX+12) - Number of bytes to write from buffer, into BC
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_FILE_OPS )

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
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


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
;*     (IX+6)  - Pathname (null-terminated) to SD-card directory into HL
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_DIR_OPS )

_mos_cd:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; pathname, passed from C on the stack (1st arg)
    MOSCALL mos_cd              ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


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
;*     (IX+6)  - Pathname (null-terminated) of SD-card filename or pathname, into HL
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_DIR_OPS )

_mos_del:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; pathname, passed from C on the stack (1st arg)
    MOSCALL mos_del             ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


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
;*     (IX+6)  - oldname (null-terminated) on SD-card into HL
;*     (IX+9)  - newname (null-terminated) on SD-card into BC
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_DIR_OPS )

_mos_ren:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; oldname, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; newname, passed from C on the stack (2nd arg)
    MOSCALL mos_ren             ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


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
;*     (IX+6)  - Pathname (null-terminated) to make on SD-card into HL
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_DIR_OPS )

_mos_mkdir:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; pathname, passed from C on the stack (1st arg)
    MOSCALL mos_mkdir           ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


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
;*     (IX+6)  - src (null-terminated) on SD-card into HL
;*     (IX+9)  - dest (null-terminated) on SD-card into BC
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_DIR_OPS )

_mos_copy:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; src, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; dest, passed from C on the stack (2nd arg)
    MOSCALL mos_copy            ; function value in mos_api.inc
                                ; returns errno byte in A reg (0=ok)
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (errno) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_fopen( 
;*                char const * const filename, 
;*                MOS_FILE_MODE const mode, 
;*              )
;*
;* Purpose: Open a file for i/o (mode depending) on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x0a-mos_fopen
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0Ah: mos_fopen
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_fopen
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filename (null-terminated) into HL
;*     (IX+9)  - mode (combination of MOS_FILE_MODE values) into BC
;*
;* Returns:
;*     FatFS Filehandle (int)
;*     Refer to AN033301 table 2 for registers containing different return types
;*     MOS 1.04 does not return any error code, only 0 (NULL) on failure
;*     MOS 1.04 does not maintain a system _errno and api mos_errno()
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_fopen:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filename, passed from C on the stack (1st arg)
    ld BC, (IX+9)               ; mode, passed from C on the stack (2nd arg)
    MOSCALL mos_fopen           ; function value in mos_api.inc
                                ; returns file handle (or NULL) in A reg
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (file handle) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_fclose( 
;*                unsigned int const filehandle
;*              )
;*
;* Purpose: Close a previously fopened file on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x0b-mos_fclose
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0Bh: mos_fclose
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_fclose
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filehandle into C
;*
;* Returns:
;*     FatFS Number of files still open (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_fclose:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld BC, (IX+6)               ; file handle, passed from C on the stack (1st arg)
    MOSCALL mos_fclose          ; function value in mos_api.inc
                                ; returns count of remaining open files in A reg
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)
    
    push HL                     ; preserve HL reg (file handle) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned char mos_fgetc( 
;*                unsigned int const filehandle
;*              )
;*
;* Purpose: Read the byte at the current seek position from a previously 
;*          fopened file on the SD-card, and increment the seek position
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x0c-mos_fgetc
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0Ch: mos_fgetc
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_fgetc
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filehandle into C
;*
;* Returns:
;*     char in A reg (also F:C is set if eof)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_fgetc:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld BC, (IX+6)               ; file handle, passed from C on the stack (1st arg)
    MOSCALL mos_fgetc           ; function value in mos_api.inc
                                ; returns character from file in A reg
    push AF                     ; preserve A reg (character) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop AF                      ; recover A reg (F.C indicates last character in file)

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* void mos_fputc( 
;*          unsigned int const filehandle,
;*          char const c
;*      )
;*
;* Purpose: Write a byte at the current seek position to a previously fopened 
;*          file on the SD-card, and increment the seek position
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x0d-mos_fputc
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0Dh: mos_fputc
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_fputc
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filehandle into C
;*     (IX+9)  - char to write, into B
;*
;* Returns:
;*     char in A reg (also F:C is set if eof)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_fputc:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld C, (IX+6)                ; file handle, passed from C on the stack (1st arg)
    ld B, (IX+9)                ; char to write, passed from C on the stack (2nd arg)
    MOSCALL mos_fputc           ; function value in mos_api.inc
                                ; returns nothing
    call _portExitMOS           ; MOS critical exit

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned char mos_feof( 
;*                unsigned int const filehandle
;*              )
;*
;* Purpose: Test if a previously fopened file on the SD-card is at the last
;*          seek position
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x0e-mos_feof
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0Eh: mos_feof
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_feof
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.h
;* 
;* Parameters:
;*     (IX+6)  - filehandle into BC
;*
;* Returns:
;*     1 at EOF else 0, in A reg
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_feof:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld C, (IX+6)                ; file handle, passed from C on the stack (1st arg)
    MOSCALL mos_feof            ; function value in mos_api.inc
                                ; returns 1 (EOF) or 0 (not EOF) in A reg
    push AF                     ; preserve A reg (character) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop AF                      ; recover A reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_fread( 
;*                unsigned int const filehandle,
;*                void * buf,
;*                size_t const num_bytes_to_read
;*              )
;*
;* Purpose: Read up to num_bytes_to_read from the current seek position from a 
;*          previously fopened file on the SD-card into buf, and increment the 
;*          seek position
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x1a-mos_fread
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     1Ah: mos_fread
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_fread
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filehandle into C
;*     (IX+9)  - Memory address of buffer to write to, into HL
;*     (IX+12) - Max number of bytes to read into buffer, into DE
;*
;* Returns:
;*     Number of bytes read in HL reg
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_fread:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld BC, (IX+6)               ; file handle, passed from C on the stack (1st arg)
    ld HL, (IX+9)               ; buf ptr, passed from C on the stack (2nd arg)
    ld DE, (IX+12)              ; num bytes to read, passed from C on the stack (3rd arg)
    MOSCALL mos_fread           ; function value in mos_api.inc
                                ; returns number of bytes read in DE reg
    push DE                     ; preserve DE reg (count) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover count of bytes read into HL

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_fwrite( 
;*                unsigned int const filehandle,
;*                void * buf,
;*                size_t const num_bytes_to_write
;*              )
;*
;* Purpose: Write up to num_bytes_to_write to the current seek position into a 
;*          previously fopened file on the SD-card from buf, and increment the 
;*          seek position
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x1b-mos_fwrite
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     1Bh: mos_fwrite
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_fwrite
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filehandle into C
;*     (IX+9)  - Memory address of buffer to write from, into HL
;*     (IX+12) - Number of bytes to write from buffer, into DE
;*
;* Returns:
;*     Number of bytes written in HL reg
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_fwrite:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld BC, (IX+6)               ; file handle, passed from C on the stack (1st arg)
    ld HL, (IX+9)               ; buf ptr, passed from C on the stack (2nd arg)
    ld DE, (IX+12)              ; num bytes to write, passed from C on the stack (3rd arg)
    MOSCALL mos_fwrite          ; function value in mos_api.inc
                                ; returns number of bytes written in DE reg
    push DE                     ; preserve count (DE) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover count of bytes written into HL

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_flseek( 
;*                unsigned int const filehandle,
;*                unsigned int const offs
;*              )
;*
;* Purpose: Set the current seek poition in a previously fopened file on the 
;*          SD-card 
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x1c-mos_flseek
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     1Ch: mos_flseek
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_flseek
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filehandle, into C reg
;*     (IX+9)  - seek position offset from start of file, into HL
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_BUFFERED_FILE_OPS )

_mos_flseek:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld BC, (IX+6)               ; file handle, passed from C on the stack (1st arg)
    ld HL, (IX+9)               ; offset 23:0, passed from C on the stack (2nd arg)
    ld E, 0                     ; offset 31:24
    MOSCALL mos_flseek          ; function value in mos_api.inc
                                ; returns result in A reg
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get char return value from A into int HL (int return)

    push HL                     ; preserve errno (HL) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover errno (HL)

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned int mos_getsysvars( 
;*                void
;*              )
;*
;* Purpose: Retrieve a pointer to the system variables
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x08-mos_sysvars
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     08h: mos_sysvars
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_sysvars
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;* 
;* Parameters:
;*     n/a
;*
;* Returns:
;*     IX - _sysvars
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_SYSVARS )

_mos_getsysvars:

    push ix                     ; Standard prologue

    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    push IX                     ; save IX over MOS call
    MOSCALL mos_sysvars         ; function value in mos_api.inc
    push IX                     ; returns _sysvars in IX reg (0=ok)
    pop HL                      ; get _sysvars result in HL (void* return)
    pop IX                      ; retrieve IX after MOS call
    
    push HL                     ; preserve HL (_sysvars) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* void mos_geterror( 
;*          MOS_ERRNO const errno, 
;*          void * const buf,
;*          size_t const buf_size
;*      )
;*
;* Purpose: Retrieve a descriptive string for the given error number
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x0f-mos_geterror
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     0Fh: mos_geterror
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_getError
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos.c
;* 
;* Parameters:
;*     (IX+6)  - errno, into E
;*     (IX+9)  - buffer to receive string, into HL
;*     (IX+12) - buffer size, into BC
;*
;* Returns:
;*     n/a
; 
_mos_geterror:
    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld E, (IX+6)                ; errno, passed from C on the stack (1st arg)
    ld HL, (IX+9)               ; buf, passed from C on the stack (2nd arg)
    ld BC, (IX+12)              ; bufsz, passed from C on the stack (2nd arg)
    MOSCALL mos_getError        ; function value in mos_api.inc
                                ; no return value
    call _portExitMOS           ; MOS critical exit

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret


;*****************************************************************************
;* MOD_FIL * mos_getfil( 
;*               unsigned int const filehandle 
;*           )
;*
;* Purpose: Retrieve a FIL descriptor for the given filehandle
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x19-mos_getfil
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     19h: mos_getfil
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to mos_api_getfil
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos.c
;* 
;* Parameters:
;*     (IX+6)  - filehandle into C
;*
;* Returns:
;*     FatFS FIL*  pointer to a FIL structure
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_MOS_FILE_OPS )

_mos_getfil:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld BC, (IX+6)               ; file handle, passed from C on the stack (1st arg)
    MOSCALL mos_getfil          ; function value in mos_api.inc
                                ; returns pointer to FIL structure in HL reg
    push HL                     ; preserve HL reg (file handle) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;*        library data is placed in linked section DATA
    segment DATA

    end
