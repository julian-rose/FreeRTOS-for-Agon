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
;* ffsapi24.asm
;*   FFS API functions defined at https://agonconsole8.github.io/agon-docs/MOS-API
;*   Created 07/Jun/2024 by Julian Rose for Agon Light port
;*
;*****************************************************************************
include "mosConfig.inc"
include "mos_api.inc"

    xref _portEnterMOS
    xref _portExitMOS

IF( 1 == configUSE_FFS_FILE_OPS )
    xdef _ffs_fopen             ; implement call to FFS API function 80h
    xdef _ffs_fclose            ; implement call to FFS API function 81h
    xdef _ffs_fread             ; implement call to FFS API function 82h
    xdef _ffs_fwrite            ; implement call to FFS API function 83h
    xdef _ffs_flseek            ; implement call to FFS API function 84h
    xdef _ffs_feof              ; implement call to FFS API function 8Eh
ENDIF

IF( 1 == configUSE_FFS_DIR_OPS )
    xdef _ffs_stat              ; implement call to FFS API function 96h
ENDIF


;******************** Globals ************************************************
;* globals are placed in linked section BSS
    segment BSS


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
;* MOS_ERRNO ffs_fopen(
;*               MOS_FIL *filbuf,
;*               char const * const filename,
;*               MOS_FILE_MODE const mode
;*               )
;*
;* Purpose: Open a file for i/o (mode depending) on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x80-ffs_fopen
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     80h: ffs_fopen
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to ffs_api_fopen
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filbuf, into HL
;*     (IX+9)  - filename (null-terminated) into DE
;*     (IX+12) - mode (combination of MOS_FILE_MODE values) into BC
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_FFS_FILE_OPS )

_ffs_fopen:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filbuf, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; filename, passed from C on the stack (2nd arg)
    ld BC, (IX+12)              ; mode, passed from C on the stack (3rd arg)
    MOSCALL ffs_fopen           ; function value in mos_api.inc
                                ; returns errno in A reg
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get errno from A into int HL (int return)
    
    push HL                     ; preserve HL reg (file handle) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover HL reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* MOS_ERRNO ffs_fclose(
;*               MOS_FIL *filbuf
;*           );
;*
;* Purpose: Close a previously fopened file on the SD-card
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x81-ffs_fclose
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     81h: ffs_fclose
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to ffs_api_fclose
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filbuf, into HL
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_FFS_FILE_OPS )

_ffs_fclose:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filbuf, passed from C on the stack (1st arg)
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
;* MOS_ERRNO ffs_fread(
;*               MOS_FIL *filbuf,
;*               void * const buf,
;*               size_t const num_bytes_to_read,
;*               size_t * num_bytes_read
;*           );
;*
;* Purpose: Read up to num_bytes_to_read from the current seek position from a 
;*          previously ffs_fopened file on the SD-card into buf, increment the 
;*          seek position, set num_bytes_Read and return errno
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x82-ffs_fread
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     82h: ffs_fread
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to ffs_api_fread
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filbuf into HLU
;*     (IX+9)  - Memory address of buffer to write to, into DEU
;*     (IX+12) - Max number of bytes to read into buffer, into BCU
;*     (IX+15) - Num bytes read pointer, outof BC
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_FFS_FILE_OPS )

_ffs_fread:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filbuf, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; buf ptr, passed from C on the stack (2nd arg)
    ld BC, (IX+12)              ; num bytes to read, passed from C on the stack (3rd arg)
    MOSCALL ffs_fread           ; function value in mos_api.inc    
                                ; returns number of bytes read in BC reg, and errno in A reg
    ld HL, (IX+15)              ; set HL = pointer to int num_bytes_read
    ld (HL), BC                 ; save number of bytes read into what HL points at

    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get errno from A into int HL (int return)

    push HL                     ; preserve errno (HL) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover errno (HL)

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* MOS_ERRNO ffs_fwrite(
;*               MOS_FIL *filbuf,
;*               void * const buf,
;*               size_t const num_bytes_to_write,
;*               size_t * num_bytes_written
;*           );
;*
;* Purpose: Write up to num_bytes_to_write to the current seek position into a 
;*          previously ffs_fopened file on the SD-card from buf, increment the 
;*          seek position; return num_bytes_written and errno
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x83-ffs_fwrite
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     83h: ffs_fwrite
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to ffs_api_fwrite
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filbuf into HLU
;*     (IX+9)  - Memory address of buffer to write from, into DEU
;*     (IX+12) - Max number of bytes to write into buffer, into BCU
;*     (IX+15) - Num bytes written, outof BC
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_FFS_FILE_OPS )

_ffs_fwrite:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filbuf, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; buf ptr, passed from C on the stack (2nd arg)
    ld BC, (IX+12)              ; num bytes to write, passed from C on the stack (3rd arg)
    MOSCALL ffs_fwrite          ; function value in mos_api.inc    
                                ; returns number of bytes written in BC reg, and errno in A reg
    ld HL, (IX+15)              ; set HL = pointer to int num_bytes_written
    ld (HL), BC                 ; save number of bytes written into what HL points at

    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get errno from A into int HL (int return)

    push HL                     ; preserve errno (HL) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover errno (HL)

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* MOS_ERRNO ffs_flseek(
;*               MOS_FIL *filbuf,
;*               unsigned long int const offset
;*           )
;*
;* Purpose: Set the current seek position in a previously ffs_fopened file on 
;*          the SD-card 
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x84-ffs_fseek
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     84h: ffs_flseek
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to ffs_api_flseek
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filbuf, into HLU
;*     (IX+9)  - seek position offset from start of file, into DEU
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
;
IF( 1 == configUSE_FFS_FILE_OPS )

_ffs_flseek:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filbuf, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; offset 23:0, passed from C on the stack (2nd arg)
    ld C, 0                     ; offset 31:24
    MOSCALL ffs_flseek          ; function value in mos_api.inc
                                ; returns result in A reg
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get errno from A into int HL (int return)

    push HL                     ; preserve errno (HL) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover errno (HL)

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* unsigned char ffs_feof(
;*                   MOS_FIL *filbuf
;*               );
;*
;* Purpose: Test if a previously fopened file on the SD-card is at the last
;*          seek position
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x8e-ffs_feof
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     8Eh: ffs_feof
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to ffs_api_feof
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.h
;* 
;* Parameters:
;*     (IX+6)  - filbuf, into HLU
;*
;* Returns:
;*     1 at EOF else 0, in A reg
;*     Refer to AN033301 table 2 for registers containing different return types
; 
IF( 1 == configUSE_FFS_FILE_OPS )

_ffs_feof:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filbuf, passed from C on the stack (1st arg)
    MOSCALL ffs_feof            ; function value in mos_api.inc
                                ; returns 1 (EOF) or 0 (not EOF) in A reg
    push AF                     ; preserve A reg (character) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop AF                      ; recover A reg

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;* MOS_ERRNO ffs_fstat(
;*               char const * const filename,
;*               MOS_FILINFO *filbuf
;*           )
;*
;* Purpose: Retrieves file information for the named file on SD-card 
;*
;* Method:
;*   https://agonconsole8.github.io/agon-docs/MOS-API/#0x96-ffs_stat
;*
;*   Invoke a MOS function call, through RST 8, passing in function number:
;*     96h: ffs_stat
;*   Implemented in MOS vectors16.asm::_rst_08_handler -> call mos_api
;*     mos_api.asm::mos_api -> switch-like call to ffs_api_stat
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src/mos_api.asm
;*     https://github.com/breakintoprogram/agon-mos/blob/main/src_fatfs/ff.c
;* 
;* Parameters:
;*     (IX+6)  - filbuf, into HLU
;*     (IX+9)  - filename (null-terminated) into DEU
;*
;* Returns:
;*     FatFS Errno (int)
;*     Refer to AN033301 table 2 for registers containing different return types
;
IF( 1 == configUSE_FFS_DIR_OPS )

_ffs_stat:

    push ix                     ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS          ; MOS critical enter

    ld HL, (IX+6)               ; filbuf, passed from C on the stack (1st arg)
    ld DE, (IX+9)               ; filename, passed from C on the stack (2nd arg)
    MOSCALL ffs_stat            ; function value in mos_api.inc
                                ; returns result in A reg
    SET_HLU24 0                 ; ld 0 (clear) HLU upper byte
    ld HL, 0                    ; clear HL (now HLU=0)
    ld L, A                     ; get errno from A into int HL (int return)

    push HL                     ; preserve errno (HL) over call to portExitMOS
    call _portExitMOS           ; MOS critical exit
    pop HL                      ; recover errno (HL)

    ld sp, ix                   ; Standard epilogue
    pop ix

    ret

ENDIF


;*****************************************************************************
;*        library data is placed in linked section DATA
    segment DATA

    end
