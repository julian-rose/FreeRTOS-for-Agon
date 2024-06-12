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
;*  sbrk.asm
;*
;*  Copyright (c) 2001-2008 Zilog, Inc.
;*  Replace the Zilog compiler version for the Agon Light FreeRTOS port
;*   The Zilog memory model assumes the stack will always be located
;*   above the heap. This is not the cse for FreeRTOS, as each task has its
;*   own stack allocated within the heap.
;*
;* These functions should not normally be application-user altered.
;*****************************************************************************


    ;library code resides in linker section CODE
    segment CODE
    .assume ADL = 1


;*****************************************************************************
;* void *_s_sbrk( unsigned int const num_bytes );
;*
;* Purpose: (Replace the Zilog) Library Routine to grab memory from the heap
;*
;* Method: Malloc maintains a queue of allocated memory.chunks.
;*         When malloc lacks a chunk of memory large enough for the
;*         user program request, it calls sbrk to grab another chunk.
;*
;*         The Zilog version compares the heap address to the stack pointer.
;*         The Zilog memory model assumes the stack will always be located
;*         above the heap; and fails if the grabbed memory is a higher address 
;*         than the stack pointer (which grows down), assuming stack corruption.
;*         With FreeRTOS we allocate each task with its own stack from the 
;*         heap. So the Zilog routine will always fail. 
;*         This version of _sbrk for FreeRTOS grabs chunks from the heap 
;*         without checking the stack pointer,
;* 
;* Parameters:
;*     nbytes - Number of bytes requested (24-bit integer), on the stack
;* 
;* Returns:
;*     HLU: Address of allocated memory (24-bit pointer); 0 on fail
;* 
    .ref    __heapbot        ; configured in linker directve file
    .ref    __heaptop        ; configured in linker directive file
    .def    __s_sbrk         ; this library routine


__s_sbrk:
    ld     hl, 3               ; retrieve param nbytes from the stack into HL
    add     hl, sp
    ld     hl, (hl)
    
    ld     de, (__sbrkbase)    ; get the start of unallocated heap into DE
    add     hl, de              ; add the number of bytes requested
    jr     c,  _fail           ; fail if the sum is greater than a 24-bit number

    push hl                  ; hl is the next un-allocated address
    ld   bc, __heaptop       ; set bc to the uppermost heap location
    sbc  hl, bc              ; subtract __heaptop from HL (carry flag set if negative)
    pop  hl
    jr     nc, _fail           ; fail if hl >= __heaptop

_ok:
    ex     hl, de              ; get the old __sbrkbase into HL for  returning
    ld     (__sbrkbase), de    ; update the new start of unallocated heap
    ret                      ; allocated address in HL

_fail:
    or     a,  a
    sbc     hl, hl
    ret                      ; NULL in HL


;*****************************************************************************
;*        library data is placed in linked section DATA

    SEGMENT DATA
__sbrkbase:                  ; current base of unallocated heap
    .trio    __heapbot        ; initially __heapbot (grows towards __heaptop)

    end
