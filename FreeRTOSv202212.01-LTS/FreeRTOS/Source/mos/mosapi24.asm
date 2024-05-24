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

	xdef _mos_getkey


;*****************************************************************************
;*		library code is placed in linked section CODE
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
;*     Get keycode retruend from VDP (Requires MOS 1.03 or above)
;*     Implemented in MOS:
;*       vectors16.asm::_rst_08_handler -> call mos_api
;*       mos_api.asm::mos_api -> switch-like call to mos_api_getkey
;*         https://github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm
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
    push ix                 ; Standard prologue
    ld ix, 0
    add ix, sp

    call _portEnterMOS		; MOS critical enter
	
	MOSCALL mos_getkey   	; function value in mos_api.inc
							; returns ASCII byte in A

	push af					; MOS critical exit
	call _portExitMOS
	pop af

    ld sp, ix               ; Standard epilogue
	pop ix
	ret


;*****************************************************************************
;*		library data is placed in linked section DATA
    segment DATA

    end
