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
 * ffsapi.h for Agon Light
 *
 * portions based on mos_api.inc by Dean Belfield 
 *
 * FFS API for FreeRTOS for Agon Light by Julian Rose, 2024, with copyright 
 * assigned to Amazon Web Services as for FreeRTOS version 10.5.
 *
 * FFS API specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * Definitions for FFS interface functions for FreeRTOS for Agon Light 
 * (and compatibles).
 * Refer to https://agonconsole8.github.io/agon-docs/MOS-API/#fatfs-commands
 *          http://elm-chan.org/fsw/ff/doc/open.html
*/

#ifndef FFSAPI_H
#define FFSAPI_H


/*----- Type Definitions ----------------------------------------------------*/
typedef struct _fil_object_id  // File Object Identifier
{                              //   refer to mos_api.inc (and src_fatfs/ff.h)
    void *         fs;         // Pointer to the hosting volume of this object (FATFS *)
    unsigned short id;         // Hosting volume mount ID
    unsigned char  attr;       // Object attribute
    unsigned char  stat;       // Object chain status
    unsigned long  sclust;     // Object data start cluster (0:no cluster or root directory)
    unsigned long  objsize;    // Object size (valid when sclust != 0)
} MOS_FFOBJID;


typedef struct _fil            // File Descriptor
{                              //   refer to mos_api.inc
    MOS_FFOBJID     obj;       // Object identifier
    unsigned char   flag;      // File status flags
    unsigned char   err;       // Abort flag (error code)
    unsigned long   fptr;      // File read/write pointer (Zeroed on file open)
    unsigned long   clust;     // Current cluster of fpter (invalid when fptr is 0)
    unsigned long   sect;      // Sector number appearing in buf[] (0:invalid)
    unsigned long   dir_sect;  // Sector number containing the directory entry
    unsigned char * dir_ptr;   // Pointer to the directory entry in the win[]
} MOS_FIL;


typedef struct _filinfo            // File Information
{                                  //   refer to mos_api.inc
    unsigned long   fsize;         // File size
    unsigned short  fdate;         // Modified date
    unsigned short  ftime;         // Modified time
    unsigned char   fattrib;       // File attribute
    char            altname[ 13 ]; // Alternative file name
    char            fname[ 256 ];  // Primary file name
} MOS_FILINFO;


typedef struct _dir           // Directory Descriptor
{                             //   refer to mos_api.inc
    MOS_FFOBJID    obj;       // Object identifier
    unsigned long  dptr;      // Current read/write offset
    unsigned long  clust;     // Current cluster
    unsigned long  sect;      // Current sector (0:Read operation has terminated)
    unsigned int   dir;       // Pointer to the directory item in the win[]
    char           fn[ 12 ];  // SFN (in/out) {body[8],ext[3],status[1]}
    unsigned long  blk_ofs;   // Offset of current entry block being processed (0xFFFFFFFF:Invalid)
} MOS_DIR;


/*----- Function Declarations -----------------------------------------------*/
    /* FFS_API: ffs_fopen:        EQU    80h
       Open a file on the SD card
       Defined in ffsapi24.asm */
MOS_ERRNO ffs_fopen(
              MOS_FIL *filbuf,
              char const * const filename,
              MOS_FILE_MODE const mode
          );


    /* FFS_API: ffs_fclose:       EQU    81h
       Close a previously opened file on the SD card
       Defined in ffsapi24.asm */
MOS_ERRNO ffs_fclose(
              MOS_FIL *filbuf
          );


    /* FFS_API: ffs_fread:        EQU    82h
       Read a stream of bytes from a previously opened file
       Defined in ffsapi24.asm */
MOS_ERRNO ffs_fread(
              MOS_FIL *filbuf,
              void * const buf,
              size_t const num_bytes_to_read,
              size_t * num_bytes_read
          );


    /* FFS_API: ffs_fwrite:       EQU    83h
       Write a stream of bytes to a previously opened file
       Defined in ffsapi24.asm */
MOS_ERRNO ffs_fwrite(
              MOS_FIL *filbuf,
              void * const buf,
              size_t const num_bytes_to_write,
              size_t * num_bytes_written
          );


    /* FFS_API: ffs_flseek:        EQU    84h
       Seek to the start + offset position of a previously opened file
       Defined in ffsapi24.asm */
MOS_ERRNO ffs_flseek(
              MOS_FIL *filbuf,
              unsigned long int const offset
          );


    /* FFS_API: ffs_feof:         EQU    8Eh
       Test whether a previously opened file position is at the end of file
       Defined in ffsapi24.asm */
unsigned char ffs_feof(
                  MOS_FIL *filbuf
              );


    /* FFS_API: ffs_dopen:        EQU    91h     NOT SUPPORTED
       Open a directory on the SD card
       Requires MOS 2.2.0 */


    /* FFS_API: ffs_dclose:       EQU    92h     NOT SUPPORTED
       Close a previously opened directory on the SD card
       Requires MOS 2.2.0 */


    /* FFS_API: ffs_dread:        EQU    93h     NOT SUPPORTED
       Read a stream of bytes from a previously opened file
       Requires MOS 2.2.0 */


    /* FFS_API: ffs_fstat:        EQU    96h
       Retrieves file information for the named file on SD-card
       Defined in ffsapi24.asm */
MOS_ERRNO ffs_fstat(
              char const * const filename,
              MOS_FILINFO *filbuf
          );


    /* FFS_API: ffs_getcwd:       EQU    9Eh     NOT SUPPORTED
       Read a stream of bytes from a previously opened file
       Requires MOS 2.2.0 */


#endif /* FFSAPI_H */
