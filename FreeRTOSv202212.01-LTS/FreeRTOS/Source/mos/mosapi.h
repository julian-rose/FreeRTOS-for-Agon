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
 * mosapi.h for Agon Light
 *
 * portions based on mos_api.inc by Dean Belfield 
 *
 * MOS API for FreeRTOS for Agon Light by Julian Rose, 2024, with copyright 
 * assigned to Amazon Web Services as for FreeRTOS version 10.5.
 *
 * MOS API specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * Definitions for MOS interface functions for FreeRTOS for Agon Light 
 * (and compatibles).
 * Refer to https://agonconsole8.github.io/agon-docs/MOS-API/#the-mos-api
*/

#ifndef MOSAPI_H
#define MOSAPI_H


/*----- Constants -----------------------------------------------------------*/
#define ESC  27
#define CRET 13
#define LF   10
#define BS    8

#if !defined( EOF )
# define EOF -1
#endif

#define RTC_EPOCH_YEAR 1980


/*----- Enumeration Types ---------------------------------------------------*/
typedef enum _mos_errnum              // MOS error numbers
{
    /* refer to /agon-mos-main/src_fatfs/ff.h
                /agon-mos-main/mos.c */
    MOS_ERR_OK                 = 0,
    MOS_ERR_SDCARD_ACCESS      = 1,
    MOS_ERR_ASSERT             = 2,
    MOS_ERR_SDCARD_FAIL        = 3,
    MOS_ERR_FILE_NOT_EXIST     = 4,
    MOS_ERR_PATH_NOT_EXIST     = 5,
    MOS_ERR_PATHNAME_INVALID   = 6,
    MOS_ERR_DIRECTORY_FULL     = 7,
    MOS_ERR_ACCESS_DENIED      = 8,
    MOS_ERR_INVALID_FILE       = 9,
    MOS_ERR_SDCARD_WRITE       = 10,
    MOS_ERR_DRIVE_NUM          = 11,
    MOS_ERR_VOL_WORKAREA       = 12,
    MOS_ERR_VOL_FILESYSTEM     = 13,
    MOS_ERR_MKFS               = 14,
    MOS_ERR_VOL_TIMEOUT        = 15,
    MOS_ERR_VOL_LOCKED         = 16,
    MOS_ERR_LFN_BUF            = 17,
    MOS_ERR_TOO_MANY_FILES     = 18,
    MOS_ERR_PARAM_INVALID      = 19,
    MOS_ERR_COMMAND_INVALID    = 20,
    MOS_ERR_EXECUTABLE_INVALID = 21,
    
    MOS_ERR_END

} MOS_ERRNO;


typedef enum _mos_file_access_modes    // MOS FAT file access modes
{
    MOS_FAM_READ           = 0x01,     // open for reading only (writing locked)
    MOS_FAM_WRITE          = 0x02,     // open for reading and writing (fail if file is read-only)
    MOS_FAM_CREATE         = 0x04,     // CREATE_NEW create iff file does not already exist else fail
    MOS_FAM_TRUNC          = 0x08,     // CREATE_ALWAYS or OVERWRITE if the file already exists
    MOS_FAM_OPEN_ALWAYS    = 0x10,     // open if file already exists else create
    MOS_FAM_APPEND         = 0x20,     // seek to end of file on opening (see FA_SEEKEND in ff.c)

        // extra definitions
    MOS_FAM_OPEN_APPEND    = 0x30,     // (following http://elm-chan.org/fsw/ff/doc/open.html)
    MOS_FAM_OPEN_EXISTING  = 0x00      // (following http://elm-chan.org/fsw/ff/doc/open.html)

} MOS_FILE_MODE;


typedef enum _mos_i2c_freq       // defined in MOS src/i2c.h
{
    MOS_I2C_FREQ_57600     = 1,
    MOS_I2C_FREQ_115200    = 2,
    MOS_I2C_FREQ_230400    = 3

} MOS_I2C_FREQ;


typedef enum _mos_i2c_result
{
    MOS_I2C_OK      = 0,         // Transfer succeeded
    MOS_I2C_NO_RESP = 1,         // if 127 < address or if addressed device nacks
    MOS_I2C_NACK =    2,         // if device does not acknowledge data receipt
    MOS_I2C_ARB =     3,         // if I2C bus handshake times-out during transfer
    MOS_I2C_BUSERR =  4          // if I2C bus has been reset

} MOS_I2C_RESULT;


/*----- Type Definitions ----------------------------------------------------*/
typedef struct _mos_sysvars_desc                // System Variables Descriptor
{
    unsigned long  MOS_SYSVAR_TIME;             // Clock timer in centiseconds (incremented by 2 every VBLANK)
    unsigned char  MOS_SYSVAR_VPD_PFLAGS;       // Flags to indicate completion of VDP commands
    unsigned char  MOS_SYSVAR_KEY_ASCII;        // ASCII keycode, or 0 if no key is pressed
    unsigned char  MOS_SYSVAR_KEY_MODS;         // Keycode modifiers
    unsigned char  MOS_SYSVAR_CURSOR_X;         // Cursor X position
    unsigned char  MOS_SYSVAR_CURSOR_Y;         // Cursor Y position
    unsigned char  MOS_SYSVAR_SCR_CHAR;         // Character read from screen
    unsigned int   MOS_SYSVAR_SCR_PIXEL;        // Pixel data read from screen (R,B,G)
    unsigned char  MOS_SYSVAR_AUDIO_CHANNEL;    // Audio channel 
    unsigned char  MOS_SYSVAR_AUDIO_SUCCESS;    // Audio channel note queued (0 = no, 1 = yes)
    unsigned short MOS_SYSVAR_SCR_WIDTH;        // Screen width in pixels
    unsigned short MOS_SYSVAR_SCR_HEIGHT;       // Screen height in pixels
    unsigned char  MOS_SYSVAR_SCR_COLS;         // Screen columns in characters
    unsigned char  MOS_SYSVAR_SCR_ROWS;         // Screen rows in characters
    unsigned char  MOS_SYSVAR_SCR_COLOURS;      // Number of colours displayed
    unsigned char  MOS_SYSVAR_SCR_PIXEL_INDEX;  // Index of pixel data read from screen
    unsigned char  MOS_SYSVAR_VKEY_CODE;        // Virtual key code from FabGL
    unsigned char  MOS_SYSVAR_VKEY_DOWN;        // Virtual key state from FabGL (0=up, 1=down)
    unsigned char  MOS_SYSVAR_VKEY_COUNT;       // Incremented every time a key packet is received
    unsigned char  MOS_SYSVAR_RTC[ 6 ];         // Real time clock data
    unsigned short MOS_SYSVAR_SPARE1;           // Deprecated, previously used by rtc
    unsigned short MOS_SYSVAR_KEY_DELAY;        // Keyboard repeat delay
    unsigned short MOS_SYSVAR_KEY_RATE;         // Keyboard repeat rate
    unsigned char  MOS_SYSVAR_KEY_LED;          // Keyboard LED status
    unsigned char  MOS_SYSVAR_SCR_MODE;         // Screen mode
    unsigned char  MOS_SYSVAR_RTC_ENABLE;       // RTC enable flag (0: disabled, 1: use ESP32 RTC)
    unsigned short MOS_SYSVAR_MOUSE_X;          // Mouse X position
    unsigned short MOS_SYSVAR_MOUSE_Y;          // Mouse Y position
    unsigned char  MOS_SYSVAR_MOUSE_BUTTONS;    // Mouse button state
    unsigned char  MOS_SYSVAR_MOUSE_WHEEL;      // Mouse wheel delta
    unsigned short MOS_SYSVAR_MOUSE_X_DELTA;    // Mouse X delta
    unsigned short MOS_SYSVAR_MOUSE_Y_DELTA;    // Mouse Y delta
} MOS_SYSVARS_DESC;


typedef struct _vdp_kb_packet    // VDP Keyboard Packet
{
    unsigned char keyascii;      // ASCII key code
    unsigned char keymod;        // SHIFT, ALT, ...
    unsigned char vkeycode;      // virtual keycode
    unsigned char keystate;      // 1=down, 0=up
} VDP_KB_PACKET;


typedef char KEYMAP[ 16 ];       // Virtual Keyboard map (16*8 bits, each bit a unique key)


typedef struct _mos_uart         // UART descriptor
{
    unsigned int  baudRate;      // baudrate (bits per sec) 
    unsigned char dataBits;      // number of databits per character to be used (in range 5..8)
    unsigned char stopBits;      // number of stopbits to be used (in range 1..2)
    unsigned char parity;        // parity bit option to be used (00b=none, 01b=odd, 11b=even)
    unsigned char flowControl;   // flow control option (0: None, 1: Hardware)
    unsigned char interrupts;    // enabled interrupts (1h=receive, 2h=transmit, 4h=line, 
                                 //                     8h=modem, 10h=sent)
} MOS_UART;


typedef char MOS_RTC_STRING_R[ 32 ];   // String output from mos_GETRTC::rtc_unpack needs 31 bytes
typedef char MOS_RTC_STRING_W[ 6 ];    // String input to mos_SETRTC needs 6 bytes


/*----- Global Names --------------------------------------------------------*/
extern void mos_printerr( char const * const callstr, MOS_ERRNO const err );


/*----- Function Declarations -----------------------------------------------*/
    /* MOS_API: mos_getkey:          EQU    00h
       Get ASCII code of any key pressed, or 0 if no key pressed, non-blocking
       Defined in mosapi24.asm
       Use: while( 0 ==( ch = mos_getkey( ))); // busy wait for a key press */
unsigned char mos_getkey( 
                void 
              );


    /* MOS_API: mos_load:            EQU    01h
       Load a file from SD card into a memory location
       Defined in mosapi24.asm
       Use: err = mos_load( "test.dat", buf, bufsz ); // copy test.dat to buf */
MOS_ERRNO mos_load( 
              char const * const filename, 
              void * const address, 
              size_t const size
          );


    /* MOS_API: mos_save:            EQU    02h
       Save a file to SD card from a memory location
       Defined in mosapi24.asm */
MOS_ERRNO mos_save(
              char const * const filename, 
              void * const address, 
              size_t const size
          );


    /* MOS_API: mos_cd:              EQU    03h
       Change current working directory
       Defined in mosapi24.asm */
MOS_ERRNO mos_cd( 
              char const * const path
          );


    /* MOS_API: mos_dir:             EQU    04h  NOT SUPPORTED
       Displays a current directory listing.
       We can support this when it returns an array of filenames.
       To Be Defined in mosapi24.asm */


    /* MOS_API: mos_del:             EQU    05h
       Delete a file from SD card
       Defined in mosapi24.asm */
MOS_ERRNO mos_del( 
              char const * const filename
          );


    /* MOS_API: mos_ren:             EQU    06h
       Rename a file on SD card
       Defined in mosapi24.asm */
MOS_ERRNO mos_ren( 
              char const * const oldName, 
              char const * const newName
          );


    /* MOS_API: mos_mkdir:           EQU    07h
       Load a file from SD card
       Defined in mosapi24.asm */
MOS_ERRNO mos_mkdir( 
              char const * const pathname
          );

    /* MOS_API: mos_sysvars:         EQU    08h
       Get a pointer to a MOS system variable
       Defined in mosapi24.asm */
void * mos_getsysvars( 
           void
       );


    /* MOS_API: mos_editline:        EQU    09h    NOT SUPPORTED
       Load a file from SD card
       NOT SUPPORTED */


    /* MOS_API: mos_fopen:           EQU    0Ah
       Open a file on the SD card
       Defined in mosapi24.asm */
unsigned int mos_fopen( 
                 char const * const filename, 
                 MOS_FILE_MODE const mode
              );


    /* MOS_API: mos_fclose:          EQU    0Bh
       Close a previously opened file
       Defined in mosapi24.asm */
unsigned int mos_fclose( 
                unsigned int const filehandle
             );


    /* MOS_API: mos_fgetc:           EQU    0Ch
       Reacd a character from a previously opened file
       Defined in mosapi24.asm */
unsigned char mos_fgetc( 
                unsigned int const filehandle
              );


    /* MOS_API: mos_fputc:           EQU    0Dh
       Write a character to a previously opened file
       Defined in mosapi24.asm */
void mos_fputc( 
                unsigned int const filehandle, 
                char const c
     );


    /* MOS_API: mos_feof:            EQU    0Eh
       Test whether a previously opened file position is at the end of file
       Defined in mosapi24.asm */
unsigned char mos_feof( 
                unsigned int const filehandle
              );


    /* MOS_API: mos_getError:        EQU    0Fh
       Get a string value for a system error code
       Defined in mosapi24.asm */
void mos_geterror( 
                MOS_ERRNO const errno, 
                void * const buffer, 
                size_t const buffersize
     );


    /* MOS_API: mos_oscli:           EQU    10h    NOT SUPPORTED
       Issue a MOS command (as if typed on the terminal keyboard)
       NOT SUPPORTED */


    /* MOS_API: mos_copy:            EQU    11h
       Make a copy of a file on the SD card
       Defined in mosapi24.asm */
MOS_ERRNO mos_copy(
              char const * const src, 
              char const * const dst
          );


    /* MOS_API: mos_getrtc:          EQU    12h
       Get the current RTC value into a (25 character) string buffer
       FMT: "DDD, dd/mm/yyyy hh:mm:ss\0"
       Defined in devapi24.asm */
void mos_getrtc(
         MOS_RTC_STRING_R const buf
     );


    /* MOS_API: mos_setrtc:          EQU    13h
       Set the RTC value from a (6 character) buffer
       FMT: "ymdhms"
       Defined in devapi24.asm */
void mos_setrtc(
         MOS_RTC_STRING_W const buf
     );


    /* MOS API: mos_setintvector     EQU    14h
       Assigns a service routine to an interrupt vector
       Defined in mosvec24.asm as it is used by the FreeRTOS port to bind the
       PIT timer */
void * mos_setintvector( 
           unsigned int, 
           void ( *intr_hndlr )( void )
       );


    /* MOS_API: mos_uopen:           EQU    15h
       Open UART1 device
         Call mos_setintvector to assign an interrupt handler
       Defined in devapi24.asm */
MOS_ERRNO mos_uopen(
              MOS_UART const * const pUART
          );


    /* MOS_API: mos_uclose:          EQU    16h
       Close UART1 device
       Defined in devapi24.asm */
void mos_uclose( 
         void
     );


    /* MOS_API: mos_ugetc:           EQU    17h
       Read a character from UART1 (blocking)
         Use mos_setintvector and an ISR with mos_uopen for non-blocking reads
         Returns 0x0 char on failure (and when received)
       Defined in devapi24.asm */
unsigned char mos_ugetc( 
                void
              );
                

    /* MOS_API: mos_uputc:           EQU    18h
       Write a character to UART1
       Defined in devapi24.asm */
MOS_ERRNO mos_uputc(
              unsigned char const ch
          );


    /* MOS_API: mos_getfil:          EQU    19h
       Retrieve the file descriptor of a previously opened filehandle
       Defined in mosapi24.asm */
void * mos_getfil(
           unsigned int const filehandle
       );


    /* MOS_API: mos_fread:           EQU    1Ah
       Read a stream of bytes from a previously opened filehandle
       Defined in mosapi24.asm */
unsigned int mos_fread(
                 unsigned int const filehandle,
                 void * const buffer, 
                 size_t const num_bytes_to_read
             );


    /* MOS_API: mos_fwrite:          EQU    1Bh
       Write a stream of bytes to a previously opened filehandle
       Defined in mosapi24.asm */
unsigned int mos_fwrite(
                 unsigned int const filehandle,
                 void const * const buffer,
                 size_t const num_bytes_to_write
             );


    /* MOS_API: mos_flseek:          EQU    1Ch
       Seek to the start + offset position of a previously opened file
       Defined in mosapi24.asm */
MOS_ERRNO mos_flseek(
              unsigned int const filehandle,
              unsigned long int const offset
          );


    /* MOS_API: mos_setkbvector:     EQU    1Dh
       Attach a user callback function to the keyboard packet receiver
       Defined in devapi24.asm */
void mos_setkbvector( 
         void ( *kybd_hndlr )( VDP_KB_PACKET* )
     );


    /* MOS_API: mos_getkbmap:        EQU    1Eh
       Get a pointer to the keyboard map of pressed keys
       Defined in devapi24.asm */
KEYMAP * mos_getkbmap( 
             void
         );


    /* MOS_API: mos_i2c_open:        EQU    1Fh
       Open the I2C device as a bus controller
       The device is "connectionless", this re-initialises the MOS driver
       Defined in devapi24.asm */
void mos_i2c_open(
         MOS_I2C_FREQ const frequency
     );


    /* MOS_API: mos_i2c_close:       EQU    20h
       Close the i2c device
       The device is "connectionless", this halts the MOS driver activity
       Defined in devapi24.asm */
void mos_i2c_close(
         void
     );


    /* MOS_API: mos_i2c_write:       EQU    21h
       Write a stream (between 1..32) of bytes to target i2c device with 
         address (0..127)
       Defined in devapi24.asm */
MOS_I2C_RESULT mos_i2c_write(
                   unsigned char const i2c_address, 
                   void const * const buffer,
                   size_t const num_bytes_to_write 
               );


    /* MOS_API: mos_i2c_read:        EQU    22h
       Read a stream (between 1..32) of bytes from target i2c device with 
         address (0..127)
       Defined in devapi24.asm */
MOS_I2C_RESULT mos_i2c_read(
                   unsigned char const i2c_address,
                   void * const buffer,
                   size_t const num_bytes_to_read
               );


#endif /* MOSAPI_H */
