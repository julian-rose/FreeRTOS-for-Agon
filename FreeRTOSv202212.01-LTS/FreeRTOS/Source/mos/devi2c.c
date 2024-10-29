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
 * devi2c.c for Agon Light
 *
 * Port specific definitions.  
 * Target:  Agon Light (eZ80)
 * Compiler: Zilog eZ80 ANSI C Compiler Version 3.4 (19101101) 
 *           ZDSII in ADL mode
 *
 * The definitions in this file support the DEV API (I2C low-level)
 * for Agon Light (and compatibles)
 * Created 20.Jun.2024 by Julian Rose for Agon Light port, shell
 * Modified from 13.August.2024 J. Rose, along the lines of devuart
 *
 * These functions should not normally be application-user altered.
*/

#include <stdlib.h>
#include <stdio.h>

#include <eZ80F92.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "devConfig.h"
#include "devapi.h"
#include "devapil.h"
#include "mosapi.h"


#if( 1 == configUSE_DRV_I2C )


/*---- Notes ------------------------------------------------------------------
 * NOTE 0: I2C bus wiring
 *  
 * Refer to NXP UM10204, section 3.1.1 and Figure 3.
 *  With Agon use the 3.3V output (pin 34) for Vdd (and Vddio), and one of the 
 *  GND outputs (pins 33, 5, 3) for Vss.
 *
 * A single I2C device can be connected one-to-one with Agon, or a small number 
 *  of I2C devices can be connected in parallel. SDA (pin 29) and SCL (pin 30) 
 *  can be taken to separate rails on a breadboard; similarly Vdd (pin 34) and 
 *  Vss (GND) can be taken to separate rails (that's 4 rails in all) to form a 
 *  parallel bus. Breadboards with 4 or 8 rails are commonly available. Best to
 *  use non-adjacent rails for SDA & SCL to avoid cross-induction.
 *
 *
 * Note 0.1: Breadboard pull-ups
 *
 * As per UM101204 section 3.1.1, both SDA and SCL are bidirectional lines, 
 *  each connected to the positive supply voltage (Vdd) via a pull-up resistor 
 *  (Rp). Such that when the bus is idle, both lines are HIGH. This establishes 
 *  the quiescent state of the I2C bus, necessary prior to Controller signalling 
 *  the START condition. 
 *
 *  The Olimex Light2 board embeds pull-up resistors of 2.2Kohm, R39 for SDA and 
 *   R40 for SCL, adjacent to the Access Bus header. In general, for a small 
 *   number of sensors or actuators this is sufficient. 2K2 values were perhaps 
 *   chosen to be the same as other Olimex boards, known to work with UEXT1.
 *   Rather than the more usual 4K7s of Light.
 *
 *  However, if you would add additional pull-ups on a breadboard, Rpsda and 
 *   Rpscl, the total resistance across each of SDA and SCL is measured in-
 *   parallel with R39 and R40 respectively. 
 *
 *   Refer to NXP UM10204, section 7.1 pull-up sizing, for calculating minimum 
 *   and maximum resistor values. The maximum value for Rp (figure 42) is 
 *   affected by the total capacitance of the number of devices to be attached, 
 *   and by the desired bus speed. In the general case, at standard bus-speed 
 *   (up to 100kbs) with Vdd=3.3v, the maximum value will be in the order of 10 
 *   to 20Kohm. The minimum value (figure 43) is affected by Vdd and by the bus 
 *   speed. The minimum value at standard speed with Vdd=3.3v is in the order 
 *   of 1.1Kohm (basically ohm's law, with a bus sink current of 3mA). 
 *
 *   With Agon running a standard clock speed of 57600bps, suitable values for 
 *   both Rpsda and Rpcsl lie between 1.5K to 10K ohms, with 3.3Kohms as a good 
 *   initial candidate. Such that the total resistance "in parallel" with R39 
 *   and R40 will be 1/R = 1/2.2K + 1/3.3K -> R= 1.3K
 *
 *-----------------------------------------------------------------------------
 * NOTE 1: Use Cases
 *  
 *  DEV I2C broadly recognises two Use Cases:
 *
 *  1/ Master-Controller is like the SPI use case. In this, a sole controller 
 *     accesses a number of slave devices (sensors and/or actuators) on the 
 *     I2C-bus. (MOS implements a subset of this Use Case: 7-bit addressing, 
 *     and no restart - sufficient for most typical uses.)
 *
 *  2/ Multi-Master is a network pattern, in which a number of controllers 
 *     inter-connect via the I2C bus. An Agon Cluster is such an arrangement
 *     (see note 2.1). There may also be slave devices on this I2C bus, such
 *     that Master-Controller is a subset of Multi-Master, with one Master. 
 *
 *-----------------------------------------------------------------------------
 * NOTE 2: Why not just use MOS I2C?
 *
 * MOS I2C supports two of the possible four roles: Master Transmit and Master 
 *  Receive. (MOS omits 10-bit addressing and Restart.) These Single-Master 
 *  roles are sufficient for controlling many sensors and actuators attached to 
 *  the I2C bus (similar to SPI use-cases). Applications may open DEV I2C in 
 *  DEV_MODE_I2C_SINGLE_MASTER mode for this purpose. Or they may continue to 
 *  use MOS I2C (and disable DEV I2C to save RAM footprint) if multi-tasking is
 *  not used. In applications with multi-tasking, DEV I2C is necessary. 
 *
 * DEV API shall support I2C-bus Multi-Mastering, adding the other two roles, 
 *  Slave Transmit and Slave Receive, allowing other devices to act as bus
 *  Master. This supports networking multiple intelligent devices on the bus. 
 *  Applications shall open DEV I2C in DEV_MODE_I2C_MULTI_MASTER mode for this 
 *  purpose; an optional callback parameter allows the application to respond
 *  to Slave Tranmsit requests from remote Master receivers. 
 *
 *
 * Note 2.1 - Agon Clusters
 *
 * I2C specifies General Call Addressing and the Device ID protocol which 
 *  can be used to determine the type of each bus-connected Device. DEV I2C 
 *  shall support this, in order that multiple Agon devices can find each other. 
 *
 * While none of the other interfaces (UART, GPIO, SPI) lend themselves well to 
 *  local networking, in connecting a number of Agons using I2C we observe: 
 *
 *  i/ Olimex Light2 embeds 2K2 pull-up resistors on each of SCL & SDA (see 
 *  note 0). Connecting Agons together combines their I2C pull-ups in-parallel. 
 *  When powered up the resulting 1/R sum for N=2 Light2 gives 1K1, the minimum 
 *  for an operational I2C-bus. This might work for N=2 Light2 [it doesn't], 
 *  but will sum to be out of specification for N>2. (By comparison Agon Light 
 *  embeds 4K7s, such that for N=2 1/R sums to 2K3; and Zilog eZ80AcclaimPlus! 
 *  Development Board embeds 5K pullups, such that for N=2 1/R sums to 2K5; 
 *  both boards well within specification.) Moreover, if any I2C-connected Agon 
 *  is powered-down, its SDA & SCL resistors effectively create a circuit to 
 *  GND from any powered-up Agon. This pulls the SDA & SCL lines out of 
 *  specification, such that any powered Agon in the circuit will be unable to 
 *  access the bus successfully. 
 *
 *  ii/ In general, to create an N>1 Agon network, N P82B96 buffer ICs should 
 *  be wired in parallel, such that: the Tx/Rx pins of all P82B96s are wired to
 *  form a single common signalling backplane; and the Sx/Sy pins of each 
 *  P82B96 are wired to bridge separate local I2C busses to which a single Agon
 *  is attached. Each Agon will power its own local I2C bus, and the devices
 *  attached to it. Slave devices attached to any of the local I2C busses will 
 *  be visible to all Agon Masters across the backplane. This arrangement will 
 *  function successfully for any powered Agon; un-powered local I2C busses and 
 *  devices will not be visible, while the backplane will continue to function. 
 *
 *  iii/ I2C was designed for intra-board connectivity, but is widely used to 
 *  inter-connect devices with up to 2m of wire requiring no further hardware. 
 *  For longer distance connectivity, P82B96 buffers can be used, to reach up 
 *  to 200m. To inter-connect Agon over even longer distances, we might use 
 *  DEV UART with Modems or Bridges or Wifi, with an increase in cost and 
 *  complexity and energy consumption. 
 *
 *-----------------------------------------------------------------------------
 * NOTE 3: To transact (like DEV UART) or not (like DEV SPI)
 * 
 * Transactions work well for DEV UART because the UART device has a hardware 
 *  FIFO (of depth 16). Each interrupt allows up to 16 bytes to be transceived
 *  without CPU involvement. Enough time for a concurrent task to do useful 
 *  work. Hence, for the UART (DEV_MODE_BUFFERED) non-blocking transactions are 
 *  useful. 
 *
 *  Whereas, the eZ80 I2C device (like the SPI device) does not have a FIFO;
 *  such that each transceived byte will be reported to the CPU by an interrupt,
 *  causing a  context switch from the current task to the ISR each time. This 
 *  interrupt frequency leaves less useful time for a concurrent task to run. 
 *
 *  Moreover, unlike DEV UART in which reading and writing are asynchronous and 
 *  can be full duplex with concurrent tasks (using DEV UART), access to the 
 *  I2C-bus is limited to one active transaction at any time.
 *
 * However, transactions are a useful way to implement ISR-driven state 
 *  transitions, with minimal user-application involvement. (In particular, for
 *  I2C bus multi-mastering Slave Receive and Slave Transmit roles). Therefore, 
 *  DEV I2C shall use Blocking Transactions, such that DEV I2C-invoking tasks 
 *  will block for the duration of each call, including Restarts. While Non-
 *  blocking transactions shall be omitted. All transactions will execute in 
 *  the context of i2cisr using transaction buffers, similar to UART.
 *
 *-----------------------------------------------------------------------------
 * NOTE 4: Master Transmit and Receive role handling
 *
 * Master Transmit and Master Receive roles shall be invoked by application 
 *  calls to i2c_writem and i2c_readm respecitvely. DEV I2C shall not provide 
 *  buffers for these functions, as the task-provided parameters are sufficient, 
 *  and avoid the need for memcpy. 
 *
 *  Calling i2c_open with parameter DEV_MODE_I2C_SINGLE_MASTER will access DEV I2C
 *   in Master role only (no Slave role).
 *
 *-----------------------------------------------------------------------------
 * NOTE 5: Slave Receive role handling (including Global Call Address)
 *
 * Calling i2c_open with parameter DEV_MODE_I2C_MULTI_MASTER will access DEV
 *  I2C for both Master and Slave roles.
 *
 * In DEV_MODE_I2C_MULTI_MASTER mode DEV I2C shall respond automatically to 
 *  requests for Device ID, and shall support General Call Addressing, provided 
 *  the application has enabled GCA (by calling i2c_ioctl). 
 *
 * Slave Receive role is the activity of receiving data from a remote Master. 
 *  DEV I2C shall manage Slave Receive (including GCA) automatically by using 
 *  (configurable sized) private buffers. The application shall subsequently 
 *  retrieve Slave Receive data by calling i2c_reads, and needs to do so in a 
 *  timeframe that avoids private buffer overrun. The rate of Slave Receive 
 *  data arrival is application-specific.
 *
 *  DEV I2C supports the Global Call Address, which is not enabled by default.
 *  To enable GCA the application shall call i2c_ioctl with command paramater
 *  DEV_IOCTL_I2C_ENABLE_GENERAL_CALL_ADDRESS. The format of the General Call 
 *  Structure is defined in NXP UM10204, section 3.1.13. 
 *
 * I2C messages do not capture the originating remote master transmitter 
 *  address. So, while each message is loaded into and self-contained within an 
 *  individual buffer, the sum of messages from all remote master transmitters 
 *  will share the available private buffer array. The number of buffers to 
 *  allocate in the array is given by devConfig configDRV_I2C_BUFFER_NUM; and 
 *  the size of each individual buffer by configDRV_I2C_BUFFER_SZ. 
 *
 *-----------------------------------------------------------------------------
 * NOTE 6: Slave Transmit role handling
 * 
 * Calling i2c_open with parameter DEV_MODE_I2C_MULTI_MASTER will access DEV
 *  I2C for both Master and Slave roles.
 *
 * Slave Transmit role is the i2c activity of sending data to a remote Master
 *  Receiver on-demand. If i2c_open is not given an application callback 
 *  parameter, DEV I2C shall respond with an automatic 'who am i' consisting of 
 *  device address (set previously through i2c_ioctl) and type ('Agon'). Or, if
 *  i2c_open is given a callback parameter, here named i2cHandler, DEV I2C will 
 *  invoke it instead. 
 *
 *  i2cHandler shall run in the context of i2cisr (see Note 7 interrupt handling
 *  below). This shall execute while the I2C-bus is idled (by clock stretching), 
 *  so i2cHandler SHALL be prompt (meaning data should be readied in advance, 
 *  for various cases), in order to minimise the time it holds-up the bus. 
 *  i2cHandler SHALL respond to the remote Master by calling i2c_writes. 
 *
 *  Data sent via i2c_writes is application-specific. 
 *
 *  Slave Transmit identifies neither the remote master address nor its type. 
 *  Any handshake or protocol needs to be established by application-specific 
 *  design. 
 *
 * DEV I2C shall not provide buffers for Slave Transmit, as the task-provided 
 *  parameters to i2c_writes are sufficient, and avoid the need for memcpy. 
 *
 *-----------------------------------------------------------------------------
 * NOTE 7: Interrupt handling
 *
 * Design of the ISR Handler Task would be based on that initially foreseen for 
 *  DEV UART. This would allow interrupt handling to be passed to a highest 
 *  priority task, and would enable concurrent interrupts from other devices. 
 *  BUT as with the UART, the eZ80 I2C interrupt hardware also needs the event 
 *  cleared down before exiting the ISR, otherwise an infinite trigger results. 
 *  Such that the entire handling must be run within the ISR context. Sigh.
 *
 * #if( 1 == configUSE_PREEMPTION )
 *   static TaskHandle_t I2CIsrHandlerTaskHandle = NULL;
 *   static void I2CIsrHandlerTask( void * params );
 * #endif
 *
 * #if( 1 == configUSE_PREEMPTION )
 * static POSIX_ERRNO createI2CIsrHandlerTask( void )
 * #endif
 * 
 * #if( 1 == configUSE_PREEMPTION )
 * /* I2CIsrHandlerTask
 *      High-priority I2C interrupt handler task.
 *      Will run immediately on exit of i2cisr_0;
 *        iff the current task is not blocking inside MOS * /
 * static void I2CIsrHandlerTask( void * params )
 * #endif
 *
 *
 * i2cisr_0 - address written into the interrupt vector table
 *    Standard ISR reti epilogue * /
 * static void i2cisr_0( void )
 * {
 *     /* The current task context SHALL be saved first on entry to an ISR * /
 *     portSAVE_CONTEXT( );
 * 
 *     __higherPriorityTaskWokenI2C = pdFALSE;
 * 
 * #   if( 1 == configUSE_PREEMPTION )
 *     {
 *         /* run in context of high-priority interrupt handler UartIsrHandlerTask.
 *              Other hardware interrupts may be raised concurrently.
 *              If the i2c isr handling isn't served quickly enough, you can
 *               - either increase the i2c handler task priority, 
 *               - or call i2cisr_1 directly from i2cisr_0 * /
 *         vTaskNotifyGiveFromISR( I2CIsrHandlerTaskHandle, 
 *                                 &__higherPriorityTaskWokenI2C );
 *     }
 * #   else
 *     {
 *         /* run in context of this ISR.
 *              Other hardware interrupts are blocked out for the duration. * /
 *         i2cisr_1( );
 *     }
 * #   endif
 *  
 *     /* If __higherPriorityTaskWokenI2C is now set to pdTRUE then a 
 *        context switch should be performed to ensure the 
 *        interrupt returns directly to the highest priority task * /
 *     if( pdTRUE == __higherPriorityTaskWokenI2C )
 *     {
 *         asm( "\txref _vPortYieldFromISR_2   ; reti from vPortYieldFromISR" );
 *         asm( "\tJP _vPortYieldFromISR_2     ;   with saved context" );
 *     }
 *     else
 *     {
 *         asm( "\t                            ; reti from here" );
 * 
 *         /* RESTORE CONTEXT SHALL be the last action prior to pop ix; reti.L * /
 *         portRESTORE_CONTEXT( );
 * 
 *         asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
 *         asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
 *         asm( "               ;   __default_mi_handler" );
 *         asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );  // *2
 *         asm( "\t reti.L      ; Long ret (ADL_CALL_MODE_LIL + 3 byte PC)" ); // *3
 *         asm( "               ; as per UM007715 table 25 for IM 2 ADL=1 MADL=1" );
 *         asm( "               ; compiler-inserted epilogue below will not be executed" );
 *     }
 * }
 *
 *-----------------------------------------------------------------------------
 * NOTE 8: I2C Locking Bug
 *
 * Refer to Zilog UP0049 "Errata for eZ80F92" Issue #4
 *  A pulse on the SCL line prior to a START condition or after a STOP 
 *   condition causes the I2C bus to lock. This situation occurs regardless of
 *   the I2C control register ENAB settings (I2C_CTL). If this situation occurs,
 *   an I2C software reset does not unlock the I2C bus.
 *  Refer to PS015317 Resetting the I2C Registers, p:153, for Software Reset 
 *   function.
 *
 * Workaround:
 *  To prevent a lock from occurring: it is possible to completely disable the 
 *   I2C block prior to any bus activity using the Clock Peripheral Power-Down 
 *   Register 1 (CLK_PPD1). Disable the I2C block before setting ENAB in the 
 *   I2C Control register.
*/


/*----- Macro Definitions ---------------------------------------------------*/
#define _putchfc( c )\
{\
    int temp;\
    temp =( c & 0xF0 )>> 4;\
    temp =( 9 < temp )?( temp - 10 )+ 'A': temp + '0';\
    _putchf( temp );\
    temp =( c & 0x0F );\
    temp =( 9 < temp )?( temp - 10 )+ 'A': temp + '0';\
    _putchf( temp );\
}


/*---- Constants ------------------------------------------------------------*/
/* Sequences of transactions are supported primarily for register-addressable 
   devices with Master Receive, in which the register address is first 
   transmitted followed by a receive to recover the value(s). These two 
   transactions are connected with a Restart between them such that the pair 
   are not interrupted by a remote Master pending Start. At present this is 
   neither generalised nor do we have an API to make sequences available to 
   the programmer, so it's not in devConfig.h */
#define configDRV_I2C_TRANSACT_MAX        2


/* The permissable frequencies divided down from Agon's 18,432Mhz clock
*/
static unsigned short int const scl[ NUM_I2C_SCL_FREQ ]=
{           // Constants for CCR divisor computed at compile time
    0x0005, // I2C_SCL_FREQ_57600 (Default, standard mode)
    0x0004, // I2C_SCL_FREQ_115200 (Fast mode)
    0x0003  // I2C_SCL_FREQ_230400 (Fast mode)
};


static void i2cisr_idle( const unsigned char );
static void i2cisr_master_transceive( const unsigned char );
static void i2cisr_slave_transmit( const unsigned char );
static void i2cisr_slave_receive( const unsigned char );

/* State Transitions for the various roles */
static void( * const i2cisr[ ])( const unsigned char )=
{
    i2cisr_idle,
    i2cisr_master_transceive, // master transmit
    i2cisr_master_transceive, // master receive
    i2cisr_slave_transmit,    // slave transmit
    i2cisr_slave_receive,     // individual slave addressed
    i2cisr_slave_receive      // general call slaves addressed (broadcast)
};


/*----- Enum Type Definitions -----------------------------------------------*/
/* DEV I2C state
   I2C_STATE_INITIAL and I2C_STATE_READY are mutually exclusive.
   I2C_STATE_READY, I2C_STATE_RUN, I2C_STATE_LOCK are mutual bitmasks..
   We split the semantics of I2C_STATE_RUN (which controls data collection) 
   into I2C_STATE_LOCK (which guarantees at most one reader task or thread). */
typedef enum _i2c_state
{                        // I2C connection and transaction states
    I2C_STATE_INITIAL   =( 0<<0 ),  // before i2c_dev_open and after i2c_dev_close
    I2C_STATE_READY     =( 1<<0 ),  // after i2c_dev_open and before i2c_dev_close
    I2C_STATE_RUN       =( 1<<1 ),  // i2cisr i2c-bus transceive in progress
    I2C_STATE_LOCK      =( 1<<2 ),  // application task in-progress

} I2C_STATE;


/* I2C defines four operating modes (refer to Zilog PS0153 Operating Modes;
    called roles here to avoid confusion with mode parameters in DEV I2C), 
    which are entered on i2c-bus activity:
      Master Transmit entered when the application calls i2c_writem;
      Master Receive entered when the application calls i2c_readm;
      Slave Transmit entered with receipt of START + slave address + read bit;
      Slave Receive entered with receipt of START + slave address + write bit;
        and a fifth General Call Address, broadcast version of Slave Receive */
typedef enum _i2c_role
{
    I2C_ROLE_IDLE = 0x0,  // Idle bus
    I2C_ROLE_MTX  = 0x1,  // Master Transmit
    I2C_ROLE_MRX  = 0x2,  // Master Receive
    I2C_ROLE_STX  = 0x3,  // Slave Transmit
    I2C_ROLE_SRX  = 0x4,  // Slave Receive
    I2C_ROLE_GCA  = 0x5   // General Call Address, version of Slave Receive
 
} I2C_ROLE;


/* I2C defines 7-bit addresses with the LSB of an 8-bit char indicating the 
   direction of data transfer required (R/W_). */
typedef enum _i2c_bus_data_direction
{
    I2C_BUS_DATA_DIRECTION_TRANSMIT = 0,
    I2C_BUS_DATA_DIRECTION_RECEIVE = 1

} I2C_BUS_DATA_DIRECTION;


/* I2C Master Transceive mini-states
   Refer to Zilog PS0153 Operating Modes, Master Tranmsit, tables 78 & 79 & 80
    and Master Receive, tables 81, 82 & 83
   Master Transmit consists of address write steps, followed by a data write 
    step. Address write will be two steps if 10-bit addressing is used
   Master Receive consists of address write steps, followed by a data read 
    step. Address write will be three steps if 10-bit addressing is used. */
typedef enum _i2c_mtrans_state
{
    I2C_MTRANS_STATE_IDLE   = 0,
    I2C_MTRANS_STATE_ADDR_1 = 1,  // 7-bit address or first part 10-bit address
    I2C_MTRANS_STATE_ADDR_2 = 2,  // second part 10-bit address
    I2C_MTRANS_STATE_ADDR_3 = 3,  // first part 10-bit address Receive restart
    I2C_MTRANS_STATE_DATA   = 4   // Read or Write data

} I2C_MTRANS_STATE;


/* Refer to Zilog PS015317 I2C Registers, Control Register */
typedef enum _i2c_ctl_reg
{
    I2C_CTL_REG_IEN_OFF  =( 0 << 7 ),
    I2C_CTL_REG_IEN_ON   =( 1 << 7 ),
    I2C_CTL_REG_ENAB_OFF =( 0 << 6 ),
    I2C_CTL_REG_ENAB_ON  =( 1 << 6 ),
    I2C_CTL_REG_STA_OFF  =( 0 << 5 ),
    I2C_CTL_REG_STA_ON   =( 1 << 5 ),
    I2C_CTL_REG_STP_OFF  =( 0 << 4 ),
    I2C_CTL_REG_STP_ON   =( 1 << 4 ),
    I2C_CTL_REG_IFLG_OFF =( 0 << 3 ),
    I2C_CTL_REG_IFLG_ON  =( 1 << 3 ),
    I2C_CTL_REG_AAK_OFF  =( 0 << 2 ),
    I2C_CTL_REG_AAK_ON   =( 1 << 2 ),

    I2C_CTL_RESERVED     =( 3 << 0 )

} I2C_CTL_REG;


/* Status codes, Refer to Zilog PS0153 table 89
   eZ80 I2C-bus Status Register, State transition codes
   Used to drive i2cisr */
typedef enum _i2c_status_code
{
    I2C_STATUS_CODE_BUS_ERROR                        = 0x00,
    I2C_STATUS_CODE_START_TRANSMITTED                = 0x08,
    I2C_STATUS_CODE_REPEATED_START                   = 0x10,
    I2C_STATUS_CODE_ADDR_WRITE_ACK                   = 0x18,
    I2C_STATUS_CODE_ADDR_WRITE_NACK                  = 0x20,
    I2C_STATUS_CODE_MASTER_DATA_WRITE_ACK            = 0x28,
    I2C_STATUS_CODE_MASTER_DATA_WRITE_NACK           = 0x30,
    I2C_STATUS_CODE_ARB_ADDR_DATA                    = 0x38,
    I2C_STATUS_CODE_ADDR_READ_ACK                    = 0x40,
    I2C_STATUS_CODE_ADDR_READ_NACK                   = 0x48,
    I2C_STATUS_CODE_DATA_READ_ACK                    = 0x50,
    I2C_STATUS_CODE_DATA_READ_NACK                   = 0x58,
    I2C_STATUS_CODE_SLAVE_ADDR_WRITE_ACK             = 0x60,
    I2C_STATUS_CODE_MASTER_ARB_SLAVE_ADDR_WRITE_ACK  = 0x68,
    I2C_STATUS_CODE_GCA_ACK                          = 0x70,
    I2C_STATUS_CODE_MASTER_ARB_GENERAL_CALL_ADDR_ACK = 0x78,
    I2C_STATUS_CODE_SLAVE_DATA_READ_ACK              = 0x80,
    I2C_STATUS_CODE_SLAVE_DATA_READ_NACK             = 0x88,
    I2C_STATUS_CODE_GCA_DATA_READ_ACK                = 0x90,
    I2C_STATUS_CODE_GCA_DATA_READ_NACK               = 0x98,
    I2C_STATUS_CODE_SLAVE_STOP                       = 0xA0,
    I2C_STATUS_CODE_SLAVE_ADDR_READ_ACK              = 0xA8,
    I2C_STATUS_CODE_MASTER_ARB_SLAVE_ADDR_READ_ACK   = 0xB0,
    I2C_STATUS_CODE_SLAVE_DATA_WRITE_ACK             = 0xB8,
    I2C_STATUS_CODE_SLAVE_DATA_WRITE_NACK            = 0xC0,
    I2C_STATUS_CODE_SLAVE_LAST_DATA_WRITE_ACK        = 0xC8,
    I2C_STATUS_CODE_SECOND_ADDR_WRITE_ACK            = 0xD0,
    I2C_STATUS_CODE_SECOND_ADDR_WRITE_NACK           = 0xD8,
    I2C_STATUS_CODE_NULL                             = 0xF8

} I2C_STATUS_CODE;


/*----- Type Definitions ----------------------------------------------------*/
/* Input from other I2C bus masters is buffered, so as to make data reception 
   by i2cisr asynchronous with respect to its subsequent collection by the user 
   application through i2c_reads.
   I2C Buffer is only used for Slave Receive operations, since this is driven
   asynchronously by a remote master transmitter. */
typedef struct _i2c_buffer
{
    unsigned char Index;                              // next input position
    unsigned char Outdex;                             // next output position
    unsigned char  _data[ configDRV_I2C_BUFFER_SZ ];  // storage

} I2C_BUFFER;


/* A transaction is constructed when an application task calls i2c_readm or
   i2c_writem. The transaction is executed in context of i2cisr; during which 
   time the calling task is blocked, and woken up on completion. A transaction 
   is also constructed when the application Slave Transmit callback invokes 
   i2c_writes; and is similarly blocked until completion. */
typedef struct _i2c_transaction
{
    I2C_ADDR *targetAddr;     // Target Slave Address                      IN
    unsigned char *buf;       // Dest or Src buffer                    IN/OUT
    size_t len;               // number of bytes to read/write             IN
    size_t lenTRx;            // local tracker of bytes transceived      TEMP
    size_t *bufTRx;           // destination buffer for lenTRx            OUT
    POSIX_ERRNO *bufRes;      // destination buffer for result            OUT
    
} I2C_TRANSACTION;


/*---- Global Variables -----------------------------------------------------*/
/* Changing any global variable from a task needs to be done within
    portENTER_CRITICAL( );
    {
    }
    portEXIT_CRITICAL( );
*/
static I2C_STATE i2cState = I2C_STATE_INITIAL;

static SemaphoreHandle_t i2cRunSemaphore = NULL;
  // configDRV_I2C_TRANSACT_MAX + 1 for increment and test of i2cTransact
static I2C_TRANSACTION transactBuf[ configDRV_I2C_TRANSACT_MAX + 1 ]=
                           {{ NULL, NULL, 0, 0, NULL, NULL }};
static I2C_TRANSACTION *i2cTransact = &transactBuf[ 0 ];
static unsigned short i2cEvent = I2C_EVENT_NONE;   // buffer for transaction results

  /* Own slave address, set by application through i2c_ioctl() */
static I2C_ADDR slaveAddr ={ 0, 0 };

  /* Slave Receive role buffers */
static I2C_BUFFER slaveReceiveBuf[ configDRV_I2C_BUFFER_NUM ]={{ 0, 0,{ 0 }}};
static I2C_BUFFER *currentSlaveReceiveBuf = &slaveReceiveBuf[ 0 ];
static unsigned char slaveReceiveBufIn = 0;   // next to populate by i2cisr
static unsigned char slaveReceiveBufOut = 0;  // next to collect by i2c_reads()

static I2C_HANDLER i2cHandler = NULL;  // application callback for SLAVE TRANSMIT

  /* ISR variables */
static I2C_ROLE i2cRole = I2C_ROLE_IDLE;
static I2C_MTRANS_STATE masterTransitionState = I2C_MTRANS_STATE_IDLE;
  /* Initialise masterTransitionRetryCount prior to each I2C bus transaction, and
     count down to spend it */
static unsigned short int masterTransitionRetryCount = 
    configDRV_I2C_TRANSITION_RETRY;
  /* state transition for i2c bus on exit from ISR */
static unsigned char ctl = I2C_CTL_REG_IEN_ON | I2C_CTL_REG_ENAB_ON;

/*
#if( 1 == configUSE_PREEMPTION )
    static TaskHandle_t I2CIsrHandlerTaskHandle = NULL;
    static void I2CIsrHandlerTask( void * params );
    static _Bool I2CIsrHandlerTaskHandleCreated = false;
#endif
*/

/* __higherPriorityTaskWokenI2C is tested to decide if a context switch is 
   necessary on exit from i2cisr. */
static BaseType_t __higherPriorityTaskWokenI2C;

static void( *prevI2cISR )( void )= NULL;    // MOS ISR for re-attach on i2c_close


/*----- Initialistion functions ---------------------------------------------*/
static POSIX_ERRNO initialiseI2cSemaphore( SemaphoreHandle_t * const s )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    if( NULL == *s )
    {
        // one-time creation
        *s = xSemaphoreCreateBinary( );
    }

    if( NULL == *s )
    {
        // critical error, failed to create uart semaphore
#       if defined( _DEBUG )
        {
            ( void )printf( "%s : %d : Failed to allocate I2C semaphore\r\n", 
                            "devi2c.c", __LINE__ );
        }
#       endif
        ret = POSIX_ERRNO_ENOLOCKS;
    }
    else
    {
        /* We may call i2c_dev_open successive times; ensure the sempahore is 
           in the blocking state, after any previous use. 
           For a binary semaphore uxSemaphoreGetCount returns 1 if it is 
           available, and 0 if it is not available. */
        UBaseType_t cnt = uxSemaphoreGetCount( *s );
        if( cnt )
        {
            xSemaphoreTake( *s, 0 );
        }
    }

    return( ret );
}


/*
#if( 1 == configUSE_PREEMPTION )
/* createI2CIsrHandlerTask
   I2CIsrHandlerTaskHandle is a pointer to the highest priority task. It is run
   as an "interrupt handler" in a task context, so as not to block other eZ80 
   interrupts during its execution.
* /
static POSIX_ERRNO createI2CIsrHandlerTask( void )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    BaseType_t r;

    if( false == I2CIsrHandlerTaskHandleCreated )
    {
        if( NULL == I2CIsrHandlerTaskHandle )
        {
            r = xTaskCreate( 
                    I2CIsrHandlerTask, 
                    "I2C ISR Handler", 
                    configMINIMAL_STACK_SIZE, 
                    NULL,                      // no params
                    configMAX_PRIORITIES,      // highest possible priority
                    &I2CIsrHandlerTaskHandle );
            if( pdPASS != r )
            {
#               if defined( _DEBUG )
                {
                    ( void )printf( "Failed to allocate I2CIsrHandlerTask: %d\r\n", r );
                }
#               endif

                ret = POSIX_ERRNO_ENSRCH;
            }
            else
            {
                I2CIsrHandlerTaskHandleCreated = true;

#               if defined( _DEBUG )
                {
                    ( void )printf( "Created I2CIsrHandlerTask\r\n" );
                }
#               endif
            }
        }
    }

    return( ret );
}
#endif
*/


/*----- I2C device functions ------------------------------------------------*/
#define i2cReset( )\
    I2C_SRR = 1

#define i2cInterruptEnable( )\
    ( I2C_CTL |= I2C_CTL_REG_IEN_ON )

#define i2cInterruptDisable( )\
    ( I2C_CTL &= I2C_CTL_REG_IEN_OFF )

#define i2cEnable( )\
    ( I2C_CTL |= I2C_CTL_REG_ENAB_ON )

#define i2cDisable( )\
    ( I2C_CTL &= I2C_CTL_REG_ENAB_OFF )

#define i2cEnableTest( )\
    ( I2C_CTL | I2C_CTL_REG_ENAB_ON )

#define i2cStart( )\
    ( ctl |= I2C_CTL_REG_STA_ON )

#define i2cStartSent( )\
    ( I2C_CTL_REG_STA_OFF ==( I2C_CTL & I2C_CTL_REG_STA_OFF ))

#define i2cStartTest( )\
    ( I2C_CTL & I2C_CTL_REG_STA_ON )

#define i2cStop( )\
    ( ctl |= I2C_CTL_REG_STP_ON )

#define i2cStopSent( )\
    ( I2C_CTL_REG_STP_OFF ==( I2C_CTL & I2C_CTL_REG_STP_OFF ))

#define i2cStopTest( )\
    ( I2C_CTL & I2C_CTL_REG_STP_ON )

#define i2cIflagTest( )\
    ( I2C_CTL & I2C_CTL_REG_IFLG_ON )

#define i2cIflagClr( )\
    ( I2C_CTL &=( 0xFF - I2C_CTL_REG_IFLG_ON ))

#define i2cAck( )\
    ( ctl |= I2C_CTL_REG_AAK_ON )

#define i2cNack( )\
    ( ctl &=( 0xFF - I2C_CTL_REG_AAK_OFF ))

#define i2cAckTest( )\
    ( I2C_CTL & I2C_CTL_REG_AAK_ON )

#define i2cNackTest( )\
    ( I2C_CTL_REG_AAK_OFF ==( I2C_CTL & I2C_CTL_REG_AAK_OFF ))


/* i2cSetClockSpeed
   Program the I2C CCR
   Data on the I2C bus can be transferred at a rate of up to 100 kbps in 
   STANDARD mode, or up to 400 kbps in FAST mode. 
   One clock pulse is generated for each data bit transferred. 
   On startup the default rate 57600 is selected.
   Applications can change the clock rate for each device they address, using
   i2c_ioctl with the command DEV_IOCTL_I2C_SET_FREQUENCY. */
static void i2cSetClockSpeed( I2C_SCL_FREQ const freq )
{
    switch( freq )
    {
        default :
        {
#           if defined( _DEBUG )
            {
                ( void )printf( "i2cSetClockSpeed defaulting to 57600\r\n" );
            }
#           endif

            I2C_CCR = scl[ I2C_SCL_FREQ_57600 ];
            i2cEvent |=( 1 << I2C_EVENT_SYS_CLK_SPEED );
        }
        break;

        case I2C_SCL_FREQ_57600 :
        case I2C_SCL_FREQ_115200 :
        case I2C_SCL_FREQ_230400 :
        {
            I2C_CCR = scl[ freq ];
        }
        break;
    }
}


/* i2cInit
   Power-up the I2C
   Refer to Zilog ps015317, resetting the I2C registers p:153
   Refer to Note 8 above.
   Compare to MOS i2c.c:init_I2C() */
static void i2cInit( void )
{
#   if defined( configTRACE )
        _putchf( 'L' );
#   endif

    i2cInterruptDisable( );

    i2cDisable( );

    // power down I2C peripheral clock before enabling I2C (note 8)
    CLK_PPD1 |= PERIPHERAL_CLK_REG_I2C_OFF; 
    {
        /* Refer to Zilog PS015317, Clock Peripheral Power-Down Registers p:36
           When powered down, the peripheral control registers are not 
           accessible for Read or Write access.*/
        // something to insert a delay between CLK_PPD1 OFF and back ON
        i2cSetClockSpeed( I2C_SCL_FREQ_57600 );
    }
    CLK_PPD1 &=( 0xFF - PERIPHERAL_CLK_REG_I2C_OFF );
    // power up the i2c peripheral clock

    /// Still have runs when only an electrical power cycle exits i2c lockup
    /// Lock-up not resolved yet
    /// Could be Light2 pullups borderline out of specification

    // Software reset the I2C device (clr STA, STP, IFLG) and to Idle
    i2cReset( );

    i2cSetClockSpeed( I2C_SCL_FREQ_57600 );  // default standard clock speed

    i2cEnable( );  // enable I2C bus (SCL/SDA) events

    /* Set Interrupt Enable Register */
    i2cInterruptEnable( );
}


/*------ IOCTL functions ----------------------------------------------------*/
#define enableGeneralCallAddress( )\
    ( I2C_SAR |= 0x1 )

#define disableGeneralCallAddress( )\
    ( I2C_SAR &= 0xFE )


/* storeSlaveAddress
   Save our I2C slave address. Used to test when we are addressed.
   DEV I2C supports bus multi-mastering. This enables other devices on the I2C 
   bus to the Master role. Those devices can address us using our application-
   selected Slave Address. 
   The I2C Address is either a 7-bit value or a 10-bit value
   Refer to NXP UM10204 section 3.1.11 and 3.1.12
   Bitmask for the 7-bit address byte is [0]=0xXnnnnnnn 
    such that the bits 6:0 form the required address 
    Assignable 7-bit address values are in the range 8..119
    While values in the ranges 0..7 and 120..127 are reserved:
     Refer to NXP UM10204 section 3.1.12 Table 4.
    For example, to initialise a 7-bit address 0x34 (52) in an application:
     I2C_ADDR mySlave ={ 0x34 };
   Bitmasks for the 10-bit address are bytes [0]=0xX11110nn [1]=0xnnnnnnnn 
    such that the bottom 2 bits of byte [0] are the MSBs and all 8-bits of 
    byte [1] are the LSBs of the 10-bit address. 
    Valid values for byte[0] bits are in the range 120..123; refer to NXP 
     UM10204 section 3.1.12 Table 4.
     The value of the byte[0] is left-shifted when stored in I2C_SAR, so
      I2C_10BIT_ADDR_MASK corresponds to 0xFX (0x78<<1).
      Valid 10-bit address first byte values are in the range 0xF0 (0x78<<1), 
      0xF2 (0x79<<1), 0xF4 (0x7A<<1) or 0xF6 (0x7B<<1).
      I2C_10BIT_ADDR_MASK doesn't distinguish that 0xF8..0xFF are invalid.
    To initialise a 10-bit address (values from 0x0..0x3ff) in an application: 
      I2C_ADDR mySlave ={ I2C_10BIT_ADDR_MASK | 0x0..0x3, 0x0..0xff };
   */
static void storeSlaveAddress( I2C_ADDR const * const addr )
{
    unsigned char const gca =( I2C_SAR & 0x1 );
    _Bool const is10bit =
        ( I2C_10BIT_ADDR_MASK ==( addr->byte[ 0 ]& I2C_10BIT_ADDR_MASK ))
            ? true
            : false;

    //Refer to ...\ZDSII_eZ80Acclaim!_5.3.5\ZTP2.5.1\RZK\eZ80_BSP\I2C\I2C.c
    //This sets I2C_SAR before enabling interrupt I2C_CTL | 0x80
    //Is this necessary for the eZ80 I2C device to recognise its Slave address?
    //Duplicating this sequence had no effect by experiment.
    //_Bool const isIntrEnabled = 
    //    ( i2cEnableTest( ))
    //        ? true
    //        : false;

    slaveAddr = *addr;  // save our I2C slave address in the global variable

    //if( true == isIntrEnabled )  // see Note on Zilog RZK code
    //{
    //    i2cDisable( );
    //}

    if( true == is10bit )
    {  // 10-bit address
        I2C_SAR =(( addr->byte[ 0 ]<< 1 )| gca );
        I2C_XSAR = addr->byte[ 1 ];
    }
    else
    {   // PS015317 p:153 I2C_SAR[7:1]= sla[6:0] 
        // the 7-bit address of the I2C when in 7-bit SLAVE mode
        I2C_SAR =(( addr->byte[ 0 ]<< 1 )| gca );
        I2C_XSAR = 0;
    }
    
    //if( true == isIntrEnabled )  // see Note on Zilog RZK code
    //{
    //    i2cEnable( );
    //}
}


/*------ Transaction functions ----------------------------------------------*/
#define CREATE_I2C_TRANSACTION( _tAddr, _buf, _bufSz, _numTransacted, _bufResult )\
    i2cTransact->targetAddr =( _tAddr ), \
    i2cTransact->buf =( _buf ), \
    i2cTransact->len =( _bufSz ), \
    i2cTransact->bufTRx =( _numTransacted ), \
    i2cTransact->bufRes =( _bufResult ), \
    i2cTransact->lenTRx =0


#define CLEAR_I2C_TRANSACTION( ) \
    i2cTransact->targetAddr = NULL, \
    i2cTransact->buf = NULL, \
    i2cTransact->len = 0, \
    i2cTransact->bufTRx = NULL, \
    i2cTransact->bufRes = NULL, \
    i2cTransact->lenTRx = 0


#if defined( _DEBUG )
  // printf is okay when run in context of a calling task
# define PRINT_I2C_TRANSACTION( )\
    ( void )printf( "i2cTransact: addr = 0x%p; buf=0x%p; " \
                    "len=%d; bufTRx=%p; bufRes=0x%p; lenTRX=%d\r\n", \
                    i2cTransact->targetAddr, \
                    i2cTransact->buf, \
                    i2cTransact->len, \
                    i2cTransact->bufTRx, \
                    i2cTransact->bufRes, \
                    i2cTransact->lenTRx )
#endif


/* resetState
   Notify pending task or deactive transaction state.
   Reset role and transition state. */
static void resetState( )
{
    int i;

    if(( I2C_ROLE_MTX == i2cRole )||
       ( I2C_ROLE_MRX == i2cRole )||
       ( I2C_ROLE_STX == i2cRole ))
    {
        /* Clear I2C_STATE_RUN here because another interrupt may follow 
           immediately, before the user task is able to take i2cRunSemaphore. */
        i2cState &=( I2C_STATE )( ~I2C_STATE_RUN );

#       if( 1 == configUSE_PREEMPTION )
        {
            xSemaphoreGiveFromISR( 
                i2cRunSemaphore, &__higherPriorityTaskWokenI2C );
        }
#       else
        {
            i2cTransact = &transactBuf[ 0 ];  // results pointers in [0]
            finalisei2cTransaction( POSIX_ERRNO_EALREADY | i2cEvent );
            i2cEvent = I2C_EVENT_NONE;
            
            for( i=0; configDRV_I2C_TRANSACT_MAX > i; i++ )
            {
                CLEAR_I2C_TRANSACTION( );
                i2cTransact++;
            }

            i2cTransact = &transactBuf[ 0 ];
            i2cState &=( I2C_STATE )( ~I2C_STATE_LOCK );
        }
#       endif /*( 1 == configUSE_PREEMPTION )*/
    }
    else
    if( I2C_ROLE_SRX == i2cRole )
    {
        i2cState &=( I2C_STATE )( ~( I2C_STATE_LOCK | I2C_STATE_RUN ));
    }

    i2cRole = I2C_ROLE_IDLE;

    masterTransitionRetryCount = configDRV_I2C_TRANSITION_RETRY;
    masterTransitionState = I2C_MTRANS_STATE_IDLE;
}


/* writeTargetAddress
   For Master Transmit role:
    - 7-bit target slave address is sent as one byte (I2C_MTRANS_STATE_ADDR_1) 
    - 10-bit address is sent as two bytes (I2C_MTRANS_STATE_ADDR_1 then 
      I2C_MTRANS_STATE_ADDR_2), +R/W_ bit (0<<0).
   For Master Receive role:
    - 7-bit target slave address is sent as one byte (I2C_MTRANS_STATE_ADDR_1) 
      plus the Read R/W_ bit (1<<0)
    - 10-bit address is first sent as two bytes with the Write R/W_ bit (0<<0) 
      as per Master Transmit, followed by a Restart with the first address byte 
      repeated (I2C_MTRANS_STATE_ADDR_3) plus the Read R/W_ bit (1<<0) set. */
static POSIX_ERRNO writeTargetAddress( I2C_BUS_DATA_DIRECTION const dir )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    unsigned char dr = 0;
    
#   if defined( configTRACE )
        _putchf( 'Z' );
#   endif

    if( i2cTransact->targetAddr )
    {
        switch( masterTransitionState )
        {
            case I2C_MTRANS_STATE_ADDR_1 :  // 7-bit or 10-bit first byte address
            case I2C_MTRANS_STATE_ADDR_3 :  // 10-bit first byte for Receive restart
            {
#   if defined( configTRACE )
        _putchf( '1' );
#   endif
                /* a 7-bit address range  6:3=0b0001..0b1110 (8 through 112)
                   a 10-bit address range 6:2=0b11110 (120, 0x78) */
                dr = i2cTransact->targetAddr->byte[ 0 ];
                
                /* lsb of an I2C address is R/W_, so shift address bits left */
                dr <<= 1;  

                // set the lsb R/W_                
                if( I2C_BUS_DATA_DIRECTION_RECEIVE == dir )
                {
                    dr |= 0x1;  // R/W_
                }
            }
            break;
        
            case I2C_MTRANS_STATE_ADDR_2 :  // 10-bit second byte address
            {
#   if defined( configTRACE )
        _putchf( '2' );
#   endif
                dr = i2cTransact->targetAddr->byte[ 1 ];

                // all 8 bits are address, no R/W_ dir bit
            }
            break;
            
            default :
            {
#   if defined( configTRACE )
        _putchf( '3' );
#   endif
                i2cStop( );
                i2cEvent |=( 1 << I2C_EVENT_DRV_ERROR );
                ret = POSIX_ERRNO_EFAULT;
            }
            break;
        }

        // write the address byte to I2C_DR
        I2C_DR = dr;
    }
    else
    {
#   if defined( configTRACE )
        _putchf( '4' );
#   endif
        i2cStop( );
        ret = POSIX_ERRNO_EFAULT;
    }

    return( ret );
}


/* Refer to Zilog PS0153, Operating Modes, Master Transmit
   In MASTER TRANSMIT mode, the I2C transmits a number of bytes to a slave 
   receiver. 
   Enter MASTER TRANSMIT mode by setting the STA bit in the I2C_CTL register
   to 1. The I2C then tests the I2C bus and transmits a START condition when 
   the bus is free.*/
#define startMasterTransmit( )  i2cRole = I2C_ROLE_MTX;\
                                I2C_CTL |= I2C_CTL_REG_STA_ON

/* Refer to Zilog PS0153, Operating Modes, Master Receive
   In MASTER RECEIVE mode, the I2C receives a number of bytes from a slave 
   transmitter. 
   Enter MASTER RECEIVE mode by setting the STA bit in the I2C_CTL register
   to 1. The I2C then tests the I2C bus and transmits a START condition when 
   the bus is free.*/
#define startMasterReceive( )   i2cRole = I2C_ROLE_MRX;\
                                I2C_CTL |= I2C_CTL_REG_STA_ON


static void masterTransmitNextByte( void )
{
    /* Refer to Zilog PS0153, Operating Modes, Master Transmit
       The [next] data byte to be transmitted is loaded into the I2C_DR 
       register (and the IFLG bit cleared - done in i2cisr_1). */
    configASSERT( I2C_MTRANS_STATE_DATA == masterTransitionState );
#   if defined( configTRACE )
        _putchf( 'X' );
#   endif

    if( i2cTransact->len > i2cTransact->lenTRx )
    {
#       if defined( configTRACE )
            _putchf( '1' );
#       endif

        I2C_DR = i2cTransact->buf[ i2cTransact->lenTRx ];
        ++( i2cTransact->lenTRx );
    }
    else
    {
        if(( i2cTransact + 1 )->len )
        {
            /* perform next consecutive transaction after a RESTART
               Currently hard-coded for Master Receive with register-addressed 
               devices */
#           if defined( configTRACE )
                _putchf( '2' );
#           endif

            i2cTransact++;
            // kick off the master receive transaction
            startMasterReceive( );  // will set i2cStart() to cause a Restart
        }
        else
        {
            /* Refer to Zilog PS0153, Operating Modes, Master Transmit, after 
               table 80. Also refer to NXP UM10204 Figure 9. When all bytes are 
               transmitted, the microcontroller should write 1 to the STP bit 
               in the I2C_CTL register. The I2C then transmits a STOP condition, 
               clears the STP bit and returns to the idle state. */

#           if defined( configTRACE )
                _putchf( '3' );
#           endif
            i2cStop( );

            i2cEvent |=( 1 << I2C_EVENT_TX_DONE );
            resetState( );
        }
    }
}


static void masterReceiveNextByte( void )
{
    /* Refer to Zilog PS0153, Operating Modes, Master Receive
       When all bytes are received, a NACK should be sent from the remote 
       master transmitter, then the microcontroller should write a 1 to the 
       STP bit in the I2C_CTL register. The I2C then transmits a STOP 
       condition, clears the STP bit and returns to the idle state. */
    configASSERT( I2C_MTRANS_STATE_DATA == masterTransitionState );
#   if defined( configTRACE )
        _putchf( 'Y' );
#   endif

    *( i2cTransact->buf )++ = I2C_DR;
    ++( i2cTransact->lenTRx );    
}


/* startSlaveTransmit
   Runs in response to a remote master SLAVE TRANSMIT demand.
   If the application passed the callback to i2c_open, then we invoke that,
    and that SHALL call i2c_writes to send a response. 
   Else we call i2c_writes with a default DEV I2C 'who am i' message. */
static void startSlaveTransmit( void )
{
    if( NULL != i2cHandler )
    {
        i2cHandler( );  // SHALL call i2c_writes() to respond
    }
    else
    {
        /* Customised use of GCA
           DEV I2C 'who am i' string format;
           Id made static to persist after startSlaveTransmit returns. */
        static unsigned char Id[ ]=  
            // default string: '@' 2-byte address, '='type '\0'
            { '@', 0, 0,
              '=', 'A', 'g', 'o', 'n', '\0' };
        Id[ 1 ]= slaveAddr.byte[ 0 ];
        Id[ 2 ]= slaveAddr.byte[ 1 ];
        i2c_dev_writes( Id, 9, NULL, NULL );  // respond with a default string
    }
}


static void slaveTransmitNextByte( void )
{
    /* Refer to Zilog PS0153, Operating Modes, Slave Transmit
       The [next] data byte to be transmitted is loaded into the I2C_DR 
       register (and the IFLG bit cleared - done in i2cisr_1) 
       When the final byte to be transmitted is loaded into the I2C_DR 
       register, the AAK bit shall be cleared before the IFLG is cleared. 
    */
    I2C_DR = i2cTransact->buf[ i2cTransact->lenTRx ];

    i2cTransact->lenTRx ++;
    if( i2cTransact->len > i2cTransact->lenTRx )
    {
        i2cAck( );
    }
    else
    {
        i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_DONE );

        /* When the final byte to be transmitted is loaded into the 
           I2C_DR register, the AAK bit is cleared when the IFLG is 
           cleared. */
        i2cNack( );
    }
}


/* Normally invoked from I2C_STATUS_CODE_SLAVE_DATA_READ_ACK
   Refer to Zilog PS0153, Operating Modes, Slave Receive */
static void slaveReceiveNextByte( void )
{
    currentSlaveReceiveBuf->_data[ currentSlaveReceiveBuf->Index ]= I2C_DR;

    currentSlaveReceiveBuf->Index ++;
    if( configDRV_I2C_BUFFER_SZ > currentSlaveReceiveBuf->Index )
    {
        i2cAck( );
    }
    else
    {
        i2cEvent |=( 1 << I2C_EVENT_RX_OVERFLOW );
        i2cNack( );
    }
}


static void slaveReceiveDone( void )
{
    i2cState &=( I2C_STATE )( ~I2C_STATE_RUN );

    if( configDRV_I2C_BUFFER_NUM <= ++slaveReceiveBufIn )
    {
        slaveReceiveBufIn = 0;
    }
    
    currentSlaveReceiveBuf = &slaveReceiveBuf[ slaveReceiveBufIn ];
    if( currentSlaveReceiveBuf->Index )
    {
        // received message(s) have not yet been read by the application
        i2cEvent |=( 1 << I2C_EVENT_RX_FULL );
    }
}


static void slaveReceiveClear( void )
{
    currentSlaveReceiveBuf->Index = 0;  // overwrite new current buffer
    currentSlaveReceiveBuf->Outdex = 0;
}


/* finalisei2cTransaction
     On completion or abandonment of an i2c bus transaction, set the calling 
     task callback hooks.
*/
static void finalisei2cTransaction( POSIX_ERRNO const ret )
{
    if( i2cTransact->bufTRx )
    {
        *( i2cTransact->bufTRx )= i2cTransact->lenTRx;
    }
    
    if( i2cTransact->bufRes )
    {
        *( i2cTransact->bufRes )= ret;
    }
}


/*====== I2C interrupt handler ==============================================*/
/* i2cisr_idle
   Interrupt Handler - Idle role
   Idle role is the default I2C condition, while we have neither Mastered the 
     bus nor been addressed by a remote Master. 
     Contention may result between concurrent remote and local Mastering bids.
     We might have begun Mastering and set I2C_STATE_LOCK, then be interrupted 
     by a remote Master start prior to starting our bus transaction. So we must 
     test and set I2C_STATE_LOCK before transitioning to a Slave role.
*/
static void i2cisr_idle( unsigned char const sr )
{
#   if defined( configTRACE )
        _putchf( 'I' );
#   endif

    switch( sr )
    {
        case I2C_STATUS_CODE_BUS_ERROR :  // 0x0
        {
            /* refer to Zilog PS0153, following table 89:
               If an illegal condition occurs on the I2C bus, the bus error 
               state is entered (status code 00h). To recover from this state, 
               the STP bit in the I2C_CTL register must be set and the IFLG bit 
               cleared. 
               A bus error signifies, if you were doing a Master transaction, 
               that DEV I2C signalled or sequenced incorrectly; or if you were
               doing a Slave transaction, then DEV I2C may have got something 
               wrong, or a remote Master got something wrong.  */
#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            // attempt to recover from bus error
            i2cInit( );

            // terminate the current transaction
            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );            
        }
        break;

        case I2C_STATUS_CODE_START_TRANSMITTED : // 0x08
        case I2C_STATUS_CODE_REPEATED_START :    // 0x10
        {
            // a (Repeated) Start may not be followed by our address
            /* we expect I2C_STATUS_CODE_SLAVE_ADDR_READ_ACK, 
                         I2C_STATUS_CODE_SLAVE_ADDR_WRITE_ACK
                         I2C_STATUS_CODE_GCA_ACK */
#           if defined( configTRACE )
                _putchf( '2' );
#           endif
        }
        break;

        case I2C_STATUS_CODE_ARB_ADDR_DATA: // 0x38
        {
            /* When bus arbitration is lost, the IFLG bit is 1 and the status 
               code in the I2C_SR register is 38h. This will happen when another
               master gains arbitration over our in-progress transaction. */
#           if defined( configTRACE )
                _putchf( '3' );
#           endif
        }
        break;

        case I2C_STATUS_CODE_SLAVE_ADDR_WRITE_ACK: // 0x60 (START for Slave Receive)
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Receive:
               The I2C enters SLAVE RECEIVE mode when it receives its own slave 
               address and a Write bit (lsb = 0) after a START condition. The 
               I2C transmits an acknowledge bit and sets the IFLG bit in the 
               I2C_CTL register and the I2C_SR register contains the status 
               code 60h. */
#           if defined( configTRACE )
                _putchf( '4' );
#           endif

            if( I2C_STATE_LOCK & i2cState )
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif

                i2cNack( );
            }
            else
            {
                slaveReceiveClear( );
            
                i2cRole = I2C_ROLE_SRX;
                i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
                i2cAck( );
            }
        }
        break;
        
        case I2C_STATUS_CODE_GCA_ACK: // 0x70 (START for GCA Receive)
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Receive:
               The I2C also enters SLAVE RECEIVE mode when it receives the 
               general call address 00h, if the GCE bit in the I2C_SAR 
               register is set. The status code is then 70h. 
               To enable GCA the application must call i2c_ioctl with command
               DEV_IOCTL_I2C_ENABLE_GENERAL_CALL_ADDRESS; it is not enabled by
               default. */
#           if defined( configTRACE )
                _putchf( '5' );
#           endif

            if( I2C_STATE_LOCK & i2cState )
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif

                i2cNack( );
            }
            else
            {
                slaveReceiveClear( );
            
                i2cRole = I2C_ROLE_GCA;
                i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
                i2cAck( );
            }
        }
        break;
        
        case I2C_STATUS_CODE_SLAVE_ADDR_READ_ACK: // 0xA8 (START for Slave Transmit)
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Transmit:
               The I2C enters SLAVE TRANSMIT mode when it receives its own 
               slave address and a Read bit (lsb = 1) after a START condition. 
               The I2C then transmits an acknowledge bit (if the AAK bit is set
               to 1) and sets the IFLG bit in the I2C_CTL register and the 
               I2C_SR register contains the status code A8h. */
#           if defined( configTRACE )
                _putchf( '6' );
#           endif

            if( I2C_STATE_LOCK & i2cState )
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif

                i2cNack( );
            }
            else
            {
#               if defined( configTRACE )
                    _putchf( '+' );
#               endif

                i2cRole = I2C_ROLE_STX;
                i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
                startSlaveTransmit( );
                i2cAck( );
            }
        }
        break;

        case I2C_STATUS_CODE_NULL: // 0xF8
        {
#           if defined( configTRACE )
                _putchf( '7' );
#           endif
        }
        break;

        default :
        {
#           if defined( configTRACE )
                _putchf( '8' );
                _putchfc( sr );
#           endif

            i2cInit( );

            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );
        }
        break;
    }
}


/* i2cisr_slave_transmit
   Interrupt Handler - Slave Transmit role
   Refer to Zilog PS015317 Operating Modes, Slave Transmit
*/
static void i2cisr_slave_transmit( unsigned char const sr )
{
#   if defined( configTRACE )
        _putchf( 'S' );
#   endif

    switch( sr )
    {
        case I2C_STATUS_CODE_BUS_ERROR :  // 0x00
        {
            /* refer to Zilog PS0153, following table 89:
               If an illegal condition occurs on the I2C bus, the bus error 
               state is entered (status code 00h). To recover from this state, 
               the STP bit in the I2C_CTL register must be set and the IFLG bit 
               cleared.
               A bus error signifies, in a Slave transaction, then DEV I2C may 
               have signalled incorrectly, or a remote Master got something 
               wrong. */
#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            i2cInit( );

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );
        }
        break;

        case I2C_STATUS_CODE_ARB_ADDR_DATA: // 0x38
        {
            /* When bus arbitration is lost, the IFLG bit is 1 and the status 
               code in the I2C_SR register is 38h. This will happen when another
               master gains arbitration over our in-progress transaction. */
#           if defined( configTRACE )
                _putchf( '3' );
#           endif

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            resetState( );
        }
        break;

        case I2C_STATUS_CODE_SLAVE_STOP : // 0xA0
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Transmit:
               If a STOP condition or a repeated START condition is detected 
               after the acknowledge bit, the IFLG bit is set and the I2C_SR 
               register contains status code A0h. */
#           if defined( configTRACE )
                _putchf( '4' );
#           endif

            if( NULL == i2cTransact->buf )
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif
                /* On entering I2C_ROLE_STX, we would have created a 
                   transaction; if none exists then the Master has dropped 
                   our selection, so fall back to Idle */
                resetState( );
            }
            else
            if( 0 == i2cTransact->lenTRx )//&&( NULL != i2cTransaction->buf ))
            {
#               if defined( configTRACE )
                    _putchf( '+' );
#               endif
               /* A STOP will be received after the master has addressed the 
                  slave, then executes a Repeated Start in order to read the 
                  data from it. The Slave must remember it has been addressed:
                  On entering I2C_ROLE_STX, we would have created a non-zero
                  length transaction, but nothing would yet have be sent. */
                asm( "\tnop    ; nothing to do" );
            }
            else
            //if( 0 < i2cTransact->lenTRx )//&&( NULL != i2cTransaction->buf ))
            {
#               if defined( configTRACE )
                    _putchf( '/' );
#               endif
                /* The Master has received enough, or lost arbitration; so we
                   abort the transaction */
                i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_SLAVE_DATA_WRITE_ACK: // 0xB8
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Transmit:
            The data byte to be transmitted is loaded into the I2C_DR register 
            and the IFLG bit cleared. After the I2C transmits the byte and 
            receives an acknowledge, the IFLG bit is set and the I2C_SR 
            register contains B8h.  */
#           if defined( configTRACE )
                _putchf( '6' );
#           endif

            /* When the final byte to be transmitted is loaded into the 
               I2C_DR register, the AAK bit is cleared when the IFLG is 
               cleared. After the final byte is transmitted, IFLG is set 
               and I2C_SR contains I2C_STATUS_CODE_SLAVE_LAST_DATA_WRITE_ACK */
            slaveTransmitNextByte( );
        }
        break;

        case I2C_STATUS_CODE_SLAVE_DATA_WRITE_NACK: // 0xC0
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Transmit:
            If no acknowledge is received after transmitting a byte, the IFLG 
            is set and the I2C_SR register contains C0h. The I2C then returns 
            to the idle state. */
#           if defined( configTRACE )
                _putchf( '7' );
#           endif

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            resetState( );
        }
        break;
        
        case I2C_STATUS_CODE_SLAVE_LAST_DATA_WRITE_ACK: // 0xC8
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Transmit:
            After the final byte is transmitted, the IFLG is set and the I2C_SR 
            register contains C8h and the I2C returns to the idle state. */
#           if defined( configTRACE )
                _putchf( '8' );
#           endif

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_DONE );
            resetState( );
        }
        break;
        
        case I2C_STATUS_CODE_NULL: // 0xF8
        {
#           if defined( configTRACE )
                _putchf( '9' );
#           endif
        }
        break;

        default :
        {
#           if defined( configTRACE )
                _putchf( 'A' );
                _putchfc( sr )
#           endif

            i2cInit( );

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );
        }
        break;
    }
}


/* i2cisr_slave_receive
   Interrupt Handler - Slave Receiver role
   In SLAVE RECEIVE mode, a number of data bytes are received from a master 
   transmitter.
   Execution does not involve the user application. Only after a data buffer
    has completely received, then the application can retrieve it.
*/
static void i2cisr_slave_receive( unsigned char const sr )
{
    int i;
    
#   if defined( configTRACE )
        _putchf( 'R' );
#   endif

    switch( sr )
    {
        case I2C_STATUS_CODE_BUS_ERROR :  // 0x00
        {
            /* refer to Zilog PS0153, following table 89:
               If an illegal condition occurs on the I2C bus, the bus error 
               state is entered (status code 00h). To recover from this state, 
               the STP bit in the I2C_CTL register must be set and the IFLG bit 
               cleared.
               A bus error signifies, in a Slave transaction, then DEV I2C may 
               have signalled incorrectly, or a remote Master got something 
               wrong. */
#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            i2cInit( );

            slaveReceiveClear( );

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );
        }
        break;

        case I2C_STATUS_CODE_ARB_ADDR_DATA: // 0x38
        {
            /* When bus arbitration is lost, the IFLG bit is 1 and the status 
               code in the I2C_SR register is 38h. This will happen when another
               master gains arbitration over our in-progress transaction. */
#           if defined( configTRACE )
                _putchf( '2' );
#           endif

            slaveReceiveClear( );

            resetState( );
        }
        break;

        case I2C_STATUS_CODE_SLAVE_DATA_READ_ACK :  // 0x80
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Receive:
               In processing I2C_STATUS_CODE_SLAVE_ADDR_READ_ACK, if the AAK 
               bit in the I2C_CTL register is set to 1 then an acknowledge bit 
               is transmitted and the IFLG bit is set after each byte is 
               received. The I2C_SR register contains the status code 80h. */
#           if defined( configTRACE )
                _putchf( '3' );
#           endif

            if( I2C_ROLE_SRX == i2cRole )
            {
                slaveReceiveNextByte( );
            }
            else
            {
                i2cStop( );

                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_SLAVE_DATA_READ_NACK : // 0x88
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Receive:
               If the AAK bit is cleared to 0 during a transfer, the I2C 
               transmits a Nak bit after the next byte is received, and sets 
               the IFLG bit. The I2C_SR register contains the status code 88h.
               The I2C returns to the idle state when the IFLG bit is cleared 
               to 0. */
#           if defined( configTRACE )
                _putchf( '4' );
#           endif

            if( I2C_ROLE_SRX == i2cRole )
            {
                slaveReceiveNextByte( );
                slaveReceiveDone( );
                resetState( );
            }
            else
            {
                i2cStop( );

                resetState( );
            }
        }
        break;
        
        case I2C_STATUS_CODE_GCA_DATA_READ_ACK :  // 0x90,
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Receive:
               In processing I2C_STATUS_CODE_SLAVE_ADDR_READ_ACK, if the AAK 
               bit in the I2C_CTL register is set to 1 then an acknowledge bit 
               is transmitted and the IFLG bit is set after each byte is 
               received. The I2C_SR register contains the status code 90h if 
               SLAVE RECEIVE mode is entered with the general call address. */
#           if defined( configTRACE )
                _putchf( '5' );
#           endif

            if( I2C_ROLE_GCA == i2cRole )
            {
                slaveReceiveNextByte( );
            }
            else
            {
                i2cStop( );

                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_GCA_DATA_READ_NACK : // 0x98,
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Receive:
               If the AAK bit is cleared to 0 during a transfer, the I2C 
               transmits a Nak bit after the next byte is received, and sets 
               the IFLG bit. The I2C_SR register contains the status code 98h 
               if SLAVE RECEIVE mode is entered with the general call address. 
               The I2C returns to the idle state when the IFLG bit is cleared 
               to 0. */
#           if defined( configTRACE )
                _putchf( '6' );
#           endif

            if( I2C_ROLE_GCA == i2cRole )
            {
                slaveReceiveNextByte( );
                slaveReceiveDone( );
                resetState( );
            }
            else
            {
                i2cStop( );

                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_SLAVE_STOP : // 0xA0
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Receive:
               If a STOP condition or a repeated START condition is detected 
               after the acknowledge bit, the IFLG bit is set and the I2C_SR 
               register contains status code A0h. */
#           if defined( configTRACE )
                _putchf( '7' );
#           endif

            if(( I2C_ROLE_GCA == i2cRole )||( I2C_ROLE_SRX == i2cRole ))
            {
                slaveReceiveDone( );  // saves what we have received so far
            }

            resetState( );
        }
        break;

        case I2C_STATUS_CODE_NULL: // 0xF8
        {
#           if defined( configTRACE )
                _putchf( '8' );
#           endif
        }
        break;

        default :
        {
#           if defined( configTRACE )
                _putchf( '9' );
                _putchfc( sr );
#           endif

            i2cInit( );

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );
        }
        break;
    }
}


/* i2cisr_master_transceive
   Interrupt Handler - Master Transmit and Master Receive roles
   We merge the two roles together because, at least for 10-bit addressing, 
   they are inseperable. And Receive adds the status codes 0x40 and 0x48 to
   those of Transmit, plus a few if..elses to distinguish details.
   In the Master Transmit role, the I2C transmits a number of bytes to a slave 
   receiver. Refer to Zilog PS0153 Operating Modes, Master Transmit & tables 78, 
   79, 80. Refer to NXP UM10204 Figure 14, "master-transmitter addresses a
   slave-receiver with a 10-bit address".
   In MASTER RECEIVE mode, the I2C receives a number of bytes from the addressed
   slave transmitter. Refer to Zilog PS0153 Operating Modes, Master Receive & 
   tables 81, 82, 83; and note
    "If 10-bit addressing is being used, the slave is first addressed using the
     full 10-bit address plus the Write bit. The master then issues a restart 
     followed by the first part of the 10-bit address again, but with the Read 
     bit. The status code then becomes 40h or 48h. It is the responsibility of 
     the slave to remember that it had been selected prior to the restart."
     Refer to NXP UM10204 Figure 15, "master-receiver addresses a slave-
     transmitter with a 10-bit address".
*/
static void i2cisr_master_transceive( unsigned char const sr )
{
#   if defined( configTRACE )
        _putchf( 'T' );
#   endif

    switch( sr )
    {
        case I2C_STATUS_CODE_BUS_ERROR :  // 0x0
        {
            /* refer to Zilog PS0153, following table 89:
               If an illegal condition occurs on the I2C bus, the bus error 
               state is entered (status code 00h). To recover from this state, 
               the STP bit in the I2C_CTL register must be set and the IFLG bit 
               cleared.
               A bus error signifies, during a Master transaction, that DEV I2C 
               signalled or sequenced incorrectly; or a remote Slave got some
               thing wrong.  */
#           if defined( configTRACE )
                _putchf( '1' );
#           endif

            i2cInit( );

            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );
        }
        break;

        case I2C_STATUS_CODE_REPEATED_START :    //0x10
#           if defined( configTRACE )
                _putchf( '>' );
#           endif
            /* deliberate fall through */
        case I2C_STATUS_CODE_START_TRANSMITTED : // 0x08
        {
            /* After a START condition is transmitted, the I2C hardware sets 
               IFLG bit to 1 and the I2C_SR register to 08h. 
               For the Master Transmit role, the I2C_DR register must be loaded 
               with either a 7-bit slave address or the first part of a 10-bit 
               slave address, with the lsb cleared to 0 to specify TRANSMIT.
               For the Master Receiver role, either:
                - the I2C_DR register must be loaded with a 7-bit slave address 
                  with the lsb set to 1 to specify RECEIVE; or
                - the I2C_DR register must be loaded with the first part of a 
                  10-bit slave address with the lsb cleared to 0 to specify 
                  TRANSMIT. */
            _Bool const is10bit =
                ( I2C_10BIT_ADDR_MASK ==
                    ( i2cTransact->targetAddr->byte[ 0 ]& I2C_10BIT_ADDR_MASK ))
                        ? true
                        : false;

#           if defined( configTRACE )
                _putchf( '2' );
#           endif

            if(( I2C_ROLE_MTX == i2cRole )||
               (( I2C_ROLE_MRX == i2cRole )&&( true == is10bit )))
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif

                masterTransitionState = I2C_MTRANS_STATE_ADDR_1;
                writeTargetAddress( I2C_BUS_DATA_DIRECTION_TRANSMIT );  // I2C_MTRANS_STATE_ADDR_1 = 1st byte
                // we next expect SR = I2C_STATUS_CODE_ADDR_WRITE_ACK
            }
            else
            if(( I2C_ROLE_MRX == i2cRole )&&( false == is10bit )) // 7-bit address 
            {
#               if defined( configTRACE )
                    _putchf( '+' );
#               endif

                masterTransitionState = I2C_MTRANS_STATE_ADDR_1;
                writeTargetAddress( I2C_BUS_DATA_DIRECTION_RECEIVE );  // I2C_MTRANS_STATE_ADDR_1 = 1st byte
                // we next expect SR = I2C_STATUS_CODE_ADDR_READ_ACK
            }
            else
            {
#           if defined( configTRACE )
                _putchf( '*' );
#           endif
                i2cStop( );
                
                if( I2C_ROLE_MTX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
                }
                else
                if( I2C_ROLE_MRX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
                }
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_ADDR_WRITE_ACK: // 0x18
        {
            /* When the 7-bit slave address or the first part of a 10-bit address 
            is acknowledged, the IFLG bit is 1 and the status code in the I2C_SR 
            register is 18h. The I2C_DR register must be loaded with either the 
            second part of a 10-bit slave address, or the first data byte, with 
            the lsb cleared to 0 to specify TRANSMIT mode. The IFLG bit should 
            now be cleared to 0 to prompt the transfer to continue.*/
            _Bool const is10bit =
                ( I2C_10BIT_ADDR_MASK ==
                ( i2cTransact->targetAddr->byte[ 0 ]& I2C_10BIT_ADDR_MASK ))
                    ? true
                    : false;

#           if defined( configTRACE )
                _putchf( '3' );
#           endif

            if( I2C_MTRANS_STATE_ADDR_1 == masterTransitionState )
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif

                if((( I2C_ROLE_MTX == i2cRole )&&( true == is10bit ))||
                   (( I2C_ROLE_MRX == i2cRole )&&( true == is10bit )))
                {
#                   if defined( configTRACE )
                        _putchf( 'a' );
#                   endif

                    // 10-bit address
                    masterTransitionState = I2C_MTRANS_STATE_ADDR_2;
                    writeTargetAddress( I2C_BUS_DATA_DIRECTION_TRANSMIT );  // I2C_MTRANS_STATE_ADDR_2 = 2nd byte
                    // we expect I2C_STATUS_CODE_SECOND_ADDR_WRITE_ACK next
                }
                else
                if( I2C_ROLE_MTX == i2cRole )
                {
#                   if defined( configTRACE )
                        _putchf( 'b' );
#                   endif

                    // 7-bit address done, start data
                    masterTransitionState = I2C_MTRANS_STATE_DATA;
                    masterTransmitNextByte( );
                    // we expect I2C_STATUS_CODE_MASTER_DATA_WRITE_ACK next
                }
                else
                if( I2C_ROLE_MRX == i2cRole )
                {
#                   if defined( configTRACE )
                        _putchf( 'c' );
#                   endif
                }
            }
            else
            {
#               if defined( configTRACE )
                    _putchf( '+' );
#               endif

                i2cStop( );

                if( I2C_ROLE_MTX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
                }
                else
                if( I2C_ROLE_MRX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
                }
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_ADDR_WRITE_NACK: // 0x20
        {
            /* When the 7-bit slave address or the first part of a 10-bit 
               address is not acknowledged, the IFLG bit is 1 and the status code 
               in the I2C_SR register is 20h. */
#           if defined( configTRACE )
                _putchf( '4' );
#           endif

            i2cStop( );

            if( I2C_ROLE_MTX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
            }
            else
            if( I2C_ROLE_MRX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
            }
            resetState( );
        }
        break;

        case I2C_STATUS_CODE_MASTER_DATA_WRITE_ACK: // 0x28
        {
            /* When the data byte is acknowledged, the IFLG bit is 1 and the 
               status code in the I2C_SR register is 28h. The I2C_DR register 
               must be loaded with the next data byte, with the lsb cleared to 
               0 to specify TRANSMIT mode. The IFLG bit should now be cleared 
               to 0 to prompt the transfer to continue.*/
#           if defined( configTRACE )
                _putchf( '5' );
#           endif

            if(( I2C_ROLE_MTX == i2cRole )&&
               ( I2C_MTRANS_STATE_DATA == masterTransitionState ))
            {
                masterTransmitNextByte( );
            }
            else
            {
                i2cStop( );

                i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_MASTER_DATA_WRITE_NACK: // 0x30
        {
            /* When the data byte is not acknowledged, the IFLG bit is 1 and 
               the status code in the I2C_SR register is 30h. */
#           if defined( configTRACE )
                _putchf( '6' );
#           endif

            i2cStop( );

            i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
            resetState( );
        }
        break;

        case I2C_STATUS_CODE_ARB_ADDR_DATA: // 0x38
        {
            /* When bus arbitration is lost, the IFLG bit is 1 and the status 
               code in the I2C_SR register is 38h. This will happen when another
               master gains arbitration over our in-progress transaction. */
#           if defined( configTRACE )
                _putchf( '7' );
#           endif

            if( --masterTransitionRetryCount )
            {
                /* Re-try from START, set I2C_CTL STA bit and clear IFLG to 
                   re-start the i2c bus transaction when it becomes free. */
                i2cStart( );
                masterTransitionState = I2C_MTRANS_STATE_IDLE;
            }
            else
            {
                if( I2C_ROLE_MTX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_TX_INTRPT );
                }
                else
                if( I2C_ROLE_MRX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_RX_INTRPT );
                }
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_ADDR_READ_ACK: // 0x40
        {
            /* When the 7-bit address (or first part of the re-transmitted 
               10-bit address following I2C_STATUS_CODE_SECOND_ADDR_WRITE_ACK)
               is acknowledged, the IFLG bit is 1 and the status code in the 
               I2C_SR register is 40h. The IFLG bit should now be cleared to 0 
               to prompt the data transfer to continue. */
#           if defined( configTRACE )
                _putchf( '8' );
#           endif

            if(( I2C_ROLE_MRX == i2cRole )&&
               (( I2C_MTRANS_STATE_ADDR_1 == masterTransitionState )||  // 7-bit address
                ( I2C_MTRANS_STATE_ADDR_3 == masterTransitionState )))   // 10-bit address 
            {
#               if defined( configTRACE )
                    _putchf( '+' );
#               endif

                // Either 7-bit or 10-bit address done, start receiving data
                masterTransitionState = I2C_MTRANS_STATE_DATA;
                
                if( 1 >= i2cTransact->len )
                {
#                   if defined( configTRACE )
                        _putchf( '-' );
#                   endif
                    i2cNack( );
                }
                else
                {
#                   if defined( configTRACE )
                        _putchf( '+' );
#                   endif
                    i2cAck( );
                }
            }
            else
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif

                i2cStop( );
                
                i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_ADDR_READ_NACK: // 0x48
        {
            /* When the 7-bit slave address or the first part of a 10-bit 
               address is not acknowledged, the IFLG bit is 1 and the status code 
               in the I2C_SR register is 48h. */
#           if defined( configTRACE )
                _putchf( '9' );
#           endif

            if(( I2C_ROLE_MRX == i2cRole )&&
               (( I2C_MTRANS_STATE_ADDR_1 == masterTransitionState )||  // 7-bit address
                ( I2C_MTRANS_STATE_ADDR_3 == masterTransitionState ))&& // 10-bit address 
               ( --masterTransitionRetryCount ))
            {
                /* Re-try the address. Set both I2C_CTL STA and STP bit then 
                   clear IFLG to re-start the i2c bus transaction.*/
                masterTransitionState = I2C_MTRANS_STATE_ADDR_1;
                writeTargetAddress( I2C_BUS_DATA_DIRECTION_TRANSMIT );
                i2cStop( );
                i2cStart( );
            }
            else
            {
                i2cStop( );

                i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_DATA_READ_ACK: // 0x50
        {
            /* After the next received data byte is acknowledged, the IFLG bit 
               is 1 and the status code in the I2C_SR register is 50h. 
               The received data byte should be retrieved from the I2C_DR 
               register, and either AAK set to 1 to indicate another byte can
               be received or AAK cleared to 0 to indicate another byte cannot
               be received. The IFLG bit should now be cleared to 0 to prompt 
               the transfer to continue.*/
#           if defined( configTRACE )
                _putchf( 'a' );
#           endif

            if(( I2C_ROLE_MRX == i2cRole )&&
               ( I2C_MTRANS_STATE_DATA == masterTransitionState ))
            {
#               if defined( configTRACE )
                    _putchf( '-' );
#               endif

                masterReceiveNextByte( );

                if(( i2cTransact->len - i2cTransact->lenTRx )== 1 )
                {
                    /* Nack the last byte to be received */
#                   if defined( configTRACE )
                        _putchf( '*' );
#                   endif
                    i2cNack( );
                }
                else
                if(( i2cTransact->len - i2cTransact->lenTRx )== 0 )
                {
                    /* Halt after the last byte is received 
                       We shouldn't get here; actually be in 
                       I2C_STATUS_CODE_DATA_READ_NACK */
#                   if defined( configTRACE )
                        _putchf( '/' );
#                   endif

                    i2cStop( );
                    i2cEvent |=( 1 << I2C_EVENT_RX_READY );
                    resetState( );
                }
                 else /* ACK if more bytes to follow */
                {
                    i2cAck( );
                }
            }
            else
            {
#               if defined( configTRACE )
                    _putchf( '+' );
#               endif

                i2cStop( );

                i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_DATA_READ_NACK: // 0x58
        {
            /* After the next data byte is NAKed, the IFLG bit is 1 and the 
                status code in the I2C_SR register is 58h. This ends the
                master receive bus transaction. */
#           if defined( configTRACE )
                _putchf( 'b' );
#           endif

            if(( I2C_ROLE_MRX == i2cRole )&&
               ( I2C_MTRANS_STATE_DATA == masterTransitionState ))
            {
#               if defined( configTRACE )
                    _putchf( '1' );
#               endif

                /* Receive the last byte. */
                masterReceiveNextByte( );

                i2cEvent |=( 1 << I2C_EVENT_RX_READY );
            }
            else
            {
#               if defined( configTRACE )
                    _putchf( '2' );
#               endif

                i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
            }

            i2cStop( );
            resetState( );
        }
        break;

        case I2C_STATUS_CODE_MASTER_ARB_SLAVE_ADDR_WRITE_ACK: // 0x68
        {
            /* When bus arbitration is lost, and another master addresses us as
               the slave (Slave Receive), the IFLG bit is 1 and the status code 
               in the I2C_SR register is 68h. We can chose either to enter Slave 
               Receive role with an ACK, or to reject the role with a NACK. 
               Either way, we abandon the Master Transmit. */
#           if defined( configTRACE )
                _putchf( 'c' );
#           endif

            if( I2C_ROLE_MTX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_TX_INTRPT );
            }
            else
            if( I2C_ROLE_MRX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_RX_INTRPT );
            }
            resetState( );

            i2cRole = I2C_ROLE_SRX;
            i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
            i2cAck( );
        }
        break;

        case I2C_STATUS_CODE_MASTER_ARB_GENERAL_CALL_ADDR_ACK: // 0x78
        {
            /* When bus arbitration is lost, and another master issues the 
               general call address, the IFLG bit is 1 and the status code 
               in the I2C_SR register is 78h. We can chose either to enter 
               Slave Receive role with an ACK, or to reject the role with a 
               NACK. Either way, we abandon the Master Transmit. */
#           if defined( configTRACE )
                _putchf( 'd' );
#           endif
            i2cAck( );

            if( I2C_ROLE_MTX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_TX_INTRPT );
            }
            else
            if( I2C_ROLE_MRX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_RX_INTRPT );
            }
            resetState( );

            i2cRole = I2C_ROLE_GCA;
            i2cState |= I2C_STATE_RUN;
        }
        break;

        case I2C_STATUS_CODE_MASTER_ARB_SLAVE_ADDR_READ_ACK: // 0xB0
        {
            /* Refer to Zilog PS0153 Operating Modes, Slave Transmit:
               I2C goes from MASTER mode to SLAVE TRANSMIT mode when arbitration 
               is lost during the transmission of an address, and the slave 
               address and Read bit are received. This action is represented by 
               the status code B0h in the I2C_SR register. */
#           if defined( configTRACE )
                _putchf( 'e' );
#           endif

            if( I2C_ROLE_MTX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_TX_INTRPT );
            }
            else
            if( I2C_ROLE_MRX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_RX_INTRPT );
            }
            resetState( );

            i2cRole = I2C_ROLE_STX;
            i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
            startSlaveTransmit( );
            i2cAck( );  // set ACK if more than one byte to tranmsit
        }
        break;

        case I2C_STATUS_CODE_SECOND_ADDR_WRITE_ACK: // 0xD0
        {
            /* When the second part of a 10-bit address is acknowledged, the 
               IFLG bit is 1 and the status code in the I2C_SR register is D0h. 
               For the Master Transmit role, I2C_DR register must be loaded 
               with the first data byte, with the lsb cleared to 0 to specify 
               TRANSMIT mode. The IFLG bit should then be cleared to 0 to 
               prompt the transfer to continue.
               For the Master Receive role, a restart should be issued followed 
               by the first part of the 10-bit address again, but with the Read 
               bit set. [Refer to NXP UM10204 Figure 15]. The next expected 
               status code will be either 40h or 48h. */
#           if defined( configTRACE )
                _putchf( 'f' );
#           endif

            if( I2C_MTRANS_STATE_ADDR_2 == masterTransitionState )
            {
                if( I2C_ROLE_MTX == i2cRole )
                {
                    masterTransitionState = I2C_MTRANS_STATE_DATA;
                    masterTransmitNextByte( );
                    // we next expect I2C_STATUS_CODE_MASTER_DATA_WRITE_ACK
                }
                else
                if( I2C_ROLE_MRX == i2cRole )
                {
                    masterTransitionState = I2C_MTRANS_STATE_ADDR_3;
                    writeTargetAddress( I2C_BUS_DATA_DIRECTION_RECEIVE );  // I2C_MTRANS_STATE_ADDR_3 = 1st byte
                    i2cStop( );
                    i2cStart( );
                    // we expect I2C_STATUS_CODE_ADDR_READ_ACK to follow
                }
            }
            else
            {
                i2cStop( );
                
                if( I2C_ROLE_MTX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
                }
                else
                if( I2C_ROLE_MRX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
                }
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_SECOND_ADDR_WRITE_NACK: // 0xD8
        {
            /* When the second part of a 10-bit address is not acknowledged, 
               the IFLG bit is 1 and the status code in the I2C_SR register is 
               D8h. */
#           if defined( configTRACE )
                _putchf( 'g' );
#           endif

            if(( I2C_MTRANS_STATE_ADDR_2 == masterTransitionState )&&
               ( --masterTransitionRetryCount ))
            {
                /* Re-try the address. Set both I2C_CTL STA and STP bit then 
                   clear IFLG to re-start the i2c bus transaction.
                   We must re-start the address anew after a STOP/START. */
                masterTransitionState = I2C_MTRANS_STATE_ADDR_1;
                writeTargetAddress( I2C_BUS_DATA_DIRECTION_TRANSMIT );  // I2C_MTRANS_STATE_ADDR_1 = 1st byte
                i2cStop( );
                i2cStart( );
                // we next expect I2C_STATUS_CODE_ADDR_WRITE_ACK
            }
            else
            {
                i2cStop( );

                if( I2C_ROLE_MTX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
                }
                else
                if( I2C_ROLE_MRX == i2cRole )
                {
                    i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
                }
                resetState( );
            }
        }
        break;

        case I2C_STATUS_CODE_NULL: // 0xF8
        {
#           if defined( configTRACE )
                _putchf( 'h' );
#           endif
        }
        break;

        default :
        {
#           if defined( configTRACE )
                _putchf( 'i' );
                _putchfc( sr );
#           endif

            i2cInit( );
            
            if( I2C_ROLE_MTX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_TX_ERROR );
            }
            else
            if( I2C_ROLE_MRX == i2cRole )
            {
                i2cEvent |=( 1 << I2C_EVENT_RX_ERROR );
            }
            i2cEvent |=( 1 << I2C_EVENT_TX_STRANS_INTRPT );
            i2cEvent |=( 1 << I2C_EVENT_SYS_ERROR );
            resetState( );
        }
        break;
    }
}


/* i2cisr_1
   Main body of the Interrupt Handler
   Should enter here iff i2c Control Register IFLAG bit is set
   Execute role + status register condition.
*/
static void i2cisr_1( void )
{
    unsigned char sr = I2C_SR;  // I2C_SR is reset (to I2C_STATUS_CODE_NULL) when CTL_IFLAG is cleared

#   if defined( configTRACE )
        _putchf( 'W' );
#   endif
    if( i2cIflagTest( ))
    {
        configASSERT(( I2C_ROLE_IDLE <= i2cRole )&&( I2C_ROLE_GCA >= i2cRole ));
#       if defined( configTRACE )
            _putchf( '1' );
#       endif
        // handle status register condition depending on role
        i2cisr[ i2cRole ]( sr );            
    }

    // by clearing CTL IFLAG bit, with ENAB set, STA/STP/ACK will be executed
    ///    i2cIflagClr( );  // the next i2c device transition will follow clearing iFlag
    I2C_CTL = ctl;
    while( i2cStopTest( ));  // wait for STP (if set) to complete

    ctl = I2C_CTL_REG_IEN_ON | I2C_CTL_REG_ENAB_ON;
    if( I2C_ROLE_IDLE == i2cRole )
    {
        /* In the Idle state, I2C can be Slave-addressed by a remote master.
            We need to enable ACK in advance in order to respond positively. */
        ctl |= I2C_CTL_REG_AAK_ON;
        I2C_CTL = ctl;
    }
}


/*
#if( 1 == configUSE_PREEMPTION )
/* I2CIsrHandlerTask
 *      High-priority I2C interrupt handler task.
 *      Will run immediately on exit of _uartisr_0;
 *        iff the current task is not blocking inside MOS * /
static void I2CIsrHandlerTask( void * params )
{
    unsigned long notification;

    for( ;; )
    {
        notification = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
#   if defined( configTRACE )
        _putchf( 'V' );
#   endif
        if( notification )
        {
            i2cisr_1( );
        }
    }
    
    ( void )params;
}
#endif /*( 1 == configUSE_PREEMPTION )* /
*/


/* i2cisr_0 - address written into the interrupt vector table
     Standard ISR reti epilogue */
static void i2cisr_0( void )
{
    /* The current task context SHALL be saved first on entry to an ISR */
    portSAVE_CONTEXT( );

#   if defined( configTRACE )
        _putchf( 'U' );
#   endif

    __higherPriorityTaskWokenI2C = pdFALSE;

    /* run in context of this ISR.
         Other hardware interrupts are blocked out for the duration. */
    i2cisr_1( );
 
    /* If __higherPriorityTaskWokenI2C is now set to pdTRUE then a 
       context switch should be performed to ensure the 
       interrupt returns directly to the highest priority task */
    if( pdTRUE == __higherPriorityTaskWokenI2C )
    {
#       if defined( configTRACE )
            _putchf( '#' );
#       endif

        asm( "\txref _vPortYieldFromISR_2   ; reti from vPortYieldFromISR" );
        asm( "\tJP _vPortYieldFromISR_2     ;   with saved context" );
    }
    else
    {
#       if defined( configTRACE )
            _putchf( '$' );
#       endif

        asm( "\t                            ; reti from here" );

        /* RESTORE CONTEXT SHALL be the last action prior to pop ix; reti.L */
        portRESTORE_CONTEXT( );

        asm( "\t pop ix      ; epilogue, restore IX pushed in prolog");
        asm( "               ; like github.com/breakintoprogram/agon-mos/blob/main/src_startup/vectors16.asm" );
        asm( "               ;   __default_mi_handler" );
        asm( "\t ei          ; re-enable interrupts (on completion of following ret)" );  // *2
        asm( "\t reti.L      ; Long ret (ADL_CALL_MODE_LIL + 3 byte PC)" ); // *3
        asm( "               ; as per UM007715 table 25 for IM 2 ADL=1 MADL=1" );
        asm( "               ; compiler-inserted epilogue below will not be executed" );
    }
}


/*====== I2C low-level functions ============================================*/
/* These routines normally called from devapi.c                              */

/* i2c_dev_open
   Device-specific i2c open function configuration.
   The eZ80 I2C uses a dedicated port, rather than share a multi-function 
   port; no pin modes for Alternate Function number 0
   Callback is required with DEV_MODE_I2C_MULTI_MASTER mode, for processing 
   remote-Master requests. */
POSIX_ERRNO i2c_dev_open(
                DEV_MODE const mode,
                ...
            )
{
    POSIX_ERRNO ret;
    va_list args;
    int i;

#   if defined( configTRACE )
        _putchf( 'O' );
#   endif

    if( I2C_STATE_INITIAL != i2cState )
    {
        ret = POSIX_ERRNO_EADDRINUSE;
        goto i2c_dev_open_end;
    }

#   if defined( configTRACE )
        _putchf( '1' );
#   endif
    /* 1. Reset the I2C semaphore on each uart_dev_open() */
    ret = initialiseI2cSemaphore( &i2cRunSemaphore );
    if( POSIX_ERRNO_ENONE != ret )
    {
        goto i2c_dev_open_end;
    }
#   if defined( configTRACE )
        _putchf( '2' );
#   endif

    /* 2. Create the ISR Handler Task * /
#   if( 1 == configUSE_PREEMPTION )
    {
        ret = createI2CIsrHandlerTask( );
        if( POSIX_ERRNO_ENONE != ret )
        {
            goto i2c_dev_open_end;
        }
    }
#   endif
    */

    /* 5. Initialise the State ready for Interrupts */
    portENTER_CRITICAL( );
    {
        i2cTransact = &transactBuf[ 1 ];
        CLEAR_I2C_TRANSACTION( );
        i2cTransact = &transactBuf[ 0 ];
        CLEAR_I2C_TRANSACTION( );
        i2cState = I2C_STATE_READY;
    }
    portEXIT_CRITICAL( );

    /* 6. Remember Slave Receive callback */
    switch( mode )
    {
        case DEV_MODE_I2C_SINGLE_MASTER :
            // no extra parameters
        break;
        
        case DEV_MODE_I2C_MULTI_MASTER :
        {
            I2C_ADDR *addr;

            va_start( args, mode );
            addr = va_arg( args, void* );
            if( addr )
            {
                storeSlaveAddress( addr );
            }
            i2cHandler = va_arg( args, void* );
            va_end( args );
#           if defined( _DEBUG )&& 0
            {
                ( void )printf( "%s : %d : I2C_SAR = 0x%x : I2C_XSAR = 0x%x\r\n", 
                                "devi2c.c", __LINE__, I2C_SAR, I2C_XSAR );
                ( void )printf( "%s : %d : callback = %p\r\n", 
                                "devi2c.c", __LINE__, i2cHandler );
            }
#           endif
        }
        break;

        default :
        {
#           if defined( _DEBUG )
            {
                ( void )printf( "%s : %d : i2c_open : INVALID mode\r\n", 
                                "devi2c.c", __LINE__ );
            }
#           endif
        }
        break;
    }

#   if defined( configTRACE )
        _putchf( '4' );
#   endif

    /* 7.1 Attach the ISR (disable MOS I2C routines) */
    prevI2cISR =  // remember old vector to restore on i2c_close
        mos_setintvector( I2C_IVECT, i2cisr_0 );
#   if defined( _DEBUG )&& 0
    {
        ( void )printf( "%s : %d : prevI2cISR = 0x%x,  i2cisr_0 = 0x%x\r\n", 
                        "devi2c.c", __LINE__, prevI2cISR, i2cisr_0 );
    }
#   endif

    /* 7.2. Initialise the I2C controller (including enable interrupts) */
    i2cInit( );
    
    // 7.3 set the CTL register ACK bit to 1, to enable Slave bus transaction ack
    //  refer to Zilog PS015317 I2C Operating Modes, Slave Transmit
    i2cAck( );

i2c_dev_open_end:
#   if defined( configTRACE )
        _putchf( '5' );
#   endif
    return( ret );
}


/* i2c_dev_close
   Device-specific i2c close function for device shutdown
   Power-down the I2C
   Refer to Zilog ps015317, resetting the I2C registers */
void i2c_dev_close(
         void
     )
{
    /* Clear Interrupt Enable Register */
    i2cInterruptDisable( );
    portENTER_CRITICAL( );
    {
        // 1. Detach any interrupt handler
        if( prevI2cISR )
        {
            mos_setintvector( I2C_IVECT, prevI2cISR );
            prevI2cISR = NULL;
        }
            
        // 2. Disconnect the application callback
        i2cHandler = NULL;

        i2cState = I2C_STATE_INITIAL;
    }
    portEXIT_CRITICAL( );

    i2cDisable( );

    // Software reset of the I2C device (clr STA, STP, IFLG) and to Idle
    i2cReset( );

    // power down I2C peripheral clock?
///    CLK_PPD1 |= PERIPHERAL_CLK_REG_I2C_OFF; 
}


/* DEV_API: i2c_dev_readm (Master Receive)
   Read from a target slave, through previously opened I2C.
     The receive parameters are used to create a transaction.
     If message contains a non-zero registerAddr, then a pair of transactions
      are created; the first to write the required register address; followed 
      by a second to read data from the (consecutive) register(s). Completion
      of the first transaction is ended with a RESTART on the I2C bus, followed
      by the second transaction. Refer to Bosch BMP280 section 5.2.2 as an 
      example.
     The calling task will be suspended while the read is in progress.
   While I2C_STATE_RUN == i2cState, all other I2C calls (read or write) will 
     be refused with the POSIX_ERRNO_EBUSY error code.  They should retry 
     after a short delay.
   Refer to Zilog PS015317 I2C Operating Modes, Master Receiver */
POSIX_ERRNO i2c_dev_readm(
                I2C_MSG const * message,        // IN
                size_t const num_bytes_to_read, // IN
                size_t * num_bytes_read,        // OUT
                POSIX_ERRNO *res                // OUT
            )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    int i;

#   if defined( configTRACE )
        _putchf( 'M' );
#   endif

    portENTER_CRITICAL( );
    {
        if(( I2C_STATE_READY & i2cState )&&
           ( 0 ==( I2C_STATE_LOCK & i2cState )))  // if not already busy
        {
            i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
        }
        else
        {
            ret = POSIX_ERRNO_EBUSY;
        }
    }
    portEXIT_CRITICAL( );

    if( POSIX_ERRNO_ENONE == ret )
    {
#       if defined( configTRACE )
            _putchf( '1' );
#       endif

        if( 0 == message->registerAddr )
        {
#           if defined( configTRACE )
                _putchf( '2' );
#           endif

            // read from a monotonic device (without registers)
            CREATE_I2C_TRANSACTION( 
                &( message->targetAddr ), 
                message->buf, num_bytes_to_read, 
                num_bytes_read, res );

            // kick off the master receive transaction
            startMasterReceive( );
        }
        else
        {
#           if defined( configTRACE )
                _putchf( '3' );
#           endif

            /* create a pair of transactions to read a register-addressable device
                1. write the 1-byte register address to the slave
                2. read data from the slave */
            CREATE_I2C_TRANSACTION( 
                &( message->targetAddr ), 
                &( message->registerAddr ), sizeof( unsigned char ), 
                num_bytes_read, res );   // results pointers in [0]

            i2cTransact = &transactBuf[ 1 ];
            CREATE_I2C_TRANSACTION( 
                &( message->targetAddr ), 
                message->buf, num_bytes_to_read, 
                NULL, NULL );

            // kick off the master transmit transaction, to write register address
            i2cTransact = &transactBuf[ 0 ];
            startMasterTransmit( );      // results pointers in [0]
        }

        /* The calling task is suspended until the I2C master read completes or 
           an error happens, when the ISR will wake the calling task. Other i2c 
           transactions will be refused until this read completes. */
        if( pdTRUE == xSemaphoreTake( 
                          i2cRunSemaphore, 
                          configDRV_I2C_MAX_DELAY ))
        {
#           if defined( configTRACE )
                _putchf( '4' );
#           endif
            
            // calling task should check callback result
            ret = POSIX_ERRNO_EALREADY;
        }
        else
        {
#           if defined( configTRACE )
                _putchf( '5' );
#           endif

            // calling task should check callback result
            ret = POSIX_ERRNO_ETIMEDOUT;
        }

        portENTER_CRITICAL( );
        {
            ret |= i2cEvent;
            i2cEvent = I2C_EVENT_NONE;
            i2cTransact = &transactBuf[ 0 ];  // results pointers in [0]
            finalisei2cTransaction( ret );
            
            for( i=0; configDRV_I2C_TRANSACT_MAX > i; i++ )
            {
                CLEAR_I2C_TRANSACTION( );
                i2cTransact++;
            }
            i2cTransact = &transactBuf[ 0 ];
            
            /* i2cisr will clear I2C_STATE_RUN before giving the semaphore. 
               We need to clear it here too in case of no interrupt and 
               POSIX_ERRNO_ETIMEDOUT */
            i2cState &=( I2C_STATE )~( I2C_STATE_RUN );
        }
        portEXIT_CRITICAL( );

        i2cState &=( I2C_STATE )( ~I2C_STATE_LOCK );
    }
 
    return( ret );
}


/* DEV_API: i2c_dev_writem (Master Transmit)
   Initiate a Write to a target slave through previously opened I2C */
POSIX_ERRNO i2c_dev_writem(
                I2C_MSG const * const message,   // IN
                size_t const num_bytes_to_write, // IN
                size_t * num_bytes_sent,         // OUT
                POSIX_ERRNO *res                 // OUT
            )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    int i;

#   if defined( configTRACE )
        _putchf( 'N' );
#   endif

    portENTER_CRITICAL( );
    {
        if(( I2C_STATE_READY & i2cState )&&
           ( 0 ==( I2C_STATE_LOCK & i2cState )))  // if not already busy
        {
            i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
        }
        else
        {
            ret = POSIX_ERRNO_EBUSY;
        }
    }
    portEXIT_CRITICAL( );

    if( POSIX_ERRNO_ENONE == ret )
    {
#       if defined( configTRACE )
            _putchf( '1' );
#       endif
        CREATE_I2C_TRANSACTION( 
            &( message->targetAddr ), 
            message->buf, num_bytes_to_write, 
            num_bytes_sent, res );
#       if defined( _DEBUG )&& 0
            PRINT_I2C_TRANSACTION( );
#       endif

        // kick off the master transmit transaction
        startMasterTransmit( );

#       if defined( configTRACE )
            _putchf( '2' );
#       endif
        /* The calling task is suspended until the I2C master read completes or 
           an error happens, when the ISR will wake the calling task. Other i2c 
           transactions will be refused until this read completes. */
        if( pdTRUE == xSemaphoreTake( 
                          i2cRunSemaphore, 
                          configDRV_I2C_MAX_DELAY ))
        {
#       if defined( configTRACE )
            _putchf( '3' );
#       endif
            // calling task should check callback result
            ret = POSIX_ERRNO_EALREADY;
        }
        else
        {
            // calling task should check callback result
#       if defined( configTRACE )
            _putchf( '4' );
#       endif
            ret = POSIX_ERRNO_ETIMEDOUT;
        }

        portENTER_CRITICAL( );
        {
            ret |= i2cEvent;
            i2cEvent = I2C_EVENT_NONE;
            i2cTransact = &transactBuf[ 0 ];  // results pointers in [0]
            finalisei2cTransaction( ret );
            
            for( i=0; configDRV_I2C_TRANSACT_MAX > i; i++ )
            {
                CLEAR_I2C_TRANSACTION( );
                i2cTransact++;
            }
            i2cTransact = &transactBuf[ 0 ];
            
            /* i2cisr will clear I2C_STATE_RUN before giving the semaphore. 
               We need to clear it here too in case of no interrupt and 
               POSIX_ERRNO_ETIMEDOUT */
            i2cState &=( I2C_STATE )~( I2C_STATE_RUN );
        }
        portEXIT_CRITICAL( );

        i2cState &=( I2C_STATE )( ~I2C_STATE_LOCK );
    }
 
    return( ret );
}


/* DEV_API: i2c_dev_reads (Slave Receive)
   In SLAVE RECEIVE role, a number of data bytes are received from a remote
   master transmitter.
   The I2C enters SLAVE RECEIVE mode when it receives its own slave address 
   with the Write bit (lsb = 0) after a START condition. 
   Data is stored into receive buffers, without application involvement. Up
   to configDRV_I2C_BUFFER_NUM messages, each of configDRV_I2C_BUFFER_SZ bytes
   can be buffered. The remote master address is not captured. 
   The same procedure follows when the remote master transmitter issues a 
   General Call Address (address 0x0 with the Write bit, lsb = 0), to address
   all slaves, and if the application has first enabled GCA by calling 
   i2c_ioctl with command DEV_IOCTL_I2C_ENABLE_GENERAL_CALL_ADDRESS; it is not 
   enabled by default.
   An application task will periodically call i2c_reads or i2c_poll to collect 
   any receive buffered Slave Receive (or GCA) messages into its own application
   buffer. The application needs to poll periodically to avoid I2C_EVENT_RX_FULL 
   being returned.
   Result codes:
     POSIX_ERRNO_ENONE      data in application buffer, receive buffer emptied 
     POSIX_ERRNO_ENOTEMPTY  data in application buffer, receive buffer not empty
     POSIX_ERRNO_ENODATA    no data in receive buffer */
POSIX_ERRNO i2c_dev_reads(
                unsigned char * const buffer,  // IN
                size_t const num_bytes_to_read,      // IN
                size_t * num_bytes_read,             // OUT
                POSIX_ERRNO *res                     // OUT
            )
{
    POSIX_ERRNO ret;

    /* we only read data once a whole message has been received; we determine
       this when the input buffer is not the same as the next output one.
       We can execute i2c_reads while I2C_STATE_RUN is active. */
    if( slaveReceiveBufOut != slaveReceiveBufIn )
    {
        int i;
        unsigned char start = slaveReceiveBuf[ slaveReceiveBufOut ].Outdex;
        unsigned char end = slaveReceiveBuf[ slaveReceiveBufOut ].Index - start;
        unsigned char n =(( end - start )> num_bytes_to_read )
                          ? num_bytes_to_read
                          :( end - start );
        end = start + n;

        if( 0 == end - start )
        {
            ret = POSIX_ERRNO_ENODATA;
        }
        else
        {
            for( i = start; end > i; i++ )
            {
                (( unsigned char * )buffer )[ i ]= 
                    slaveReceiveBuf[ slaveReceiveBufOut ]._data[ i ];
            }
            slaveReceiveBuf[ slaveReceiveBufOut ].Outdex += n;
        
            if( configDRV_I2C_BUFFER_SZ <= 
                  slaveReceiveBuf[ slaveReceiveBufOut ].Outdex )
            {
                // mark buffer as read
                slaveReceiveBuf[ slaveReceiveBufOut ].Index =
                  slaveReceiveBuf[ slaveReceiveBufOut ].Outdex = 0;

                // ready for next output
                if( configDRV_I2C_BUFFER_NUM >= ++slaveReceiveBufOut )
                {
                    slaveReceiveBufOut = 0;
                }
            
                ret = POSIX_ERRNO_ENONE;  // buffer emptied
            }
            else
            {
                ret = POSIX_ERRNO_ENOTEMPTY;   // more to read from this buffer
            }
        }
    }
    else
    {
        /* no Slave Receive data buffered */
        ret = POSIX_ERRNO_ENODATA;
    }
    
    return( ret );
}


/* DEV_API: i2c_dev_writes (Slave Transmit)
   In SLAVE TRANSMIT mode, a number of bytes are transmitted to a master 
   Receiver.
   The eZ80 I2C enters SLAVE TRANSMIT mode when it receives its own slave 
   address and a Read bit after a START condition. 
   This routine is not normally called by an application. Managed by i2cisr, 
   the application callback, i2cHandler, registered in i2c_open will be 
   invoked either from i2cisr (case 0 == configUSE_PREEMPTION), or from 
   I2CIsrHandlerTask (case 1 == configUSE_PREEMPTION) running as a highest-
   priority ISR Handler Task, shall interpret the remote master request then
   call i2c_writes to send the responding data. 
   If no callback is registered in i2c_open, then DEV I2C will respond to the
   Slave Transmit request with an automatic 'who am i' string. This will
   include the addressed stored when the application calls i2c_ioctl with
   parameter DEV_IOCTL_I2C_SET_SLAVE_ADDRESS. */
POSIX_ERRNO i2c_dev_writes(
                unsigned char const * const buffer,    // IN
                size_t const num_bytes_to_write, // IN
                size_t * num_bytes_sent,         // OUT
                POSIX_ERRNO *res                 // OUT
            )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;
    int i;
    
    /// we don't want to re-enable interrupts with portEXIT_CRITICAL from i2cisr
    ///portENTER_CRITICAL( );
    {
        if(( I2C_STATE_READY & i2cState )&&
           ( 0 ==( I2C_STATE_LOCK & i2cState )))  // if not already busy
        {
            i2cState |=( I2C_STATE )( I2C_STATE_RUN | I2C_STATE_LOCK );
        }
        else
        {
            ret = POSIX_ERRNO_EBUSY;
        }
    }
    ///portEXIT_CRITICAL( );

    if( POSIX_ERRNO_ENONE == ret )
    {
        CREATE_I2C_TRANSACTION( 
            NULL, 
            buffer, num_bytes_to_write, 
            num_bytes_sent, res );

        // kick off the slave transmit transaction
        slaveTransmitNextByte( );

#       if( 1 == configUSE_PREEMPTION )
        {
            /* The calling task is suspended until the I2C master read completes or 
               an error happens, when the ISR will wake the calling task. Other i2c 
               transactions will be refused until this read completes. */
            if( pdTRUE == xSemaphoreTake( 
                              i2cRunSemaphore, 
                              configDRV_I2C_MAX_DELAY ))
            {
                // calling task should check callback result
                ret = POSIX_ERRNO_EALREADY;
            }
            else
            {
                // calling task should check callback result
                ret = POSIX_ERRNO_ETIMEDOUT;
            }

            portENTER_CRITICAL( );
            {
                ret |= i2cEvent;
                i2cEvent = I2C_EVENT_NONE;
                i2cTransact = &transactBuf[ 0 ];  // results pointers in [0]
                finalisei2cTransaction( ret );
            
                for( i=0; configDRV_I2C_TRANSACT_MAX > i; i++ )
                {
                    CLEAR_I2C_TRANSACTION( );
                    i2cTransact++;
                }

                i2cTransact = &transactBuf[ 0 ];
            
                /* i2cisr will clear I2C_STATE_RUN before giving the semaphore. 
                   We need to clear it here too in case of no interrupt and 
                   POSIX_ERRNO_ETIMEDOUT */
                i2cState &=( I2C_STATE )~( I2C_STATE_RUN );
            }
            portEXIT_CRITICAL( );
        
            i2cState &=( I2C_STATE )( ~I2C_STATE_LOCK );
        }
#       else
        {
            /* resetState( ) will perform the completion cleardown 
               as if i2cRunSemaphore had been taken */
        }
#       endif /* 1 == configUSE_PREEMPTION */
    }
 
    return( ret );
}


/* i2c_dev_ioctl
   Device-specific I2C IO Control function
*/
POSIX_ERRNO i2c_dev_ioctl(
                DEV_IOCTL const cmd,
                void * const param
            )
{
    POSIX_ERRNO ret = POSIX_ERRNO_ENONE;

    portENTER_CRITICAL( );
    {
        if( 0 ==( I2C_STATE_LOCK & i2cState ))  // if not already busy
        {
            switch( cmd )
            {
                case DEV_IOCTL_I2C_SET_FREQUENCY :
                {
                    I2C_SCL_FREQ f = *( I2C_SCL_FREQ* )param;

                    i2cSetClockSpeed( f );
                }
                break;

                case DEV_IOCTL_I2C_SET_SLAVE_ADDRESS :
                {
                    I2C_ADDR const * const addr =( I2C_ADDR* )param;

                    if( addr )
                    {
                        storeSlaveAddress( addr );
                    }
printf( "I2C_SAR = 0x%x : I2C_XSAR = 0x%x\r\n", I2C_SAR, I2C_XSAR );
                }
                break;

                case DEV_IOCTL_I2C_ENABLE_GENERAL_CALL_ADDRESS :
                {
                    enableGeneralCallAddress( );
                }
                break;

                case DEV_IOCTL_I2C_DISABLE_GENERAL_CALL_ADDRESS :
                {
                    disableGeneralCallAddress( );
                }
                break;
        
                default:
                {
                    ret = POSIX_ERRNO_EINVAL;
                }
                break;
            }
        }
        else
        {
            ret = POSIX_ERRNO_EBUSY;
        }
    }
    portEXIT_CRITICAL( );
    
    return( ret );
}

#endif  /* 1 == configUSE_DRV_I2C */
