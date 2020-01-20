//------------------------------------------------------------------------------
// MCP_Registers.h
//------------------------------------------------------------------------------
// Copyright (C) 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Description:
//
// This file defines addresses for MCP registers and APIs to access them. MCP
// registers are divided into two groups depending on register size. First group
// contains 8-bit registers while second group contain 16-bit registers. 16-bit
// registers are defined in big-endian format.
//
// Release 0.0 - April 11, 2013 mufayyuz
//    -Initial Revision.
//
// At this stage MCP Register and UART memory usage is:
// data=59.3 xdata=33 code=1132
//

#ifndef MCP_REGISTERS_H
#define MCP_REGISTERS_H

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// BLDC Register Map - 8-bit registers
// It contains 48 registers (Addresses 0x00 to 0x2F) but only few are allocated.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// MCP_Reg_Firmware_Revision:
// It is 8-bit read-only register. It is updated only at build time with F/W
// revision (version/release) number. It is read and sent to controller in
// response to Read Register Command (?regaddr\n) or status update (if enabled).
//------------------------------------------------------------------------------
#define MCP_REG_FIRMWARE_REVISION         0x00

//------------------------------------------------------------------------------
// MCP_Reg_Present_Motor_Direction:
// It is 8-bit read-only register. It is updated when motor direction is changed
// either by controller or button. It is read and sent to controller in response
// to Read Register Command (?regaddr\n) or status update (if enabled).
// Values:
//     0: clockwise
//     1: counter-clockwise
//------------------------------------------------------------------------------
#define MCP_REG_PRESENT_MOTOR_DIRECTION   0x01

//------------------------------------------------------------------------------
// MCP_Reg_Target_Motor_Direction:
// It is 8-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n) when controller is selected as a source of target
// direction. It is read and sent to controller in response to Read Register
// Command (?regaddr\n) or status update (if enabled).
// Values:
//     0: clockwise
//     1: counter-clockwise
//------------------------------------------------------------------------------
#define MCP_REG_TARGET_MOTOR_DIRECTION    0x02

//------------------------------------------------------------------------------
// MCP_Reg_Motor_Control_Source:
// It is 8-bit read-only register. It is read and sent to controller in response
// to Read Register Command (?regaddr\n) or status update (if enabled).
// Values:
//     --7---6---5---------4--------------3-------2-------1------0----
//     | X | X | X | Direction Src | RPM or PWM | X | RPM/PWM Source |
//     ---------------------------------------------------------------
//     RPM/PWM Source:
//         0: Controller is the source of Speed/PWM control.
//         1: External PWM input is the source of RPM/PWM control.
//         2: ADC input is the source of RPM/PWM control.
//     RPM or PWM:
//         0: Motor's target RPM is selected if RPM/PWM Source is Controller.
//         1: Motor's target PWM is selected.
//     Direction Source:
//         0: Motor direction is set by Controller.
//         1: Motor direction is set by button input.
//------------------------------------------------------------------------------
#define MCP_REG_MOTOR_CONTROL_SOURCE      0x03

//------------------------------------------------------------------------------
// MCP_Reg_Over_Current_Threshold:
// It is 8-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n). It is read and sent to controller in response to Read
// Register Command (?regaddr\n) or status update (if enabled).
// Values:
//     0: Disable over-current shutdown feature
//     Non-Zero: Threshold for over-current detection and shutdown feature
//------------------------------------------------------------------------------
#define MCP_REG_OVER_CURRENT_THRESHOLD    0x04

//------------------------------------------------------------------------------
// MCP_Reg_Over_Current_Persistence:
// It is 8-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n). It is read and sent to controller in response to Read
// Register Command (?regaddr\n) or status update (if enabled).
// Values:
//     0: Disable over-current shutdown feature
//     Non-Zero: Number of consecutive current samples that must exceeded
//               current threshold in order to detect over current.
//------------------------------------------------------------------------------
#define MCP_REG_OVER_CURRENT_PERSISTANCE  0x05

//------------------------------------------------------------------------------
// MCP_Reg_Motor_Pole_Pairs_Count:
// It is 8-bit read-only register. It is initialized at build time. It is read
// and sent to controller in response to Read Register Command (?regaddr\n) or
// status update (if enabled).
// Values:
//     Number of pairs of motor's poles.
//------------------------------------------------------------------------------
#define MCP_REG_MOTOR_POLE_PAIRS_COUNT    0x06

//------------------------------------------------------------------------------
// MCP_Reg_PWM_Switching:
// It is 8-bit read-only register. It is initialized at build time.
// It is read and sent to controller in response to Read Register Command
// (?regaddr\n) or status update (if enabled).
// Values:
//     0: High side is PWMed
//     1: Low side is PWMed
//     2: Diagonal/Symmetric/Bipolar
//------------------------------------------------------------------------------
#define MCP_REG_PWM_SWITCHING             0x07

//------------------------------------------------------------------------------
// MCP_Reg_Commutation_Method:
// It is 8-bit read-only register. It is initialized at build time.
// It is read and sent to controller in response to Read Register Command
// (?regaddr\n) or status update (if enabled).
// Values:
//     0: Count-down timing from zero-crossing
//     1: Flux integration
//     2: Hall sensor
//     x: Others ...
//------------------------------------------------------------------------------
#define MCP_REG_COMMUTATION_METHOD        0x08


//Rest of 8-bit registers are reserved
#define MCP_REG_RESERVED_START            0x09
#define MCP_REG_RESERVED_END              0x2F

//------------------------------------------------------------------------------
// BLDC Register Map - 16-bit registers
// It contains 104 registers (Addresses 0x30 to 0xFE) but only few are allocated.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// MCP_Reg_Present_Motor_Speed:
// It is 16-bit read-only register. It is updated internally by F/W to reflect
// present speed of motor in units of 10 RPMs. That is, the resolution of this
// register is (or 1 LSB represents) 10 RPM.
// It is read and sent to controller in response to Read Register Command
// (?regaddr\n) or status update (if enabled).
// Value:
//     Present RPM of motor in units of 10 RPMs
//------------------------------------------------------------------------------
#define MCP_REG_PRESENT_MOTOR_SPEED       0x30

//------------------------------------------------------------------------------
// MCP_Reg_Motor_Coil_Current:
// It is 16-bit read-only register. It is updated internally by F/W to reflect
// amount of current flowing in the motor coil in 0.01 Amperes. It is read and
// sent to controller in response to Read Register Command (?regaddr\n) or
// status update (if enabled).
// Value:
//     Bits 15-14: active phase
//         1: U-V phase
//         2: V-W phase
//         3: W-U phase
//     Bits 13-0: motor coil current in 0.01 Amperes
//------------------------------------------------------------------------------
#define MCP_REG_MOTOR_COIL_CURRENT        0x32

//------------------------------------------------------------------------------
// MCP_Reg_Motor_Operating_Voltage:
// It is 16-bit read-only register. It is updated internally by F/W to reflect
// operating voltage of motor in 0.01 Volts. It is read and sent to controller
// in response to Read Register Command or status update (if enabled).
// Value:
//     Operating voltage in 0.01 Volts
//------------------------------------------------------------------------------
#define MCP_REG_MOTOR_OPERATING_VOLTAGE   0x34

//------------------------------------------------------------------------------
// MCP_Reg_Active_PWM_Duty_Cycle:
// It is 16-bit read-only register. It is updated internally by F/W to reflect
// the duty cycle of active PWM. It is read and sent to controller in
// response to Read Register Command (?regaddr\n) or status update (if enabled).
// Value:
//     Duty cycle (0xFF is 100%)
//------------------------------------------------------------------------------
#define MCP_REG_ACTIVE_PWM_DUTY_CYCLE     0x36

//------------------------------------------------------------------------------
// MCP_Reg_Current_Sampling_Period:
// It is 16-bit read-only register. It is initialized at build time.
// It is read and sent to controller in response to Read Register Command
// (?regaddr\n) or status update (if enabled).
// Value:
//     Sampling period in 0.1 msecs
//------------------------------------------------------------------------------
#define MCP_REG_CURRENT_SAMPLING_PERIOD   0x38

//------------------------------------------------------------------------------
// MCP_Reg_Maximum_Speed:
// It is 16-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n). It is read and sent to controller in response to Read
// Register Command (?regaddr\n) or status update (if enabled).
// Value:
//     It represents the maximum speed of motor in units of 10 RPMs. It is used
//     to set target speed when input speed parameter is at full scale.
//------------------------------------------------------------------------------
#define MCP_REG_MAXIMUM_OPERATIONAL_SPEED 0x3A

//------------------------------------------------------------------------------
// MCP_Reg_Target_Motor_Speed:
// It is 16-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n) if controller is selected as source to set motor speed
// in MCP_REG_MOTOR_CONTROL_SOURCE. It is read and sent to controller in
// response to Read Register Command (?regaddr\n) or status update (if enabled).
// Value:
//     It represents target speed of motor in units of 10 RPMs. It should not be
//     greater that MCP_REG_MAXIMUM_OPERATIONAL_SPEED.
//------------------------------------------------------------------------------
#define MCP_REG_TARGET_MOTOR_SPEED        0x3C

//------------------------------------------------------------------------------
// MCP_Reg_Target_PWM_Duty_Cycle:
// It is 16-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n) if controller (???) is selected as source to set PWM
// in MCP_reg_motor_control_source. It is read and sent to controller in
// response to Read Register Command (?regaddr\n) or status update (if enabled).
//------------------------------------------------------------------------------
#define MCP_REG_TARGET_PWM_DUTY_CYCLE     0x3E

//------------------------------------------------------------------------------
// MCP_Reg_Acceleration_Step_Size:
// It is 16-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n). It is read and sent to controller in response to Read
// Register Command (?regaddr\n) or status update (if enabled).
// Value:
//     PWM duty cycle is incremented by this amount to achieve target speed.
//------------------------------------------------------------------------------
#define MCP_REG_ACCELERATION_STEP_SIZE    0x40

//------------------------------------------------------------------------------
// MCP_Reg_Deceleration_Step_Size:
// It is 16-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n). It is read and sent to controller in response to Read
// Register Command (?regaddr\n) or status update (if enabled).
// Value:
//     PWM duty cycle is decremented by this amount to achieve target speed.
//------------------------------------------------------------------------------
#define MCP_REG_DECELERATION_STEP_SIZE    0x42

//------------------------------------------------------------------------------
// MCP_Reg_Proportional_Gain:
// It is 16-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n). It is read and sent to controller in response to Read
// Register Command (?regaddr\n) or status update (if enabled).
// Value:
//     It is used as proportional gain in PI control loop.  This value is
//     scaled up by a factor of 256.
//------------------------------------------------------------------------------
#define MCP_REG_PROPORTIONAL_GAIN         0x44

//------------------------------------------------------------------------------
// MCP_Reg_Integral_Gain:
// It is 16-bit read-write register. It is updated by Write Register Command
// (=regaddr regvalue\n). It is read and sent to controller in response to Read
// Register Command (?regaddr\n) or status update (if enabled).
// Value:
//     It is used as integral gain in PI control loop.  This value is
//     scaled up by a factor of 256.
//------------------------------------------------------------------------------
#define MCP_REG_INTEGRAL_GAIN             0x46


//Rest of 16 bit registers are undefined
#define MCP_REG_UNDEFINED_START           0x48


//------------------------------------------------------------------------------
// MCP_init_registers:
// void MCP_init_registers (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     This function initializes MCP registers to default values and must be
//     called by MCP at start-up.
//------------------------------------------------------------------------------
void MCP_init_registers(void);

//------------------------------------------------------------------------------
// MCP_set_8bit_register:
// bit MCP_set_8bit_register (U8 address, U8 value, bit external_access)
//
// Return Value:
//     0: Success
//     1: Failure - Invalid address or attempt to write to a read-only register
//
// Parameters:
//     address: A unique ID of an 8-bit MCP register
//     value: Value to be written to register.
//     external_access:
//         0: Function is called internally by F/W
//         1: Function is called as a result of Write Register Command ('=')
//
// Description:
//     This function is called by MCP when a Write Register Command for 8-bit
//     register is received from controller. If register is reserved/undefined
//     then it returns 1. For a valid register, it returns 1 if it is read-only
//     and returns 0 otherwise. A read-only register is read-only for external
//     access via Write Register Command. All registers are writable internally
//     by F/W.
//
//------------------------------------------------------------------------------
bit MCP_set_8bit_register (U8 address, U8 value, bit external_access);

//------------------------------------------------------------------------------
// MCP_set_16bit_register:
// bit MCP_set_16bit_register (U8 address, U16 value, bit external_access)
//
// Return Value:
//     0: Success
//     1: Failure - Invalid address or attempt to write to a read-only register
//
// Parameters:
//     address: A unique ID of an 16-bit MCP register
//     value: Value to be written to register.
//     external_access:
//         0: Function is called internally by F/W
//         1: Function is called as a result of Write Register Command ('=')
//
// Description:
//     This function is called by MCP when a Write Register Command for 16-bit
//     register is received from controller. If register is reserved/undefined
//     then it returns 1. For a valid register, it returns 1 if it is read-only
//     and returns 0 otherwise. A read-only register is read-only for external
//     access via Write Register Command. All registers are writable internally
//     by F/W.
//
//------------------------------------------------------------------------------
bit MCP_set_16bit_register (U8 address, U16 value, bit external_access);

//------------------------------------------------------------------------------
// MCP_set_8bit_register_isr:
// void MCP_set_8bit_register_isr (U8 address, U8 value)
//
// Return Value:
//     None
//
// Parameters:
//     address: A unique ID of an MCP register
//     value: Value to be written to register.
//
// Description:
//     This function is called by ISR to update a register. It assumes that the
//     address of valid 8-bit register is passed to it. It can write all 8-bit
//     registers.
//
//------------------------------------------------------------------------------
void MCP_set_8bit_register_isr (U8 address, U8 value);

//------------------------------------------------------------------------------
// MCP_set_16bit_register_isr:
// void MCP_set_16bit_register_isr (U8 address, U16 value)
//
// Return Value:
//     None
//
// Parameters:
//     address: A unique ID of an MCP register
//     value: Value to be written to register.
//
// Description:
//     This function is called by ISR to update a register. It assumes that the
//     address of valid 16-bit register is passed to it. It can write all 16-bit
//     registers.
//
//------------------------------------------------------------------------------
void MCP_set_16bit_register_isr (U8 address, U16 value);

//------------------------------------------------------------------------------
// MCP_get_8bit_register:
// bit MCP_get_8bit_register (U8 address, U8 * value)
//
// Return Value:
//     Contents of addressed register
//
// Parameters:
//     address: A unique ID of an MCP register
//
// Description:
//     This function is called by MCP when a Read Register Command is received
//     from controller. It assumes that address validity is checked by caller.
//------------------------------------------------------------------------------
U8 MCP_get_8bit_register (U8 address);

//------------------------------------------------------------------------------
// MCP_get_16bit_register:
// U16 MCP_get_16bit_register (U8 address)
//
// Return Value:
//     Contents of addressed register
//
// Parameters:
//     address: A unique ID of an MCP register
//
// Description:
//     This function is called by MCP when a Read Register Command is received
//     from controller. It assumes that address validity is checked by caller.
//------------------------------------------------------------------------------
U16 MCP_get_16bit_register (U8 address);

//------------------------------------------------------------------------------
// MCP_get_8bit_register_isr:
// U8 MCP_get_8bit_register_isr (U8 address)
//
// Return Value:
//     Contents of addressed register
//
// Parameters:
//     address: A unique ID of an MCP register
//
// Description:
//     This function is called by ISR to read a register. It assumes that the
//     address of valid 8-bit register is passed to it.
//
//------------------------------------------------------------------------------
U8 MCP_get_8bit_register_isr (U8 address);

//------------------------------------------------------------------------------
// MCP_get_16bit_register_isr:
// U16 MCP_get_16bit_register_isr (U8 address)
//
// Return Value:
//     Contents of addressed register
//
// Parameters:
//     address: A unique ID of an MCP register
//
// Description:
//     This function is called by ISR to read a register. It assumes that the
//     address of valid 16-bit register is passed to it.
//
//------------------------------------------------------------------------------
U16 MCP_get_16bit_register_isr (U8 address);

//------------------------------------------------------------------------------
// MCP_status_update_register:
// bit MCP_status_update_register (U8 address, bit enable)
//
// Return Value:
//     0: Success
//     1: Failure - Invalid address
//
// Parameters:
//     address: A unique ID of an MCP register
//     enable: Flag indicating enable or disable
//         0: Disable sending given register in status update transfer
//         1: Enable sending given register in status update transfer
//
// Description:
//     This function is called by MCP when a Enable/Disable Register in Status
//     Update Command is received from controller. It sets/resets a register
//     specific bit in an internal map register depending on enable parameter.
//     Enabled registers will be sent to UART regularly in binary mode.
//
//------------------------------------------------------------------------------
bit MCP_status_update_register (U8, bit);

//------------------------------------------------------------------------------
// MCP_is_8bit_status_update_enabled:
// bit MCP_is_8bit_status_update_enabled (U8 address)
//
// Return Value:
//     0: Register is not enabled for status update
//     1: Register is enabled for status update
//
// Parameters:
//     address: A unique ID of an 8-bit MCP register
//
// Description:
//     This function is called by MCP during Status Update in binary mode to
//     check register can be read and included in status update process.
//
//------------------------------------------------------------------------------
bit MCP_is_8bit_status_update_enabled (U8);

//------------------------------------------------------------------------------
// MCP_is_16bit_status_update_enabled:
// bit MCP_is_16bit_status_update_enabled (U8 address)
//
// Return Value:
//     0: Register is not enabled for status update
//     1: Register is enabled for status update
//
// Parameters:
//     address: A unique ID of a 16-bit MCP register
//
// Description:
//     This function is called by MCP during Status Update in binary mode to
//     check register can be read and included in status update process.
//
//------------------------------------------------------------------------------
bit MCP_is_16bit_status_update_enabled (U8);

#endif //MCP_REGISTERS_H

//------------------------------------------------------------------------------
// End Of File
//------------------------------------------------------------------------------
