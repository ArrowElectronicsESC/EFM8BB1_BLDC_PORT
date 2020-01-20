//------------------------------------------------------------------------------
// MCP_Registers.c
//------------------------------------------------------------------------------
// Copyright (C) 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Description:
//
// This file allocates memory for MCP registers. It also allocates 2 bits per
// register. These bits define access type and and status update enable/disable.
// BLDC registers are divided into two groups depending on register size. First
// group contains 8-bit registers while second group contain 16-bit registers.
// 16-bit registers are defined in big-endian format.
//
// Release 0.0 - April 11, 2013 mufayyuz
//    -Initial Revision.
//

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "bldcdk.h"

#define MCP_REG_MEM_TYPE xdata

#define BLDC_RD_FW_VERSION      1
#define BLDC_RD_DEFAULT_DIR     0 //0: clockwise, 1: counter-clockwise
#define BLDC_RD_CURRENT_TH      MOTOR_OC                //Over current threshold value
#define BLDC_RD_CURRENT_COUNT   MOTOR_OC_MAX_DEBOUNCE   //Over current persistence

#define BLDC_RD_CURRENT_SAMPLING_PERIOD     1 //0.1 msec
#define BLDC_RD_MAXIMUM_OPERAIONAL_SPEED    MOTOR_MAX_RPM
#define BLDC_RD_ACCELERATION_STEP_SIZE      ACCELERATION_STEP_SIZE
#define BLDC_RD_DECELERATION_STEP_SIZE      DECELERATION_STEP_SIZE
#define BLDC_RD_PI_PROPORTIONAL_GAIN        PI_PROPORTIONAL_GAIN
#define BLDC_RD_PI_INTEGRAL_GAIN            PI_INTEGRAL_GAIN

#define INDEX_PRESENT_MOTOR_SPEED       0
#define INDEX_MOTOR_COIL_CURRENT        1
#define INDEX_MOTOR_OPERATING_VOLTAGE   2
#define INDEX_ACTIVE_PWM_DUTY_CYCLE     3
#define INDEX_CURRENT_SAMPLING_PERIOD   4
#define INDEX_MAXIMUM_OPERATIONAL_SPEED 5
#define INDEX_TARGET_MOTOR_SPEED        6
#define INDEX_TARGET_PWM_DUTY_CYCLE     7
#define INDEX_ACCELERATION_STEP_SIZE    8
#define INDEX_DECELERATION_STEP_SIZE    9
#define INDEX_PROPORTIONAL_GAIN         10
#define INDEX_INTEGRAL_GAIN             11

#define MCP_ATOMIC_ACCESS_START() \
do                                \
{                                 \
    int_status = IE_EA;              \
    IE_EA = 0;                       \
} while (0)

#define MCP_ATOMIC_ACCESS_END()   \
do                                \
{                                 \
    IE_EA = int_status;              \
} while (0)

//------------------------------------------------------------------------------
// Data
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// MCP_Reg8_Status_Update_Map:
// It is 16-bit read-write register. It is updated by Enable/Disable Status
// Command (#regaddr 0/1\n). It is read by F/W to select individual registers
// during status update.
// Values:
//     Each bit enables/disables one of 8-bit registers for status update.
// WARNING: Only 9 registers are defined in this revision of firmware.
//------------------------------------------------------------------------------
volatile U16 data MCP_reg8_status_update_map;

//------------------------------------------------------------------------------
// MCP_Reg16_Status_Update_Map:
// It is 32-bit read-write register. It is updated by Enable/Disable Status
// Command (#regaddr 0/1\n). It is read by F/W to select individual registers
// during status update.
// Values:
//     Each bit enables/disables one of 16-bit register for status update.
// WARNING: Only 12 registers are defined in this revision of firmware.
//------------------------------------------------------------------------------
//volatile U32 data MCP_reg16_status_update_map;
volatile U8 data MCP_reg16_status_update_map[4];

//------------------------------------------------------------------------------
// MCP_Reg8_Dirty_Bit_Map:
// It is 16-bit read-write register. Each bit represents one of 8-bit registers.
// A particular bit is set when corresponding register is written/updated. It
// is cleared when corresponding register value is sent during status update.
//
// WARNING: Only 9 registers are defined in this revision of firmware.
//------------------------------------------------------------------------------
volatile U16 data MCP_reg8_dirty_bit_map;

//------------------------------------------------------------------------------
// MCP_Reg16_Dirty_Bit_Map:
// It is 32-bit read-write register. Each bit represents one of 16-bit registers
// A particular bit is set when corresponding register is written/updated. It
// is cleared when corresponding register value is sent during status update.
//
// WARNING: Only 12 registers are defined in this revision of firmware.
//------------------------------------------------------------------------------
//volatile U32 data MCP_reg16_dirty_bit_map;
volatile U8 data MCP_reg16_dirty_bit_map[4];

//------------------------------------------------------------------------------
// MCP_Reg8_Access_Type:
// It is 16-bit read-only register. It is updated only at build time. It is read
// by F/W to check if an 8-bit register is writable.
// Values:
//     0: Corresponding register is read-only
//     1: Corresponding register is read-write
// If source of direction is not Controller then restrict 'target direction'
// register to read-only operations
// WARNING: Only 9 registers are defined in this revision of firmware.
//------------------------------------------------------------------------------
code U16 MCP_reg8_access_type = 0x0074 & ~(0 << 2);

//------------------------------------------------------------------------------
// MCP_Reg16_Access_Type:
// It is 32-bit read-only register. It is updated only at build time. It is read
// by F/W to check if an 16-bit register is writable.
// Values:
//     0: Corresponding register is read-only
//     1: Corresponding register is read-write
// If speed is not being controlled then restrict 'maximum operational speed'
// register to read-only operations
// If speed is not being controlled or it is being controlled but its source
// of control is not Controller then restrict 'target motor speed' register
// to read-only operations
// If PWM is not being controlled then restrict 'target PWM duty cycle'
// register to read-only operations
// WARNING: Only 12 registers are defined in this revision of firmware.
//------------------------------------------------------------------------------
//code U32 MCP_reg16_access_type = 0x00000FE0 &
//             ~(BLDC_RD_RPM_OR_PWM << 5) &
//             ~(((BLDC_RD_RPM_OR_PWM == 1) | (BLDC_RD_RPM_PWM_SRC != 0)) << 6) &
//             ~(((BLDC_RD_RPM_OR_PWM == 0) | (BLDC_RD_RPM_PWM_SRC != 0)) << 7);

code U8 MCP_reg16_access_type[4] =
{
    0xE0 &
    ~(BLDC_RD_RPM_OR_PWM << 5) &
    ~(((BLDC_RD_RPM_OR_PWM == PWM_PARAMETER)) << 6) &
    ~(((BLDC_RD_RPM_OR_PWM == RPM_PARAMETER)) << 7),
    0x0F,
    0x00,
    0x00
};

//------------------------------------------------------------------------------
// BLDC Register Map - 8-bit registers
// It contains 48 registers (Addresses 0x00 to 0x2F) but only few are allocated.
//
// Post Review 1: All 8-bit registers are grouped together into an array and
// are no longer mapped to a fixed memory address (7 dummy registers).
//------------------------------------------------------------------------------
volatile U8 MCP_REG_MEM_TYPE    MCP_8_bit_registers[16];

//------------------------------------------------------------------------------
// BLDC Register Map - 16-bit registers
// It contains 104 registers (Addresses 0x30 to 0xFE) but only few are allocated.
//
// Post Review 1: All 16-bit registers are grouped together into an array and
// are no longer mapped to a fixed memory address (4 dummy registers).
//------------------------------------------------------------------------------
volatile U16 MCP_REG_MEM_TYPE    MCP_16_bit_registers[16];

//------------------------------------------------------------------------------
// Local Functions Prototypes
//------------------------------------------------------------------------------
static bit MCP_check_address (U8 address);

//------------------------------------------------------------------------------
// APIs
//------------------------------------------------------------------------------

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
void MCP_init_registers (void)
{
    //8-bit register default settings
    MCP_8_bit_registers[MCP_REG_FIRMWARE_REVISION] = BLDC_RD_FW_VERSION;
    MCP_8_bit_registers[MCP_REG_PRESENT_MOTOR_DIRECTION] = BLDC_RD_DEFAULT_DIR;
    MCP_8_bit_registers[MCP_REG_TARGET_MOTOR_DIRECTION] = BLDC_RD_DEFAULT_DIR;
    // no more DIRECTION_BY_HOST.
    MCP_8_bit_registers[MCP_REG_MOTOR_CONTROL_SOURCE] =
                                           ((BLDC_RD_RPM_PWM_SRC & 0x03) << 0) |
                                           ((BLDC_RD_RPM_OR_PWM & 0x01) << 3)  |
                                           ((0x01) << 4);

    MCP_8_bit_registers[MCP_REG_OVER_CURRENT_THRESHOLD] = BLDC_RD_CURRENT_TH;
    MCP_8_bit_registers[MCP_REG_OVER_CURRENT_PERSISTANCE] =
                                                          BLDC_RD_CURRENT_COUNT;
    MCP_8_bit_registers[MCP_REG_MOTOR_POLE_PAIRS_COUNT] = BLDC_RD_NUM_POLES/2;
    MCP_8_bit_registers[MCP_REG_PWM_SWITCHING] = BLDC_RD_PWM_METHOD;
    MCP_8_bit_registers[MCP_REG_COMMUTATION_METHOD] = BLDC_RD_COMMUT_METHOD;

    //16-bit register default settings
    MCP_16_bit_registers[INDEX_PRESENT_MOTOR_SPEED] = 0;
    MCP_16_bit_registers[INDEX_MOTOR_COIL_CURRENT] = 0;
    MCP_16_bit_registers[INDEX_MOTOR_OPERATING_VOLTAGE] = 0;
    MCP_16_bit_registers[INDEX_ACTIVE_PWM_DUTY_CYCLE] = 0;
    MCP_16_bit_registers[INDEX_CURRENT_SAMPLING_PERIOD] =
                                                BLDC_RD_CURRENT_SAMPLING_PERIOD;
    MCP_16_bit_registers[INDEX_MAXIMUM_OPERATIONAL_SPEED] =
                                               BLDC_RD_MAXIMUM_OPERAIONAL_SPEED;
    MCP_16_bit_registers[INDEX_TARGET_MOTOR_SPEED] = 0;
    MCP_16_bit_registers[INDEX_TARGET_PWM_DUTY_CYCLE] = 0;
    MCP_16_bit_registers[INDEX_ACCELERATION_STEP_SIZE] =
                                                 BLDC_RD_ACCELERATION_STEP_SIZE;
    MCP_16_bit_registers[INDEX_DECELERATION_STEP_SIZE] =
                                                 BLDC_RD_DECELERATION_STEP_SIZE;
    MCP_16_bit_registers[INDEX_PROPORTIONAL_GAIN] =
                                                   BLDC_RD_PI_PROPORTIONAL_GAIN;

    MCP_16_bit_registers[INDEX_INTEGRAL_GAIN] = BLDC_RD_PI_INTEGRAL_GAIN;
}

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
//     register is received from controller. It can also be called internally
//     by other components of F/W. If register is reserved/undefined then it
//     returns 1. For a valid register, it returns 1 if it is read-only and
//     returns 0 otherwise. A read-only register is read-only for external
//     access via Write Register Command. All registers are writable internally
//     by F/W. When a register is written (modified), corresponding dirty bit
//     is also set.
//------------------------------------------------------------------------------
bit MCP_set_8bit_register (U8 address, U8 value, bit external_access)
{
    U16 temp16;

    if (MCP_REG_RESERVED_START <= address)
    {
        return 1;
    }

    temp16 = ((U16)1 << address);

    if (external_access && (0 == (MCP_reg8_access_type & temp16)))
    {
        return 1;
    }

    MCP_8_bit_registers[address] = value;

    MCP_reg8_dirty_bit_map |= temp16;

    return 0;
}

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
//     register is received from controller. It can also be called internally
//     by other components of F/W. If register is reserved/undefined then it
//     returns 1. For a valid register, it returns 1 if it is read-only and
//     returns 0 otherwise. A read-only register is read-only for external
//     access via Write Register Command. All registers are writable internally
//     by F/W. When a register is written (modified), corresponding dirty bit
//     is also set.
//
//------------------------------------------------------------------------------
bit MCP_set_16bit_register (U8 address, U16 value, bit external_access)
{
    //This bit is used by MCP_ATOMIC_ACCESS_START and MCP_ATOMIC_ACCESS_END
    bit int_status;
    U16 MCP_REG_MEM_TYPE *ptr;
    U8 byte_num;
    U8 bit_mask;

    if (MCP_REG_UNDEFINED_START <= address)
    {
        return 1;
    }

    address = (address - MCP_REG_PRESENT_MOTOR_SPEED) >> 1;

    ptr = &MCP_16_bit_registers[address];

    byte_num = (address & 0xF8) >> 3;
    bit_mask = 1 << (address & 0x07);

    if (external_access && (0 == (MCP_reg16_access_type[byte_num] & (bit_mask))))
    {
        return 1;
    }

    MCP_ATOMIC_ACCESS_START();

    *ptr = value;

    MCP_ATOMIC_ACCESS_END();

    MCP_reg16_dirty_bit_map[byte_num] |= (bit_mask);

    return 0;
}

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
//     registers. When a register is written (modified), corresponding dirty bit
//     is also set.
//
//------------------------------------------------------------------------------
void MCP_set_8bit_register_isr (U8 address, U8 value)
{
    MCP_8_bit_registers[address] = value;

    MCP_reg8_dirty_bit_map |= ((U16)1 << address);
}

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
//     registers. When a register is written (modified), corresponding dirty bit
//     is also set.
//
//------------------------------------------------------------------------------
void MCP_set_16bit_register_isr (U8 address, U16 value)
{
    U16 MCP_REG_MEM_TYPE *ptr;
    U8 byte_num;
    U8 bit_mask;

    address = (address - MCP_REG_PRESENT_MOTOR_SPEED) >> 1;

    ptr = &MCP_16_bit_registers[address];

    byte_num = (address & 0xF8) >> 3;
    bit_mask = 1 << (address & 0x07);

    IE_EA = 0;

    *ptr = value;

    IE_EA = 1;

    MCP_reg16_dirty_bit_map[byte_num] |= (bit_mask);
}

//------------------------------------------------------------------------------
// MCP_get_8bit_register:
// U8 MCP_get_8bit_register (U8 address)
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
U8 MCP_get_8bit_register (U8 address)
{
    return(MCP_8_bit_registers[address]);
}

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
U16 MCP_get_16bit_register (U8 address)
{
    //This bit is used by MCP_ATOMIC_ACCESS_START and MCP_ATOMIC_ACCESS_END
    bit int_status;
    U16 value;
    U16 MCP_REG_MEM_TYPE *ptr;

    ptr = &MCP_16_bit_registers[(address - MCP_REG_PRESENT_MOTOR_SPEED) >> 1];

    MCP_ATOMIC_ACCESS_START();

    value = *ptr;

    MCP_ATOMIC_ACCESS_END();

    return(value);
}

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
U8 MCP_get_8bit_register_isr (U8 address)
{
    return(MCP_8_bit_registers[address]);
}

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
U16 MCP_get_16bit_register_isr (U8 address)
{
    U16 value;
    U16 MCP_REG_MEM_TYPE *ptr;

    ptr = &MCP_16_bit_registers[(address - MCP_REG_PRESENT_MOTOR_SPEED) >> 1];

    IE_EA = 0;

    value = *ptr;

    IE_EA = 1;

    return(value);
}

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
bit MCP_status_update_register (U8 address, bit enable)
{
    U16 temp16;
    U8 byte_num;
    U8 bit_mask;

    if (MCP_check_address(address))
    {
        return 1;
    }

    temp16 = (U16)1 << address;

    if (MCP_REG_PRESENT_MOTOR_SPEED > address)
    {
        if (enable)
        {
            MCP_reg8_status_update_map |= temp16;
            //MCP_reg8_dirty_bit_map     |= ((U16)1 << address);
        }
        else
        {
            MCP_reg8_status_update_map &= ~(temp16);
            //MCP_reg8_dirty_bit_map     &= ~((U16)1 << address);
        }
    }
    // This check avoids a potential problem. If address is 0x70 or above then
    // byte_num becomes 4 and therefore, MCP_reg16_status_update_map[byte_num]
    // will reference MCP_reg8_dirty_bit_map.
    else if (MCP_REG_UNDEFINED_START > address)
    {
        address = (address - MCP_REG_PRESENT_MOTOR_SPEED) >> 1;

        byte_num = (address & 0xF8) >> 3;
        bit_mask = 1 << (address & 0x07);

        if (enable)
        {
            MCP_reg16_status_update_map[byte_num] |= (bit_mask);
            //MCP_reg16_dirty_bit_map     |= ((U32)1 << address);
        }
        else
        {
            MCP_reg16_status_update_map[byte_num] &= ~(bit_mask);
            //MCP_reg16_dirty_bit_map      &= ~((U16)1 << address);
        }
    }
    return 0;
}

//------------------------------------------------------------------------------
// MCP_is_8bit_status_update_enabled:
// bit MCP_is_8bit_status_update_enabled (U8 address)
//
// Return Value:
//     0: Register is not enabled for status update
//     1: Register is enabled for status update
//
// Parameters:
//     address: A unique ID of an 8 bit MCP register
//
// Description:
//     This function is called by MCP during Status Update in binary mode to
//     check register can be read and included in status update process.
//
//------------------------------------------------------------------------------
bit MCP_is_8bit_status_update_enabled (U8 address)
{
    U16 temp16;

    temp16 = (U16)1 << address;

    if ((MCP_reg8_status_update_map & (temp16))  &&
        (MCP_reg8_dirty_bit_map     & (temp16)))
    {
        MCP_reg8_dirty_bit_map &= ~(temp16);
        return 1;
    }

    return 0;
}

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
bit MCP_is_16bit_status_update_enabled (U8 address)
{
    U8 byte_num;
    U8 bit_mask;

    address = (address - MCP_REG_PRESENT_MOTOR_SPEED) >> 1;

    byte_num = (address & 0xF8) >> 3;
    bit_mask = 1 << (address & 0x07);

    if ((MCP_reg16_status_update_map[byte_num] & (bit_mask))  &&
        (MCP_reg16_dirty_bit_map[byte_num] & (bit_mask)))
    {
        MCP_reg16_dirty_bit_map[byte_num] &= ~(bit_mask);
        return 1;
    }

    return 0;
}

//------------------------------------------------------------------------------
// Local Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// MCP_check_address:
// static bit MCP_check_address (U8)
//
// Return Value:
//     0: if address of supported register is passed
//     1: if address of unsupported register is passed
//
// Parameters:
//     Address of MCP register
//
// Description:
//     It checks if the register referred by address is supported or not.
//------------------------------------------------------------------------------
static bit MCP_check_address (U8 address)
{
    if (((MCP_REG_RESERVED_START <= address)
                       &&
        (MCP_REG_RESERVED_END    >= address))
                       ||
        (MCP_REG_UNDEFINED_START <= address))
    {
        return 1;
    }

    return 0;
}

//------------------------------------------------------------------------------
// End Of File
//------------------------------------------------------------------------------
