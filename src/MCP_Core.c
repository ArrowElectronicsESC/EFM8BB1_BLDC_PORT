//------------------------------------------------------------------------------
// MCP_Core.c
//------------------------------------------------------------------------------
// Copyright (C) 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Description:
//
// This file implements core of MCP. It receives commands from controller over
// UART interface, parse and process them. Most of commands read or write MCP
// registers. It also sends response and/or MCP register values to controller
// over UART interface. It also support some non-register commands like mode
// switching commands. MCP can operate in two modes: Text Mode or Binary Mode.
// Default mode is Text Mode. It can switch from one mode to other. Each mode
// can process a set of commands. Unsupported commands are responded with ERROR.
//
// Release 0.0 - April 12, 2013 mufayyuz
//    -Initial Revision.
//

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "bldcdk.h"

#ifdef BUILD_FOR_PROTOCOL

//#define PRINT_IN_HEX
#define PRINT_IN_DEC

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------
#define MCP_TEXT_MODE   0
#define MCP_BINARY_MODE 1

#define MCP_DEBUG_BUF_SIZE 32

#define MCP_STATUS_BUF_SIZE 16

#define MCP_COMMAND_SIZE 20
#define MCP_COMMAND_TYPE_INDEX 0
#define MCP_COMMAND_ADDR_INDEX 1
#define MCP_COMMAND_DATA_INDEX 4

//------------------------------------------------------------------------------
// DATA TYPES
//------------------------------------------------------------------------------
typedef enum _cmd
{
    MCP_COMMAND_NULL,
    MCP_COMMAND_RESET,
    MCP_COMMAND_TEXT_MODE,
    MCP_COMMAND_BIN_MODE,
    MCP_COMMAND_WRITE_REG,
    MCP_COMMAND_READ_REG,
    MCP_COMMAND_ENABLE_UPDATE,
    MCP_COMMAND_TUNE_PID,
    /*TODO: add more if needed*/
    MCP_COMMAND_INVALID,
} mcp_command_t;

//------------------------------------------------------------------------------
// Local Variables
//------------------------------------------------------------------------------
static U8 MCP_mode;
static mcp_command_t MCP_cmd_type;

#ifdef BUILD_FOR_DEBUG
#ifdef PRINT_IN_HEX
static U8 code MCP_aschextable[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                                    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
#endif
#endif

//------------------------------------------------------------------------------
// MCP_Command[] has a capacity of only one command. Any data received during
// processing of last received command will be ignored.
//------------------------------------------------------------------------------
static U8 SEG_IDATA MCP_command[MCP_COMMAND_SIZE];
static U8 SEG_IDATA *MCP_command_ptr;
static U8 SEG_IDATA *MCP_command_ptr_2;

// Buffer used for ASCII <-> number conversions
static U8 SEG_IDATA MCP_numcnvbuf[10];

#define MCP_INIT_CMDPTR()   do { MCP_command_ptr = &MCP_command[0]; } while (0)

#ifdef BUILD_FOR_DEBUG
static U8 MCP_debug_head;
static U8 MCP_debug_tail;
static U8 SEG_XDATA MCP_debug_buffer[MCP_DEBUG_BUF_SIZE];

#define MCP_debug_add(b)   do {     \
    *(U8 SEG_XDATA *)MCP_debug_tail = (b);  \
} while (0)

#define MCP_debug_get()   *(U8 SEG_XDATA *)MCP_debug_head

#define MCP_debug_INCPTR(p)    do { \
        (p)++;  \
        if ((p) == (U8)&MCP_debug_buffer[MCP_DEBUG_BUF_SIZE])   \
            (p) = (U8)&MCP_debug_buffer[0];    \
} while (0)
#endif

static U8 MCP_status_head;
static U8 MCP_status_tail;
static U8 SEG_XDATA MCP_status_buffer[MCP_STATUS_BUF_SIZE];

#define MCP_status_add(b)   do {    \
    *(U8 SEG_XDATA *)MCP_status_tail = (b); \
} while (0)

#define MCP_status_get()   *(U8 SEG_XDATA *)MCP_status_head

#define MCP_status_INCPTR(p)    do {    \
        (p)++;  \
        if ((p) == (U8)&MCP_status_buffer[MCP_STATUS_BUF_SIZE]) \
            (p) = (U8)&MCP_status_buffer[0];    \
} while (0)

static bit MCP_txqueue_lock;

static bit MCP_temp1;
static bit MCP_reset_cmd;
static U8 SEG_DATA MCP_temp8_1;
static U8 SEG_DATA MCP_temp8_2;
static U16 SEG_DATA MCP_temp16_1;
static U16 SEG_DATA MCP_temp16_2;

struct pt SEG_XDATA mcp;
struct pt SEG_XDATA mcp_tune;
struct pt SEG_XDATA mcp_status;

#ifdef BUILD_FOR_DEBUG
struct pt SEG_XDATA mcp_debug;
#endif

//Pointers to call back functions
U8 (* MCP_tune_callback) (U16, U16);
bit (* MCP_reset_callback) (void);

//------------------------------------------------------------------------------
// Local Functions Prototypes
//------------------------------------------------------------------------------
static bit MCP_receive_command (void);
static bit MCP_get_register_address(void);
static bit MCP_process_command (void);
static void MCP_update_status (void);
static void MCP_send_ascii_value (U16);
static void MCP_init_commandbuf(void);
static U8 MCP_dec_to_ascnumbuf(U32 value);

static U8 MCP_command_task(void);
static U8 MCP_status_update_task(void);

#ifdef BUILD_FOR_DEBUG
static U8 MCP_print_task(void);
#endif

//------------------------------------------------------------------------------
// APIs
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// MCP_init:
// void MCP_init (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     This function initializes MCP core and MCP registers. It should be called
//     by application at startup.
//------------------------------------------------------------------------------
void MCP_init (void)
{
    MCP_reset_cmd = 0;
    MCP_mode = MCP_TEXT_MODE;
    MCP_cmd_type = MCP_COMMAND_INVALID;

    MCP_init_commandbuf();
    MCP_command_ptr_2 = MCP_command_ptr;

#ifdef BUILD_FOR_DEBUG
    MCP_debug_head = (U8)&MCP_debug_buffer[0];
    MCP_debug_tail = MCP_debug_head;
#endif

    MCP_status_head = (U8)&MCP_status_buffer[0];
    MCP_status_tail = MCP_status_head;

    MCP_txqueue_lock = 0;

    PT_INIT(&mcp);

#ifdef BUILD_FOR_DEBUG
    PT_INIT(&mcp_debug);
#endif

    PT_INIT(&mcp_status);

    MCP_init_registers();

}

//------------------------------------------------------------------------------
// MCP_task:
// void MCP_task (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     It is supposed to run forever. It should be called repeatedly by
//     application at regular interval. It runs different threads.
//------------------------------------------------------------------------------
void MCP_task (void)
{
    MCP_command_task();
    MCP_status_update_task();

#ifdef BUILD_FOR_DEBUG
    MCP_print_task();
#endif
}

//------------------------------------------------------------------------------
// MCP Debug Print
// void MCP_debug_Print (const U8 *str, U32 value)
//
// Return Value:
//     None
//
// Parameters:
//     Debug string/message to be printed.
//
// Description:
//     This function can be used by application to print simple debug messages.
//     It does not support printing values. Each message is started with 0xFF to
//     indicate that it is debug message so that terminal program/console can
//     distinguish it from MCP responses.
//
//------------------------------------------------------------------------------
#ifdef BUILD_FOR_DEBUG

void MCP_debug_print (const U8 * data str, U32 value)
{
    U8 idx;

    //This indicates that following string as a debug message
    MCP_debug_add(0xFF);
    MCP_debug_INCPTR(MCP_debug_tail);

    while (0 != *str)
    {
        if ('%' == *str)
        {

#ifdef PRINT_IN_HEX
            if ('x' == *(str + 1))
            {
                str += 2;

                MCP_debug_add('0');
                MCP_debug_INCPTR(MCP_debug_tail);
                MCP_debug_add('x');
                MCP_debug_INCPTR(MCP_debug_tail);

                // Convert it to hex number format
                // in reverse byte order
                for (idx = 0; idx < 10; idx++)
                {
                    MCP_numcnvbuf[idx] = value & 0x0f;
                    value >>= 4;
                    if (0 == value)
                    {
                        break;
                    }
                }

                for (; idx != 0xff; idx--)
                {
                    MCP_debug_add(MCP_aschextable[MCP_numcnvbuf[idx]]);
                    MCP_debug_INCPTR(MCP_debug_tail);
                }
                continue;   // Skip incrementing the index
            }
            else
#endif //PRINT_IN_HEX

#ifdef PRINT_IN_DEC
            if ('d' == *(str + 1))
            {
                str += 2;

                // Convert it to hex number format
                // in reverse byte order
                idx = MCP_dec_to_ascnumbuf(value);

                for (; idx != 0xff; idx--)
                {
                    MCP_debug_add(MCP_numcnvbuf[idx]);
                    MCP_debug_INCPTR(MCP_debug_tail);
                }
                continue;   // Skip incrementing the index
            }
            else
#endif //PRINT_IN_DEC
            {
                    str += 2;
            }
        }
        else
        {
            MCP_debug_add(*str);
            str++;
        }

        MCP_debug_INCPTR(MCP_debug_tail);
    }
}
#endif

//------------------------------------------------------------------------------
// Local Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// MCP_dec_to_ascnumbuf:
// static U8 MCP_dec_to_ascnumbuf(U32 value)
//
// Return Value:
//     index into last used byte in MCP_numcnvbuf
//
// Parameters:
//     value - value to be converted to ASCII decimal number
//
// Description:
//     This function converts a decimal value into an ASCII string in reverse
//     order and stores it in MCP_numcnvbuf[]. The caller can read it downwards
//     using returned index.
//
//------------------------------------------------------------------------------
static U8 MCP_dec_to_ascnumbuf(U32 value)
{
    U8 idx;
    U8 bval;

    // Convert it to hex number format in reverse byte order
    for (idx = 0; idx < 10; idx++)
    {
        bval = value % 10;
        MCP_numcnvbuf[idx] = bval + '0';
        value /= 10;
        if (0 == value)
        {
            break;
        }
    }

    return idx;
}

//------------------------------------------------------------------------------
// MCP_init_commandbuf:
// static void MCP_init_commandbuf (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     This function initializes MCP command buffer to zero.  It clears the old
//     command and allows new commands to be read and processed. It should be
//     called when previous command is processed completely.
//------------------------------------------------------------------------------
static void MCP_init_commandbuf(void)
{
    for (MCP_command_ptr = &MCP_command[0];
         MCP_command_ptr != &MCP_command[MCP_COMMAND_SIZE]; ++MCP_command_ptr)
    {
        *MCP_command_ptr = 0;
    }

    MCP_command_ptr = &MCP_command[0];
}

//------------------------------------------------------------------------------
// MCP_command_task:
// void MCP_command_task (void)
//
// Return Value:
//     Proto-thread state
//         0: PT_WAITING
//         1: PT_YIELDED
//         2: PT_EXITED
//         3: PT_ENDED
//
// Parameters:
//     None
//
// Description:
//     This is the command handling thread. It is supposed to run forever. It is
//     called by MCP_task() which is run repeatedly by application at regular
//     interval. It waits for command. Once complete command is received, it
//     processes it and sends response if required. Command processing is mode
//     dependent.
//------------------------------------------------------------------------------
static U8 MCP_command_task (void)
{
    PT_BEGIN(&mcp);

    while (1)
    {
        PT_WAIT_UNTIL(&mcp, (0 == MCP_receive_command()));

        if (MCP_BINARY_MODE == MCP_mode)
        {
            MCP_INIT_CMDPTR();

            //Only one command is accepted in BIN mode - switch to TEXT mode
            if (MCP_COMMAND_TEXT_MODE == MCP_cmd_type)
            {
                MCP_process_command();
            }
        }
        else if (MCP_TEXT_MODE == MCP_mode)
        {
            //Perform command specific action
            if (0 == MCP_process_command())
            {
#ifdef FEATURE_PID_TUNE_FUNCTION
                //TUNE PID has a special handling. Response is not sent immediately.
                if (MCP_COMMAND_TUNE_PID == MCP_cmd_type)
                {
                    //Wait here until call-back function returns 0.
                    PT_SPAWN(&mcp, &mcp_tune, MCP_tune_callback(MCP_temp16_1,
                                                                 MCP_temp16_2));

                    MCP_INIT_CMDPTR();
                    MCP_temp16_1 =
                              MCP_get_16bit_register(MCP_REG_PROPORTIONAL_GAIN);
                    MCP_send_ascii_value(MCP_temp16_1);
                    *MCP_command_ptr++ = ',';

                    MCP_temp16_1 =
                                  MCP_get_16bit_register(MCP_REG_INTEGRAL_GAIN);
                    MCP_send_ascii_value(MCP_temp16_1);
                    *MCP_command_ptr++ = '\n';
                }
#endif
                //If command is switch to BIN mode, do not send "OK\n"
                if (MCP_COMMAND_BIN_MODE != MCP_cmd_type)
                {
                    *MCP_command_ptr++ = 'O';
                    *MCP_command_ptr++ = 'K';
                    *MCP_command_ptr++ = '\n';
                }
            }
            else
            {
                *MCP_command_ptr++ = 'E';
                *MCP_command_ptr++ = 'R';
                *MCP_command_ptr++ = 'R';
                *MCP_command_ptr++ = 'O';
                *MCP_command_ptr++ = 'R';
                *MCP_command_ptr++ = '\n';
            }
        }

        //Command is processed and response is prepared. Now send the response.
        if (MCP_COMMAND_TEXT_MODE != MCP_cmd_type)
        {
            MCP_command_ptr_2 = &MCP_command[0];

            PT_WAIT_UNTIL(&mcp, (0 == MCP_txqueue_lock));
            MCP_txqueue_lock = 1;   // Lock the UART tx queue

            while (MCP_command_ptr_2 != MCP_command_ptr)
            {
                PT_WAIT_UNTIL(&mcp, (1 < UART_get_tx_buffer_free_bytes()));
                UART_send_byte(*MCP_command_ptr_2);
                MCP_command_ptr_2++;
            }
            MCP_txqueue_lock = 0;
        }

        //Last command was RST command
        if (1 == MCP_reset_cmd)
        {
            PT_WAIT_UNTIL(&mcp, (UART_BUF_SIZE == UART_get_tx_buffer_free_bytes()));

            MCP_temp8_1 = 0;
            while (254 > MCP_temp8_1)
            {
                MCP_temp8_1++;
            }

            RSTSRC = 0x10;
            while(1);
        }

        //Command has been processed. Clear last command and accept new.
        MCP_init_commandbuf();

        //Allow next command to be processed
        MCP_cmd_type = MCP_COMMAND_INVALID;
    }

    PT_END(&mcp);
}


//------------------------------------------------------------------------------
// MCP_status_update_task:
// static U8 MCP_status_update_task (void)
//
// Return Value:
//     Proto-thread state
//         0: PT_WAITING
//         1: PT_YIELDED
//         2: PT_EXITED
//         3: PT_ENDED
//
// Parameters:
//     None
//
// Description:
//     This is the status update thread. It is supposed to run forever. It is
//     called by MCP_task() which is run repeatedly by application at regular
//     interval. In binary mode, it prepares data to be sent during status
//     update process. Then it sends status data if any.
//------------------------------------------------------------------------------
static U8 MCP_status_update_task(void)
{
    if (MCP_BINARY_MODE == MCP_mode)
    {
        MCP_update_status();
    }

    PT_BEGIN(&mcp_status);

    while (1)
    {
        PT_WAIT_UNTIL(&mcp_status, (MCP_status_tail != MCP_status_head) && (0 == MCP_txqueue_lock));

        MCP_txqueue_lock = 1;    // Lock the UART tx queue
        while (MCP_status_tail != MCP_status_head)
        {
            PT_WAIT_UNTIL(&mcp_status, (1 < UART_get_tx_buffer_free_bytes()));
            UART_send_byte(MCP_status_get());
            MCP_status_INCPTR(MCP_status_head);
        }
        MCP_txqueue_lock = 0;
    }
    PT_END(&mcp_status);
}

#ifdef BUILD_FOR_DEBUG
//------------------------------------------------------------------------------
// MCP_print_task:
// static void MCP_print_task (void)
//
// Return Value:
//     Proto-thread state
//         0: PT_WAITING
//         1: PT_YIELDED
//         2: PT_EXITED
//         3: PT_ENDED
//
// Parameters:
//     None
//
// Description:
//     This is the printing thread. It is supposed to run forever. It is called
//     by MCP_task() which is run repeatedly by application at regular interval.
//     It waits until there is something to print.
//------------------------------------------------------------------------------
static U8 MCP_print_task(void)
{
    PT_BEGIN(&mcp_debug);

    while (1)
    {
        PT_WAIT_UNTIL(&mcp_debug, (MCP_debug_tail != MCP_debug_head) &&
                (0 == MCP_txqueue_lock));

        MCP_txqueue_lock = 1;   // Lock the UART tx queue
        while (MCP_debug_tail != MCP_debug_head)
        {
            PT_WAIT_UNTIL(&mcp_debug, (1 < UART_get_tx_buffer_free_bytes()));
            UART_send_byte(MCP_debug_get());
            MCP_debug_INCPTR(MCP_debug_head);
        }
        MCP_txqueue_lock = 0;
    }

    PT_END(&mcp_debug);
    return PT_ENDED;
}

#endif //BUILD_FOR_DEBUG

//------------------------------------------------------------------------------
// MCP_receive_command
// static bit MCP_receive_command (void)
//
// Return Value:
//     0: Command received completely
//     1: Command not received
//
// Parameters:
//     None
//
// Description:
//     It is uses UART API to get received byte to accumulate full command.
//     Command is considered completely received if '\n' is received (because
//     most of commands are terminated by '\n'). However, mode switch commands
//     are not terminated by '\n' and therefore handled separately.
//
//------------------------------------------------------------------------------
static bit MCP_receive_command (void)
{
    if (UART_SUCCESS == UART_receive_byte(&MCP_temp8_1))
    {
        *MCP_command_ptr = MCP_temp8_1;

        if (MCP_command_ptr == &MCP_command[MCP_COMMAND_TYPE_INDEX + 2])
        {
            if (('+' == MCP_command[MCP_COMMAND_TYPE_INDEX + 0]) &&
                ('+' == MCP_command[MCP_COMMAND_TYPE_INDEX + 1]) &&
                ('+' == MCP_command[MCP_COMMAND_TYPE_INDEX + 2]))
            {
                MCP_cmd_type = MCP_COMMAND_TEXT_MODE;
                return 0;
            }
            else if (('^' == MCP_command[MCP_COMMAND_TYPE_INDEX + 0]) &&
                     ('^' == MCP_command[MCP_COMMAND_TYPE_INDEX + 1]) &&
                     ('^' == MCP_command[MCP_COMMAND_TYPE_INDEX + 2]))
            {
                MCP_cmd_type = MCP_COMMAND_BIN_MODE;
                return 0;
            }
        }
        else if ('\n' == *MCP_command_ptr)
        {
            return 0;
        }

        MCP_command_ptr++;
        // something wrong...(maybe wrong uart baud rate in PC side)
        if( MCP_command_ptr == &MCP_command[MCP_COMMAND_SIZE])
        {
            // stop motor first.
            MCP_reset_callback();
            RSTSRC = 0x10;
            while(1);
        }
    }
    return 1;
}

//------------------------------------------------------------------------------
// MCP_hex_to_dec
// static U8 MCP_hex_to_dec (U8 aschex)
//
// Return Value:
//     0x00 to 0x0F: If input is a valid ASCII Hex number.
//     0xFF: If input is not a valid ASCII Hex number.
//
// Parameters:
//     Ascii Hex digit.
//
// Description:
//     It converts ACSII Hex digit to corresponding decimal value. It checks if
//     input is a valid ASCII Hex digit or not. If valid, it returns a valid
//     number between 0x00 and 0x0F otherwise it returns 0xFF.
//     Valid ASCII hex digits are 0, 1, ..,9, A, B, ..., F, a, b, ..., f.
//------------------------------------------------------------------------------
static U8 MCP_hex_to_dec(U8 aschex)
{
    if (('0' <= aschex) && ('9' >= aschex))
    {
        return aschex - '0';
    }
    else if (('a' <= aschex) && ('f' >= aschex))
    {
        return (aschex - 'a' + 10);
    }
    else if (('A' <= aschex) && ('F' >= aschex))
    {
        return (aschex - 'A' + 10);
    }

    return 0xFF;
}

//------------------------------------------------------------------------------
// MCP_get_register_address
// static bit MCP_get_register_address (void)
//
// Return Value:
//     0: If address field in the command represents a valid Hex value
//     1: If address field in the command does not represent valid Hex value
//
// Parameters:
//     None
//
// Description:
//     It checks both ASCII bytes of address field in the command. If both bytes
//     are valid ASCII Hex values then it constructs a decimal value from them
//     and store it in MCP_temp8_1.
//------------------------------------------------------------------------------
static bit MCP_get_register_address(void)
{
    U8 temp_byte;

    temp_byte = MCP_hex_to_dec(MCP_command[MCP_COMMAND_ADDR_INDEX + 0]);
    if (0xFF != temp_byte)
    {
        MCP_temp8_1 = temp_byte << 4;
        temp_byte = MCP_hex_to_dec(MCP_command[MCP_COMMAND_ADDR_INDEX + 1]);
        if (0xFF != temp_byte)
        {
            MCP_temp8_1 += temp_byte;
            return 0;
        }
    }

    return 1;
}

//------------------------------------------------------------------------------
// MCP_process_command
// static bit MCP_process_command (void)
//
// Return Value:
//     0: Command successfully processed
//     1: Command processing failed
//
// Parameters:
//     None
//
// Description:
//     It is parses and processes the received command by performing command
//     specific operation. It also prepares response if required by the command.
//
//------------------------------------------------------------------------------
static bit MCP_process_command (void)
{
    U8 bval;
    U8 digit_count;

    MCP_temp16_1 = 0;

    MCP_temp1 = MCP_get_register_address();

    if ('\n' == MCP_command[MCP_COMMAND_TYPE_INDEX])
    {
        MCP_temp16_1 = MCP_get_8bit_register(MCP_REG_FIRMWARE_REVISION);

        MCP_command[0] = 'B';
        MCP_command[1] = 'L';
        MCP_command[2] = 'D';
        MCP_command[3] = 'C';
        MCP_command[4] = 'R';
        MCP_command[5] = 'E';
        MCP_command[6] = 'F';
        MCP_command[7] = ' ';

        MCP_command_ptr = &MCP_command[8];
        MCP_send_ascii_value(MCP_temp16_1);
        *MCP_command_ptr++ = '\n';

        return 0;
    }
    else if (('R' == MCP_command[MCP_COMMAND_TYPE_INDEX + 0]) &&
             ('S' == MCP_command[MCP_COMMAND_TYPE_INDEX + 1]) &&
             ('T' == MCP_command[MCP_COMMAND_TYPE_INDEX + 2]))
    {
        // MCP_reset_callback() will generate software_reset(SWRSF).
        // MCP_init_registers() called during booting time.
        MCP_INIT_CMDPTR();
        MCP_reset_callback();

        MCP_reset_cmd = 1;

//        MCP_init_registers();
        return 0;
    }
    else if (MCP_COMMAND_TEXT_MODE == MCP_cmd_type)
    {
        //Switch to TEXT mode
        MCP_mode = MCP_TEXT_MODE;
        return 0;
    }
    else if (MCP_COMMAND_BIN_MODE == MCP_cmd_type)
    {
        //Switch to BIN mode
        MCP_mode = MCP_BINARY_MODE;
        MCP_INIT_CMDPTR();
        *MCP_command_ptr++ = 0x00;

        //First register for status update process
        MCP_temp8_2 = MCP_REG_FIRMWARE_REVISION;
        return 0;
    }
    else if ('=' == MCP_command[MCP_COMMAND_TYPE_INDEX])
    {
        MCP_temp16_1 = 0;
        digit_count = 0;
        for (MCP_command_ptr_2 = &MCP_command[MCP_COMMAND_DATA_INDEX];
             MCP_command_ptr_2 < MCP_command_ptr; MCP_command_ptr_2++)
        {
            digit_count++;

            bval = *MCP_command_ptr_2 - '0';
            if (9 >= bval)
            {
                MCP_temp16_1 = MCP_temp16_1 * 10 + bval;
            }
            else
            {
                break;
            }
        }

        if (0 == MCP_temp1)
        {
            if (MCP_REG_PRESENT_MOTOR_SPEED > MCP_temp8_1)
            {
                if (3 < digit_count)
                {
                    MCP_INIT_CMDPTR();
                    return 1;
                }

                MCP_temp1 = MCP_set_8bit_register(MCP_temp8_1,
                                                           (U8)MCP_temp16_1, 1);
            }
            else
            {
                if (5 < digit_count)
                {
                    MCP_INIT_CMDPTR();
                    return 1;
                }

                MCP_temp1 = MCP_set_16bit_register(MCP_temp8_1,
                                                               MCP_temp16_1, 1);
            }
        }

        MCP_INIT_CMDPTR();
        return(MCP_temp1);
    }
    else if ('?' == MCP_command[MCP_COMMAND_TYPE_INDEX])
    {
        MCP_INIT_CMDPTR();

        if (0 == MCP_temp1)
        {
            MCP_temp1 = 1;
            if (MCP_REG_PRESENT_MOTOR_SPEED > MCP_temp8_1)
            {
                if (MCP_REG_RESERVED_START > MCP_temp8_1)
                {
                    MCP_temp16_1 = (U16)MCP_get_8bit_register(MCP_temp8_1);
                    MCP_send_ascii_value(MCP_temp16_1);
                    MCP_temp1 = 0;
                }
            }
            else
            {
                if ((MCP_REG_PRESENT_MOTOR_SPEED <= MCP_temp8_1) &&
                    (MCP_REG_UNDEFINED_START     > MCP_temp8_1))
                {
                    MCP_temp16_1 = MCP_get_16bit_register(MCP_temp8_1);
                    MCP_send_ascii_value(MCP_temp16_1);
                    MCP_temp1 = 0;
                }
            }
        }

        if (0 == MCP_temp1)
        {
            *MCP_command_ptr++ = '\n';
        }

        return(MCP_temp1);
    }
    else if ('#' == MCP_command[MCP_COMMAND_TYPE_INDEX])
    {
        MCP_INIT_CMDPTR();
        MCP_temp16_1 = MCP_command[MCP_COMMAND_DATA_INDEX + 0] - '0';

        if (0 == MCP_temp1)
        {
            MCP_temp1 = MCP_status_update_register(MCP_temp8_1, MCP_temp16_1);
        }

        return(MCP_temp1);
    }
#ifdef FEATURE_PID_TUNE_FUNCTION
    else if (('T' == MCP_command[MCP_COMMAND_TYPE_INDEX + 0]) &&
             ('N' == MCP_command[MCP_COMMAND_TYPE_INDEX + 1]))
    {
        MCP_temp16_1 = MCP_get_16bit_register(MCP_REG_PRESENT_MOTOR_SPEED);

        if (0 == MCP_temp16_1)
        {
            digit_count = 0;
            MCP_temp16_1 = 0;
            for (MCP_command_ptr_2 = &MCP_command[MCP_COMMAND_DATA_INDEX - 1];
                 MCP_command_ptr_2 < MCP_command_ptr; MCP_command_ptr_2++)
            {
                bval = *MCP_command_ptr_2 - '0';
                if (9 >= bval)
                {
                    digit_count++;
                    if (5 < digit_count)
                    {
                        MCP_INIT_CMDPTR();
                        return 1;
                    }

                    MCP_temp16_1 = MCP_temp16_1 * 10 + bval;
                }
                else
                {
                    break;
                }
            }
            MCP_command_ptr_2++;    // Skip past non-number character
            digit_count = 0;
            MCP_temp16_2 = 0;
            for (; MCP_command_ptr_2 < MCP_command_ptr; MCP_command_ptr_2++)
            {
                bval = *MCP_command_ptr_2 - '0';
                if (9 >= bval)
                {
                    digit_count++;
                    if (5 < digit_count)
                    {
                        MCP_INIT_CMDPTR();
                        return 1;
                    }

                    MCP_temp16_2 = MCP_temp16_2 * 10 + bval;
                }
                else
                {
                    break;
                }
            }

            MCP_cmd_type = MCP_COMMAND_TUNE_PID;

            MCP_INIT_CMDPTR();
            //Note: Remaining part of this command is processed in MCP_Task()
            return 0;
        }

        else
        {
            MCP_INIT_CMDPTR();
            return 1;
        }
    }
#endif
    MCP_INIT_CMDPTR();
    return 1;
}

//------------------------------------------------------------------------------
// MCP_update_status
// static void MCP_update_status (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     It reads all enabled registers, one at a time, and stores address and
//     corresponding value into status buffer.
//
// WARNING:
//     If data is not taken out (get) from status buffer at a sufficient rate,
//     it may over-write unsent data.
//------------------------------------------------------------------------------
static void MCP_update_status (void)
{
    if (MCP_REG_RESERVED_START > MCP_temp8_2)
    {
        if (MCP_is_8bit_status_update_enabled(MCP_temp8_2))
        {
            MCP_temp16_1 = MCP_get_8bit_register(MCP_temp8_2);

            //Send register address in binary form
            MCP_status_add(MCP_temp8_2);
            MCP_status_INCPTR(MCP_status_tail);

            //Send 8-bit value
            MCP_status_add((U8)(MCP_temp16_1 & 0x00FF));
            MCP_status_INCPTR(MCP_status_tail);
        }
        MCP_temp8_2++;
    }
    else if (MCP_REG_PRESENT_MOTOR_SPEED > MCP_temp8_2)
    {
        // Skip reserved registers
        MCP_temp8_2 = MCP_REG_PRESENT_MOTOR_SPEED;
    }
    else if (MCP_REG_UNDEFINED_START > MCP_temp8_2)
    {
        if (MCP_is_16bit_status_update_enabled(MCP_temp8_2))
        {
            MCP_temp16_1 = MCP_get_16bit_register(MCP_temp8_2);

            //Send register address in binary form
            MCP_status_add(MCP_temp8_2);
            MCP_status_INCPTR(MCP_status_tail);

            //Send upper byte of 16 bit value
            MCP_status_add((U8)(MCP_temp16_1 >> 8));
            MCP_status_INCPTR(MCP_status_tail);

            //Send lower byte of 16 bit value
            MCP_status_add((U8)(MCP_temp16_1 & 0x00FF));
            MCP_status_INCPTR(MCP_status_tail);
        }
        MCP_temp8_2 += 2;
    }
    else
    {
        // Skip undefined registers and start again
        MCP_temp8_2 = MCP_REG_FIRMWARE_REVISION;
    }
}

//------------------------------------------------------------------------------
// MCP_send_ascii_value
// static void MCP_send_ascii_value (U16 value)
//
// Return Value:
//     None
//
// Parameters:
//     value - the 16-bit value to convert to ASCII decimal
//
// Description:
//     This function calls MCP_dec_to_ascnumbuf() to converts value to ASCII
//     decimal and stores it in MCP_command[] buffer. It is normally called in
//     in response to some commands and the value is normally register contents.
//
//------------------------------------------------------------------------------
static void MCP_send_ascii_value (U16 value)
{
    U8 idx;

    // Convert it to hex number format in reverse byte order
    idx = MCP_dec_to_ascnumbuf(value);

    for (; idx != 0xff; idx--)
    {
        *MCP_command_ptr++ = MCP_numcnvbuf[idx];
    }
}

#else //BUILD_FOR_PROTOCOL
//------------------------------------------------------------------------------
// APIs
//------------------------------------------------------------------------------

//Pointers to call back functions
U8 (* MCP_tune_callback) (U16, U16);
bit (* MCP_reset_callback) (void);

//------------------------------------------------------------------------------
// MCP_init:
// void MCP_init (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     This function initializes MCP Registers.
//------------------------------------------------------------------------------
void MCP_init (void)
{
    MCP_init_registers();
}

//------------------------------------------------------------------------------
// MCP_task:
// void MCP_task (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     This is a dummy function.
//------------------------------------------------------------------------------
void MCP_task (void)
{

}

//------------------------------------------------------------------------------
// MCP Debug Print
// void MCP_debug_Print (const U8 *str, U32 value)
//
// Return Value:
//     None
//
// Parameters:
//     Debug string/message to be printed.
//
// Description:
//     This is a dummy function.
//------------------------------------------------------------------------------
#ifdef BUILD_FOR_DEBUG

void MCP_debug_print (const U8 * data str, U32 value)
{

}
#endif //BUILD_FOR_DEBUG

#endif //BUILD_FOR_PROTOCOL

//------------------------------------------------------------------------------
// End Of File
//------------------------------------------------------------------------------
