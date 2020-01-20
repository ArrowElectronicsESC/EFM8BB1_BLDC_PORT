//------------------------------------------------------------------------------
// MCP_Core.h
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
// Default mode is text mode. It can switch from one mode to other. Each mode
// can process a set of commands. Unsupported commands are responded with ERROR.
//
// Release 0.0 - April 12, 2013 mufayyuz
//    -Initial Revision.
//
// At this stage MCP Core, MCP Register and UART memory usage is:
// data=89.4 xdata=33 code=2231

#ifndef MCP_CORE_H
#define MCP_CORE_H

//------------------------------------------------------------------------------
//Exposed Data
//
//These function pointers should be pointed to appropriate call-back functions
//------------------------------------------------------------------------------
extern U8 (* MCP_tune_callback) (U16, U16);
extern bit (* MCP_reset_callback) (void);

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
void MCP_init (void);

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
//     application at regular interval.
//------------------------------------------------------------------------------
void MCP_task (void);

//------------------------------------------------------------------------------
// MCP_debug_print
// void MCP_debug_print (const U8 *str)
//
// Return Value:
//     None
//
// Parameters:
//     Debug string/message to be printed.
//
// Description:
//     This function can be used by application to print simple debug message.
//     It does not support printing values. Each message is started with 0xFF to
//     indicate that it is debug message so that terminal program/console can
//     distinguish it from MCP responses.
//
//------------------------------------------------------------------------------
#ifdef BUILD_FOR_DEBUG
void MCP_debug_print (const U8 *str, U32 value);
#else
#define MCP_debug_print(a, b)
#endif

#endif //MCP_CORE_H

//------------------------------------------------------------------------------
// End Of File
//------------------------------------------------------------------------------
