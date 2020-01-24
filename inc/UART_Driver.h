//------------------------------------------------------------------------------
// UART_Driver.h
//------------------------------------------------------------------------------
// Copyright (C) 2013, Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Description:
//
// This file defines APIs for UART of C8051F85x_6x devices. These APIs allow
// users to initialize and configure UART and to receive and send data via UART
//
// Resources Used:
//     1) UART0
//     2) TIMER1
//     3) Pins P0.4 and P0.5 using Crossbar
// These resources can not be used for any other purpose.
//
// Release 0.0 - April 10, 2013 mufayyuz
//    -Initial Revision.
//
// At this stage UART memory usage is:
// data=45.1 xdata=0 code=273
//

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

//------------------------------------------------------------------------------
// CONSTANTS
//
// UART_BUF_SIZE is the size of internal Tx and Rx buffers.
//------------------------------------------------------------------------------
#define UART_BUF_SIZE 16 //UART buffer size in bytes

//------------------------------------------------------------------------------
// DATA TYPES
//------------------------------------------------------------------------------
typedef enum _error
{
    UART_SUCCESS,
    UART_ERROR_UNSUPPORTED_BAUD = 10,
    UART_ERROR_TX_BUFFER_FULL   = 20,
    UART_ERROR_RX_BUFFER_EMPTY  = 21,
    /*TODO: add more*/
} uart_error_t;

//------------------------------------------------------------------------------
// API
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// UART_init
// void UART_init (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     This function is called by application to initialize UART for 921600 bps.
//     Once this function is called successfully, application can send data over
//     UART. It also enables UART for data reception.
//------------------------------------------------------------------------------
void UART_init (void);

//------------------------------------------------------------------------------
// UART_send_byte
// U8 UART_send_byte (U8)
//
// Return Value:
//     Returns error code of type uart_error_t
//
// Parameters:
//     Data byte to sent over UART
//
// Description:
//     This function is called by application to send a data byte over UART.
//     UART driver doesn't impose any restriction on the value of data byte.
//     The data byte is put in an internal buffer and will only be send when all
//     previous data bytes are sent. If driver is unable to accept new data byte
//     then it doesn't block, instead it returns UART_ERROR_TX_BUFFER_FULL.
//     Application need to handle such situation.
//------------------------------------------------------------------------------
U8 UART_send_byte (U8);

//------------------------------------------------------------------------------
// UART_receive_byte
// U8 UART_receive_byte (U8 *)
//
// Return Value:
//     Returns error code of type uart_error_t
//
// Parameters:
//     Pointer to the variable where received byte is to be copied
//
// Description:
//     This function is called by application to read a data byte from UART. The
//     driver receives data from UART independent of application calls and saves
//     it into internal buffer. This function returns oldest unread data byte
//     from internal buffer. If buffer doesn't contain any unread byte then it
//     doesn't block, instead it returns UART_ERROR_RX_BUFFER_EMPTY. In this
//     case, application has to retry.
//------------------------------------------------------------------------------
U8 UART_receive_byte (U8 data *);

//------------------------------------------------------------------------------
// UART_init_buffers
// void UART_init_buffers (void)
//
// Return Value:
//     None
//
// Parameters:
//     None
//
// Description:
//     Application can call this function to initialize Tx and Rx buffers such
//     that both buffers are empty and respective Head and Tail indices are 0.
//
// WARNING: Use with care. Any unread data or data-in-wait will be lost.
//------------------------------------------------------------------------------
void UART_init_buffers (void);

//------------------------------------------------------------------------------
// UART_get_tx_buffer_free_bytes
// U8 UART_get_tx_buffer_free_bytes (void)
//
// Return Value:
//     Returns the number of free bytes in UART_tx_buffer[]
//
// Parameters:
//     None
//
// Description:
//     This function can be called by application to know how much space (bytes)
//     is available in tx buffer. Application can wait until required amount of
//     bytes is not free.
//
//------------------------------------------------------------------------------
U8 UART_get_tx_buffer_free_bytes (void);

#endif //UART_DRIVER_H

//------------------------------------------------------------------------------
// End Of File
//------------------------------------------------------------------------------
