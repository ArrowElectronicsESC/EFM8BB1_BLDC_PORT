/*
 * UART_Driver.c
 *
 *  Created on: Jan 20, 2020
 *      Author: a92862
 */


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "bldcdk.h"

#ifdef BUILD_FOR_UART

//------------------------------------------------------------------------------
// Local Constants
//------------------------------------------------------------------------------
#define TIMER1_INT 0

#if 1
#define UART_TX_BUFFER_FREE_BYTES \
             ((UART_tx_buffer_head - UART_tx_buffer_tail) & (UART_BUF_SIZE - 1))

#define UART_TX_BUFFER_FULL  (1 == (UART_TX_BUFFER_FREE_BYTES))

#define UART_TX_BUFFER_EMPTY (0 == (UART_TX_BUFFER_FREE_BYTES))

#define UART_INC_TXPTR(p) do {  \
    (p)++;  \
    if ((p) == &UART_tx_buffer[UART_BUF_SIZE])    \
        (p) = &UART_tx_buffer[0];   \
} while (0)

#define UART_RX_BUFFER_FREE_BYTES \
             ((UART_rx_buffer_head - UART_rx_buffer_tail) & (UART_BUF_SIZE - 1))

#define UART_RX_BUFFER_FULL  (1 == (UART_RX_BUFFER_FREE_BYTES))

#define UART_RX_BUFFER_EMPTY (0 == (UART_RX_BUFFER_FREE_BYTES))

#define UART_INC_RXPTR(p) do {  \
    (p)++;  \
    if ((p) == &UART_rx_buffer[UART_BUF_SIZE])    \
        (p) = &UART_rx_buffer[0];   \
} while (0)

#else

#define UART_TX_BUFFER_FULL                               \
   ((UART_tx_buffer_tail == (UART_tx_buffer_head - 1)) || \
   ((UART_tx_buffer_tail == (UART_BUF_SIZE - 1)) && (UART_tx_buffer_head == 0)))

#define UART_TX_BUFFER_EMPTY   (UART_tx_buffer_tail == UART_tx_buffer_head)

#define UART_RX_BUFFER_FULL                               \
   ((UART_rx_buffer_tail == (UART_rx_buffer_head - 1)) || \
   ((UART_rx_buffer_tail == (UART_BUF_SIZE - 1)) && (UART_rx_buffer_head == 0)))

#define UART_RX_BUFFER_EMPTY   (UART_rx_buffer_tail == UART_rx_buffer_head)

#endif

//------------------------------------------------------------------------------
// Local functions
//------------------------------------------------------------------------------
INTERRUPT_PROTO (UART0_ISR, INTERRUPT_UART0);

//------------------------------------------------------------------------------
// Local Variables
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// TX BUFFER
// UART_Tx_Buffer[] holds data to be send over UART. The next data to be sent
// over UART is at index UART_tx_buffer_head and next data byte can be queued at
// index UART_tx_buffer_tail.
// UART_Tx_Busy is set when data byte is written to UART register and cleared
// when that byte is transmitted successfully.
//------------------------------------------------------------------------------
volatile U8 SEG_IDATA UART_tx_buffer[UART_BUF_SIZE];
volatile U8 SEG_IDATA *UART_tx_buffer_head;
volatile U8 SEG_IDATA *UART_tx_buffer_tail;
volatile bit UART_tx_busy;


//------------------------------------------------------------------------------
// RX BUFFER
// UART_Rx_Buffer[] holds data received from UART. The next data byte to be read
// by application is at index UART_rx_buffer_head and next received byte will be
// stored at index UART_rx_buffer_tail.
//------------------------------------------------------------------------------
volatile U8 SEG_IDATA UART_rx_buffer[UART_BUF_SIZE];
volatile U8 SEG_IDATA *UART_rx_buffer_head;
volatile U8 SEG_IDATA *UART_rx_buffer_tail;

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
//     This function is called at startup to initialize Tx and Rx buffers such
//     that both buffers are empty and respective Head and Tail indices are set
//     to 0. It can also be called by application.
//
// WARNING: Use with care. Any unread data or data-in-wait will be lost.
//------------------------------------------------------------------------------
void UART_init_buffers (void)
{
    UART_tx_buffer_head = &UART_tx_buffer[0];
    UART_tx_buffer_tail = &UART_tx_buffer[0];
    UART_rx_buffer_head = &UART_rx_buffer[0];
    UART_rx_buffer_tail = &UART_rx_buffer[0];
}

//------------------------------------------------------------------------------
// UART_send_byte
// U8 UART_send_byte (U8)
//
// Return Value:
//     Returns error code of type uart_error_t
//
// Parameters:
//     Data byte to be sent over UART
//
// Description:
//     This function is called by application to send a data byte over UART.
//     UART driver doesn't impose any restriction on the value of data byte.
//     The data byte is put in an internal buffer and will only be send when all
//     previous data bytes are sent. If driver is unable to accept new data byte
//     then it doesn't block, instead it returns UART_ERROR_TX_BUFFER_FULL. Such
//     condition should be handled by application.
//------------------------------------------------------------------------------
U8 UART_send_byte (U8 value)
{
    //Return error if Tx buffer is full.
    if (UART_TX_BUFFER_FULL)
    {
        return UART_ERROR_TX_BUFFER_FULL;
    }

    //Queue new byte at tail of buffer, update tail and count
    *UART_tx_buffer_tail = value;

    IE_ES0 = 0;
    UART_INC_TXPTR(UART_tx_buffer_tail);
    IE_ES0 = 1;

    if (0 == UART_tx_busy)
    {
        //Since UART is free for transmission, trigger transmission now
    	SCON0_TI = 1;
    }

    return UART_SUCCESS;
}

//------------------------------------------------------------------------------
// UART_receive_byte
// U8 UART_receive_byte (U8 data *)
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
U8 UART_receive_byte (U8 data *value)
{
    //Return error if Rx buffer is empty.
    if (UART_RX_BUFFER_EMPTY)
    {
        return UART_ERROR_RX_BUFFER_EMPTY;
    }

    //Queue new byte at tail of buffer, update tail and count
    *value = *UART_rx_buffer_head;

    IE_ES0 = 0;
    UART_INC_RXPTR(UART_rx_buffer_head);
    IE_ES0 = 1;

    return UART_SUCCESS;
}
#if 1
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
U8 UART_get_tx_buffer_free_bytes (void)
{
    if (UART_tx_buffer_head != UART_tx_buffer_tail)
    {
        return UART_TX_BUFFER_FREE_BYTES;
    }
    else
    {
        return UART_BUF_SIZE;
    }

}
#endif


//-----------------------------------------------------------------------------
// UART0_ISR
//-----------------------------------------------------------------------------
//
// UART0 ISR Content goes here. Remember to clear flag bits:
// SCON0::RI (Receive Interrupt Flag)
// SCON0::TI (Transmit Interrupt Flag)
//
//-----------------------------------------------------------------------------
SI_INTERRUPT (UART0_ISR, UART0_IRQn)
{
	if (SCON0_RI == 1)
	    {
	        //This bit is not cleared by hardware, so clear it here.
			SCON0_RI = 0;

	#if 0 //To test the behaviour if alternate byte is not read from SBUF0
	        if (0 == temp)
	        {
	            UART_rx_buffer[UART_rx_buffer_tail] = SBUF0;
	            UART_rx_buffer_tail++;

				if (UART_BUF_SIZE <= UART_rx_buffer_tail)
	            {
	                UART_rx_buffer_tail = 0;
	            }
	        }
	        else
	        {

	        }
	        temp ^= 1;
	#else

	        //If UART Rx buffer is full then discard new byte otherwise save it
	        if (!UART_RX_BUFFER_FULL)
	        {
	            *UART_rx_buffer_tail = SBUF0;
	            UART_INC_RXPTR(UART_rx_buffer_tail);
	        }
	#endif

	    }

	    //Last byte was transmitted successfully
	    if (SCON0_TI == 1)
	    {
	        //This bit is not cleared by hardware, so clear it here.
	    	SCON0_TI = 0;

	        //After transmission of last byte driver is now ready for next (if any?)
	        UART_tx_busy = 0;

	        if (UART_tx_buffer_head != UART_tx_buffer_tail)
	        {
	            //Indicate transmission in progress, get next byte in queue, send it
	            //and update head and count

	            UART_tx_busy = 1;
	            SBUF0 = *UART_tx_buffer_head;
	            UART_INC_TXPTR(UART_tx_buffer_head);
	        }
	    }
}


#else //BUILD_FOR_UART

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
//     This is a dummy function.
//------------------------------------------------------------------------------
void UART_init (void)
{

}

#ifdef BUILD_FOR_PROTOCOL

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
//     This is a dummy function.
//------------------------------------------------------------------------------
void UART_init_buffers (void)
{

}

//------------------------------------------------------------------------------
// UART_send_byte
// U8 UART_send_byte (U8)
//
// Return Value:
//     Returns error code of type uart_error_t
//
// Parameters:
//     Data byte to be sent over UART
//
// Description:
//     This is a dummy function.
//------------------------------------------------------------------------------
U8 UART_send_byte (U8 value)
{
    return UART_SUCCESS;
}

//------------------------------------------------------------------------------
// UART_receive_byte
// U8 UART_receive_byte (U8 data *)
//
// Return Value:
//     Returns error code of type uart_error_t
//
// Parameters:
//     Pointer to the variable where received byte is to be copied
//
// Description:
//     This is a dummy function.
//------------------------------------------------------------------------------
U8 UART_receive_byte (U8 data *value)
{
    //Nothing received.
    return UART_ERROR_RX_BUFFER_EMPTY;
}

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
//     This is a dummy function.
//------------------------------------------------------------------------------
U8 UART_get_tx_buffer_free_bytes (void)
{
    return 0;
}

#endif //BUILD_FOR_PROTOCOL

#endif //BUILD_FOR_UART

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
