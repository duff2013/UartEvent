/*
 ||
 || @file       Uart3Event.cpp
 || @version 	6
 || @author 	Colin Duffy
 || @contact 	http://forum.pjrc.com/members/25610-duff
 || @license
 || | Copyright (c) 2015 Colin Duffy
 || | This library is free software; you can redistribute it and/or
 || | modify it under the terms of the GNU Lesser General Public
 || | License as published by the Free Software Foundation; version
 || | 2.1 of the License.
 || |
 || | This library is distributed in the hope that it will be useful,
 || | but WITHOUT ANY WARRANTY; without even the implied warranty of
 || | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 || | Lesser General Public License for more details.
 || |
 || | You should have received a copy of the GNU Lesser General Public
 || | License along with this library; if not, write to the Free Software
 || | Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 || #
 ||
 */

#include "UartEvent.h"
#include "utility/memcpy.h"

#define SCGC4_UART2_BIT     12

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
#define TX_BUFFER_SIZE TX2_BUFFER_SIZE // number of outgoing bytes to buffer
#define RX_BUFFER_SIZE RX2_BUFFER_SIZE // number of incoming bytes to buffer
//#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

#ifdef SERIAL_9BIT_SUPPORT
static uint8_t use9Bits = 0;
#define BUFTYPE uint16_t
#else
#define BUFTYPE uint8_t
#define use9Bits 0
#endif

DMAMEM static volatile BUFTYPE __attribute__((aligned(TX_BUFFER_SIZE))) tx_buffer[TX_BUFFER_SIZE];
DMAMEM static volatile BUFTYPE __attribute__((aligned(RX_BUFFER_SIZE))) rx_buffer[RX_BUFFER_SIZE];

#if TX_BUFFER_SIZE > 255
static volatile uint16_t tx_buffer_head = 0;
static volatile uint16_t tx_buffer_tail = 0;
#else
static volatile uint8_t tx_buffer_head  = 0;
static volatile uint8_t tx_buffer_tail  = 0;
#endif
#if RX_BUFFER_SIZE > 255
static volatile uint16_t rx_buffer_head = 0;
static volatile uint16_t rx_buffer_tail = 0;
#else
static volatile uint8_t rx_buffer_head  = 0;
static volatile uint8_t rx_buffer_tail  = 0;
#endif

static volatile uint8_t transmitting  = 0;
static volatile uint8_t *transmit_pin = NULL;
static volatile uint8_t BUFFER_FULL   = false;

event_t           Uart3Event::event;
DMAChannel        Uart3Event::tx;
DMAChannel        Uart3Event::rx;
Uart3Event::ISR Uart3Event::txEventHandler;
Uart3Event::ISR Uart3Event::rxEventHandler;
// -------------------------------------------ISR------------------------------------------
void Uart3Event::serial_dma_tx_isr( void ) {
    tx.clearInterrupt( );
    
    uint32_t head = tx_buffer_head;
    uint32_t tail = tx_buffer_tail;
    uint32_t ELINKNO = tx.TCD->CITER_ELINKNO;
    
    if ( ( tail + ELINKNO ) >= TX_BUFFER_SIZE ) tail += ELINKNO - TX_BUFFER_SIZE;
    else tail += ELINKNO;
    
    if ( head != tail ) {
        transmitting = true;
        int size;
        if ( tail > head ) size = ( TX_BUFFER_SIZE - tail ) + head;
        else size = head - tail;
        
        __disable_irq( );
        tx.TCD->CITER = size;
        tx.TCD->BITER = size;
        tx.enable();
        __enable_irq( );
    } else {
        transmitting = false;
        txEventHandler( );
    }
    if ( transmit_pin ) *transmit_pin = 0;
    tx_buffer_tail = tail;
    //UART2_C2 |= UART_C2_TIE;
}

void Uart3Event::serial_dma_rx_isr( void ) {
    rx.clearInterrupt( );
    
    if ( event.term_rx_character != -1 ) {
        static uint32_t byteCount_rx = 1;
        if (( ( uint8_t )*event.currentptr_rx == event.term_rx_character ) || ( byteCount_rx == RX_BUFFER_SIZE ) ) {
            event.currentptr_rx = ( uintptr_t * )rx.destinationAddress( );
            *event.currentptr_rx = 0;
            rx.destinationCircular( rx_buffer, 1 );
            rxEventHandler( );
            byteCount_rx = 1;
            rx_buffer_head = rx_buffer_tail = 0;
        }
        else {
            ++byteCount_rx;
            event.currentptr_rx = ( uintptr_t * )rx.destinationAddress( );        }
    }
    else {
        BUFFER_FULL = true;
        rxEventHandler( );
        rx_buffer_head = rx_buffer_tail = 0;
        if ( RX2_BUFFER_SIZE == 1 ) rx.destinationCircular( rx_buffer, 1 );
        BUFFER_FULL = false;
    }
    //
}
// -------------------------------------------CODE------------------------------------------
void Uart3Event::serial_dma_begin( uint32_t divisor ) {
    // Enable UART2 clock
    BITBAND_U32( SIM_SCGC4, SCGC4_UART2_BIT ) = 0x01;
    /****************************************************************
     * some code lifted from Teensyduino Core serial1.c
     ****************************************************************/
    CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX( 3 );
    CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX( 3 );
    UART2_BDH = ( divisor >> 13 ) & 0x1F;
    UART2_BDL = ( divisor >> 5 ) & 0xFF;
    UART2_C4 = divisor & 0x1F;
    UART2_C1 = 0;//UART_C1_ILT;
    UART2_C2 = C2_TX_INACTIVE;
    UART2_C5 = UART_DMA_ENABLE; // setup Serial1 tx,rx to use dma
    if ( loopBack ) UART2_C1 |= UART_C1_LOOPS; // Set internal loop1Back
    /****************************************************************
     * DMA TX setup
     ****************************************************************/
    tx.destination( UART2_D );
    tx.sourceCircular( tx_buffer, TX_BUFFER_SIZE );
    tx.attachInterrupt( serial_dma_tx_isr );
    tx.interruptAtCompletion( );
    tx.disableOnCompletion( );
    tx.triggerAtHardwareEvent( DMAMUX_SOURCE_UART2_TX );
    event.priority = NVIC_GET_PRIORITY( IRQ_DMA_CH0 + tx.channel );
    /****************************************************************
     * DMA RX setup
     ****************************************************************/
    if ( rxTermCharacter == -1 && rxTermString == NULL ) {
        rx.destinationCircular( rx_buffer, RX_BUFFER_SIZE );
    }
    else {
        rx.destinationCircular( rx_buffer, 1 );
        event.term_rx_character = rxTermCharacter;
    }
    rx.source( UART2_D );
    rx.attachInterrupt( serial_dma_rx_isr );
    rx.interruptAtCompletion( );
    rx.triggerContinuously( );
    rx.triggerAtHardwareEvent( DMAMUX_SOURCE_UART2_RX );
    rx.enable( );
    for (int i = 0; i < RX_BUFFER_SIZE; i++) rx_buffer[i] = 0;
    for (int i = 0; i < TX_BUFFER_SIZE; i++) tx_buffer[i] = 0;
}

void Uart3Event::serial_dma_format(uint32_t format) {
    /****************************************************************
     * serial1 format, from teensduino core, serial1.c
     ****************************************************************/
    uint8_t c;
    c = UART2_C1;
    c = ( c & ~0x13 ) | ( format & 0x03 );      // configure parity
    if (format & 0x04) c |= 0x10;           // 9 bits (might include parity)
    UART2_C1 = c;
    if ( ( format & 0x0F ) == 0x04 ) UART2_C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
    c = UART2_S2 & ~0x10;
    if ( format & 0x10 ) c |= 0x10;           // rx invert
    UART2_S2 = c;
    c = UART2_C3 & ~0x10;
    if ( format & 0x20 ) c |= 0x10;           // tx invert
    UART2_C3 = c;
}

void Uart3Event::serial_dma_end( void ) {
    if ( !( SIM_SCGC7 & SIM_SCGC7_DMA ) ) return;
    if ( !( SIM_SCGC6 & SIM_SCGC6_DMAMUX ) ) return;
    if ( !( SIM_SCGC4 & SIM_SCGC4_UART2 ) ) return;
    flush( );
    delay(20);
    /****************************************************************
     * serial1 end, from teensduino core, serial1.c
     ****************************************************************/
    UART2_C2 = 0;
    CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX( 1 );
    CORE_PIN8_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX( 1 );
    // clear Serial1 dma enable rx/tx bits
    UART2_C5 = UART_DMA_DISABLE;
    tx_buffer_head = tx_buffer_tail = 0;
}

void Uart3Event::serial_dma_set_transmit_pin( uint8_t pin ) {
    // TODO: need to update var when finish transmitting serial for RS485
    pinMode( pin, OUTPUT );
    digitalWrite( pin, LOW );
    transmit_pin = portOutputRegister( pin );
}

void Uart3Event::serial_dma_putchar( uint32_t c ) {
    serial_dma_write( &c, 1 );
}

void Uart3Event::serial_dma_write( const void *buf, unsigned int count ) {
    uint8_t * buffer = ( uint8_t * )buf;
    uint32_t head = tx_buffer_head;
    uint32_t tail = tx_buffer_tail;
    uint32_t next = head + count;
    
    uint32_t free_buffer = serial_dma_write_buffer_free( );
    uint32_t cnt = count;
    
    if ( cnt > TX_BUFFER_SIZE ) return;
    
    bool bufwrap = next >= TX_BUFFER_SIZE ? true : false;
    if ( bufwrap ) {
        uint32_t over = next - TX_BUFFER_SIZE;
        uint32_t under = TX_BUFFER_SIZE - head;
        memcpy_fast( tx_buffer+head, buffer, under );
        memcpy_fast( tx_buffer, buffer+under, over );
        head = over;
    }
    else {
        memcpy_fast( tx_buffer+head, buffer, count );
        head += cnt;
    }
    
    tx_buffer_head = head;
    
    if ( !transmitting ) {
        transmitting = true;
        __disable_irq( );
        tx.TCD->CITER = cnt;
        tx.TCD->BITER = cnt;
        tx.enable( );
        __enable_irq( );
    }
}

void Uart3Event::serial_dma_flush( void ) {
    // wait for any remainding dma transfers to complete
    int head = tx_buffer_head;
    int tail = tx_buffer_tail;
    raise_priority( );
    while ( head != tail ) {
        yield( );
        head = tx_buffer_head;
        tail = tx_buffer_tail;
    }
    while ( transmitting ) yield( );
    lower_priority( );
}

int Uart3Event::serial_dma_write_buffer_free( void ) {
    uint32_t head, tail;
    head = tx_buffer_head;
    tail = tx_buffer_tail;
    if (head >= tail) return TX_BUFFER_SIZE - 1 - head + tail;
    return tail - head - 1;
}

int Uart3Event::serial_dma_available( void ) {
    uint32_t head, tail, ELINKNO;
    if ( BUFFER_FULL ) {
        head = rx_buffer_head;
        tail = rx_buffer_tail;
        return (RX_BUFFER_SIZE) - (tail);
    }
    ELINKNO = rx.TCD->CITER_ELINKNO;
    head = RX_BUFFER_SIZE - ELINKNO;
    tail = rx_buffer_tail;
    if ( head >= tail ) return head - tail;
    return RX_BUFFER_SIZE + head - tail;
}

int Uart3Event::serial_dma_getchar( void ) {
    uint32_t head, tail, ELINKNO;
    int c;
    if ( BUFFER_FULL ) {
        head = rx_buffer_head;
        tail = rx_buffer_tail;
        c = rx_buffer[tail];
        if ( ++tail > RX_BUFFER_SIZE ) tail = 0;
        
    } else {
        ELINKNO = rx.TCD->CITER_ELINKNO;
        head = RX_BUFFER_SIZE - ELINKNO;
        tail = rx_buffer_tail;
        c = rx_buffer[tail];
        if ( head == tail ) return -1;
        if ( ++tail >= RX_BUFFER_SIZE ) tail = 0;
    }
    rx_buffer_tail = tail;
    return c;
}

int Uart3Event::serial_dma_peek( void ) {
    uint32_t head, ELINKNO;
    ELINKNO = rx.TCD->CITER_ELINKNO;
    head = RX_BUFFER_SIZE - ELINKNO;
    return head;
}

void Uart3Event::serial_dma_clear( void ) {
    rx.destinationCircular( rx_buffer, RX2_BUFFER_SIZE );
    rx_buffer_head = rx_buffer_tail;
}
