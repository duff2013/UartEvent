/*
 ||
 || @file       Uart1Event.cpp
 || @version 	6.6
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

#define SCGC4_UART0_BIT     10

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
#define TX_BUFFER_SIZE TX0_BUFFER_SIZE // number of outgoing bytes to buffer
#define RX_BUFFER_SIZE RX0_BUFFER_SIZE // number of incoming bytes to buffer
#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest
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
#define TX_RETURN_TYPE uint8_t
static volatile uint8_t tx_buffer_head  = 0;
static volatile uint8_t tx_buffer_tail  = 0;
#endif
#if RX_BUFFER_SIZE > 255
static volatile uint16_t rx_buffer_head  = 0;
static volatile uint16_t rx_buffer_tail  = 0;
static volatile uint16_t rx_buffer_count = 0;
#else
static volatile uint8_t rx_buffer_head  = 0;
static volatile uint8_t rx_buffer_tail  = 0;
static volatile uint8_t rx_buffer_count = 0;
#endif

static volatile uint8_t transmitting  = 0;
static volatile uint8_t *transmit_pin = NULL;
static volatile uint8_t BUFFER_FULL   = false;

DMAChannel        Uart1Event::tx;
DMAChannel        Uart1Event::rx;
Uart1Event::ISR   Uart1Event::txEventHandler;
Uart1Event::ISR   Uart1Event::rxEventHandler;

volatile int16_t  Uart1Event::priority;
volatile int      Uart1Event::rxTermCharacterTrigger;
volatile int      Uart1Event::rxBufferSizeTrigger;

uint32_t *Uart1Event::elink;
// -------------------------------------------ISR------------------------------------------
void Uart1Event::user_isr_tx( void ) {
    txEventHandler( );
}

void Uart1Event::serial_dma_tx_isr( void ) {
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
        NVIC_SET_PENDING( IRQ_UART0_ERROR );
    }
    if ( transmit_pin ) *transmit_pin = 0;
    tx_buffer_tail = tail;
}

void Uart1Event::user_isr_rx( void ) {
    rxEventHandler( );
}

void Uart1Event::serial_dma_rx_isr( void ) {
    rx.clearInterrupt( );
    uint32_t head, tail;
    int term_trigger, size_trigger;
    head = rx_buffer_head;
    head = ( head + 1 )&( RX_BUFFER_SIZE - 1 );
    rx_buffer_head = head;
    term_trigger = rxTermCharacterTrigger;
    size_trigger = rxBufferSizeTrigger;
    if ( size_trigger != -1 ) {
        
        uint16_t bufferFree;
        tail = rx_buffer_tail;
        if ( head >= tail ) bufferFree = head - tail;
        else bufferFree = RX_BUFFER_SIZE + head - tail;
        if ( bufferFree >= size_trigger ) {
            NVIC_SET_PENDING( IRQ_UART0_STATUS );
            *elink = 1;
            return;
        }
        else if ( term_trigger == -1 ) {
            *elink = 1;
            return;
        }
    }
    if ( term_trigger != -1 ) {
        char current = rx_buffer[head];
        if ( current == term_trigger ) {
            NVIC_SET_PENDING( IRQ_UART0_STATUS );
        }
        *elink = 1;
    }
    else {
        NVIC_SET_PENDING( IRQ_UART0_STATUS );
        *elink = 1;
    }
}
// -------------------------------------------CODE------------------------------------------
void Uart1Event::serial_dma_begin( uint32_t divisor ) {
    // Enable UART0 clock
    BITBAND_REG_U32( SIM_SCGC4, SCGC4_UART0_BIT ) = 0x01;
    /****************************************************************
     * some code lifted from Teensyduino Core serial1.c
     ****************************************************************/
    CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX( 3 );
    CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX( 3 );
    UART0_BDH = ( divisor >> 13 ) & 0x1F;
    UART0_BDL = ( divisor >> 5 ) & 0xFF;
    UART0_C4 = divisor & 0x1F;
    UART0_C1 = 0;//UART_C1_ILT;
    // TODO: Use UART0 fifo with dma
    UART0_TWFIFO = 2; // tx watermark, causes C5_TDMAS DMA request
    UART0_RWFIFO = 1; // rx watermark, causes C5_RDMAS DMA request
    UART0_PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;
    UART0_C2 = C2_TX_INACTIVE;
    UART0_C5 = UART_DMA_ENABLE; // setup Serial1 tx,rx to use dma
    if ( loopBack ) UART0_C1 |= UART_C1_LOOPS; // Set internal loop1Back
    /****************************************************************
     * DMA TX setup
     ****************************************************************/
    tx.destination( UART0_D );
    tx.sourceCircular( tx_buffer, TX_BUFFER_SIZE );
    tx.attachInterrupt( serial_dma_tx_isr );
    tx.interruptAtCompletion( );
    tx.disableOnCompletion( );
    tx.triggerAtHardwareEvent( DMAMUX_SOURCE_UART0_TX );
    attachInterruptVector( IRQ_UART0_ERROR, user_isr_tx );
    NVIC_SET_PRIORITY( IRQ_UART0_ERROR, 192 ); // 255 = lowest priority
    NVIC_ENABLE_IRQ( IRQ_UART0_ERROR );
    NVIC_SET_PRIORITY( IRQ_DMA_CH0 + tx.channel, IRQ_PRIORITY );
    priority = NVIC_GET_PRIORITY( IRQ_DMA_CH0 + tx.channel );
    /****************************************************************
     * DMA RX setup
     ****************************************************************/
    rx.source( UART0_D );
    rx.destinationCircular( rx_buffer+1, RX_BUFFER_SIZE );
    rx.attachInterrupt( serial_dma_rx_isr );
    rx.interruptAtCompletion( );
    rx.triggerContinuously( );
    rx.triggerAtHardwareEvent( DMAMUX_SOURCE_UART0_RX );
    attachInterruptVector( IRQ_UART0_STATUS, user_isr_rx );
    NVIC_SET_PRIORITY( IRQ_UART0_STATUS, 192 ); // 255 = lowest priority
    NVIC_ENABLE_IRQ( IRQ_UART0_STATUS );
    NVIC_SET_PRIORITY( IRQ_DMA_CH0 + rx.channel, IRQ_PRIORITY );
    elink = ( uint32_t * )&rx.TCD->CITER_ELINKNO;
    *elink = 1;
    rx.enable( );
}

void Uart1Event::serial_dma_format(uint32_t format) {
    /****************************************************************
     * serial1 format, from teensduino core, serial1.c
     ****************************************************************/
    uint8_t c;
    c = UART0_C1;
    c = ( c & ~0x13 ) | ( format & 0x03 );      // configure parity
    if (format & 0x04) c |= 0x10;           // 9 bits (might include parity)
    UART0_C1 = c;
    if ( ( format & 0x0F ) == 0x04) UART0_C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
    c = UART0_S2 & ~0x10;
    if ( format & 0x10 ) c |= 0x10;           // rx invert
    UART0_S2 = c;
    c = UART0_C3 & ~0x10;
    if ( format & 0x20 ) c |= 0x10;           // tx invert
    UART0_C3 = c;
#ifdef SERIAL_9BIT_SUPPORT
    c = UART0_C4 & 0x1F;
    if ( format & 0x08 ) c |= 0x20;           // 9 bit mode with parity (requires 10 bits)
    UART0_C4 = c;
    use9Bits = format & 0x80;
#endif
}

void Uart1Event::serial_dma_end( void ) {
    if ( !( SIM_SCGC7 & SIM_SCGC7_DMA ) ) return;
    if ( !( SIM_SCGC6 & SIM_SCGC6_DMAMUX ) ) return;
    if ( !( SIM_SCGC4 & SIM_SCGC4_UART0 ) ) return;
    attachInterruptVector( IRQ_UART0_STATUS, uart0_status_isr );
    // flush Uart1Event tx buffer
    flush( );
    delay(20);
    /****************************************************************
     * serial1 end, from teensduino core, serial1.c
     ****************************************************************/
    UART0_C2 = 0;
    CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX( 1 );
    CORE_PIN1_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX( 1 );
    UART0_C5 = UART_DMA_DISABLE;
    tx_buffer_head = tx_buffer_tail = 0;
}

void Uart1Event::serial_dma_set_transmit_pin( uint8_t pin ) {
    while ( transmitting ) yield();
    pinMode( pin, OUTPUT);
    digitalWrite( pin, LOW );
    transmit_pin = portOutputRegister( pin );
}

void Uart1Event::serial_dma_putchar( uint32_t c ) {
    serial_dma_write( &c, 1 );
}

int Uart1Event::serial_dma_write( const void *buf, unsigned int count ) {
    uint8_t * buffer = ( uint8_t * )buf;
    uint32_t head = tx_buffer_head;
    uint32_t cnt = count;
    uint32_t free = serial_dma_write_buffer_free( );
    if ( cnt > free ) cnt = free;
    uint32_t next = head + cnt;
    bool wrap = next >= TX_BUFFER_SIZE ? true : false;
    if ( wrap ) {
        uint32_t over = next - TX_BUFFER_SIZE;
        uint32_t under = TX_BUFFER_SIZE - head;
        memcpy_fast( tx_buffer+head, buffer, under );
        memcpy_fast( tx_buffer, buffer+under, over );
        head = over;
    }
    else {
        memcpy_fast( tx_buffer+head, buffer, cnt );
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
    return cnt;
}

void Uart1Event::serial_dma_flush( void ) {
    // wait for any remainding dma transfers to complete
    uint16_t head, tail;
    //raise_priority( );
    do {
        yield( );
        head = tx_buffer_head;
        tail = tx_buffer_tail;
    } while ( head != tail );
    while ( transmitting ) yield( );
    //lower_priority( );
}

int Uart1Event::serial_dma_write_buffer_free( void ) {
    uint32_t head, tail;
    head = tx_buffer_head;
    tail = tx_buffer_tail;
    if ( head >= tail ) return TX_BUFFER_SIZE - 1 - head + tail;
    return tail - head - 1;
}

int Uart1Event::serial_dma_available( void ) {
    uint32_t head, tail;
    head = rx_buffer_head;
    tail = rx_buffer_tail;
    if ( head >= tail ) return head - tail;
    return RX_BUFFER_SIZE + head - tail;
}

int Uart1Event::serial_dma_getchar( void ) {
    uint32_t head, tail;
    int c;
    head = rx_buffer_head;
    tail = rx_buffer_tail;
    if ( head == tail ) return -1;
    tail = ( tail + 1 )&( RX_BUFFER_SIZE - 1 );
    c = rx_buffer[tail];
    rx_buffer_tail = tail;
    return c;
}

int Uart1Event::serial_dma_peek( void ) {
    uint32_t head, tail;
    head = rx_buffer_head;
    tail = rx_buffer_tail;
    if ( head == tail ) return -1;
    tail = ( tail + 1 )&( RX_BUFFER_SIZE - 1 );
    return rx_buffer[tail];
}

void Uart1Event::serial_dma_clear( void ) {
    rx_buffer_tail = rx_buffer_head;
}
