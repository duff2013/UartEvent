/*
 ||
 || @file 		Serial1Event.cpp
 || @version 	4
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 || @license
 || | Copyright (c) 2014 Colin Duffy
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

#include "SerialEvent.h"

#define SCGC4_UART0_BIT     10

Serial1Event::ISR       Serial1Event::RX_CALLBACK;
Serial1Event::ISR       Serial1Event::TX_CALLBACK;

event_params_t          Serial1Event::event;
DMAChannel              Serial1Event::tx;
DMAChannel              Serial1Event::rx;

volatile uint8_t        Serial1Event::txUsedMemory;
volatile uint8_t        Serial1Event::rxUsedMemory;
volatile unsigned char  *Serial1Event::rxBuffer;
volatile uint32_t       Serial1Event::rxBufferSize;

volatile unsigned char  *Serial1Event::txBuffer;

volatile int Serial1Event::sendSize;
// -------------------------------------------ISR------------------------------------------
void Serial1Event::serial_dma_tx_isr( void ) {
    tx.clearInterrupt();
    int head = event.txHead;
    int tail = event.txTail;
    int mysize = sendSize;
    if (sendSize) {
        __disable_irq();
        event.isTransmitting = true;
        tail += sendSize;
        if (tail >= event.TX_BUFFER_SIZE) {
            tail -= event.TX_BUFFER_SIZE;
        }
        tx.TCD->CITER = sendSize;
        tx.TCD->BITER = sendSize;
        sendSize = 0;
        tx.enable();
        __enable_irq();
    } else {
        __disable_irq();
        tail = head;
        __enable_irq();
        TX_CALLBACK();
        event.isTransmitting = false;
    }
    event.txTail = tail;
    UART0_C2 |= UART_C2_TIE;
}

void Serial1Event::serial_dma_rx_isr( void ) {
    rx.clearInterrupt();
    if ( event.term_rx_character != -1 ) {
        static uint32_t byteCount_rx = 1;
        rxBufferSize = byteCount_rx;
        if (( (uint8_t)*event.currentptr_rx == event.term_rx_character) || ( byteCount_rx == event.RX_BUFFER_SIZE ) ) {
            event.currentptr_rx = (uintptr_t*)rx.TCD->DADDR;
            *event.currentptr_rx = 0;
            rx.TCD->DADDR = event.zeroptr_rx;
            byteCount_rx = 1;
            RX_CALLBACK();
        }
        else { ++byteCount_rx; }
        event.currentptr_rx = (uintptr_t*)rx.TCD->DADDR;
    }
    else {
        RX_CALLBACK();
    }
    
}
// -------------------------------------------CODE------------------------------------------
void Serial1Event::serial_dma_begin( uint32_t divisor ) {
    TX_CALLBACK = txEventHandler;
    RX_CALLBACK = rxEventHandler;
    // Enable UART0 clock
    BITBAND_U32( SIM_SCGC4, SCGC4_UART0_BIT ) = 0x01;
    /****************************************************************
     * some code lifted from Teensyduino Core serial1.c
     ****************************************************************/
    CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
	CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
	UART0_BDH = ( divisor >> 13 ) & 0x1F;
	UART0_BDL = ( divisor >> 5 ) & 0xFF;
	UART0_C4 = divisor & 0x1F;
	UART0_C1 = UART_C1_ILT;
    // TODO: Use UART0 fifo with dma
	UART0_TWFIFO = 1; // tx watermark, causes C5_TDMAS DMA request
	UART0_RWFIFO = 1; // rx watermark, causes C5_RDMAS DMA request
	UART0_PFIFO = UART_PFIFO_TXFE;
    UART0_C2 = C2_TX_INACTIVE;
    UART0_C5 = UART_DMA_ENABLE; // setup Serial1 tx,rx to use dma
    if ( loopBack ) UART0_C1 |= UART_C1_LOOPS; // Set internal loop1Back
    /****************************************************************
     * DMA TX setup
     ****************************************************************/
    tx.destination( UART0_D );
    tx.sourceCircular(txBuffer, event.TX_BUFFER_SIZE);
    tx.attachInterrupt( serial_dma_tx_isr );
    tx.interruptAtCompletion();
    tx.disableOnCompletion();
    tx.triggerAtHardwareEvent(DMAMUX_SOURCE_UART0_TX);
    int sync_address = (int)&txBuffer[0] - (int)tx.TCD->SADDR;
    Serial.printf("SADDR: %p | txBuffer: %p | sync_address: %X\n\n", tx.TCD->SADDR, &txBuffer[0], sync_address);
    /****************************************************************
     * DMA RX setup
     ****************************************************************/
    if ( rxTermCharacter == -1 && rxTermString == NULL ) {
        rxBufferSize = event.RX_BUFFER_SIZE;
        rx.destinationBuffer(rxBuffer, event.RX_BUFFER_SIZE);
    }
    else {
        rx.destinationCircular(rxBuffer, 1);
        event.term_rx_string = rxTermString;
        event.term_rx_character = rxTermCharacter;
        event.zeroptr_rx = (uintptr_t*)rx.TCD->DADDR;
        event.currentptr_rx = event.zeroptr_rx;
    }
    rx.source(UART0_D);
    rx.attachInterrupt(serial_dma_rx_isr);
    rx.interruptAtCompletion();
    rx.triggerContinuously();
    rx.triggerAtHardwareEvent(DMAMUX_SOURCE_UART0_RX);
    rx.enable();
    /*Serial.printf("rx.channel:\t\t%i\n", rx.channel);
     Serial.printf("rx.TCD->DADDR:\t\t%p\n", rx.TCD->DADDR);
     Serial.printf("rx.TCD->SADDR:\t\t%p\n", rx.TCD->SADDR);
     Serial.printf("rx.TCD->SLAST:\t\t%i\n", rx.TCD->SLAST);
     Serial.printf("rx.TCD->SOFF:\t\t%i\n", rx.TCD->SOFF);
     Serial.printf("rx.TCD->DOFF:\t\t%i\n", rx.TCD->DOFF);
     Serial.printf("rx.TCD->DLASTSGA:\t%i\n", rx.TCD->DLASTSGA);
     Serial.printf("rx.TCD->ATTR_DST:\t%i\n", rx.TCD->ATTR_DST);
     Serial.printf("rx.TCD->ATTR_SRC:\t%i\n", rx.TCD->ATTR_SRC);
     Serial.printf("rx.TCD->BITER_ELINKNO:\t%i\n", rx.TCD->BITER_ELINKNO);
     Serial.printf("rx.TCD->CITER_ELINKNO:\t%i\n", rx.TCD->CITER_ELINKNO);
     Serial.printf("rx.TCD->NBYTES:\t\t%i\n", rx.TCD->NBYTES);*/
}

void Serial1Event::serial_dma_format(uint32_t format) {
    /****************************************************************
     * serial1 format, from teensduino core, serial1.c
     ****************************************************************/
    uint8_t c;
    c = UART0_C1;
    c = ( c & ~0x13 ) | ( format & 0x03 );      // configure parity
    if (format & 0x04) c |= 0x10;           // 9 bits (might include parity)
    UART0_C1 = c;
    if ( ( format & 0x0F ) == 0x04 ) UART0_C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
    c = UART0_S2 & ~0x10;
    if ( format & 0x10 ) c |= 0x10;           // rx invert
    UART0_S2 = c;
    c = UART0_C3 & ~0x10;
    if ( format & 0x20 ) c |= 0x10;           // tx invert
    UART0_C3 = c;
}

void Serial1Event::serial_dma_end( void ) {
    if ( !( SIM_SCGC7 & SIM_SCGC7_DMA ) ) return;
    if ( !( SIM_SCGC6 & SIM_SCGC6_DMAMUX ) ) return;
    if ( !( SIM_SCGC4 & SIM_SCGC4_UART0 ) ) return;
    while ( sendSize ) ;
    while ( event.isTransmitting ) ;
    //delay(20);
    /****************************************************************
     * serial1 end, from teensduino core, serial1.c
     ****************************************************************/
    UART0_C2 = 0;
	CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
	CORE_PIN1_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
    // clear Serial1 dma enable rx/tx bits
    UART0_C5 = UART_DMA_DISABLE;
    event.rxHead = event.rxTail = 0;
}

void Serial1Event::serial_dma_set_transmit_pin( uint8_t pin ) {
    // TODO: need to update var when finish transmitting serial for RS485
    pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	event.transmit_pin = portOutputRegister( pin );
}

void Serial1Event::serial_dma_putchar( uint32_t c ) {
    serial_dma_write( &c, 1 );
}

void Serial1Event::serial_dma_write( const void *buf, unsigned int count ) {
    int head = event.txHead;
    int tail = event.txTail;
    int next = head + count;
    sendSize += count;
    if (sendSize > event.TX_BUFFER_SIZE) {
        if (event.isTransmitting) DMA_CR |= DMA_CR_ECX;
        tx.TCD->CITER = 1;
        tx.TCD->BITER = 1;
        tx.TCD->SADDR = &txBuffer[0];
        head = 1;
        event.txHead = head;
        sendSize = 0;
        return;
    }
    if (next >= event.TX_BUFFER_SIZE) {
        int over = next - event.TX_BUFFER_SIZE;
        int under = event.TX_BUFFER_SIZE - head;
        if (under) {
            __disable_irq();
            memcpy_fast(txBuffer+head, buf, under);
            __enable_irq();
        }
        if (over) {
            __disable_irq();
            memcpy_fast(txBuffer, buf+under, over);
            __enable_irq();
        }
        head = over;
    }
    else {
        __disable_irq();
        memcpy_fast(txBuffer+head, buf, count);
        head += count;
        __enable_irq();
    }
    event.txHead = head;
    if (!event.isTransmitting) {
        __disable_irq();
        tx.TCD->CITER = count;
        tx.TCD->BITER = count;
        tx.enable();
        sendSize -= count;
        event.isTransmitting = true;
        __enable_irq();
        //Serial.println();
    }
}

void Serial1Event::serial_dma_flush( void ) {
    // wait for any remainding dma transfers to complete
    while ( sendSize ) ;
    while ( event.isTransmitting ) ;
}

int Serial1Event::serial_dma_available( void ) {
    uint32_t head, tail;
    head = event.RX_BUFFER_SIZE - rx.TCD->CITER_ELINKNO;
    tail = event.rxTail;
    if ( head >= tail ) return head - tail;
    return event.RX_BUFFER_SIZE + head - tail;
}

int Serial1Event::serial_dma_getchar( void ) {
    uint32_t head, tail;
    int c;
    head = event.RX_BUFFER_SIZE - rx.TCD->CITER_ELINKNO;
    tail = event.rxTail;
    c = rxBuffer[tail];
    if ( head == tail ) return -1;
    if ( ++tail >= event.RX_BUFFER_SIZE ) tail = 0;
    event.rxTail = tail;
    return c;
}

int Serial1Event::serial_dma_peek( void ) {
    uint32_t head;
    head = event.RX_BUFFER_SIZE - rx.TCD->CITER_ELINKNO;
    return head;
}

void Serial1Event::serial_dma_clear( void ) {
    
}