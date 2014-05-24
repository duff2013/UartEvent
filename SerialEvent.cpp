/*
 ||
 || @file 		SerialEvent.cpp
 || @version 	1
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 ||
 || @description
 || | A simple DMA Hardware Serial class for transfering data in the background.
 || | This allows you to send data and move on very quickly. The DMA engine will
 || | complete the transfer without CPU intervention. Sending is buffered so once
 || | the first transfer is complete any messages in the buffer are automatically
 || | sent without user intervention. Recieving is buffered by user settings but
 || | defaults to a single byte.
 || |
 || #
 ||
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
#include "util/atomic.h"

#define UART_C5_TDMAS           (uint8_t)0x80
#define UART_C5_RDMAS           (uint8_t)0x20

#define SEND_DONE_TX        !(DMA_TCD0_CSR & DMA_TCD_CSR_DONE)
#define RECIEVE_DONE_RX1    !(DMA_TCD1_CSR & DMA_TCD_CSR_DONE)
#define RECIEVE_DONE_RX2    !(DMA_TCD2_CSR & DMA_TCD_CSR_DONE)
#define RECIEVE_DONE_RX3    !(DMA_TCD3_CSR & DMA_TCD_CSR_DONE)

#define UART_DMA_ENABLE       UART_C5_TDMAS | UART_C5_RDMAS
#define UART_DMA_DISABLE    0
#define IRQ_PRIORITY        64  // 0 = highest priority, 255 = lowest

#define likely(x)	__builtin_expect(!!(x), 1)
#define unlikely(x)	__builtin_expect(!!(x), 0)

#define BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define BITBAND_U32(reg, bit) (*(uint32_t *)BITBAND_ADDR((reg), (bit)))
#define BITBAND_U8(reg, bit) (*(uint8_t *)BITBAND_ADDR((reg), (bit)))

SerialEvent::ISR SerialEvent::RX1_CALLBACK;
SerialEvent::ISR SerialEvent::RX2_CALLBACK;
SerialEvent::ISR SerialEvent::RX3_CALLBACK;

volatile bool SerialEvent::txDone;
volatile int SerialEvent::txCount;

volatile uint32_t SerialEvent::txHead;
volatile uint32_t SerialEvent::txTail;
volatile uint32_t SerialEvent::_memory;

volatile uint16_t SerialEvent::bufSize_rx1;
volatile uint16_t SerialEvent::bufSize_rx2;
volatile uint16_t SerialEvent::bufSize_rx3;

volatile uint8_t SerialEvent::term_rx1;
volatile uint8_t SerialEvent::term_rx2;
volatile uint8_t SerialEvent::term_rx3;

volatile uintptr_t *SerialEvent::currentptr_rx1;
volatile uintptr_t *SerialEvent::currentptr_rx2;
volatile uintptr_t *SerialEvent::currentptr_rx3;

volatile uintptr_t *SerialEvent::zeroptr_rx1;
volatile uintptr_t *SerialEvent::zeroptr_rx2;
volatile uintptr_t *SerialEvent::zeroptr_rx3;

bool SerialEvent::dma_ch_enabled[4];

// transmit memory fifo
DMAMEM static tx_fifo_t txfifo[TX_FIFO_SIZE];

// -----------------------------------------------ch0_isr------------------------------------------------------------
void dma_ch0_isr() {
    // clear dma interrupt request 0
    DMA_CINT |= DMA_CINT_CINT(0);
    // access fifo to free already sent packet
    tx_fifo_t* sent = &txfifo[SerialEvent::txTail];
    // check if packet is allocated on the buffer
    if (sent->allocated) {
        // reduce packet count in fifo
        SerialEvent::txCount--;
        // update memory var, since we are releasing it here
        SerialEvent::_memory += sent->size;
        // free memory of already transmitted packet
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { delete [] sent->packet; }
        // buffer freed so update fifo
        sent->allocated = false;
    }
    else {
        Serial.println("ERROR: FREE ALLOCATED FAILED");
        // increment fifo tail
        SerialEvent::txTail = SerialEvent::txTail < (TX_FIFO_SIZE - 1) ? SerialEvent::txTail + 1 : 0;
        return;
    }
    // increment fifo tail
    SerialEvent::txTail = SerialEvent::txTail < (TX_FIFO_SIZE - 1) ? SerialEvent::txTail + 1 : 0;
    // user or default tx callback event
    sent->txEventHandler();
    /***********************************************************************************
     / here if the fifo has packets to send, we setup the dma destination address       /
     / and source registers for transmission on the queued serial port. We will         /
     / keep transmitting these queued packets in the "background" until the fifo        /
     / is exhausted.                                                                    /
     ***********************************************************************************/
    if ( SerialEvent::txCount > 0 ) {
        tx_fifo_t* queued = &txfifo[SerialEvent::txTail];
        if (queued->port == 1) {
            //BITBAND_U8(UART0_C2, 7) = 0x01;
            UART0_C2 |= UART_C2_TIE;
            DMA_TCD0_DADDR = &UART0_D;
            DMAMUX0_CHCFG0 &= ~0x3F;
            DMAMUX0_CHCFG0 |= DMAMUX_SOURCE_UART0_TX;
        } else if (queued->port == 2) {
            DMA_TCD0_DADDR = &UART1_D;
            UART1_C2 |= UART_C2_TIE;
            DMAMUX0_CHCFG0 &= ~0x3F;
            DMAMUX0_CHCFG0 |= DMAMUX_SOURCE_UART1_TX;
        }else if (queued->port == 3) {
            DMA_TCD0_DADDR = &UART2_D;
            UART2_C2 |= UART_C2_TIE;
            DMAMUX0_CHCFG0 &= ~0x3F;
            DMAMUX0_CHCFG0 |= DMAMUX_SOURCE_UART2_TX;
        }
        // Number of bytes to transfer (in each service request)
        DMA_TCD0_CITER_ELINKNO = queued->size;
        DMA_TCD0_BITER_ELINKNO = queued->size;
        DMA_TCD0_SADDR = queued->packet;
        BITBAND_U32(DMA_ERQ, 0x00) = 0x01;
        return;
        
    } else {
        // no more packets to transmitt
        SerialEvent::txDone = true;
        return;
    }
}
// -----------------------------------------------ch1_isr------------------------------------------------------------
void dma_ch1_isr() {
    // Clear Interrupt Request 1
    DMA_CINT |= DMA_CINT_CINT(1);
    // Enable DMA Request 1
    BITBAND_U32(DMA_ERQ, 1) = 0x01;
    //DMA_ERQ |= DMA_ERQ_ERQ1;
    if (SerialEvent::term_rx1 != 0) {
        static uint16_t byteCount_rx1 = 0;
        if (( (uint8_t)*SerialEvent::currentptr_rx1 == SerialEvent::term_rx1) || ( byteCount_rx1 == (SerialEvent::bufSize_rx1-1) )) {
            SerialEvent::currentptr_rx1 = (uintptr_t*)DMA_TCD1_DADDR;
            DMA_TCD1_DADDR = SerialEvent::zeroptr_rx1;
            byteCount_rx1 = 0;
            *SerialEvent::currentptr_rx1 = 0;
            SerialEvent::RX1_CALLBACK();
        }
        else { byteCount_rx1++; }
        SerialEvent::currentptr_rx1 = (uintptr_t*)DMA_TCD1_DADDR;
    }
    else {
        // user or default rx1 callback event
        SerialEvent::RX1_CALLBACK();
    }
}
// -----------------------------------------------ch2_isr------------------------------------------------------------
void dma_ch2_isr() {
    // Clear Interrupt Request 2
    DMA_CINT |= DMA_CINT_CINT(2);
    // Enable DMA Request 2
    BITBAND_U32(DMA_ERQ, 2) = 0x01;
    if (SerialEvent::term_rx2 != 0) {
        static uint16_t byteCount_rx2 = 0;
        if (( (uint8_t)*SerialEvent::currentptr_rx2 == SerialEvent::term_rx2) || ( byteCount_rx2 == (SerialEvent::bufSize_rx2-1) )) {
            SerialEvent::currentptr_rx2 = (uintptr_t*)DMA_TCD2_DADDR;
            DMA_TCD2_DADDR = SerialEvent::zeroptr_rx2;
            byteCount_rx2 = 0;
            *SerialEvent::currentptr_rx2 = 0;
            SerialEvent::RX2_CALLBACK();
        }
        else { byteCount_rx2++; }
        SerialEvent::currentptr_rx2 = (uintptr_t*)DMA_TCD2_DADDR;
    }
    else {
        // user or default rx2 callback event
        SerialEvent::RX2_CALLBACK();
    }
}
// -----------------------------------------------ch3_isr------------------------------------------------------------
void dma_ch3_isr() {
    // Clear Interrupt Request 3
    DMA_CINT |= DMA_CINT_CINT(3);
    // Enable DMA Request 3
    BITBAND_U32(DMA_ERQ, 3) = 0x01;
    if (SerialEvent::term_rx3 != 0) {
        static uint16_t byteCount_rx3 = 0;
        if (( (uint8_t)*SerialEvent::currentptr_rx3 == SerialEvent::term_rx3) || ( byteCount_rx3 == (SerialEvent::bufSize_rx3-1) )) {
            SerialEvent::currentptr_rx3 = (uintptr_t*)DMA_TCD3_DADDR;
            DMA_TCD3_DADDR = SerialEvent::zeroptr_rx3;
            byteCount_rx3 = 0;
            *SerialEvent::currentptr_rx3 = 0;
            SerialEvent::RX3_CALLBACK();
        }
        else { byteCount_rx3++; }
        SerialEvent::currentptr_rx3 = (uintptr_t*)DMA_TCD3_DADDR;
    }
    else {
        // user or default rx3 callback event
        SerialEvent::RX3_CALLBACK();
    }
}
// -------------------------------------------Constructor------------------------------------------------------------
SerialEvent::SerialEvent() :
    txEventHandler(defaultCallback),
    rxEventHandler(defaultCallback),
    rxBuffer(defaultBuffer_RX),
    rxBufferSize(2),
    loopBack(false),
    termCharacter(0)
{
    txHead = 0;
    txTail = 0;
    txCount = 0;
    _memory = 5520;
    txDone = true;
    dma_ch_enabled[0] = false;
    dma_ch_enabled[1] = false;
    dma_ch_enabled[2] = false;
    dma_ch_enabled[3] = false;
}
// user defined TX memory
SerialEvent::SerialEvent(uint32_t memmory) :
txEventHandler(defaultCallback),
rxEventHandler(defaultCallback),
rxBuffer(defaultBuffer_RX),
rxBufferSize(2),
loopBack(false),
termCharacter(0)
{
    txHead = 0;
    txTail = 0;
    txCount = 0;
    _memory = memmory;
    txDone = true;
    dma_ch_enabled[0] = false;
    dma_ch_enabled[1] = false;
    dma_ch_enabled[2] = false;
    dma_ch_enabled[3] = false;
}
// ----------------------------------------------begin---------------------------------------------------------------
void SerialEvent::begin( uint32_t baud, uint32_t format) {
    // Enable DMA, DMAMUX clocks
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    // Use default configuration
    if (DMA_CR != 0) DMA_CR = 0;
    bool  ch_enabled = ( ( dma_ch_enabled[1]) && (dma_ch_enabled[2]) && (dma_ch_enabled[3]) );
    if (!ch_enabled) {
        // setup tx dma transfers, all Serial ports share the same
        // dma channel for transmitted packets.
        dma_TX_begin();
        dma_ch_enabled[0] = true;
    }
    if (port == &Serial1) {
        Serial1.begin(baud, format);
        // disable Serial1 ISR
        NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
        if (loopBack) {
            // Set internal Loopback
            UART0_C1 |= UART_C1_LOOPS;
        }
        // set tx watermark (1 byte) TODO: use tx fifo
        UART0_TWFIFO = 1;
        // set rx watermark (1 byte) TODO: use rx fifo
        UART0_RWFIFO = 1;
        //UART0_C2 |= UART_C2_RIE;
        // enable Serial1 tx dma 
        UART0_C5 = UART_DMA_ENABLE;
        current_port = SERIAL1;
        dma_RX_begin();
        dma_ch_enabled[SERIAL1] = true;
    }
    else if (port == &Serial2) {
        Serial2.begin(baud, format);
        // disable Serial2 ISR
        NVIC_DISABLE_IRQ(IRQ_UART1_STATUS);
        if (loopBack) {
            // Set internal Loopback
            UART1_C1 |= UART_C1_LOOPS;
        }
        // enable Serial2 tx DMA
        UART1_C5 = UART_DMA_ENABLE;
        current_port = SERIAL2;
        dma_RX_begin();
        dma_ch_enabled[SERIAL2] = true;
    }
    else if (port == &Serial3) {
        Serial3.begin(baud, format);
        // disable Serial3 ISR
        NVIC_DISABLE_IRQ(IRQ_UART2_STATUS);
        if (loopBack) {
            // Set internal Loopback
            UART2_C1 |= UART_C1_LOOPS;
        }
        // enable Serial3 tx DMA
        UART2_C5 = UART_DMA_ENABLE;
        current_port = SERIAL3;
        dma_RX_begin();
        dma_ch_enabled[SERIAL3] = true;
    }
}
// --------------------------------------------serialTXInit----------------------------------------------------------
int SerialEvent::dma_TX_begin(void) {
    // Restore source address after major loop
    DMA_TCD0_SLAST = 0;
    // source address offset (1 byte)
    DMA_TCD0_SOFF = 1;
    // Destination offset (0 byte)
    DMA_TCD0_DOFF = 0;
    // Restore destination address after major loop
    DMA_TCD0_DLASTSGA = 0;
    // Source and destination size 8 bit
    DMA_TCD0_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    // Number of bytes to transfer (in each service request)
    DMA_TCD0_NBYTES_MLNO = 1;//UART0_TWFIFO;
    // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
    DMA_TCD0_CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
    // enable DMA MUX
    DMAMUX0_CHCFG0 = DMAMUX_ENABLE;
    // set TX priority
    NVIC_SET_PRIORITY(IRQ_DMA_CH0, IRQ_PRIORITY);
    // enable irq
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0);
    return 1;
}
// --------------------------------------------serialRXInit----------------------------------------------------------
int SerialEvent::dma_RX_begin(void) {
    if ( port == &Serial1 ) {
        RX1_CALLBACK = rxEventHandler;
        // Source address
        DMA_TCD1_SADDR = &UART0_D;
        // Restore source address after major loop
        DMA_TCD1_SLAST = 0;
        // source address offset (0 bytes)
        DMA_TCD1_SOFF = 0;
        // Destination address
        DMA_TCD1_DADDR = &rxBuffer[0];
        // Destination offset (0 byte)
        DMA_TCD1_DOFF = 1;
        if (termCharacter == 0) {
            // Restore destination address after major loop
            DMA_TCD1_DLASTSGA = -(rxBufferSize);
            // Set loop counts / channel2channel linking disabled
            DMA_TCD1_CITER_ELINKNO = rxBufferSize;//UART0_RWFIFO;
            DMA_TCD1_BITER_ELINKNO = rxBufferSize;//UART0_RWFIFO;
            term_rx1 = 0;
        }
        else {
            // Restore destination address after major loop
            DMA_TCD1_DLASTSGA = 0;
            // Set loop counts / channel2channel linking disabled
            DMA_TCD1_CITER_ELINKNO = 1;//UART0_RWFIFO;
            DMA_TCD1_BITER_ELINKNO = 1;//UART0_RWFIFO;
            term_rx1 = termCharacter;
            bufSize_rx1 = rxBufferSize;
            zeroptr_rx1 = (uintptr_t*)DMA_TCD1_DADDR;
            currentptr_rx1 = zeroptr_rx1;
            //Serial.printf("DMA_TCD1_DLASTSGA: %i\t| DMA_TCD1_DADDR: %p | zeroptr_rx1: %p\n\n",  DMA_TCD1_DLASTSGA, DMA_TCD1_DADDR, zeroptr_rx1);
        }
        // Source and destination size 8 bit
        DMA_TCD1_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
        // Number of bytes to transfer (in each service request)
        DMA_TCD1_NBYTES_MLNO = 1;//UART0_TWFIFO;
        // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
        DMA_TCD1_CSR = DMA_TCD_CSR_INTMAJOR  | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
        // Set Serial1 as source (CH 1), enable DMA MUX
        DMAMUX0_CHCFG1 = DMAMUX_DISABLE;
        DMAMUX0_CHCFG1 = DMAMUX_SOURCE_UART0_RX | DMAMUX_ENABLE;
        // Enable DMA Request 1
        DMA_ERQ |= DMA_ERQ_ERQ1;
        // enable irq CH1
        NVIC_SET_PRIORITY(IRQ_DMA_CH1, IRQ_PRIORITY);
        NVIC_ENABLE_IRQ(IRQ_DMA_CH1);
        return 1;
    }
    else if ( port == &Serial2 ) {
        RX2_CALLBACK = rxEventHandler;
        //Serial.println("Serial2 Begin");
        // Source address
        DMA_TCD2_SADDR = &UART1_D;
        // Restore source address after major loop
        DMA_TCD2_SLAST = 0;
        // source address offset (0 bytes)
        DMA_TCD2_SOFF = 0;
        // Destination address
        DMA_TCD2_DADDR = rxBuffer;
        // Destination offset (0 byte)
        DMA_TCD2_DOFF = 1;
        if (termCharacter == 0) {
            // Restore destination address after major loop
            DMA_TCD2_DLASTSGA = -(rxBufferSize);
            // Set loop counts / channel2channel linking disabled
            DMA_TCD2_CITER_ELINKNO = rxBufferSize;//UART0_RWFIFO;
            DMA_TCD2_BITER_ELINKNO = rxBufferSize;//UART0_RWFIFO;
            term_rx2 = 0;
        }
        else {
            // Restore destination address after major loop
            DMA_TCD2_DLASTSGA = 0;
            // Set loop counts / channel2channel linking disabled
            DMA_TCD2_CITER_ELINKNO = 1;//UART0_RWFIFO;
            DMA_TCD2_BITER_ELINKNO = 1;//UART0_RWFIFO;
            term_rx2 = termCharacter;
            bufSize_rx2 = rxBufferSize;
            zeroptr_rx2 = (uintptr_t*)DMA_TCD2_DADDR;
            currentptr_rx2 = zeroptr_rx2;
            //Serial.printf("DMA_TCD2_DLASTSGA: %i\t| DMA_TCD2_DADDR: %p | zeroptr_rx1: %p\n\n",  DMA_TCD2_DLASTSGA, DMA_TCD2_DADDR, zeroptr_rx2);
        }
        // Source and destination size 8 bit
        DMA_TCD2_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
        // Number of bytes to transfer (in each service request)
        DMA_TCD2_NBYTES_MLNO = 1;
        // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
        DMA_TCD2_CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
        // Set Serial2 as source (CH 2), enable DMA MUX
        DMAMUX0_CHCFG2 = DMAMUX_DISABLE;
        DMAMUX0_CHCFG2 = DMAMUX_SOURCE_UART1_RX | DMAMUX_ENABLE;
        // Enable DMA Request 2
        DMA_ERQ |= DMA_ERQ_ERQ2;
        // enable irq CH2
        NVIC_SET_PRIORITY(IRQ_DMA_CH2, IRQ_PRIORITY);
        NVIC_ENABLE_IRQ(IRQ_DMA_CH2);
        return 1;
    }
    else if ( port == &Serial3 ) {
        RX3_CALLBACK = rxEventHandler;
        // Source address
        DMA_TCD3_SADDR = &UART2_D;
        // Restore source address after major loop
        DMA_TCD3_SLAST = 0;
        // source address offset (0 bytes)
        DMA_TCD3_SOFF = 0;
        // Destination address
        DMA_TCD3_DADDR = rxBuffer;
        // Destination offset (0 byte)
        DMA_TCD3_DOFF = 1;
        if (termCharacter == 0) {
            // Restore destination address after major loop
            DMA_TCD3_DLASTSGA = -(rxBufferSize);
            // Set loop counts / channel2channel linking disabled
            DMA_TCD3_CITER_ELINKNO = rxBufferSize;//UART0_RWFIFO;
            DMA_TCD3_BITER_ELINKNO = rxBufferSize;//UART0_RWFIFO;
            term_rx3 = 0;
        }
        else {
            // Restore destination address after major loop
            DMA_TCD3_DLASTSGA = 0;
            // Set loop counts / channel2channel linking disabled
            DMA_TCD3_CITER_ELINKNO = 1;//UART0_RWFIFO;
            DMA_TCD3_BITER_ELINKNO = 1;//UART0_RWFIFO;
            term_rx3 = termCharacter;
            bufSize_rx3 = rxBufferSize;
            zeroptr_rx3 = (uintptr_t*)DMA_TCD3_DADDR;
            currentptr_rx3 = zeroptr_rx3;
            //Serial.printf("DMA_TCD3_DLASTSGA: %i\t| DMA_TCD3_DADDR: %p | zeroptr_rx3: %p\n\n",  DMA_TCD3_DLASTSGA, DMA_TCD3_DADDR, zeroptr_rx3);
        }
        // Source and destination size 8 bit
        DMA_TCD3_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
        // Number of bytes to transfer (in each service request)
        DMA_TCD3_NBYTES_MLNO = 1;
        // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
        DMA_TCD3_CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
        // Set Serial3 as source (CH 3), enable DMA MUX
        DMAMUX0_CHCFG3 = DMAMUX_DISABLE;
        DMAMUX0_CHCFG3 = DMAMUX_SOURCE_UART2_RX | DMAMUX_ENABLE;
        // Enable DMA Request 3
        DMA_ERQ |= DMA_ERQ_ERQ3;
        // enable irq CH3
        NVIC_SET_PRIORITY(IRQ_DMA_CH3, IRQ_PRIORITY);
        NVIC_ENABLE_IRQ(IRQ_DMA_CH3);
        return 1;
    }
    else {
        return -1;
    }
}
// -----------------------------------------------end----------------------------------------------------------------
int SerialEvent::dma_end( void ) {
    if (!(SIM_SCGC7 & SIM_SCGC7_DMA)) return -1;
    if (!(SIM_SCGC6 & SIM_SCGC6_DMAMUX)) return -1;
    // make sure all queued packets all transmeitted and nothing
    // is being read into each rx port.
    while ( txCount > 0 ) yield();
    while (SEND_DONE_TX) yield();
    delay(20);
    if (dma_ch_enabled[SERIAL1]) {
        while (RECIEVE_DONE_RX1) yield();
    }
    if (dma_ch_enabled[SERIAL2]) {
        while (RECIEVE_DONE_RX2) yield();
    }
    if (dma_ch_enabled[SERIAL3]) {
        while (RECIEVE_DONE_RX3) yield();
    }
    if (this->current_port == SERIAL1) {
        NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
        Serial1.end();
        // clear Serial1 dma enable rx/tx bits
        UART0_C5 = UART_DMA_DISABLE;
        dma_ch_enabled[SERIAL1] = false;
    }
    else if (this->current_port == SERIAL2) {
        NVIC_DISABLE_IRQ(IRQ_DMA_CH2);
        Serial2.end();
        // clear Serial2 dma enable rx/tx bits
        UART1_C5 = UART_DMA_DISABLE;
        dma_ch_enabled[SERIAL2] = false;
    }
    else if (this->current_port == SERIAL3) {
        NVIC_DISABLE_IRQ(IRQ_DMA_CH2);
        Serial3.end();
        // clear Serial3 dma enable rx/tx bits
        UART2_C5 = UART_DMA_DISABLE;
        dma_ch_enabled[SERIAL3] = false;
    }
    else {
        return -1;
    }
    bool  allDone = ( !( dma_ch_enabled[1]) && !(dma_ch_enabled[2]) && !(dma_ch_enabled[3]) );
    if (allDone) {
        NVIC_DISABLE_IRQ(IRQ_DMA_CH0);
        NVIC_DISABLE_IRQ(IRQ_DMA_CH1);
        NVIC_DISABLE_IRQ(IRQ_DMA_CH2);
        NVIC_DISABLE_IRQ(IRQ_DMA_CH3);
        txHead = 0;
        txTail = 0;
        txCount = 0;
    }
    return this->current_port;
}
 // --------------------------------------------available------------------------------------------------------------
int SerialEvent::dma_available(void) {
    if ( this->current_port == SERIAL1 ) {
        uint32_t head, tail;
        head = this->rxBufferSize - DMA_TCD1_CITER_ELINKNO;
        tail = this->rxTail;
        if (head >= tail) return head - tail;
        return this->rxBufferSize + head - tail;
    }
    else if ( this->current_port == SERIAL2 ) {
        uint32_t head, tail;
        head = this->rxBufferSize - DMA_TCD2_CITER_ELINKNO;
        tail = this->rxTail;
        if (head >= tail) return head - tail;
        return this->rxBufferSize + head - tail;
    }
    else if ( this->current_port == SERIAL3 ) {
        uint32_t head, tail;
        head = this->rxBufferSize - DMA_TCD3_CITER_ELINKNO;
        tail = this->rxTail;
        if (head >= tail) return head - tail;
        return this->rxBufferSize + head - tail;
    }
    else return -1;
}
// ----------------------------------------------getchar-------------------------------------------------------------
int SerialEvent::dma_getchar(void) {
    if ( this->current_port == SERIAL1 ) {
        uint32_t head, tail;
        int c;
        head = this->rxBufferSize - DMA_TCD1_CITER_ELINKNO;
        tail = this->rxTail;
        c = this->rxBuffer[tail];
        if (head == tail) return -1;
        if (++tail >= this->rxBufferSize) tail = 0;
        this->rxTail = tail;
        return c;
    }
    else if ( this->current_port == SERIAL2 ) {
        uint32_t head, tail;
        int c;
        head = this->rxBufferSize - DMA_TCD2_CITER_ELINKNO;
        tail = this->rxTail;
        c = this->rxBuffer[tail];
        if (head == tail) return -1;
        if (++tail >= this->rxBufferSize) tail = 0;
        this->rxTail = tail;
        return c;
    }
    else if ( this->current_port == SERIAL3 ) {
        uint32_t head, tail;
        int c;
        head = this->rxBufferSize - DMA_TCD3_CITER_ELINKNO;
        tail = this->rxTail;
        c = this->rxBuffer[tail];
        if (head == tail) return -1;
        if (++tail >= this->rxBufferSize) tail = 0;
        this->rxTail = tail;
        return c;
    }
    else return -1;
}
// -----------------------------------------------peek---------------------------------------------------------------
int SerialEvent::dma_peek( void ) {
    uint32_t head, tail;
    int c;
    if ( this->current_port == SERIAL1 ) {
        head = rxBufferSize - DMA_TCD1_CITER_ELINKNO;
    }
    else if ( this->current_port == SERIAL2 ) {
        head = rxBufferSize - DMA_TCD2_CITER_ELINKNO;
    }
    else if ( this->current_port == SERIAL3 ) {
        head = rxBufferSize - DMA_TCD3_CITER_ELINKNO;
    }
    else return -1;
    tail = rxTail;
    c = rxBuffer[tail];
    if (head == tail) return -1;
    if (++tail >= rxBufferSize) tail = 0;
    return c;
    /*if ( this->current_port == SERIAL1 ) {
        head = rxBufferSize - DMA_TCD1_CITER_ELINKNO;
        tail = rxTail;
        c = rxBuffer[tail];
        if (head == tail) return -1;
        if (++tail >= rxBufferSize) tail = 0;
        return c;
    }
    else if ( this->current_port == SERIAL2 ) {
        head = this->rxBufferSize - DMA_TCD2_CITER_ELINKNO;
        tail = this->rxTail;
        c = this->rxBuffer[tail];
        if (head == tail) return -1;
        if (++tail >= this->rxBufferSize) tail = 0;
        return c;
    }
    else if ( this->current_port == SERIAL3 ) {
        head = this->rxBufferSize - DMA_TCD3_CITER_ELINKNO;
        tail = this->rxTail;
        if (head == tail) return -1;
        if (++tail >= this->rxBufferSize) tail = 0;
        return this->rxBuffer[tail];
    }*/
}
// -----------------------------------------------flush--------------------------------------------------------------
int SerialEvent::dma_flush( void ) {
    while ( SerialEvent::txCount > 0 ) yield();
    while (SEND_DONE_TX) yield();
    return 1;
}
// -----------------------------------------------clear--------------------------------------------------------------
int SerialEvent::dma_clear( void ) {
    //DMA_CR |= DMA_CR_HALT;
    int fifoSize = TX_FIFO_SIZE+1;
    int i = 0;
    txCount = 0;
    while (SEND_DONE_TX) yield();
    while (--fifoSize) {
        Serial.printf("fifoSize: %i\n", i);
        tx_fifo_t* queued = &txfifo[i];
        if (queued->allocated) {
            // update memory var, since we are releasing it here
            _memory += queued->size;
            // free memory of already transmitted packet
            delete [] queued->packet;
            queued->allocated = false;
        }
        i++;
    }
    txTail = txHead = 0;
    //DMA_CR &= ~DMA_CR_HALT;
    return 1;
}
// -----------------------------------------------end----------------------------------------------------------------
inline int SerialEvent::dma_write( const uint8_t* data, uint32_t size )  {
    if (unlikely(!(SIM_SCGC7 & SIM_SCGC7_DMA))) return -1;
    if (unlikely(!(SIM_SCGC6 & SIM_SCGC6_DMAMUX))) return -1;
    //-----------------------------------------------------------------
    // if fifo counter is >= than fifo size, do not add more serial packets to buffer;
    int mem = _memory;
    mem -= size;
    if (txCount >= (TX_FIFO_SIZE) || (mem <= 0) ) { return -1; }
    _memory = mem;
    //-----------------------------------------------------------------
    // here we insert new items into the fifo circular buffer
    tx_fifo_t* p = &txfifo[txHead];
    // get out if already allocated
    if (p->allocated) {
        Serial.println("ERROR: ALLOCATE FAILED");
        return -1;
    }
    p->port = this->current_port;
    // store current packet size in the fifo buffer
    p->size = size;
    // update allocated
    p->allocated = true;
    // update tx event callback for specific port
    p->txEventHandler = txEventHandler;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        // make some space
        p->packet = new char[size];
        // fill fifo buffer with user packet
        memcpy(p->packet, data, size);
    }
    // packet count
    txCount++;
    // fifo head
    txHead = txHead < (TX_FIFO_SIZE - 1) ? txHead + 1 : 0;
    //------------------------------------------------------------------
    // return if packets are still in transmission
    if(!txDone) return txCount;
    //------------------------------------------------------------------
    if ( this->current_port == SERIAL1 ) {
        // DMA transfer requests enabled
        BITBAND_U8(UART0_C2, 7) = 0x01;
        //UART0_C2 |= UART_C2_TIE;
        // Destination address
        DMA_TCD0_DADDR = &UART0_D;
        DMAMUX0_CHCFG0 &= ~0x3F;
        DMAMUX0_CHCFG0 |= DMAMUX_SOURCE_UART0_TX;
    }
    else if ( this->current_port == SERIAL2 ) {
        // DMA transfer requests enabled
        BITBAND_U8(UART1_C2, 7) = 0x01;
        //UART1_C2 |= UART_C2_TIE;
        // Destination address
        DMA_TCD0_DADDR = &UART1_D;
        // Set UART as source (CH 0)
        DMAMUX0_CHCFG0 &= ~0x3F;
        DMAMUX0_CHCFG0 |= DMAMUX_SOURCE_UART1_TX;
    }
    else if ( this->current_port == SERIAL3 ) {
        // DMA transfer requests enabled
        BITBAND_U8(UART2_C2, 7) = 0x01;
        //UART2_C2 |= UART_C2_TIE;
        // Destination address
        DMA_TCD0_DADDR = &UART2_D;
        // Set UART as source (CH 0)
        DMAMUX0_CHCFG0 &= ~0x3F;
        DMAMUX0_CHCFG0 |= DMAMUX_SOURCE_UART2_TX;
    }
    else {
        return -1;
    }
    // set minor loop counts
    DMA_TCD0_CITER_ELINKNO = p->size;
    DMA_TCD0_BITER_ELINKNO = p->size;
    // source address
    DMA_TCD0_SADDR = p->packet;
    BITBAND_U32(DMA_ERQ, 0x00) = 0x01;
    //------------------------------------------------------------------
    // siganl packet transmission in progress, start adding to buffer
    txDone = false;
    return txCount;
}

/*void SerialEvent::memcpy8(volatile char *dest, const char *src, unsigned int count) {
    //while (txCount > 0) yield();
    //while (!(DMA_TCD3_CSR & DMA_TCD_CSR_DONE)) yield();
    DMA_TCD3_SADDR = src;
    DMA_TCD3_SOFF = 1;
    DMA_TCD3_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0); //8bit
    DMA_TCD3_NBYTES_MLNO = count;
    DMA_TCD3_SLAST = 0;
    DMA_TCD3_DADDR = dest;
    DMA_TCD3_DOFF = 1;
    DMA_TCD3_CITER_ELINKNO = 1;
    DMA_TCD3_DLASTSGA = 0;
    DMA_TCD3_BITER_ELINKNO = 1;
    DMA_TCD3_CSR = 0;
    DMA_SSRT = DMA_SSRT_SSRT(3);
    //while (SEND_DONE_TX) yield();
    dest[count] = 0x00;
    dma_RX_begin();
}
 */


