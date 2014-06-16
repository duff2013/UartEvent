/*
 ||
 || @file 		Serial3Event.cpp
 || @version 	3
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
// -------------------------------------------Serial3------------------------------------------
#define SEND_DONE_TX        !(DMA_TCD8_CSR & DMA_TCD_CSR_DONE)
#define RECIEVE_DONE_RX     !(DMA_TCD9_CSR & DMA_TCD_CSR_DONE)

#define SCGC4_UART2_BIT     12

Serial3Event::ISR   Serial3Event::RX_CALLBACK;
Serial3Event::ISR   Serial3Event::TX_CALLBACK;

volatile int        Serial3Event::txFifoCount;
volatile int        Serial3Event:: term_rx_character;
volatile uint32_t   Serial3Event::txHead;
volatile uint32_t   Serial3Event::txTail;
volatile uint32_t   Serial3Event::rxHead;
volatile uint32_t   Serial3Event::rxTail;
volatile uint16_t   Serial3Event::bufSize_rx;
volatile uint16_t   Serial3Event::RX_BUFFER_SIZE;
volatile uint8_t   *Serial3Event::transmit_pin;
volatile uint8_t    Serial3Event::TX_FIFO_SIZE;
volatile uint8_t   *Serial3Event::rxBuffer;
volatile boolean    Serial3Event::txDone;
volatile uintptr_t *Serial3Event::currentptr_rx;
volatile uintptr_t *Serial3Event::zeroptr_rx;
tx3_fifo_t         *Serial3Event::tx_memory_pool;

char               *Serial3Event::term_rx_string;
volatile uint8_t    Serial3Event::txUsedMemory;
volatile uint8_t    Serial3Event::rxUsedMemory;
// ------------Serial1 ISR----------------
void dma_ch8_isr() {
    //Serial.printf("ISR   | count: %02i | head: %02i | tail: %02i\n",Serial3Event::txFifoCount, Serial3Event::txHead, Serial3Event::txTail);
    // clear dma interrupt request 0
    DMA_CINT |= DMA_CINT_CINT(8);
    int tail  = Serial3Event::txTail;
    Serial3Event::txFifoCount--;
    // increment fifo tail
    Serial3Event::txTail = Serial3Event::txTail < (Serial3Event::TX_FIFO_SIZE - 1) ? Serial3Event::txTail + 1 : 0;
    /***********************************************************************************
     / here if the fifo has packets to send, we setup the dma destination address       /
     / and source registers for transmission on the queued serial port. We will         /
     / keep transmitting these queued packets in the "background" until the fifo        /
     / is exhausted.                                                                    /
     ***********************************************************************************/
    if ( Serial3Event::txFifoCount > 0 ) {
        tx3_fifo_t* queued = &Serial3Event::tx_memory_pool[Serial3Event::txTail];
        // Number of bytes to transfer (in each service request)
        DMA_TCD8_CITER_ELINKNO = queued->size;
        DMA_TCD8_BITER_ELINKNO = queued->size;
        DMA_TCD8_SADDR = queued->packet;
        DMA_SERQ = 8;
    } else {
        // no more packets to transmitt
        Serial3Event::txDone = true;
    }
    tx3_fifo_t* sent = &Serial3Event::tx_memory_pool[tail];
    // user or default tx callback event
    if (sent->eventTrigger) Serial3Event::TX_CALLBACK();
}

void dma_ch9_isr() {
    // Clear Interrupt Request 1
    DMA_CINT |= DMA_CINT_CINT(9);
    //BITBAND_U32(DMA_ERQ, 5) = 0x01;
    //DMA_ERQ |= DMA_ERQ_ERQ1;
    if (Serial3Event::term_rx_character != -1) {
        //int len=sizeof(Serial3Event::termChar);
        /*uint8_t *tmpterm = Serial3Event::term1_rx;
         int inc = 0;
         Serial.printf("size: %i\n ", termSize);
         do {
         //Serial.println((char)*tmpterm++);
         if (*tmpterm++ == (uint8_t)*Serial3Event::currentptr_rx ) Serial.println("Hello2");
         } while (--termSize) ;*/
        static uint32_t byteCount_rx = 0;
        Serial3Event::rxUsedMemory = (100*byteCount_rx)/Serial3Event::RX_BUFFER_SIZE;
        //Serial.printf("rxUsedMemory: %i\n", Serial3Event::rxUsedMemory);
        if (( (uint8_t)*Serial3Event::currentptr_rx == Serial3Event:: term_rx_character) || ( byteCount_rx == (Serial3Event::RX_BUFFER_SIZE-1) )) {
            Serial3Event::currentptr_rx = (uintptr_t*)DMA_TCD9_DADDR;
            *Serial3Event::currentptr_rx = 0;
            DMA_TCD9_DADDR = Serial3Event::zeroptr_rx;
            byteCount_rx = 0;
            Serial3Event::RX_CALLBACK();
        }
        else { ++byteCount_rx; }
        Serial3Event::currentptr_rx = (uintptr_t*)DMA_TCD9_DADDR;
    }
    else {
        // user or default rx1 callback event
        Serial3Event::RX_CALLBACK();
    }
    // Enable DMA Request 1
    DMA_SERQ = 9;
}

void Serial3Event::serial_dma_begin(uint32_t divisor) {
    TX_CALLBACK = txEventHandler;
    RX_CALLBACK = rxEventHandler;
    // Enable DMA, DMAMUX and UART2 clocks
    BITBAND_U32(SIM_SCGC7, SCGC7_DMA_BIT)       = 0x01;
    BITBAND_U32(SIM_SCGC6, SCGC6_DMAMUX_BIT)    = 0x01;
    BITBAND_U32(SIM_SCGC4, SCGC4_UART2_BIT)     = 0x01;
    /****************************************************************
     * some code lifted from Teensyduino Core serial3.c
     ****************************************************************/
    CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
	CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
	UART2_BDH = (divisor >> 13) & 0x1F;
	UART2_BDL = (divisor >> 5) & 0xFF;
	UART2_C4 = divisor & 0x1F;
	UART2_C1 = 0;//UART_C1_ILT;
    // TODO: Use UART2 fifo with dma
	//UART2_TWFIFO = 1; // tx watermark, causes C5_TDMAS DMA request
	//UART2_RWFIFO = 1; // rx watermark, causes C5_RDMAS DMA request
	UART2_PFIFO = 0;//UART_PFIFO_TXFE | UART_PFIFO_RXFE;
    UART2_C2 = C2_TX_INACTIVE;
    UART2_C5 = UART_DMA_ENABLE; // setup Serial1 tx,rx to use dma
    if (loopBack) UART2_C1 |= UART_C1_LOOPS; // Set internal loop1Back
    /****************************************************************
     * DMA TX setup
     ****************************************************************/
    // Use default configuration
    if (DMA_CR != 0) DMA_CR = 0;
    // Destination address
    DMA_TCD8_DADDR = &UART2_D;
    // Restore source address after major loop
    DMA_TCD8_SLAST = 0;
    // source address offset (1 byte)
    DMA_TCD8_SOFF = 1;
    // Destination offset (0 byte)
    DMA_TCD8_DOFF = 0;
    // Restore destination address after major loop
    DMA_TCD8_DLASTSGA = 0;
    // Source and destination size 8 bit
    DMA_TCD8_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    // Number of bytes to transfer (in each service request)
    DMA_TCD8_NBYTES_MLNO = 1;//UART2_TWFIFO;
    // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
    //DMA_TCD8_CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    DMA_TCD8_CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    DMAMUX0_CHCFG8 = DMAMUX_DISABLE;
    // enable DMA MUX
    DMAMUX0_CHCFG8 = DMAMUX_SOURCE_UART2_TX | DMAMUX_ENABLE;
    // set TX priority
    NVIC_SET_PRIORITY(IRQ_DMA_CH8, IRQ_PRIORITY);
    // enable irq
    NVIC_ENABLE_IRQ(IRQ_DMA_CH8);
    /****************************************************************
     * DMA RX setup
     ****************************************************************/
    // Source address
    DMA_TCD9_SADDR = &UART2_D;
    // Restore source address after major loop
    DMA_TCD9_SLAST = 0;
    // source address offset (0 bytes)
    DMA_TCD9_SOFF = 0;
    // Destination address
    DMA_TCD9_DADDR = &rxBuffer[0];
    // Destination offset (0 byte)
    DMA_TCD9_DOFF = 1;
    if (rxTermCharacter == -1) {
        // Restore destination address after major loop
        DMA_TCD9_DLASTSGA = -RX_BUFFER_SIZE;
        // Set loop counts / channel2channel linking disabled
        DMA_TCD9_CITER_ELINKNO = RX_BUFFER_SIZE;//UART2_RWFIFO;
        DMA_TCD9_BITER_ELINKNO = RX_BUFFER_SIZE;//UART2_RWFIFO;
        term_rx_character = -1;
    }
    else {
        // Restore destination address after major loop
        DMA_TCD9_DLASTSGA = 0;
        // Set loop counts / channel2channel linking disabled
        DMA_TCD9_CITER_ELINKNO = 1;//UART2_RWFIFO;
        DMA_TCD9_BITER_ELINKNO = 1;//UART2_RWFIFO;
        term_rx_string = rxTermString;
        term_rx_character = rxTermCharacter;
        bufSize_rx = RX_BUFFER_SIZE;
        zeroptr_rx = (uintptr_t*)DMA_TCD9_DADDR;
        currentptr_rx = zeroptr_rx;
    }
    // Source and destination size 8 bit
    DMA_TCD9_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    // Number of bytes to transfer (in each service request)
    DMA_TCD9_NBYTES_MLNO = 1;//UART2_TWFIFO;
    // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
    DMA_TCD9_CSR = DMA_TCD_CSR_INTMAJOR  | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    // Set Serial1 as source (CH 1), enable DMA MUX
    DMAMUX0_CHCFG9 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG9 = DMAMUX_SOURCE_UART2_RX | DMAMUX_ENABLE;
    // Enable DMA Request 1
    DMA_ERQ |= DMA_ERQ_ERQ9;
    // enable irq CH1
    NVIC_SET_PRIORITY(IRQ_DMA_CH9, IRQ_PRIORITY);
    NVIC_ENABLE_IRQ(IRQ_DMA_CH9);
}

void Serial3Event::serial_dma_format(uint32_t format) {
    /****************************************************************
     * serial1 format, from teensduino core, serial1.c
     ****************************************************************/
    uint8_t c;
    c = UART2_C1;
    c = (c & ~0x13) | (format & 0x03);      // configure parity
    if (format & 0x04) c |= 0x10;           // 9 bits (might include parity)
    UART2_C1 = c;
    if ((format & 0x0F) == 0x04) UART2_C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
    c = UART2_S2 & ~0x10;
    if (format & 0x10) c |= 0x10;           // rx invert
    UART2_S2 = c;
    c = UART2_C3 & ~0x10;
    if (format & 0x20) c |= 0x10;           // tx invert
    UART2_C3 = c;
}

void Serial3Event::serial_dma_end(void) {
    if (!(SIM_SCGC7 & SIM_SCGC7_DMA)) return;
    if (!(SIM_SCGC6 & SIM_SCGC6_DMAMUX)) return;
    if (!(SIM_SCGC4 & SIM_SCGC4_UART2)) return;
    while ( txFifoCount > 0 ) yield();
    while (SEND_DONE_TX) yield();
    delay(20);
    /****************************************************************
     * serial1 end, from teensduino core, serial1.c
     ****************************************************************/
    UART2_C2 = 0;
	CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
	CORE_PIN8_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
    // clear Serial1 dma enable rx/tx bits
    UART2_C5 = UART_DMA_DISABLE;
    rxHead = rxTail = 0;
}

void Serial3Event::serial_dma_set_transmit_pin(uint8_t pin) {
    // TODO: need to update var when finish transmitting serial for RS485
    pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	transmit_pin = portOutputRegister(pin);
}

void Serial3Event::serial_dma_putchar(uint32_t c) {
    serial_dma_write(&c, 1);
}

void Serial3Event::serial_dma_write(const void *buf, unsigned int count) {
    const uint8_t *buffer = (const uint8_t *)buf;
    int fifoSlots = count/TX3_PACKET_SIZE;
    int remainder = count%TX3_PACKET_SIZE;
    int totalPacks = fifoSlots + txFifoCount + 1;
    term_rx_character = rxTermCharacter;
    if ( totalPacks > TX_FIFO_SIZE ) return;
    if ( fifoSlots != 0 ) {
        for (int i = 0; i < fifoSlots; i++) {
            // start inserting new items into the fifo circular buffer
            tx3_fifo_t* p = &tx_memory_pool[txHead];
            volatile uint8_t* fifopack = p->packet;
            // store current packet size in the fifo buffer
            p->size = TX3_PACKET_SIZE;
            // update tx event trigger
            p->eventTrigger = false;
            int pakSize = TX3_PACKET_SIZE;
            do {
                *fifopack++ = *buffer++;
            } while ( --pakSize ) ;
            // packet count
            ++txFifoCount;
            txHead = txHead < ( TX_FIFO_SIZE - 1 ) ? txHead + 1 : 0;
        }
        if ( remainder > 0 ) {
            // start inserting new items into the fifo circular buffer
            tx3_fifo_t* p = &tx_memory_pool[txHead];
            volatile uint8_t* fifopack = p->packet;
            // store current packet size in the fifo buffer
            p->size = remainder ;
            // update tx event trigger
            p->eventTrigger = true;
            do {
                *fifopack++ = *buffer++;
            } while ( --remainder ) ;
            p->packet[p->size] = 0;
            // packet count
            ++txFifoCount;
            txHead = txHead < ( TX_FIFO_SIZE - 1 ) ? txHead + 1 : 0;
        }
    }
    else {
        // start inserting new items into the fifo circular buffer
        tx3_fifo_t* p = &tx_memory_pool[txHead];
        volatile uint8_t* fifopack = p->packet;
        // store current packet size in the fifo buffer
        p->size = count;
        // update tx event trigger
        p->eventTrigger = true;
        int pakSize = p->size;
        do {
            *fifopack++ = *buffer++;
        } while ( --pakSize ) ;
        // packet count
        ++txFifoCount;
        txHead = txHead < ( TX_FIFO_SIZE - 1 ) ? txHead + 1 : 0;
    }
    //------------------------------------------------------------------
    // return if packets are still in transmission
    if(!txDone) return;
    //------------------------------------------------------------------
    //Serial.printf("WRITE | count: %02i | head: %02i | tail: %02i\n",txFifoCount, txHead, txTail);
    __disable_irq();
    tx3_fifo_t* p = &tx_memory_pool[txTail];
    // DMA transfer requests enabled
    UART2_C2 |= UART_C2_TIE;
    // set minor loop counts
    DMA_TCD8_CITER_ELINKNO = p->size;
    DMA_TCD8_BITER_ELINKNO = p->size;
    // source address
    DMA_TCD8_SADDR = p->packet;
    DMA_SERQ = 8;
    //------------------------------------------------------------------
    // siganl packet transmission in progress, start adding to buffer
    txDone = false;
    __enable_irq();
    //return txFifoCount;
    
}

void Serial3Event::serial_dma_flush(void) {
    while ( txFifoCount != 0 ) yield();
    while ( SEND_DONE_TX ) yield();
}

int Serial3Event::serial_dma_available(void) {
    uint32_t head, tail;
    head = RX_BUFFER_SIZE - DMA_TCD9_CITER_ELINKNO;
    tail = rxTail;
    if (head >= tail) return head - tail;
    return RX_BUFFER_SIZE + head - tail;
}

int Serial3Event::serial_dma_getchar(void) {
    uint32_t head, tail;
    int c;
    head = RX_BUFFER_SIZE - DMA_TCD9_CITER_ELINKNO;
    tail = rxTail;
    c = rxBuffer[tail];
    if (head == tail) return -1;
    if (++tail >= RX_BUFFER_SIZE) tail = 0;
    rxTail = tail;
    return c;
}

int Serial3Event::serial_dma_peek(void) {
    uint32_t head;
    head = RX_BUFFER_SIZE - DMA_TCD9_CITER_ELINKNO;
    return head;
}

void Serial3Event::serial_dma_clear(void) {
    
}

void Serial3Event::serial_dma_print(const char *p) {
    
}

void Serial3Event::serial_dma_phex(uint32_t n) {
    
}

void Serial3Event::serial_dma_phex16(uint32_t n) {
    
}

void Serial3Event::serial_dma_phex32(uint32_t n) {
    
}