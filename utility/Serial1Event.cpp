/*
 ||
 || @file 		Serial1Event.cpp
 || @version 	1
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
// -------------------------------------------Serial1------------------------------------------
#define SEND_DONE_TX        !(DMA_TCD4_CSR & DMA_TCD_CSR_DONE)
#define RECIEVE_DONE_RX     !(DMA_TCD5_CSR & DMA_TCD_CSR_DONE)

#define SCGC4_UART0_BIT     10

Serial1Event::ISR   Serial1Event::RX_CALLBACK;
Serial1Event::ISR   Serial1Event::TX_CALLBACK;

volatile int        Serial1Event::txFifoCount;
volatile int        Serial1Event:: term_rx_character;
volatile uint32_t   Serial1Event::txHead;
volatile uint32_t   Serial1Event::txTail;
volatile uint32_t   Serial1Event::rxHead;
volatile uint32_t   Serial1Event::rxTail;
volatile uint16_t   Serial1Event::bufSize_rx;
volatile uint16_t   Serial1Event::RX_BUFFER_SIZE;
volatile uint8_t   *Serial1Event::transmit_pin;
volatile uint8_t    Serial1Event::TX_FIFO_SIZE;
volatile uint8_t   *Serial1Event::rxBuffer;
volatile boolean       Serial1Event::txDone;
volatile uintptr_t *Serial1Event::currentptr_rx;
volatile uintptr_t *Serial1Event::zeroptr_rx;
tx1_fifo_t         *Serial1Event::tx_memory_pool;

char               *Serial1Event::term_rx_string;
volatile uint8_t    Serial1Event::txUsedMemory;
volatile uint8_t    Serial1Event::rxUsedMemory;
// ------------Serial1 ISR----------------
void dma_ch4_isr() {
    //Serial.printf("ISR   | count: %02i | head: %02i | tail: %02i\n",Serial1Event::txFifoCount, Serial1Event::txHead, Serial1Event::txTail);
    // clear dma interrupt request 0
    DMA_CINT |= DMA_CINT_CINT(4);
    int tail  = Serial1Event::txTail;
    Serial1Event::txFifoCount--;
    // increment fifo tail
    Serial1Event::txTail = Serial1Event::txTail < (Serial1Event::TX_FIFO_SIZE - 1) ? Serial1Event::txTail + 1 : 0;
    /***********************************************************************************
     / here if the fifo has packets to send, we setup the dma destination address       /
     / and source registers for transmission on the queued serial port. We will         /
     / keep transmitting these queued packets in the "background" until the fifo        /
     / is exhausted.                                                                    /
     ***********************************************************************************/
    if ( Serial1Event::txFifoCount > 0 ) {
        tx1_fifo_t* queued = &Serial1Event::tx_memory_pool[Serial1Event::txTail];
        // Number of bytes to transfer (in each service request)
        DMA_TCD4_CITER_ELINKNO = queued->size;
        DMA_TCD4_BITER_ELINKNO = queued->size;
        DMA_TCD4_SADDR = queued->packet;
        DMA_SERQ = 4;
    } else {
        // no more packets to transmitt
        Serial1Event::txDone = true;
    }
    tx1_fifo_t* sent = &Serial1Event::tx_memory_pool[tail];
    // user or default tx callback event
    if (sent->eventTrigger) Serial1Event::TX_CALLBACK();
}

void dma_ch5_isr() {
    // Clear Interrupt Request 1
    DMA_CINT |= DMA_CINT_CINT(5);
    //BITBAND_U32(DMA_ERQ, 5) = 0x01;
    //DMA_ERQ |= DMA_ERQ_ERQ1;
    if (Serial1Event::term_rx_character != -1) {
        //int len=sizeof(Serial1Event::termChar);
        /*uint8_t *tmpterm = Serial1Event::term1_rx;
        int inc = 0;
        Serial.printf("size: %i\n ", termSize);
        do {
            //Serial.println((char)*tmpterm++);
            if (*tmpterm++ == (uint8_t)*Serial1Event::currentptr_rx ) Serial.println("Hello2");
        } while (--termSize) ;*/
        static uint32_t byteCount_rx = 0;
        Serial1Event::rxUsedMemory = (100*byteCount_rx)/Serial1Event::RX_BUFFER_SIZE;
        //Serial.printf("rxUsedMemory: %i\n", Serial1Event::rxUsedMemory);
        if (( (uint8_t)*Serial1Event::currentptr_rx == Serial1Event:: term_rx_character) || ( byteCount_rx == (Serial1Event::RX_BUFFER_SIZE-1) )) {
            Serial1Event::currentptr_rx = (uintptr_t*)DMA_TCD5_DADDR;
            *Serial1Event::currentptr_rx = 0;
            DMA_TCD5_DADDR = Serial1Event::zeroptr_rx;
            byteCount_rx = 0;
            Serial1Event::RX_CALLBACK();
        }
        else { ++byteCount_rx; }
        Serial1Event::currentptr_rx = (uintptr_t*)DMA_TCD5_DADDR;
    }
    else {
        // user or default rx1 callback event
        Serial1Event::RX_CALLBACK();
    }
    // Enable DMA Request 1
    DMA_SERQ = 5;
}

void Serial1Event::serial_dma_begin(uint32_t divisor) {
    TX_CALLBACK = txEventHandler;
    RX_CALLBACK = rxEventHandler;
    // Enable DMA, DMAMUX and UART0 clocks
    BITBAND_U32(SIM_SCGC7, SCGC7_DMA_BIT)       = 0x01;
    BITBAND_U32(SIM_SCGC6, SCGC6_DMAMUX_BIT)    = 0x01;
    BITBAND_U32(SIM_SCGC4, SCGC4_UART0_BIT)     = 0x01;
    /****************************************************************
     * some code lifted from Teensyduino Core serial1.c
     ****************************************************************/
    CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3);
	CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3);
	UART0_BDH = (divisor >> 13) & 0x1F;
	UART0_BDL = (divisor >> 5) & 0xFF;
	UART0_C4 = divisor & 0x1F;
	UART0_C1 = UART_C1_ILT;
    // TODO: Use UART0 fifo with dma
	UART0_TWFIFO = 1; // tx watermark, causes C5_TDMAS DMA request
	UART0_RWFIFO = 1; // rx watermark, causes C5_RDMAS DMA request
	UART0_PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;
    UART0_C2 = C2_TX_INACTIVE;
    UART0_C5 = UART_DMA_ENABLE; // setup Serial1 tx,rx to use dma
    if (loopBack) UART0_C1 |= UART_C1_LOOPS; // Set internal loop1Back
    /****************************************************************
     * DMA TX setup
     ****************************************************************/
    // Use default configuration
    if (DMA_CR != 0) DMA_CR = 0;
    // Destination address
    DMA_TCD4_DADDR = &UART0_D;
    // Restore source address after major loop
    DMA_TCD4_SLAST = 0;
    // source address offset (1 byte)
    DMA_TCD4_SOFF = 1;
    // Destination offset (0 byte)
    DMA_TCD4_DOFF = 0;
    // Restore destination address after major loop
    DMA_TCD4_DLASTSGA = 0;
    // Source and destination size 8 bit
    DMA_TCD4_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    // Number of bytes to transfer (in each service request)
    DMA_TCD4_NBYTES_MLNO = 1;//UART0_TWFIFO;
    // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
    //DMA_TCD4_CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    DMA_TCD4_CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    DMAMUX0_CHCFG4 = DMAMUX_DISABLE;
    // enable DMA MUX
    DMAMUX0_CHCFG4 = DMAMUX_SOURCE_UART0_TX | DMAMUX_ENABLE;
    // set TX priority
    NVIC_SET_PRIORITY(IRQ_DMA_CH4, IRQ_PRIORITY);
    // enable irq
    NVIC_ENABLE_IRQ(IRQ_DMA_CH4);
    /****************************************************************
     * DMA RX setup
     ****************************************************************/
    // Source address
    DMA_TCD5_SADDR = &UART0_D;
    // Restore source address after major loop
    DMA_TCD5_SLAST = 0;
    // source address offset (0 bytes)
    DMA_TCD5_SOFF = 0;
    // Destination address
    DMA_TCD5_DADDR = &rxBuffer[0];
    // Destination offset (0 byte)
    DMA_TCD5_DOFF = 1;
    if (rxTermCharacter == -1) {
        // Restore destination address after major loop
        DMA_TCD5_DLASTSGA = -RX_BUFFER_SIZE;
        // Set loop counts / channel2channel linking disabled
        DMA_TCD5_CITER_ELINKNO = RX_BUFFER_SIZE;//UART0_RWFIFO;
        DMA_TCD5_BITER_ELINKNO = RX_BUFFER_SIZE;//UART0_RWFIFO;
        term_rx_character = -1;
    }
    else {
        // Restore destination address after major loop
        DMA_TCD5_DLASTSGA = 0;
        // Set loop counts / channel2channel linking disabled
        DMA_TCD5_CITER_ELINKNO = 1;//UART0_RWFIFO;
        DMA_TCD5_BITER_ELINKNO = 1;//UART0_RWFIFO;
        term_rx_string = rxTermString;
        term_rx_character = rxTermCharacter;
        bufSize_rx = RX_BUFFER_SIZE;
        zeroptr_rx = (uintptr_t*)DMA_TCD5_DADDR;
        currentptr_rx = zeroptr_rx;
    }
    // Source and destination size 8 bit
    DMA_TCD5_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    // Number of bytes to transfer (in each service request)
    DMA_TCD5_NBYTES_MLNO = 1;//UART0_TWFIFO;
    // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
    DMA_TCD5_CSR = DMA_TCD_CSR_INTMAJOR  | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    // Set Serial1 as source (CH 1), enable DMA MUX
    DMAMUX0_CHCFG5 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG5 = DMAMUX_SOURCE_UART0_RX | DMAMUX_ENABLE;
    // Enable DMA Request 1
    DMA_ERQ |= DMA_ERQ_ERQ5;
    // enable irq CH1
    NVIC_SET_PRIORITY(IRQ_DMA_CH5, IRQ_PRIORITY);
    NVIC_ENABLE_IRQ(IRQ_DMA_CH5);
}

void Serial1Event::serial_dma_format(uint32_t format) {
    /****************************************************************
     * serial1 format, from teensduino core, serial1.c
     ****************************************************************/
    uint8_t c;
    c = UART0_C1;
    c = (c & ~0x13) | (format & 0x03);      // configure parity
    if (format & 0x04) c |= 0x10;           // 9 bits (might include parity)
    UART0_C1 = c;
    if ((format & 0x0F) == 0x04) UART0_C3 |= 0x40; // 8N2 is 9 bit with 9th bit always 1
    c = UART0_S2 & ~0x10;
    if (format & 0x10) c |= 0x10;           // rx invert
    UART0_S2 = c;
    c = UART0_C3 & ~0x10;
    if (format & 0x20) c |= 0x10;           // tx invert
    UART0_C3 = c;
}

void Serial1Event::serial_dma_end(void) {
    if (!(SIM_SCGC7 & SIM_SCGC7_DMA)) return;
    if (!(SIM_SCGC6 & SIM_SCGC6_DMAMUX)) return;
    if (!(SIM_SCGC4 & SIM_SCGC4_UART0)) return;
    while ( txFifoCount > 0 ) yield();
    while (SEND_DONE_TX) yield();
    delay(20);
    /****************************************************************
     * serial1 end, from teensduino core, serial1.c
     ****************************************************************/
    UART0_C2 = 0;
	CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
	CORE_PIN1_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(1);
    // clear Serial1 dma enable rx/tx bits
    UART0_C5 = UART_DMA_DISABLE;
    rxHead = rxTail = 0;
}

void Serial1Event::serial_dma_set_transmit_pin(uint8_t pin) {
    // TODO: need to update var when finish transmitting serial for RS485
    pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	transmit_pin = portOutputRegister(pin);
}

void Serial1Event::serial_dma_putchar(uint32_t c) {
    serial_dma_write(&c, 1);
}

void Serial1Event::serial_dma_write(const void *buf, unsigned int count) {
    const uint8_t *buffer = (const uint8_t *)buf;
    int fifoSlots = count/TX1_PACKET_SIZE;
    int remainder = count%TX1_PACKET_SIZE;
    int totalPacks = fifoSlots + txFifoCount + 1;
    term_rx_character = rxTermCharacter;
    if ( totalPacks > TX_FIFO_SIZE ) return;
    if ( fifoSlots != 0 ) {
        for (int i = 0; i < fifoSlots; i++) {
            // start inserting new items into the fifo circular buffer
            tx1_fifo_t* p = &tx_memory_pool[txHead];
            volatile uint8_t* fifopack = p->packet;
            // store current packet size in the fifo buffer
            p->size = TX1_PACKET_SIZE;
            // update tx event trigger
            p->eventTrigger = false;
            int pakSize = TX1_PACKET_SIZE;
            do {
                *fifopack++ = *buffer++;
            } while ( --pakSize ) ;
            // packet count
            ++txFifoCount;
            txHead = txHead < ( TX_FIFO_SIZE - 1 ) ? txHead + 1 : 0;
        }
        if ( remainder > 0 ) {
            // start inserting new items into the fifo circular buffer
            tx1_fifo_t* p = &tx_memory_pool[txHead];
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
        tx1_fifo_t* p = &tx_memory_pool[txHead];
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
    tx1_fifo_t* p = &tx_memory_pool[txTail];
    // DMA transfer requests enabled
    UART0_C2 |= UART_C2_TIE;
    // set minor loop counts
    DMA_TCD4_CITER_ELINKNO = p->size;
    DMA_TCD4_BITER_ELINKNO = p->size;
    // source address
    DMA_TCD4_SADDR = p->packet;
    DMA_SERQ = 4;
    //------------------------------------------------------------------
    // siganl packet transmission in progress, start adding to buffer
    txDone = false;
    __enable_irq();
    //return txFifoCount;

}

void Serial1Event::serial_dma_flush(void) {
    while ( txFifoCount != 0 ) yield();
    while ( SEND_DONE_TX ) yield();
}

int Serial1Event::serial_dma_available(void) {
    uint32_t head, tail;
    head = RX_BUFFER_SIZE - DMA_TCD5_CITER_ELINKNO;
    tail = rxTail;
    if (head >= tail) return head - tail;
    return RX_BUFFER_SIZE + head - tail;
}

int Serial1Event::serial_dma_getchar(void) {
    uint32_t head, tail;
    int c;
    head = RX_BUFFER_SIZE - DMA_TCD5_CITER_ELINKNO;
    tail = rxTail;
    c = rxBuffer[tail];
    if (head == tail) return -1;
    if (++tail >= RX_BUFFER_SIZE) tail = 0;
    rxTail = tail;
    return c;
}

int Serial1Event::serial_dma_peek(void) {
    uint32_t head;
    head = RX_BUFFER_SIZE - DMA_TCD5_CITER_ELINKNO;
    return head;
}

void Serial1Event::serial_dma_clear(void) {
    
}

void Serial1Event::serial_dma_print(const char *p) {
    
}

void Serial1Event::serial_dma_phex(uint32_t n) {
    
}

void Serial1Event::serial_dma_phex16(uint32_t n) {
    
}

void Serial1Event::serial_dma_phex32(uint32_t n) {
    
}


/*#define UART_C5_TDMAS           (uint8_t)0x80
//#define UART_C5_RDMAS           (uint8_t)0x20

//#define SEND_DONE_TX        !(DMA_TCD4_CSR & DMA_TCD_CSR_DONE)
//#define RECIEVE_DONE_RX     !(DMA_TCD5_CSR & DMA_TCD_CSR_DONE)

//#define UART_DMA_ENABLE       UART_C5_TDMAS | UART_C5_RDMAS
//#define UART_DMA_DISABLE    0
//#define IRQ_PRIORITY        64  // 0 = highest priority, 255 = lowest

//#define likely(x)	__builtin_expect(!!(x), 1)
//#define unlikely(x)	__builtin_expect(!!(x), 0)

//#define BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
//#define BITBAND_U32(reg, bit) (*(uint32_t *)BITBAND_ADDR((reg), (bit)))
//#define BITBAND_U8(reg, bit) (*(uint8_t *)BITBAND_ADDR((reg), (bit)))

SerialEvent::ISR SerialEvent::RX_CALLBACK;
SerialEvent::ISR SerialEvent::TX_CALLBACK;

volatile bool SerialEvent::txDone;
volatile int SerialEvent::txFifoCount;

volatile uint32_t SerialEvent::txHead;
volatile uint32_t SerialEvent::txTail;

volatile uint16_t SerialEvent::bufSize_rx;

volatile uint8_t SerialEvent::term_rx;

volatile uintptr_t *SerialEvent::currentptr_rx;

volatile uintptr_t *SerialEvent::zeroptr_rx;

tx_fifo_t *SerialEvent::tx_memory_pool;
uint8_t SerialEvent::TX_FIFO_SIZE;

uint8_t *SerialEvent::rxBuffer;
uint16_t SerialEvent::RX_BUFFER_SIZE;

bool SerialEvent::dma_ch_enabled[4];
// -----------------------------------------------ch0_isr------------------------------------------------------------
void dma_ch4_isr() {
    // clear dma interrupt request 0
    DMA_CINT |= DMA_CINT_CINT(4);
    SerialEvent::txFifoCount--;
    //Serial.printf("txFifoCount: %i | txHead: %i | txTail: %i | packet: %s\n", SerialEvent::txFifoCount, SerialEvent::txHead, SerialEvent::txTail, sent->packet);
    // increment fifo tail
    SerialEvent::txTail = SerialEvent::txTail < (SerialEvent::TX_FIFO_SIZE - 1) ? SerialEvent::txTail + 1 : 0;
    // user or default tx callback event
    SerialEvent::TX_CALLBACK();
    /***********************************************************************************
     / here if the fifo has packets to send, we setup the dma destination address       /
     / and source registers for transmission on the queued serial port. We will         /
     / keep transmitting these queued packets in the "background" until the fifo        /
     / is exhausted.                                                                    /
     ***********************************************************************************/
    /*if ( SerialEvent::txFifoCount > 0 ) {
        tx_fifo_t* queued = &SerialEvent::tx_memory_pool[SerialEvent::txTail];
        //Serial.printf("ISR | count: %i | size: %i | head: %i | tail: %i | packet: %s\n",SerialEvent::txFifoCount, queued->size, SerialEvent::txHead, SerialEvent::txTail, queued->packet);
        // Number of bytes to transfer (in each service request)
        DMA_TCD4_CITER_ELINKNO = queued->size;
        DMA_TCD4_BITER_ELINKNO = queued->size;
        DMA_TCD4_SADDR = queued->packet;
        BITBAND_U32(DMA_ERQ, 0x04) = 0x01;
        return;
    } else {
        // no more packets to transmitt
        SerialEvent::txDone = true;
        return;
    }
}
// -----------------------------------------------ch1_isr------------------------------------------------------------
void dma_ch5_isr() {
    // Clear Interrupt Request 1
    DMA_CINT |= DMA_CINT_CINT(5);
    // Enable DMA Request 1
    BITBAND_U32(DMA_ERQ, 5) = 0x01;
    //DMA_ERQ |= DMA_ERQ_ERQ1;
    if (SerialEvent::term_rx != 0) {
        static uint32_t byteCount_rx = 0;
        if (( (uint8_t)*SerialEvent::currentptr_rx == SerialEvent::term_rx) || ( byteCount_rx == (SerialEvent::RX_BUFFER_SIZE-1) )) {
            SerialEvent::currentptr_rx = (uintptr_t*)DMA_TCD5_DADDR;
            DMA_TCD5_DADDR = SerialEvent::zeroptr_rx;
            byteCount_rx = 0;
            *SerialEvent::currentptr_rx = 0;
            SerialEvent::RX_CALLBACK();
        }
        else { byteCount_rx++; }
        SerialEvent::currentptr_rx = (uintptr_t*)DMA_TCD5_DADDR;
    }
    else {
        //SerialEvent::rxBuffer[SerialEvent::RX_BUFFER_SIZE] = 0;
        // user or default rx1 callback event
        SerialEvent::RX_CALLBACK();
    }
}
// -------------------------------------------Constructor------------------------------------------------------------
SerialEvent::SerialEvent() :
    txEventHandler(defaultCallback),
    rxEventHandler(defaultCallback),
    //rxBuffer(defaultBuffer_RX),
    rxBufferSize(2),
    loopBack(false),
    termCharacter(0),
    txSkipped(0)
{
    txHead = 0;
    txTail = 0;
    txFifoCount = 0;
    txDone = true;
}
// ----------------------------------------------begin---------------------------------------------------------------
void SerialEvent::begin( uint32_t baud, uint32_t format) {
    // Enable DMA, DMAMUX clocks
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    // Use default configuration
    if (DMA_CR != 0) DMA_CR = 0;
    // setup tx dma transfers, all Serial ports share the same
    // dma channel for transmitted packets.
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
    // tx dma setup
    dma_TX_begin();
    // rx dma setup
    dma_RX_begin();
}
// --------------------------------------------serialTXInit----------------------------------------------------------
int SerialEvent::dma_TX_begin(void) {
    // assign user callback
    TX_CALLBACK = txEventHandler;
    // Destination address
    DMA_TCD4_DADDR = &UART0_D;
    // Restore source address after major loop
    DMA_TCD4_SLAST = 0;
    // source address offset (1 byte)
    DMA_TCD4_SOFF = 1;
    // Destination offset (0 byte)
    DMA_TCD4_DOFF = 0;
    // Restore destination address after major loop
    DMA_TCD4_DLASTSGA = 0;
    // Source and destination size 8 bit
    DMA_TCD4_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
    // Number of bytes to transfer (in each service request)
    DMA_TCD4_NBYTES_MLNO = 1;//UART0_TWFIFO;
    // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
    DMA_TCD4_CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
    DMAMUX0_CHCFG4 = DMAMUX_DISABLE;
    // enable DMA MUX
    DMAMUX0_CHCFG4 = DMAMUX_SOURCE_UART0_TX | DMAMUX_ENABLE;
    // set TX priority
    NVIC_SET_PRIORITY(IRQ_DMA_CH4, IRQ_PRIORITY);
    // enable irq
    NVIC_ENABLE_IRQ(IRQ_DMA_CH4);
    return 1;
}
// --------------------------------------------serialRXInit----------------------------------------------------------
int SerialEvent::dma_RX_begin(void) {
        RX_CALLBACK = rxEventHandler;
        // Source address
        DMA_TCD5_SADDR = &UART0_D;
        // Restore source address after major loop
        DMA_TCD5_SLAST = 0;
        // source address offset (0 bytes)
        DMA_TCD5_SOFF = 0;
        // Destination address
        DMA_TCD5_DADDR = &rxBuffer[0];
        // Destination offset (0 byte)
        DMA_TCD5_DOFF = 1;
        if (termCharacter == 0) {
            // Restore destination address after major loop
            DMA_TCD5_DLASTSGA = -RX_BUFFER_SIZE;//-(rxBufferSize);
            // Set loop counts / channel2channel linking disabled
            DMA_TCD5_CITER_ELINKNO = RX_BUFFER_SIZE;//rxBufferSize;//UART0_RWFIFO;
            DMA_TCD5_BITER_ELINKNO = RX_BUFFER_SIZE;//rxBufferSize;//UART0_RWFIFO;
            term_rx = 0;
        }
        else {
            // Restore destination address after major loop
            DMA_TCD5_DLASTSGA = 0;
            // Set loop counts / channel2channel linking disabled
            DMA_TCD5_CITER_ELINKNO = 1;//UART0_RWFIFO;
            DMA_TCD5_BITER_ELINKNO = 1;//UART0_RWFIFO;
            term_rx = termCharacter;
            bufSize_rx = rxBufferSize;
            zeroptr_rx = (uintptr_t*)DMA_TCD5_DADDR;
            currentptr_rx = zeroptr_rx;
            //Serial.printf("DMA_TCD1_DLASTSGA: %i\t| DMA_TCD1_DADDR: %p | zeroptr_rx1: %p\n\n",  DMA_TCD1_DLASTSGA, DMA_TCD1_DADDR, zeroptr_rx1);
        }
        // Source and destination size 8 bit
        DMA_TCD5_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
        // Number of bytes to transfer (in each service request)
        DMA_TCD5_NBYTES_MLNO = 1;//UART0_TWFIFO;
        // Enable interrupt (end-of-major loop) / Clear ERQ bit at end of Major Loop
        DMA_TCD5_CSR = DMA_TCD_CSR_INTMAJOR  | DMA_TCD_CSR_DREQ | DMA_TCD_CSR_DONE;
        // Set Serial1 as source (CH 1), enable DMA MUX
        DMAMUX0_CHCFG5 = DMAMUX_DISABLE;
        DMAMUX0_CHCFG5 = DMAMUX_SOURCE_UART0_RX | DMAMUX_ENABLE;
        // Enable DMA Request 1
        DMA_ERQ |= DMA_ERQ_ERQ5;
        // enable irq CH1
        NVIC_SET_PRIORITY(IRQ_DMA_CH5, IRQ_PRIORITY);
        NVIC_ENABLE_IRQ(IRQ_DMA_CH5);
        return 1;
}
// -----------------------------------------------end----------------------------------------------------------------
int SerialEvent::dma_end( void ) {
    if (!(SIM_SCGC7 & SIM_SCGC7_DMA)) return -1;
    if (!(SIM_SCGC6 & SIM_SCGC6_DMAMUX)) return -1;
    while ( txFifoCount > 0 ) yield();
    while (SEND_DONE_TX) yield();
    delay(20);
        Serial1.end();
        // clear Serial1 dma enable rx/tx bits
        UART0_C5 = UART_DMA_DISABLE;
    return 1;
}
 // --------------------------------------------available------------------------------------------------------------
int SerialEvent::dma_available(void) {
        uint32_t head, tail;
        head = this->rxBufferSize - DMA_TCD5_CITER_ELINKNO;
        tail = this->rxTail;
        if (head >= tail) return head - tail;
        return this->rxBufferSize + head - tail;
}
// ----------------------------------------------getchar-------------------------------------------------------------
int SerialEvent::dma_getchar(void) {
        uint32_t head, tail;
        int c;
        head = this->rxBufferSize - DMA_TCD5_CITER_ELINKNO;
        tail = this->rxTail;
        c = this->rxBuffer[tail];
        if (head == tail) return -1;
        if (++tail >= this->rxBufferSize) tail = 0;
        this->rxTail = tail;
        return c;
}
// -----------------------------------------------peek---------------------------------------------------------------
int SerialEvent::dma_peek( void ) {
    uint32_t head, tail;
    int c;
    head = rxBufferSize - DMA_TCD5_CITER_ELINKNO;
}
// -----------------------------------------------flush--------------------------------------------------------------
int SerialEvent::dma_flush( void ) {
    while ( SerialEvent::txFifoCount != 0 ) yield();
    while (SEND_DONE_TX) yield();
    return 1;
}
// -----------------------------------------------clear--------------------------------------------------------------
int SerialEvent::dma_clear( void ) {

    return 1;
}
// -----------------------------------------------end----------------------------------------------------------------

inline int SerialEvent::dma_write( const uint8_t* data, uint32_t size )  {
    //-----------------------------------------------------------------
    int numPacks = size/TX_PACKET_SIZE;
    int remPacks = size%TX_PACKET_SIZE;
    int totalPacks = numPacks + txFifoCount + 1;
    //Serial.printf("size: %i | numPacks: %i | remainder: %i | total: %i\n", size, numPacks, remPacks, totalPacks);
    if (totalPacks > TX_FIFO_SIZE) {
        return -1;
    }
    
    if (numPacks != 0) {
        for (int i = 0; i < numPacks; i++) {
            // here we insert new items into the fifo circular buffer
            tx_fifo_t* p = &tx_memory_pool[txHead];
            // store current packet size in the fifo buffer
            p->size = TX_PACKET_SIZE;
            // update tx event callback for specific port
            //p->txEventHandler = txEventHandler;
            // packet count
            int inc = 0;
            int count = TX_PACKET_SIZE;
            do {
                p->packet[inc++] = *data++;
            } while (--count) ;
            //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { memcpy(p->packet, data, TX_PACKET_SIZE); }
            //p->packet[p->size] = 0;
            //Serial.printf("count: %02i | size: %03i | head: %02i | tail: %02i\n",txFifoCount, p->size, txHead, txTail);
            ++txFifoCount;
            txHead = txHead < (TX_FIFO_SIZE - 1) ? txHead + 1 : 0;
            
        }
        if (remPacks > 0) {
            // here we insert new items into the fifo circular buffer
            tx_fifo_t* p = &tx_memory_pool[txHead];
            // store current packet size in the fifo buffer
            p->size = remPacks;
            // update tx event callback for specific port
            //p->txEventHandler = txEventHandler;
            // packet count
            int inc = 0;
            do {
                p->packet[inc++] = *data++;
            } while (--remPacks) ;
            //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { memcpy(p->packet, data, remPacks); }
            p->packet[p->size] = 0;
            //Serial.printf("count: %02i | size: %03i | head: %02i | tail: %02i\n",txFifoCount, p->size, txHead, txTail);
            ++txFifoCount;
            txHead = txHead < (TX_FIFO_SIZE - 1) ? txHead + 1 : 0;
        }
    }
    else {
        // here we insert new items into the fifo circular buffer
        tx_fifo_t* p = &tx_memory_pool[txHead];
        // store current packet size in the fifo buffer
        p->size = remPacks;
        // update tx event callback for specific port
        //p->txEventHandler = txEventHandler;
        int inc = 0;
        do {
            p->packet[inc++] = *data++;
        } while (--size) ;
        p->packet[p->size] = 0;
        //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { memcpy(p->packet, data, remPacks); }
        //p->packet[p->size] = 0;
        // packet count
        ++txFifoCount;
        txHead = txHead < (TX_FIFO_SIZE - 1) ? txHead + 1 : 0;
    }
    //------------------------------------------------------------------
    // return if packets are still in transmission
    if(!txDone) return txFifoCount;
    //------------------------------------------------------------------
    tx_fifo_t* p = &tx_memory_pool[txTail];
    // DMA transfer requests enabled
    UART0_C2 |= UART_C2_TIE;
    // set minor loop counts
    DMA_TCD4_CITER_ELINKNO = p->size;
    DMA_TCD4_BITER_ELINKNO = p->size;
    // source address
    DMA_TCD4_SADDR = p->packet;
    BITBAND_U32(DMA_ERQ, 0x04) = 0x01;
    //------------------------------------------------------------------
    // siganl packet transmission in progress, start adding to buffer
    txDone = false;
    return txFifoCount;
}*/


