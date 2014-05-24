#ifndef SerialEvent_h
#define SerialEvent_h
#ifdef __cplusplus
/*
 ||
 || @file 		SerialEvent.h
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
#include "Arduino.h"

const size_t TX_FIFO_SIZE = 32;
typedef struct transmit_fifo_t {
private:
    typedef void (*EVENT)();
public:
    char* volatile packet;
    volatile uint16_t size;
    volatile uint8_t port;
    volatile bool allocated;
    EVENT txEventHandler;
    transmit_fifo_t() : allocated(false) {}
} tx_fifo_t;

class SerialEvent : public Print {
private:
    friend void dma_ch0_isr ( void );
    friend void dma_ch1_isr ( void );
    friend void dma_ch2_isr ( void );
    friend void dma_ch3_isr ( void );
    friend void callback    ( void );
    
    //void memcpy8(volatile char *dest, const char *src, unsigned int count);
    static void defaultCallback() { yield(); }
    
    int dma_TX_begin            ( void );
    int dma_RX_begin            ( void );
    int dma_write               ( const uint8_t* data, uint32_t size ) ;
    int dma_end                 ( void );
    int dma_available           ( void );
    int dma_getchar             ( void );
    int dma_peek                ( void );
    int dma_flush               ( void );
    int dma_clear               ( void );
    size_t dma_readBytesUntil   ( char terminator, char *buffer, size_t length );
    
    typedef void (*ISR)();
    static ISR RX1_CALLBACK;
    static ISR RX2_CALLBACK;
    static ISR RX3_CALLBACK;
    
    static volatile bool txDone;
    static volatile int txCount;
    static volatile uint32_t txHead;
    static volatile uint32_t txTail;
    static volatile uint32_t _memory;
    
    static volatile uint16_t bufSize_rx1;
    static volatile uint16_t bufSize_rx2;
    static volatile uint16_t bufSize_rx3;
    
    static volatile uint8_t term_rx1;
    static volatile uint8_t term_rx2;
    static volatile uint8_t term_rx3;
    
    static volatile uintptr_t *currentptr_rx1;
    static volatile uintptr_t *currentptr_rx2;
    static volatile uintptr_t *currentptr_rx3;
    
    static volatile uintptr_t *zeroptr_rx1;
    static volatile uintptr_t *zeroptr_rx2;
    static volatile uintptr_t *zeroptr_rx3;
    
    static bool dma_ch_enabled[4];
    
    volatile uint32_t rxHead;
    volatile uint32_t rxTail;
    
    volatile char defaultBuffer_RX[2];
    
    uint8_t current_port;
    enum { SERIAL1 = 1, SERIAL2 = 2, SERIAL3 = 3 };
public:
    SerialEvent();
    SerialEvent(uint32_t memmory);
    ~SerialEvent()          { dma_end(); }
    void begin              ( uint32_t baud, uint32_t format = SERIAL_8N1 );
    virtual void end        ( void ){ dma_end(); }
    virtual int available   ( void ){ return dma_available(); }
    virtual int peek        ( void ){ return dma_peek(); }
    virtual int read        ( void ){ return dma_getchar(); }
    virtual void flush      ( void ){ dma_flush(); }
    virtual void clear      ( void ){ dma_clear(); }
    size_t write( uint8_t c ) {
        int error;
        error = dma_write( &c, 1 );
        if (error == -1) return -1;
        return 1;
    }
    size_t write( const uint8_t *buffer, size_t size ) {
        int error;
        error = dma_write( buffer, size );
        if (error == -1) return -1;
        return size;
    }
    using Print::write;
    // Serial Port
    Stream *port;
    // Event Callback
    ISR txEventHandler;
    ISR rxEventHandler;
    // User supplied RX buffer array
    volatile char* rxBuffer;
    // RX buffer size
    uint32_t rxBufferSize;
    // Return TX buffer size
    uint32_t txBufferSize() const { return txCount; }
    // return memory
    uint32_t memory() const { return _memory; }
    // set serial loopback (y/n)?
    bool loopBack;
    // termination charatcter for readBytesUntil
    uint8_t termCharacter;
};
#endif  // __cplusplus
#endif
