/*
 ||
 || @file 		SerialEvent.h
 || @version 	2
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 ||
 || @description
 || | A event based Hardware Serial class for transferring data in the 
 || | background using DMA for use with the Teensy3.1 only. This library
 || | borrorowed some code from the Teensyduino Core Library, Copyright
 || | (c) 2013 PJRC.COM, LLC.
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
#ifndef SerialEvent_h
#define SerialEvent_h
#ifdef __cplusplus

#include "Arduino.h"
#include "utility/Utils.h"
//---------------------------------------Serial1Event----------------------------------------
class Serial1Event : public Stream {
private:
    void serial_dma_begin(uint32_t divisor);
    void serial_dma_format(uint32_t format);
    void serial_dma_end(void);
    void serial_dma_set_transmit_pin(uint8_t pin);
    void serial_dma_putchar(uint32_t c);
    void serial_dma_write(const void *buf, unsigned int count);
    void serial_dma_flush(void);
    int serial_dma_available(void);
    int serial_dma_getchar(void);
    int serial_dma_peek(void);
    void serial_dma_clear(void);
    void serial_dma_print(const char *p);
    void serial_dma_phex(uint32_t n);
    void serial_dma_phex16(uint32_t n);
    void serial_dma_phex32(uint32_t n);
    
    static void defaultCallback() { yield(); }
    
    friend void dma_ch4_isr ( void );
    friend void dma_ch5_isr ( void );
    
    typedef void (*ISR)();
    static ISR RX_CALLBACK;
    static ISR TX_CALLBACK;
    
    static volatile int txFifoCount;
    static volatile int term_rx_character;
    static volatile uint32_t txHead;
    static volatile uint32_t txTail;
    static volatile uint32_t rxHead;
    static volatile uint32_t rxTail;
    static volatile uint16_t bufSize_rx;
    static volatile uintptr_t* currentptr_rx;
    static volatile uintptr_t *zeroptr_rx;
    static volatile uint8_t* transmit_pin;
    static volatile uint8_t TX_FIFO_SIZE;
    static volatile uint16_t RX_BUFFER_SIZE;
    static volatile boolean txDone;
    static tx1_fifo_t * tx_memory_pool;
    
    static char *term_rx_string;
public:
    Serial1Event() :
        txEventHandler(defaultCallback),
        rxEventHandler(defaultCallback),
        rxTermCharacter(-1),
        loopBack(false),
        half(false)
    {
        txFifoCount, txHead, txTail, rxHead, rxTail, bufSize_rx = 0;
        txDone = true;
    }
    virtual void begin(uint32_t baud, uint32_t format) {
        serial_dma_begin(BAUD2DIV(baud));
        serial_dma_format(format);
    }
    virtual void begin(uint32_t baud)           { serial_dma_begin(BAUD2DIV(baud)); }
	virtual void end(void)                      { serial_dma_end(); }
	virtual void transmitterEnable(uint8_t pin) { serial_dma_set_transmit_pin(pin); }
	virtual int available(void)                 { return serial_dma_available(); }
	virtual int peek(void)                      { return serial_dma_peek(); }
	virtual int read(void)                      { serial_dma_getchar(); }
	virtual void flush(void)                    { serial_dma_flush(); }
	virtual void clear(void)                    { serial_dma_clear(); }
	virtual size_t write(uint8_t c)             { serial_dma_putchar(c); }
	virtual size_t write(unsigned long n)       { return write((uint8_t)n); }
	virtual size_t write(long n)                { return write((uint8_t)n); }
	virtual size_t write(unsigned int n)        { return write((uint8_t)n); }
	virtual size_t write(int n)                 { return write((uint8_t)n); }
    virtual size_t write9bit(uint32_t c)        {  }
    
	virtual size_t write(const uint8_t *buffer, size_t size) {
        serial_dma_write(buffer, size);
        return size;
    }
    virtual size_t write(const char *str) {
        size_t len = strlen(str);
        serial_dma_write((const uint8_t *)str, len);
        return len;
    }
    static void initialize_tx_memory(tx1_fifo_t *data, uint16_t num) {
        TX_FIFO_SIZE = num;
        txUsedMemory = 0;
        tx_memory_pool = data;
        for (int i = 0; i < num; i++) {
            tx1_fifo_t* p = &tx_memory_pool[i];
            p->size = 0;
            p->eventTrigger = false;
            for (int x = 0; x < TX1_PACKET_SIZE; x++) p->packet[x] = 0;
        }
    }
    static void initialize_rx_memory(uint8_t *data, uint16_t num) {
        RX_BUFFER_SIZE = num;
        rxUsedMemory = 0;
        rxBuffer = data;
        for (int i = 0; i < num+1; i++) rxBuffer[i] = 0;
    }
    ISR txEventHandler;
    ISR rxEventHandler;
    int rxTermCharacter;
    char *rxTermString;
    bool loopBack;
    bool half;
    static volatile uint8_t* rxBuffer;
    static volatile uint8_t txUsedMemory;
    static volatile uint8_t rxUsedMemory;
};

//---------------------------------------Serial2Event----------------------------------------
class Serial2Event : public Stream {
private:
    void serial_dma_begin(uint32_t divisor);
    void serial_dma_format(uint32_t format);
    void serial_dma_end(void);
    void serial_dma_set_transmit_pin(uint8_t pin);
    void serial_dma_putchar(uint32_t c);
    void serial_dma_write(const void *buf, unsigned int count);
    void serial_dma_flush(void);
    int serial_dma_available(void);
    int serial_dma_getchar(void);
    int serial_dma_peek(void);
    void serial_dma_clear(void);
    void serial_dma_print(const char *p);
    void serial_dma_phex(uint32_t n);
    void serial_dma_phex16(uint32_t n);
    void serial_dma_phex32(uint32_t n);
    
    static void defaultCallback() { yield(); }
    
    friend void dma_ch6_isr ( void );
    friend void dma_ch7_isr ( void );
    
    typedef void (*ISR)();
    static ISR RX_CALLBACK;
    static ISR TX_CALLBACK;
    
    static volatile int txFifoCount;
    static volatile int term_rx_character;
    static volatile uint32_t txHead;
    static volatile uint32_t txTail;
    static volatile uint32_t rxHead;
    static volatile uint32_t rxTail;
    static volatile uint16_t bufSize_rx;
    static volatile uintptr_t* currentptr_rx;
    static volatile uintptr_t *zeroptr_rx;
    static volatile uint8_t* transmit_pin;
    static volatile uint8_t TX_FIFO_SIZE;
    static volatile uint16_t RX_BUFFER_SIZE;
    static volatile boolean txDone;
    static tx2_fifo_t * tx_memory_pool;
    
    static char *term_rx_string;
public:
    Serial2Event() :
    txEventHandler(defaultCallback),
    rxEventHandler(defaultCallback),
    rxTermCharacter(-1),
    loopBack(false),
    half(false)
    {
        txFifoCount, txHead, txTail, rxHead, rxTail, bufSize_rx = 0;
        txDone = true;
    }
    virtual void begin(uint32_t baud, uint32_t format) {
        serial_dma_begin(BAUD2DIV(baud));
        serial_dma_format(format);
    }
    virtual void begin(uint32_t baud)           { serial_dma_begin(BAUD2DIV(baud)); }
	virtual void end(void)                      { serial_dma_end(); }
	virtual void transmitterEnable(uint8_t pin) { serial_dma_set_transmit_pin(pin); }
	virtual int available(void)                 { return serial_dma_available(); }
	virtual int peek(void)                      { return serial_dma_peek(); }
	virtual int read(void)                      { serial_dma_getchar(); }
	virtual void flush(void)                    { serial_dma_flush(); }
	virtual void clear(void)                    { serial_dma_clear(); }
	virtual size_t write(uint8_t c)             { serial_dma_putchar(c); }
	virtual size_t write(unsigned long n)       { return write((uint8_t)n); }
	virtual size_t write(long n)                { return write((uint8_t)n); }
	virtual size_t write(unsigned int n)        { return write((uint8_t)n); }
	virtual size_t write(int n)                 { return write((uint8_t)n); }
    virtual size_t write9bit(uint32_t c)        {  }
    
	virtual size_t write(const uint8_t *buffer, size_t size) {
        serial_dma_write(buffer, size);
        return size;
    }
    virtual size_t write(const char *str) {
        size_t len = strlen(str);
        serial_dma_write((const uint8_t *)str, len);
        return len;
    }
    static void initialize_tx_memory(tx2_fifo_t *data, uint16_t num) {
        TX_FIFO_SIZE = num;
        txUsedMemory = 0;
        tx_memory_pool = data;
        for (int i = 0; i < num; i++) {
            tx2_fifo_t* p = &tx_memory_pool[i];
            p->size = 0;
            p->eventTrigger = false;
            for (int x = 0; x < TX2_PACKET_SIZE; x++) p->packet[x] = 0;
        }
    }
    static void initialize_rx_memory(uint8_t *data, uint16_t num) {
        RX_BUFFER_SIZE = num;
        rxUsedMemory = 0;
        rxBuffer = data;
        for (int i = 0; i < num+1; i++) rxBuffer[i] = 0;
    }
    ISR txEventHandler;
    ISR rxEventHandler;
    int rxTermCharacter;
    char *rxTermString;
    bool loopBack;
    bool half;
    static volatile uint8_t* rxBuffer;
    static volatile uint8_t txUsedMemory;
    static volatile uint8_t rxUsedMemory;
};

//---------------------------------------Serial3Event----------------------------------------
class Serial3Event : public Stream {
private:
    void serial_dma_begin(uint32_t divisor);
    void serial_dma_format(uint32_t format);
    void serial_dma_end(void);
    void serial_dma_set_transmit_pin(uint8_t pin);
    void serial_dma_putchar(uint32_t c);
    void serial_dma_write(const void *buf, unsigned int count);
    void serial_dma_flush(void);
    int serial_dma_available(void);
    int serial_dma_getchar(void);
    int serial_dma_peek(void);
    void serial_dma_clear(void);
    void serial_dma_print(const char *p);
    void serial_dma_phex(uint32_t n);
    void serial_dma_phex16(uint32_t n);
    void serial_dma_phex32(uint32_t n);
    
    static void defaultCallback() { yield(); }
    
    friend void dma_ch8_isr ( void );
    friend void dma_ch9_isr ( void );
    
    typedef void (*ISR)();
    static ISR RX_CALLBACK;
    static ISR TX_CALLBACK;
    
    static volatile int txFifoCount;
    static volatile int term_rx_character;
    static volatile uint32_t txHead;
    static volatile uint32_t txTail;
    static volatile uint32_t rxHead;
    static volatile uint32_t rxTail;
    static volatile uint16_t bufSize_rx;
    static volatile uintptr_t* currentptr_rx;
    static volatile uintptr_t *zeroptr_rx;
    static volatile uint8_t* transmit_pin;
    static volatile uint8_t TX_FIFO_SIZE;
    static volatile uint16_t RX_BUFFER_SIZE;
    static volatile boolean txDone;
    static tx3_fifo_t * tx_memory_pool;
    
    static char *term_rx_string;
public:
    Serial3Event() :
    txEventHandler(defaultCallback),
    rxEventHandler(defaultCallback),
    rxTermCharacter(-1),
    loopBack(false),
    half(false)
    {
        txFifoCount, txHead, txTail, rxHead, rxTail, bufSize_rx = 0;
        txDone = true;
    }
    virtual void begin(uint32_t baud, uint32_t format) {
        serial_dma_begin(BAUD2DIV3(baud));
        serial_dma_format(format);
    }
    virtual void begin(uint32_t baud)           { serial_dma_begin(BAUD2DIV3(baud)); }
	virtual void end(void)                      { serial_dma_end(); }
	virtual void transmitterEnable(uint8_t pin) { serial_dma_set_transmit_pin(pin); }
	virtual int available(void)                 { return serial_dma_available(); }
	virtual int peek(void)                      { return serial_dma_peek(); }
	virtual int read(void)                      { serial_dma_getchar(); }
	virtual void flush(void)                    { serial_dma_flush(); }
	virtual void clear(void)                    { serial_dma_clear(); }
	virtual size_t write(uint8_t c)             { serial_dma_putchar(c); }
	virtual size_t write(unsigned long n)       { return write((uint8_t)n); }
	virtual size_t write(long n)                { return write((uint8_t)n); }
	virtual size_t write(unsigned int n)        { return write((uint8_t)n); }
	virtual size_t write(int n)                 { return write((uint8_t)n); }
    virtual size_t write9bit(uint32_t c)        {  }
    
	virtual size_t write(const uint8_t *buffer, size_t size) {
        serial_dma_write(buffer, size);
        return size;
    }
    virtual size_t write(const char *str) {
        size_t len = strlen(str);
        serial_dma_write((const uint8_t *)str, len);
        return len;
    }
    static void initialize_tx_memory(tx3_fifo_t *data, uint16_t num) {
        TX_FIFO_SIZE = num;
        txUsedMemory = 0;
        tx_memory_pool = data;
        for (int i = 0; i < num; i++) {
            tx3_fifo_t* p = &tx_memory_pool[i];
            p->size = 0;
            p->eventTrigger = false;
            for (int x = 0; x < TX3_PACKET_SIZE; x++) p->packet[x] = 0;
        }
    }
    static void initialize_rx_memory(uint8_t *data, uint16_t num) {
        RX_BUFFER_SIZE = num;
        rxUsedMemory = 0;
        rxBuffer = data;
        for (int i = 0; i < num+1; i++) rxBuffer[i] = 0;
    }
    ISR txEventHandler;
    ISR rxEventHandler;
    int rxTermCharacter;
    char *rxTermString;
    bool loopBack;
    bool half;
    static volatile uint8_t* rxBuffer;
    static volatile uint8_t txUsedMemory;
    static volatile uint8_t rxUsedMemory;
};
//---------------------------------------------End----------------------------------------------
#endif  // __cplusplus
#endif
