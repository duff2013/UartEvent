/*
 ||
 || @file       UartEvent.h
 || @version 	6.6
 || @author 	Colin Duffy
 || @contact 	http://forum.pjrc.com/members/25610-duff
 ||
 || @description
 || | A event based Hardware Serial class for transferring data in the
 || | background using DMA for use with the Teensy3.1 only. This library
 || | borrorowed some code from the Teensyduino Core Library, Copyright
 || | (c) 2013 PJRC.COM, LLC.
 || #
 ||
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
#ifndef UartEvent_h
#define UartEvent_h
#ifdef __cplusplus

#if(!defined(__MK20DX256__))
    #error "Teensy 3.1 Only!!!!"
#endif

#include "Arduino.h"
#include "DMAChannel.h"
#include "utility/Utils.h"
////////////////////////////////////////////////////////////////////////////////
#define TX0_BUFFER_SIZE 64  // Uart1 outgoing buffer size, must be power of 2 //
#define RX0_BUFFER_SIZE 64  // Uart1 incoming buffer size, must be power of 2 //
                                                                              //
#define TX1_BUFFER_SIZE 64  // Uart2 outgoing buffer size, must be power of 2 //
#define RX1_BUFFER_SIZE 64  // Uart2 incoming buffer size, must be power of 2 //
                                                                              //
#define TX2_BUFFER_SIZE 128 // Uart3 outgoing buffer size, must be power of 2 //
#define RX2_BUFFER_SIZE 128 // Uart3 incoming buffer size, must be power of 2 //
////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Uart1Event----------------------------------------
class Uart1Event : public Stream {
private:
    static uint32_t *elink;
    static volatile int16_t  priority;
    static DMAChannel tx;
    static DMAChannel rx;
    static void user_isr_rx          ( void );
    static void user_isr_tx          ( void );
    typedef void ( * ISR )           ( void );
    static void serial_dma_tx_isr    ( void ) ;
    static void serial_dma_rx_isr    ( void ) ;
    static void defaultCallback      ( void ) {  }
    
    void serial_dma_begin            ( uint32_t divisor );
    void serial_dma_format           ( uint32_t format );
    void serial_dma_end              ( void );
    void serial_dma_set_transmit_pin ( uint8_t pin );
    void serial_dma_putchar          ( uint32_t c );
    int  serial_dma_write            ( const void *buf, unsigned int count );
    int  serial_dma_write_buffer_free( void );
    void serial_dma_flush            ( void );
    int  serial_dma_available        ( void );
    int  serial_dma_getchar          ( void );
    int  serial_dma_peek             ( void );
    void serial_dma_clear            ( void );
    
    inline static void raise_priority( void ) {
        int pri = nvic_execution_priority( );
        if ( pri <= priority ) {
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + tx.channel, ( pri - 16 ) >= 0 ? pri - 16 : 0 );
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + rx.channel, ( pri - 16 ) >= 0 ? pri - 16 : 0 );
        }
    }
    
    inline static void lower_priority( void ) {
        int pri = nvic_execution_priority( );
        if ( pri <= priority ) {
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + tx.channel, priority );
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + rx.channel, priority );
        }
    }
public:
    Uart1Event( ) : loopBack( false ) {
        rxTermCharacterTrigger = -1;
        rxBufferSizeTrigger = -1;
        txEventHandler = defaultCallback;
        rxEventHandler = defaultCallback;
    }
    virtual void begin( uint32_t baud, uint32_t format ) {
        serial_dma_format( format );
        serial_dma_begin( BAUD2DIV( baud ) );
    }
    virtual void   begin            ( uint32_t baud )   { serial_dma_begin( BAUD2DIV( baud ) ); }
    virtual void   end              ( void )            { serial_dma_end( ); }
    virtual void   transmitterEnable( uint8_t pin )     { serial_dma_set_transmit_pin( pin ); }
    virtual int    available        ( void )            { return serial_dma_available( ); }
    virtual int    peek             ( void )            { return serial_dma_peek( ); }
    virtual int    read             ( void )            { return serial_dma_getchar( ); }
    virtual void   flush            ( void )            { serial_dma_flush( ); }
    virtual void   clear            ( void )            { serial_dma_clear( ); }
    virtual int    availableForWrite( void )            { return serial_dma_write_buffer_free( ); }
    virtual size_t write            ( uint8_t c )       { serial_dma_putchar( c ); return 1; }
    virtual size_t write            ( unsigned long n ) { return write( ( uint8_t )n ); }
    virtual size_t write            ( long n )          { return write( ( uint8_t )n ); }
    virtual size_t write            ( unsigned int n )  { return write( ( uint8_t )n ); }
    virtual size_t write            ( int n )           { return write( ( uint8_t )n ); }
    virtual size_t write9bit        ( uint32_t c )      {  return 0; }
    
    virtual size_t write( const uint8_t *buffer, size_t size ) {
        int s = serial_dma_write( buffer, size );
        return s;
    }
    virtual size_t write( const char *str ) {
        size_t len = strlen( str );
        int s = serial_dma_write( ( const uint8_t * )str, len );
        return s;
    }
    bool loopBack;
    static volatile int rxTermCharacterTrigger;
    static volatile int rxBufferSizeTrigger;
    static ISR txEventHandler;
    static ISR rxEventHandler;
    static const uint32_t txBufferSize = TX0_BUFFER_SIZE;
    static const uint32_t rxBufferSize = RX0_BUFFER_SIZE;
};
//---------------------------------------Uart2Event----------------------------------------
class Uart2Event : public Stream {
private:
    static uint32_t *elink;
    static volatile int16_t  priority;
    static DMAChannel tx;
    static DMAChannel rx;
    static void user_isr_rx          ( void );
    static void user_isr_tx          ( void );
    typedef void ( * ISR )           ( void );
    static void serial_dma_tx_isr    ( void ) ;
    static void serial_dma_rx_isr    ( void ) ;
    static void defaultCallback      ( void ) {  }
    
    void serial_dma_begin            ( uint32_t divisor );
    void serial_dma_format           ( uint32_t format );
    void serial_dma_end              ( void );
    void serial_dma_set_transmit_pin ( uint8_t pin );
    void serial_dma_putchar          ( uint32_t c );
    int  serial_dma_write            ( const void *buf, unsigned int count );
    int  serial_dma_write_buffer_free( void );
    void serial_dma_flush            ( void );
    int  serial_dma_available        ( void );
    int  serial_dma_getchar          ( void );
    int  serial_dma_peek             ( void );
    void serial_dma_clear            ( void );
    
    inline static void raise_priority( void ) {
        int pri = nvic_execution_priority( );
        if ( pri <= priority ) {
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + tx.channel, ( pri - 16 ) >= 0 ? pri - 16 : 0 );
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + rx.channel, ( pri - 16 ) >= 0 ? pri - 16 : 0 );
        }
    }
    
    inline static void lower_priority( void ) {
        int pri = nvic_execution_priority( );
        if ( pri <= priority ) {
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + tx.channel, priority );
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + rx.channel, priority );
        }
    }
public:
    Uart2Event( ) : loopBack( false ) {
        rxTermCharacterTrigger = -1;
        rxBufferSizeTrigger = -1;
        txEventHandler = defaultCallback;
        rxEventHandler = defaultCallback;
    }
    virtual void begin( uint32_t baud, uint32_t format ) {
        serial_dma_format( format );
        serial_dma_begin( BAUD2DIV2( baud ) );
    }
    virtual void   begin            ( uint32_t baud )   { serial_dma_begin( BAUD2DIV2( baud ) ); }
    virtual void   end              ( void )            { serial_dma_end( ); }
    virtual void   transmitterEnable( uint8_t pin )     { serial_dma_set_transmit_pin( pin ); }
    virtual int    available        ( void )            { return serial_dma_available( ); }
    virtual int    peek             ( void )            { return serial_dma_peek( ); }
    virtual int    read             ( void )            { return serial_dma_getchar( ); }
    virtual void   flush            ( void )            { serial_dma_flush( ); }
    virtual void   clear            ( void )            { serial_dma_clear( ); }
    virtual int    availableForWrite( void )            { return serial_dma_write_buffer_free( ); }
    virtual size_t write            ( uint8_t c )       { serial_dma_putchar( c ); return 1; }
    virtual size_t write            ( unsigned long n ) { return write( ( uint8_t )n ); }
    virtual size_t write            ( long n )          { return write( ( uint8_t )n ); }
    virtual size_t write            ( unsigned int n )  { return write( ( uint8_t )n ); }
    virtual size_t write            ( int n )           { return write( ( uint8_t )n ); }
    virtual size_t write9bit        ( uint32_t c )      {  return 0; }
    
    virtual size_t write( const uint8_t *buffer, size_t size ) {
        int s = serial_dma_write( buffer, size );
        return s;
    }
    virtual size_t write( const char *str ) {
        size_t len = strlen( str );
        int s = serial_dma_write( ( const uint8_t * )str, len );
        return s;
    }
    bool loopBack;
    static volatile int rxTermCharacterTrigger;
    static volatile int rxBufferSizeTrigger;
    static ISR txEventHandler;
    static ISR rxEventHandler;
    static const uint32_t txBufferSize = TX1_BUFFER_SIZE;
    static const uint32_t rxBufferSize = RX1_BUFFER_SIZE;
};
//---------------------------------------Uart3Event----------------------------------------
class Uart3Event : public Stream {
private:
    static uint32_t *elink;
    static volatile int16_t  priority;
    static DMAChannel tx;
    static DMAChannel rx;
    static void user_isr_rx          ( void );
    static void user_isr_tx          ( void );
    typedef void ( * ISR )           ( void );
    static void serial_dma_tx_isr    ( void ) ;
    static void serial_dma_rx_isr    ( void ) ;
    static void defaultCallback      ( void ) {  }
    
    void serial_dma_begin            ( uint32_t divisor );
    void serial_dma_format           ( uint32_t format );
    void serial_dma_end              ( void );
    void serial_dma_set_transmit_pin ( uint8_t pin );
    void serial_dma_putchar          ( uint32_t c );
    int  serial_dma_write            ( const void *buf, unsigned int count );
    int  serial_dma_write_buffer_free( void );
    void serial_dma_flush            ( void );
    int  serial_dma_available        ( void );
    int  serial_dma_getchar          ( void );
    int  serial_dma_peek             ( void );
    void serial_dma_clear            ( void );
    
    inline static void raise_priority( void ) {
        int pri = nvic_execution_priority( );
        if ( pri <= priority ) {
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + tx.channel, ( pri - 16 ) >= 0 ? pri - 16 : 0 );
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + rx.channel, ( pri - 16 ) >= 0 ? pri - 16 : 0 );
        }
    }
    
    inline static void lower_priority( void ) {
        int pri = nvic_execution_priority( );
        if ( pri <= priority ) {
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + tx.channel, priority );
            NVIC_SET_PRIORITY( IRQ_DMA_CH0 + rx.channel, priority );
        }
    }
public:
    Uart3Event( ) : loopBack( false ) {
        rxTermCharacterTrigger = -1;
        rxBufferSizeTrigger = -1;
        txEventHandler = defaultCallback;
        rxEventHandler = defaultCallback;
    }
    virtual void begin( uint32_t baud, uint32_t format ) {
        serial_dma_format( format );
        serial_dma_begin( BAUD2DIV3( baud ) );
    }
    virtual void   begin            ( uint32_t baud )   { serial_dma_begin( BAUD2DIV3( baud ) ); }
    virtual void   end              ( void )            { serial_dma_end( ); }
    virtual void   transmitterEnable( uint8_t pin )     { serial_dma_set_transmit_pin( pin ); }
    virtual int    available        ( void )            { return serial_dma_available( ); }
    virtual int    peek             ( void )            { return serial_dma_peek( ); }
    virtual int    read             ( void )            { return serial_dma_getchar( ); }
    virtual void   flush            ( void )            { serial_dma_flush( ); }
    virtual void   clear            ( void )            { serial_dma_clear( ); }
    virtual int    availableForWrite( void )            { return serial_dma_write_buffer_free( ); }
    virtual size_t write            ( uint8_t c )       { serial_dma_putchar( c ); return 1; }
    virtual size_t write            ( unsigned long n ) { return write( ( uint8_t )n ); }
    virtual size_t write            ( long n )          { return write( ( uint8_t )n ); }
    virtual size_t write            ( unsigned int n )  { return write( ( uint8_t )n ); }
    virtual size_t write            ( int n )           { return write( ( uint8_t )n ); }
    virtual size_t write9bit        ( uint32_t c )      {  return 0; }
    
    virtual size_t write( const uint8_t *buffer, size_t size ) {
        int s = serial_dma_write( buffer, size );
        return s;
    }
    virtual size_t write( const char *str ) {
        size_t len = strlen( str );
        int s = serial_dma_write( ( const uint8_t * )str, len );
        return s;
    }
    bool loopBack;
    static volatile int rxTermCharacterTrigger;
    static volatile int rxBufferSizeTrigger;
    static ISR txEventHandler;
    static ISR rxEventHandler;
    static const uint32_t txBufferSize = TX2_BUFFER_SIZE;
    static const uint32_t rxBufferSize = RX2_BUFFER_SIZE;
};
//---------------------------------------------End----------------------------------------------
#endif  // __cplusplus
#endif
