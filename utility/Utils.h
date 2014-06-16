/*
 ||
 || @file 		Utils.h
 || @version 	1
 || @author 	Colin Duffy
 || @contact 	cmduffy@engr.psu.edu
 ||
 || @description
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

#ifndef Utils_h
#define Utils_h

#define BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define BITBAND_U32(reg, bit) (*(uint32_t *)BITBAND_ADDR((reg), (bit)))
#define BITBAND_U8(reg, bit) (*(uint8_t *)BITBAND_ADDR((reg), (bit)))

#define UART_DMA_ENABLE     UART_C5_TDMAS | UART_C5_RDMAS
#define UART_DMA_DISABLE    0
#define IRQ_PRIORITY        64  // 0 = highest priority, 255 = lowest

#define UART_C5_TDMAS       (uint8_t)0x80
#define UART_C5_RDMAS       (uint8_t)0x20

#define C2_ENABLE           UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE
#define C2_TX_ACTIVE		C2_ENABLE | UART_C2_TIE
#define C2_TX_COMPLETING	C2_ENABLE | UART_C2_TCIE
#define C2_TX_INACTIVE		C2_ENABLE

#define SCGC7_DMA_BIT       1
#define SCGC6_DMAMUX_BIT    1

#define likely(x)           __builtin_expect(!!(x), 1)
#define unlikely(x)         __builtin_expect(!!(x), 0)

#define TX1_PACKET_SIZE 128
typedef struct transmit1_fifo_t {
    volatile uint8_t packet[TX1_PACKET_SIZE];
    volatile uint16_t size;
    volatile boolean eventTrigger;
} tx1_fifo_t;

#define TX2_PACKET_SIZE 128
typedef struct transmit2_fifo_t {
    uint8_t packet[TX2_PACKET_SIZE];
    volatile uint16_t size;
    volatile boolean eventTrigger;
} tx2_fifo_t;

#define TX3_PACKET_SIZE 128
typedef struct transmit3_fifo_t {
    uint8_t packet[TX3_PACKET_SIZE];
    volatile uint16_t size;
    volatile boolean eventTrigger;
} tx3_fifo_t;

#define SERIAL1_MEMORY_TX(num) ({                            \
    const int fifoSize = num/TX1_PACKET_SIZE + 1;            \
    DMAMEM static tx1_fifo_t data[fifoSize];                 \
    Serial1Event::initialize_tx_memory(data, fifoSize);      \
})

#define SERIAL1_MEMORY_RX(num) ({                           \
    DMAMEM static uint8_t data[num+1];                      \
    Serial1Event::initialize_rx_memory(data, num);          \
})

#define SERIAL2_MEMORY_TX(num) ({                           \
    const int fifoSize = num/TX2_PACKET_SIZE + 1;           \
    DMAMEM static tx2_fifo_t data[fifoSize];                \
    Serial2Event::initialize_tx_memory(data, fifoSize);     \
})

#define SERIAL2_MEMORY_RX(num) ({                           \
    DMAMEM static uint8_t data[num+1];                      \
    Serial2Event::initialize_rx_memory(data, num);          \
})

#define SERIAL3_MEMORY_TX(num) ({                           \
    const int fifoSize = num/TX3_PACKET_SIZE + 1;           \
    DMAMEM static tx3_fifo_t data[fifoSize];                \
    Serial3Event::initialize_tx_memory(data, fifoSize);     \
})

#define SERIAL3_MEMORY_RX(num) ({                           \
    DMAMEM static uint8_t data[num+1];                      \
    Serial3Event::initialize_rx_memory(data, num);          \
})

#endif
