/*****************************************************
 * This is the same as the Loopback example BUT!!! this
 * example cascades the all three serial ports. So when
 * Serial1 RX event is fired its buffer is printed to
 * Serial2 then when Serial2 RX event is fired its buffer
 * is then printed to Serial3 and when its RX event is
 * fired the result is printed to the serial monitor.
 *****************************************************/
#include <UartEvent.h>
#include "monroe.h"

Uart1Event Event1;
Uart2Event Event2;
Uart2Event Event3;

volatile bool print_flag = false;            // flag to indicate rx buffer full
const uint16_t BUFSIZE = Event1.rxBufferSize;// size of internal buffer
char buffer[BUFSIZE + 1];                    // user variable to hold incoming data
int pak_count = 5824 / BUFSIZE;              // number of packets to send

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(0);
    while (!Serial);
    delay(1000);
    //---------------------------------------------------------------------------------------
    Event1.loopBack = true;          // internal loopback set / "default = false"
    Event1.txEventHandler = tx1Event;// event handler Serial1 TX
    Event1.rxEventHandler = rx1Event;// event handler Serial1 RX
    Event1.begin(115200);            // start Serial1
    //---------------------------------------------------------------------------------------
    Event2.loopBack = true;          // internal loopback set / "default = false"
    Event2.txEventHandler = tx2Event;// event handler Serial2 TX
    Event2.rxEventHandler = rx2Event;// event handler Serial2 RX
    Event2.begin(115200);            // start Serial2
    //---------------------------------------------------------------------------------------
    Event3.loopBack = true;          // internal loopback set / "default = false"
    Event3.txEventHandler = tx3Event;// event handler Serial3 TX
    Event3.rxEventHandler = rx3Event;// event handler Serial3 RX
    Event3.begin(115200);            // start Serial3
    //---------------------------------------------------------------------------------------
    Serial.print("Event1 Recieving (RX) Buijhffer Size -> ");
    Serial.println(Event1.rxBufferSize);// print the size of the internal RX buffer
}

void loop() {
    for (int i = 0; i < pak_count; i++) {
        // write a part of monroe data
        Event1.write(monroe + (BUFSIZE * i), BUFSIZE);
        // wait for rxEventHandler to fire
        while (!print_flag);
        // print buffer copied from rxEventHandler
        Serial.print(buffer);
        // reset print flag
        print_flag = false;
    }
    delay(1000);
}

//--------------------------------------Serial1 Events----------------------------------
void tx1Event(void) {
    // TX Event function will be fired when the DMA is finished sending a packet
    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
}

void rx1Event(void) {
    // RX Event fired, now we will print to Uart2
    int x = 0;
    while (Event1.available()) buffer[x++] = Event1.read();
    buffer[x] = 0;
    Event2.write((uint8_t*)buffer, BUFSIZE);
}
//--------------------------------------Serial2 Events----------------------------------
void tx2Event(void) {
    // TX Event function will be fired when the DMA is finished sending a packet
    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
}

void rx2Event(void) {
    // RX Event fired, now we will print to Uart3
    int x = 0;
    while (Event2.available()) buffer[x++] = Event2.read();
    buffer[x] = 0;
    Event3.write((uint8_t*)buffer, BUFSIZE);
}
//--------------------------------------Serial3 Events----------------------------------
void tx3Event(void) {
    // TX Event function will be fired when the DMA is finished sending a packet
    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
}

void rx3Event(void) {
    // RX Event fired, now print to serial monitor
    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    int x = 0;
    while (Event3.available()) buffer[x++] = Event3.read();
    buffer[x] = 0;
    print_flag = true;
}



