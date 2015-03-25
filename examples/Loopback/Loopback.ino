/*****************************************************
 * Simple example shows the use of the loopback feature
 * that the Teensy has for testing purposes and also how
 * to send and receive sets of data using TX and RX
 * events. Since we are using the DMA to send and receive
 * you will see that there is very little CPU overhead.
 *****************************************************/

#include <UartEvent.h>
#include "monroe.h"

Uart1Event Event1;

volatile bool print_flag = false;            // flag to indicate rx buffer full
const uint16_t BUFSIZE = Event1.rxBufferSize;// size of internal buffer
char buffer[BUFSIZE + 1];                    // user variable to hold incoming data
int pak_count = sizeof(monroe) / BUFSIZE;    // number of packets to send

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(0);
    while (!Serial);
    delay(1000);
    //--------------------------UartEvent1 Configuration-----------------------------------
    Event1.loopBack = true;          // internal loopback set / "default = false"
    Event1.txEventHandler = tx1Event;// event handler Serial1 TX
    Event1.rxEventHandler = rx1Event;// event handler Serial1 RX
    Event1.begin(115200);            // start Serial1
    //---------------------------------------------------------------------------------------
    Serial.print("Event1 Recieving (RX) Buffer Size -> ");
    Serial.println(Event1.rxBufferSize);// print the size of the internal RX buffer
    delay(1000);
}

void loop() {
    for (int i = 0; i < pak_count; i++) {
        Event1.write(monroe + (BUFSIZE * i), BUFSIZE);// write a part of monroe data
        while (!print_flag);                          // wait for rxEventHandler to fire
        Serial.print(buffer);                         // print buffer copied from rxEventHandler
        print_flag = false;                           // reset print flag
    }
    int rem = sizeof(monroe) % BUFSIZE;               // num remainding bytes
    if (rem) {
        Event1.write(monroe + (sizeof(monroe) - rem), rem);// write remainding bytes
        while (Event1.available() <= rem - 1);             // wait for remainder bytes to come through
        int x = 0;
        while (Event1.available()) buffer[x++] = Event1.read();
        buffer[x] = 0;
        Serial.print(buffer);// print buffer
        /*
         * must clear RX buffer indexes and reset RX DMA because of
         * bug using the read() function in an ISR and polling.
         */
        Event1.clear();
    }
    delay(1000);
}

//--------------------------------------Serial1 Events----------------------------------
void tx1Event(void) {
    // TX Event function will fired when the DMA is finished sending a packet
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void rx1Event(void) {
    // RX Event function prints the buffer when it is full or termination character is found
    int x = 0;
    while (Event1.available()) buffer[x++] = Event1.read();// fill buffer with serial
    buffer[x] = 0;      // zero out
    print_flag = true;  // set flag true to print buffer
}