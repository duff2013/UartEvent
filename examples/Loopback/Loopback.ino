/*****************************************************
 * Simple example shows the use of the loopback feature
 * that the Teensy has for testing purposes and also how
 * to send and receive sets of data using TX and RX
 * events.
 *****************************************************/

#include <UartEvent.h>
#include "monroe.h"
//---------------------------------------------------------------------------------------
Uart1Event Event1;
//---------------------------------------------------------------------------------------
const uint16_t BUFSIZE = sizeof(monroe); // size of buffer
char buffer[BUFSIZE + 1];                // user buffer to hold incoming data
volatile bool print_flag = false;        // flag to indicate to print buffer
volatile int x = 0;                      // buffer increment
//---------------------------------------------------------------------------------------
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    while (!Serial);
    delay(1000);
    //--------------------------UartEvent1 Configuration-----------------------------------
    Event1.loopBack        = true;     // internal loopback set / "default = false"
    Event1.txEventHandler  = tx1Event; // event handler Serial1 TX
    Event1.rxEventHandler  = rx1Event; // event handler Serial1 RX
    Event1.begin(6000000);             // start Serial1
    //-------------------------------------------------------------------------------------
    Serial.print("Event1 Recieving (RX) Buffer Size -> ");
    Serial.println(Event1.rxBufferSize);// print the size of the internal RX buffer
    delay(1000);
}

void loop() {
    for (int i = 0; i < sizeof(monroe); i++) {
        while (Event1.availableForWrite() < 1);
        Event1.write(monroe[i]);
    }
    
    if (print_flag) {
        Serial.print(buffer);
        print_flag = false;
    }
    delay(1000);
}

//------------------------------------UartEvent1 Events--------------------------------
void tx1Event(void) {
    // This TX Event function will fired when the DMA is finished sending a packet
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void rx1Event(void) {
    // This RX Event will fire when termination character is found
    while (Event1.available()) { // put data into a buffer
        uint8_t c = Event1.read();
        buffer[x] = c;
        if (++x >= BUFSIZE) {
            print_flag = true;
            x = 0;
        }
    }
}