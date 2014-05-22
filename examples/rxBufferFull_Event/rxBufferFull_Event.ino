/*****************************************************
 * This SerialEvent example shows the "RX Buffer Full"
 * event where it will fire the RX event handler when
 * the buffer is full. The library  allows you to
 * assign your buffer to whatever size you want.
 *
 * By using the loopback feature we can test the 
 * sending and receiving without having to connect
 * up anything. If you want to disable this feature
 * comment it out.
 *****************************************************/
#include <SerialEvent.h>

SerialEvent Event1 = SerialEvent();

char rx1Buffer[8];// RX buffer set to eight bytes

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(0);
    while(!Serial);
//------------------------------------------------------------------------------------
    Event1.loopBack = true;// internal loopback set / "default = false"
    Event1.port = &Serial1;// set port to Serial1
    Event1.txEventHandler = tx1Event;// event handler Serial1 TX
    Event1.rxEventHandler = rx1Event;// event handler Serial1 RX
    Event1.rxBuffer = rx1Buffer;// user supplied variable to hold incoming Serial1 data
    Event1.rxBufferSize = sizeof(rx1Buffer); // size of the RX buffer
    Event1.begin(9600);// start serial port
//------------------------------------------------------------------------------------
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        Event1.print(c);
    }
}

//--------------------------------------Serial1 Events--------------------------------
void tx1Event(void) {
    // TX Event function will fire when it is finished sending a packet
}

void rx1Event(void) {
    // RX Event function prints the buffer when it is full
    Serial.printf("Buffer Full Event: %s\n", rx1Buffer);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}
