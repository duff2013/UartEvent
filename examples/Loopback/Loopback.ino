/*****************************************************
 * This SerialEvent example shows the use of the 
 * loopback feature that the Teensy has for testing 
 * purposes and also how to send and receive large sets 
 * of data using TX and RX events. Since we are using 
 * the DMA to send and receive you will see that there 
 * is very little CPU overhead.
 *
 * Transmitting data is packet based so even sending
 * large array of data takes much less time in the write
 * function than would be using the normal Serial1 write.
 *
 * Receiving allows the user to declare his/her own
 * buffer and size. If you don't know the size of your
 * receiving data you can use either polling mode or
 * set your buffer size to 1.
 *****************************************************/

#include <SerialEvent.h>
#include "monroe.h"

SerialEvent Event1 = SerialEvent();

volatile char rx1Buffer[128]; // RX Event will fire when buffer is full (128 bytes)

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(0);
    while(!Serial);
//---------------------------------------------------------------------------------------
    Event1.loopBack = true;// internal loopback set / "default = false"
    Event1.port = &Serial1;// set port to Serial1
    Event1.txEventHandler = tx1Event;// event handler Serial1 TX
    Event1.rxEventHandler = rx1Event;// event handler Serial1 RX
    Event1.rxBuffer = rx1Buffer;// user supplied variable to hold incoming Serial1 data
    Event1.rxBufferSize = sizeof(rx1Buffer); // size of the RX buffer
    Event1.begin(9600);// start Serial1
//---------------------------------------------------------------------------------------
}

void loop() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // write a data packet
    Event1.write(monroe, 5510);
    /************************************************************************
     * Since we return very quickly we can wait for the TX buffer to flush.
     * Comment out this flush and it will still work but since packet "monroe"
     * takes up most of the allocated TX memory (5520 bytes) it will not put
     * any more data on the buffer until the it has enough memory. This means 
     * you can have skipped data. This is where it differs from normal TX 
     * methods in which it will block until the data is sent. Flushing the data
     * will mimic that behavior.
     ************************************************************************/
    Event1.flush();
}

//--------------------------------------Serial1 Events----------------------------------
void tx1Event(void) {
    // TX Event function will fired when the DMA is finished sending a packet
    Serial.print("\ntx1 Event Fired\n");
}

void rx1Event(void) {
    // RX Event function prints the buffer when it is full
    Serial.print(rx1Buffer);
    
}
