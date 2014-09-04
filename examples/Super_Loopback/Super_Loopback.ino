/*****************************************************
 * This is the same as the Loopback example BUT!!! this
 * example cascades the all three serial ports. So when
 * Serial1 RX event is fired its buffer is printed to 
 * Serial2 then when Serial2 RX event is fired its buffer   
 * is then printed to Serial3 and when its RX event is  
 * fired the result is printed to the serial monitor.
 *****************************************************/

#include <SerialEvent.h>
#include "monroe.h"

#define TX_BUFFER_SIZE 1024
#define RX_BUFFER_SIZE 128

Serial1Event Event1;
Serial2Event Event2;
Serial2Event Event3;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(0);
  while(!Serial);
  delay(100);
  //---------------------------------------------------------------------------------------
  // Must declare TX buffer size in bytes
  SERIAL1_MEMORY_TX(TX_BUFFER_SIZE);
  // Must declare RX buffer size in bytes, will fire when buffer is full or term if declared
  SERIAL1_MEMORY_RX(RX_BUFFER_SIZE);
  Event1.loopBack = true;// internal loopback set / "default = false"
  Event1.txEventHandler = tx1Event;// event handler Serial1 TX
  Event1.rxEventHandler = rx1Event;// event handler Serial1 RX
  Event1.rxTermCharacter = '\n'; // RX termination character
  Event1.begin(9600);// start Serial1
  //---------------------------------------------------------------------------------------
  // Must declare TX buffer size in bytes
  SERIAL2_MEMORY_TX(TX_BUFFER_SIZE);
  // Must declare RX buffer size in bytes, will fire when buffer is full or term if declared
  SERIAL2_MEMORY_RX(RX_BUFFER_SIZE);
  Event2.loopBack = true;// internal loopback set / "default = false"
  Event2.txEventHandler = tx2Event;// event handler Serial2 TX
  Event2.rxEventHandler = rx2Event;// event handler Serial2 RX
  Event2.rxTermCharacter = '\n'; // RX termination character
  Event2.begin(9600);// start Serial2
  //---------------------------------------------------------------------------------------
  // Must declare TX buffer size in bytes
  SERIAL3_MEMORY_TX(TX_BUFFER_SIZE);
  // Must declare RX buffer size in bytes, will fire when buffer is full or term if declared
  SERIAL3_MEMORY_RX(RX_BUFFER_SIZE);
  Event3.loopBack = true;// internal loopback set / "default = false"
  Event3.txEventHandler = tx3Event;// event handler Serial3 TX
  Event3.rxEventHandler = rx3Event;// event handler Serial3 RX
  Event3.rxTermCharacter = '\n'; // RX termination character
  Event3.begin(9600);// start Serial3
  //---------------------------------------------------------------------------------------
}

void loop() {
  // write a data packet
  Event1.write(monroe, 5510);
  // wait for data to flush
  Event1.flush();
}

//--------------------------------------Serial1 Events----------------------------------
void tx1Event(void) {
  // TX Event function will be fired when the DMA is finished sending a packet
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void rx1Event(void) {
  // RX Event fired, now we will print to Serial2
  Event2.printf("%s", Event1.rxBuffer);
}
//--------------------------------------Serial2 Events----------------------------------
void tx2Event(void) {
  // TX Event function will be fired when the DMA is finished sending a packet
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void rx2Event(void) {
  // RX Event fired, now we will print to Serial3
  Event2.printf("%s", Event1.rxBuffer);

}
//--------------------------------------Serial3 Events----------------------------------
void tx3Event(void) {
  // TX Event function will be fired when the DMA is finished sending a packet
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void rx3Event(void) {
  // RX Event fired, now print to serial monitor
  Serial.printf("%s", Event3.rxBuffer);

}

