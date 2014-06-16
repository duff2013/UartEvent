/*****************************************************
 * This SerialEvent example shows the "1 Byte Receive"
 * event where it will fire the event handler for every
 * byte that is received on the selected serial port.
 * This is done by setting the RX buffer size to 1.
 *
 * By using the loopback feature we can test
 * the sending and receiving without having to connect
 * up anything. If you want to disable this feature
 * comment it out.
 *****************************************************/
#include <SerialEvent.h>

Serial1Event Event1;

#define TX_BUFFER_SIZE 128
/*******************************************************
 * when the RX buffer is set to 1, an Event will fire
 * for every byte received. If you set the buffer to
 * more than 1 it will fire every buffer size bytes.
 *******************************************************/
#define RX_BUFFER_SIZE 1

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(0);
  while(!Serial);
  //-----------------------------------------------------------------------------------------
  // Must declare TX buffer size in bytes, this must be as big as your largest packet
  SERIAL1_MEMORY_TX(TX_BUFFER_SIZE);
  // Must declare RX buffer size in bytes, will fire when buffer is full or term if declared
  SERIAL1_MEMORY_RX(RX_BUFFER_SIZE);
  Event1.loopBack = true;// internal loopback set / "default = false"
  Event1.txEventHandler = tx1Event;// event handler Serial1 TX
  Event1.rxEventHandler = rx1Event;// event handler Serial1 RX
  Event1.begin(9600);// start serial port
  //-----------------------------------------------------------------------------------------
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Event1.print(c);
  }
}

//--------------------------------------Serial1 Events------------------------------------
void tx1Event(void) {
  // TX Event function will fire when it is finished sending a packet
}

void rx1Event(void) {
  // RX Event function prints the buffer when it is full
  char myByte = Event1.rxBuffer[0];
  Serial.printf("Byte Recevied Event: %c\n", myByte);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}