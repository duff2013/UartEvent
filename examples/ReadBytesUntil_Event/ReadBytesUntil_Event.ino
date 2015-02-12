/*****************************************************
 * This UartEvent example shows the "Read Bytes Until"
 * event where it will fire the RX event handler when
 * a termination character is detected or the buffer
 * is full.
 * Open a serial monitor and type into it and end your
 * command with a line feed, "return" or carriage
 * return, "enter".
 *
 * By using the loopback feature we can test
 * the sending and receiving without having to connect
 * up anything. If you want to disable this feature
 * comment it out.
 *****************************************************/
#include <UartEvent.h>

Uart1Event Event1;

volatile bool print_flag = false;// flag to indicate termination character
const uint16_t BUFSIZE = Event1.rxBufferSize;// size of internal buffer
char buffer[BUFSIZE+1];// user variable to hold incoming data

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(0);
    while(!Serial);
    //--------------------------Uart1Event Configuration----------------------------------
    Event1.loopBack = true;           // internal loopback set / "default = false"
    Event1.txEventHandler = tx1Event; // event handler Serial1 TX
    Event1.rxEventHandler = rx1Event; // event handler Serial1 RX
    Event1.rxTermCharacter = '\n';    // this termination character will fire the RX event handler
    Event1.begin(9600);               // start serial port
    //------------------------------------------------------------------------------------
    delay(1000);
    Serial.println("TX event will fire when termination character is detected");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        Event1.print(c);
    }
    
    if(print_flag) {
        Serial.print(buffer);
        print_flag = false;
    }
}

//--------------------------------------Serial1 Events-------------------------------------
void tx1Event(void) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // TX Event function will fire when it is finished sending a packet
}

void rx1Event(void) {
    // This function fires when termination character is detected
    int x = 0;
    while(Event1.available()) buffer[x++] = Event1.read();
    buffer[x] = 0;
    print_flag = true;
}