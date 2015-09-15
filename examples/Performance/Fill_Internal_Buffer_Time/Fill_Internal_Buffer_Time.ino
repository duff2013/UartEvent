/*****************************************************
 * Show the faster copy to internal buffer than the
 * core Serial implemntation. More dramatic performance
 * is shown if you increase the default sizes of core
 * Serial1 buffer i.e. edit serial1.c (TX_BUFFER_SIZE) and
 * edit UartEvent.h (TX0_BUFFER_SIZE) from there default
 * sizes of 64 bytes.
 *****************************************************/

#include <UartEvent.h>

Uart1Event Event1;

// size of internal buffer defaults to the same as Serial1 (64 bytes)
const uint16_t BUFSIZE = Event1.txBufferSize;
uint8_t PACKET[BUFSIZE + 1];

const uint16_t INC_VAL = 16;    // Value to add to 'write' size
uint16_t PACKET_SIZE = INC_VAL; // size of sent packet

elapsedMicros time = 0;         // Time keeper
double avg_event1_micro_time;   // Event1 micro time from elapsedMicros
double avg_serial1_micro_time;  // Serial1 micro time from elapsedMicros

const int LoopCount = 4096;       // How many times to loop through the inner loop

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(0);
    while (!Serial);
    delay(1000);
    //--------------------------UartEvent1 Configuration-----------------------------------
    //Event1.txEventHandler = tx1Event;// event handler Serial1 TX
    //-------------------------------------------------------------------------------------
    Serial.print("Event1 Recieving (TX) Buffer Size -> ");
    Serial.println(BUFSIZE);// print the size of the internal RX buffer
    Serial.println("--------------------------------------------");
    delay(1000);
}

void loop() {
    if (PACKET_SIZE > BUFSIZE) PACKET_SIZE = INC_VAL;
    memset(PACKET, 'A', PACKET_SIZE);
    //------------------------Event1 Performance in a loop-----------------------------
    Event1.begin(1000000);// start Event1
    Serial.print("Starting Speed Comparison Test Event1  ");
    delay(200);
    for (int x = 0; x < LoopCount; x++) {
        time = 0;
        Event1.write(PACKET, PACKET_SIZE);
        avg_event1_micro_time += time;
        Event1.flush();
        if (!(LoopCount % x)) Serial.print('.');
    }
    Serial.println('.');
    Event1.end();// stop Event1
    //----------------------Serial1  Performance in a loop-----------------------------
    Serial1.begin(1000000);// start Serial1
    Serial.print("Starting Speed Comparison Test Serial1 ");
    delay(200);
    for (int x = 0; x < LoopCount; x++) {
        time = 0;
        Serial1.write(PACKET, PACKET_SIZE);
        avg_serial1_micro_time += time;
        Serial1.flush();
        if (!(LoopCount % x)) Serial.print('.');
    }
    Serial.println('.');
    Serial1.end();// stop Serial1
    //---------------------------------Stats--------------------------------------------
    avg_event1_micro_time = avg_event1_micro_time / LoopCount;
    avg_serial1_micro_time = avg_serial1_micro_time / LoopCount;
    float diff;
    if (avg_serial1_micro_time > avg_event1_micro_time) {
        diff = avg_serial1_micro_time - avg_event1_micro_time;
    }
    else if (avg_serial1_micro_time < avg_event1_micro_time) {
        diff = avg_event1_micro_time - avg_serial1_micro_time;
    }
    else diff = 0;
    
    delay(30);
    Serial.println("\n*************** STATS ***************");
    delay(30);
    Serial.print("Loop Count:                      ");
    Serial.println(LoopCount);
    delay(30);
    Serial.print("Bytes Sent Per loop:             ");
    Serial.println(PACKET_SIZE);
    delay(30);
    Serial.print("Event1  Avg Copy to Buffer Time: " );
    Serial.print(avg_event1_micro_time);
    Serial.println(" microseconds");
    delay(30);
    Serial.print("Serial1 Avg Copy to Buffer Time: ");
    Serial.print(avg_serial1_micro_time);
    Serial.println(" microseconds");
    delay(30);
    Serial.print("Difference:                      ");
    Serial.print(diff);
    Serial.println(" microseconds");
    delay(30);
    Serial.println("--------------------------------------------------------------");
    delay(500);
    PACKET_SIZE += INC_VAL;
}

//--------------------------------------Serial1 Events----------------------------------
void tx1Event(void) {
    // TX Event function will fired when the DMA is finished sending a packet
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}