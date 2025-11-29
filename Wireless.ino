// https://forum.arduino.cc/t/serial-input-basics-updated/382007
// https://forum.arduino.cc/t/add-character-to-string/285038/3

#include <stdio.h>
#include <stdlib.h>


//#### Declare Variables
char receivedCharTerm;
char receivedCharWire;
String dataReceived;
boolean newDataWire = false;

void setup() {
    Serial.begin(115200);    // Computer Terminal
    Serial1.begin(38400);    // Wireless Connection
    Serial.println("<Arduino is ready>");
    dataReceived = "";
}

void loop() {
    // Clear string;
    dataReceived = "";
    recvOneChar_send();
    recvOneChar_recv();
    showNewData();
    delay(1);
}

// Receive from terminal
void recvOneChar_send() {
    if (Serial.available() > 0) {
        receivedCharTerm = Serial.read();
        Serial1.print(receivedCharTerm);
    }
}

// Receive from wireless
void recvOneChar_recv() {
    if (Serial1.available() > 0) {
        delay(15); // delay for message to buffer
        while(Serial1.available()){
            receivedCharWire = Serial1.read();
            dataReceived.concat(receivedCharWire);
        }
        newDataWire = true;
    }
}

// Print inbound information to serial
void showNewData() {
    if (newDataWire == true) {
        Serial.println(dataReceived);
        newDataWire = false;
    }
}