#include <SPI.h>   // Comes with Arduino IDE
#include "RF24.h"  // Download and Install (See above)
#define  CE_PIN  9   // The pins to be used for CE and SN
#define  CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

byte addresses[][6] = {"1Node", "2Node"}; // These will be the names of the "Pipes"

class A{
  public:
  float x,y,z;  
};
A angle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  radio.begin();          
  radio.setChannel(85);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]); 
  radio.startListening();
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!radio.available());
  radio.read(&angle, sizeof(angle));
}
