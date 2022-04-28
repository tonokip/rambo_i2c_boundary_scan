#include <Wire.h>

#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)

void setup() {
  Wire.begin(13);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.end();
  //Serial.begin(115200);           // start serial for output
  
  DEBUG_PRINTLN("start");
}

uint8_t pin;
uint8_t values[70];
uint8_t dataSize;

void loop() {
  // put your main code here, to run repeatedly:

}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  DEBUG_PRINT("howMany: ");
  DEBUG_PRINTLN(howMany);
  uint8_t i = 0;
  int8_t buf[70] = {0};
  while (Wire.available()) {
    buf[i] = Wire.read();
    DEBUG_PRINTLN(buf[i]);
    i++;
  }
  dataSize = i;

  if( i < 2) return;


  if( buf[0] == 'Q' ) {
    for(uint8_t i = 1; i < dataSize; i++) {
      pinMode(buf[i], INPUT_PULLUP);
      values[i-1] = digitalRead(buf[i]);
    }
  }

  if( buf[0] == 'H' ) {
    for(uint8_t i = 1; i < dataSize; i++) {
      pinMode(buf[i], OUTPUT);
      digitalWrite(buf[i], HIGH);
    }
  }

  if( buf[0] == 'L' ) {
    for(uint8_t i = 1; i < dataSize; i++) {
      pinMode(buf[i], OUTPUT);
      digitalWrite(buf[i], LOW);
    }
  }

  if( buf[0] == 'B' ) {
    for(int i = 0; i < sizeof(values); i++) values[i]=0;
    for(int i = 1; i < dataSize; i++) {
      pinMode(buf[i], OUTPUT);
      digitalWrite(buf[i], LOW);
      for(int j=1; j < dataSize ; j++)
      {
        if(j != i && digitalRead(buf[j]) == LOW)
        {
          values[j-1]++;
          /*
          Serial.print("Driving pin ");
          Serial.print(buf[i]);
          Serial.print(" low causes pin ");
          Serial.print(buf[j]);
          Serial.println(" to be low."); */
        }
      } // For j
      pinMode(buf[i], INPUT_PULLUP);
    } // For i
  } // 'B'

}

void requestEvent() {
  Wire.write(values, dataSize);
}
