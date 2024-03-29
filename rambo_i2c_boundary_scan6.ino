#include <Wire.h>


//#define TESTING

uint8_t pins_J3[] = { 44, 46, 48, 50, 52, 69, 67, 65, 63, 61, 59, 57, 55, 45, 47, 49, 51, 53, 68, 66, 64, 62, 60, 58, 56 };

uint8_t pins_J3_no6[] = { 44, 46, 48, 50, 52,     67, 65, 63, 61, 59, 57, 55, 45, 47, 49, 51, 53, 68, 66, 64, 62, 60, 58, 56 };
uint8_t pins_J3_no8[] = { 44, 46, 48, 50, 52, 69, 67,     63, 61, 59, 57, 55, 45, 47, 49, 51, 53, 68, 66, 64, 62, 60, 58, 56 };


#ifdef TESTING
uint8_t pins_J4[] = { 44, 19, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42 };
#else
uint8_t pins_J4[] = { 17, 19, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 18, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42 };
#endif

uint8_t    dbmp[] = {  1,  2,  3,  4,  5,  6,  7 , 8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 };

#ifdef TESTING
uint8_t rj_out_J1[] = { 4,   3,  2, 17, 18, 14, 15, 16 };
#else
uint8_t rj_out_J1[] = { 4, 3, 2,  1,  0, 14, 15, 16 };
#endif

uint8_t rj_out_J2[] = { 12, 11, 10,  9,  8,  7,  6,  5 };
uint8_t  db_in_J3[] = { 52, 65, 48, 67, 67, 46, 66, 50 };
uint8_t  db_in_J4[] = { 27, 33, 23, 31, 31, 19, 32, 25 };

int8_t test(uint8_t addr, uint8_t pins[], uint8_t pinCount, uint8_t rj_out[], uint8_t db_in[], uint8_t rjCount);

#define LED_PIN 13

#define START_PIN 12
#define DEBUG 1 //send debug info over serial about what we are doing
#define DEBOUNCE 2

#if DEBUG == 0
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#else
  #define DEBUG_PRINT(x) Serial.print(x);
  #define DEBUG_PRINTLN(x) Serial.println(x);
#endif

byte startReads = 0;
unsigned long startMillis;
char currentChar; //current char we are processing
int8_t pin; //current pin we are playing with

uint8_t pins[70];
uint8_t values[70];
uint8_t index;

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(10000);
  Serial.begin(115200);  // start serial for output
  Serial.println("1");
  i2c_scan();
  pinMode(LED_PIN,OUTPUT);
  Serial.println("start");

  index = 0;
  for(int i = 0; i < sizeof(index); i++) values[i]=0; //Clear values
}

#define DEVICE_1_ADDR 8


void finished(void){
  Serial.println("ok");
}

void loop() {
  /*
  if(!digitalRead(START_PIN))
  {
    startReads++;
    if(startReads >= DEBOUNCE && millis()-startMillis >= 1000){
      Serial.println("start");
      startReads = 0;
      startMillis = millis();
    }
  }*/
  static uint8_t address = 0;

  manage_blinker();
  
  if( Serial.available() )
  {
    currentChar = Serial.read();
    //DEBUG_PRINT("Recieved command : ");
    //DEBUG_PRINTLN(currentChar);
    delay(10);
    switch (currentChar)
    {
   case '\n': break;
   case '\r': break;
   case '?' :
     {
      Serial.println("Commands: ");
      Serial.println("? (prints list of commands)");
      Serial.println("Q<pin> (Reads Pin with pullups on)");
      Serial.println("J<addr> (J1J3 test for one i2c address)");
      Serial.println("K<addr> (J2J4 test for one i2c address)");
      Serial.println("F (Run full test)\n");
      break;
     }
    case 'U' : //Uptime
      {
        Serial.println(millis());
        break;
      }
      //Read Pin but turn pullups on
      //Format: Q<pin>
      //Returns: <pin val>\n
    case 'a' :
      {
          if(isDigit(Serial.peek())){
            address = Serial.parseInt();
          }
          else
          {
            Serial.print("address: ");
            Serial.println(address);
          }
          break;
      }
    case 'i' :
      {
          if(isDigit(Serial.peek())){
            index = Serial.parseInt();
          }
          else
          {
            Serial.print("index: ");
            Serial.println(index);
          }
          break;
      }
    case 'w' :
      {
          if(isDigit(Serial.peek())){
            values[index] = Serial.parseInt();
            index++;
          }
          else
          {
            Serial.print("values: ");
            for(int i = 0; i < sizeof(values); i++)
            {
              Serial.print(values[i]);
              Serial.print(',');
            }
            Serial.println();
          }
          break;
      }
    case 'r' :
      {
          if( isDigit(Serial.peek()) )
          {
            uint8_t readCount = Serial.parseInt();
            Wire.requestFrom(address,readCount);
            uint32_t timeout = 10000; while( Wire.available() == false && timeout) { timeout--; } ;
            while( Wire.available() ) {
              uint8_t rxData = Wire.read();
              Serial.print ( rxData );
              Serial.print(',');
            }
            Serial.println();
          }
          break;
      }
    case 'b' :
      {
          J1J3_test(address);
          break;
      }
    case 'g' :
      {
          J1J3_bridge_test(address);
          break;
      }
    case 'Q' : 
      {
        if(isDigit(Serial.peek())){
          pin = Serial.parseInt();
          pinMode(pin,INPUT);
          digitalWrite(pin,HIGH);
          Serial.println(digitalRead(pin));
          pin = -1;
        }
        finished();
        break; 
      }
      //Run full J1J3 test for 1 i2c address
      //Format: J<addr>
      //Returns: <1 for passed>\n
    case 'J' : 
      {
        if(isDigit(Serial.peek())){
          uint8_t addr = Serial.parseInt();
          Serial.println(J1J3_test(addr));
          pin = -1;
        }
        finished();
        break; 
      }
    case 'K' : 
      {
        if(isDigit(Serial.peek())){
          uint8_t addr = Serial.parseInt();
          Serial.println(J2J4_test(addr));
          pin = -1;
        }
        finished();
        break; 
      }
    case 'G' : 
      {
        if(Serial.peek()){
          uint8_t addr = Serial.parseInt();
          Serial.println(J2J4_test(addr));
          pin = -1;
        }
        finished();
        break; 
      }
    case 'F' : 
      {
        run_full_rack_test();
      }
    default:
      {
        Serial.println("Invalid Command. ? for List.");
      }
    } // Switch
  }

}

void run_full_rack_test()
{
  int result = full_rack_test();
  Serial.println();
  if(result) {
    Serial.print("Faults Found: ");
    Serial.println(result);
  }
  else
  {
    Serial.println("Passed");
  }
}

void i2c_scan()
{
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address <= 20; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan
}

int full_rack_test()
{
  int retval = 0;
  
  for(int addr = 1; addr <= 16; addr++)
  {
    retval += J1J3_test(addr);
    retval += J2J4_test(addr);
  }

  return retval;
}

uint8_t i2c_pin_low(uint8_t addr, uint8_t pin) {
  Wire.beginTransmission(addr);
  Wire.write('L');
  Wire.write(pin);
  Wire.endTransmission();
}

uint8_t i2c_pin_high(uint8_t addr, uint8_t pin) {
  Wire.beginTransmission(addr);
  Wire.write('H');
  Wire.write(pin);
  Wire.endTransmission();
}

uint8_t read_i2c_pin(uint8_t addr, uint8_t pin)
{
  Wire.beginTransmission((int)addr);
  Wire.write('Q');
  Wire.write(pin);
  Wire.endTransmission();

  Wire.requestFrom((int)addr, 1);
  if ( ! Wire.available() ) {
    Wire.requestFrom((int)addr, 1);
    //delay(1);
  }
  if( Wire.available() )
  {
    return Wire.read();
  }

  return -1;
}

int8_t J1J3_bridge_test(uint8_t addr)
{
  int8_t retval;
  uint32_t start_time = millis();
  DEBUG_PRINT("\nJ1J3 test address: ");
  DEBUG_PRINT(addr);
  DEBUG_PRINT(" cable: ");
  DEBUG_PRINT((addr - 1) * 2);
  retval = bridge_test(addr, pins_J3, sizeof(pins_J3) );
  //DEBUG_PRINT("Test1 Time: ");
  //DEBUG_PRINTLN(millis() - start_time);
  return retval;
}

int8_t J1J3_test(uint8_t addr)
{
  int8_t retval;
  uint32_t start_time = millis();
  DEBUG_PRINT("\nJ1J3 test address: ");
  DEBUG_PRINT(addr);
  DEBUG_PRINT(" cable: ");
  DEBUG_PRINT((addr - 1) * 2);
  retval = test(addr, pins_J3, sizeof(pins_J3), rj_out_J1, db_in_J3, sizeof(rj_out_J1) );
  //DEBUG_PRINT("Test1 Time: ");
  //DEBUG_PRINTLN(millis() - start_time);
  return retval;
}

int8_t J2J4_test(uint8_t addr)
{
  int8_t retval;
  uint32_t start_time = millis();
  DEBUG_PRINT("\nJ2J4 test address: ");
  DEBUG_PRINT(addr);
  DEBUG_PRINT(" cable: ");
  DEBUG_PRINT((addr - 1) * 2 + 1);
  retval = test(addr, pins_J4, sizeof(pins_J4), rj_out_J2, db_in_J4, sizeof(rj_out_J2) );
  //DEBUG_PRINT("Test1 Time: ");
  //DEBUG_PRINTLN(millis() - start_time);
  return retval;
}

int8_t bridge_test(uint8_t addr, uint8_t pins[], uint8_t pinCount)
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  uint8_t test_pass=0;

  // Bridged pins Test
  Wire.beginTransmission(addr);
  Wire.write('G');
  Wire.write(pins, pinCount);
  Wire.endTransmission();

  Wire.requestFrom(addr, pinCount);
  Serial.print(" B:");
  for(int i = 0; i < pinCount; i++)
  {
    if( ! Wire.available() ) {
      Serial.print(" ?");
      test_pass++;
      continue;
    } else {
      uint8_t c = Wire.read(); // receive a byte as character
      Serial.print(',');
      Serial.print(c);
      if( c ) test_pass++;
    }
  }
  Serial.println();
  return test_pass;
}

int8_t test(uint8_t addr, uint8_t pins[], uint8_t pinCount, uint8_t rj_out[], uint8_t db_in[], uint8_t rjCount) {
  //Serial.println("start test");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  uint8_t test_pass=0;

  // Read pins using internal pullups
  Wire.beginTransmission(addr);
  Wire.write('Q'); // INPUT_PULLUP
  Wire.write(pins, pinCount);
  Wire.endTransmission();

  Wire.requestFrom(addr, pinCount);    // request 25 bytes from device

  // Short to GND Test
  Serial.print(" S:");
  for(int i = 0; i < pinCount; i++)
  {
    if( ! Wire.available() ) {
      Serial.print("?");
      test_pass++;
      continue;
    } else {
      uint8_t c = Wire.read(); // receive a byte as character
      Serial.print(c);
      if( ! c ) test_pass++;
    }
  }


  // Bridged pins Test
  Wire.beginTransmission(addr);
  Wire.write('B');
  Wire.write(pins, pinCount);
  Wire.endTransmission();

  

  Wire.requestFrom(addr, pinCount);
  Serial.print(" B:");
  for(int i = 0; i < pinCount; i++)
  {
    if( ! Wire.available() ) {
      Serial.print("?");
      test_pass++;
      continue;
    } else {
      uint8_t c = Wire.read(); // receive a byte as character
      Serial.print(c);
      if( c ) test_pass++;
    }
  }

  // RJ45 Connectivity Test
  DEBUG_PRINT(" C:");
  for (int i=0; i<rjCount; i++)
  {
    //DEBUG_PRINT(" ");
    //read_i2c_pin(addr, db_in[i]);  //pinMode(db_in_J3[i], INPUT_PULLUP);
    i2c_pin_low(addr, rj_out[i]);  //pinMode(rj_out_J1[i], OUTPUT);  //digitalWrite(rj_out_J1[i], LOW);
    int8_t c = read_i2c_pin(addr, db_in[i]);
    
    if( c == -1 ) Serial.print('?');
    else Serial.print(c);

    if (  c != LOW )
    {
      test_pass++;
      //DEBUG_PRINT("RJ output pin ");
      //DEBUG_PRINT(i + 1);
      //DEBUG_PRINTLN(" not connected to DB25 pin.");
    }

    read_i2c_pin(addr, rj_out[i]); //pinMode(rj_out_J1[i], INPUT);
  }

  Serial.println();
  return test_pass;
} //Test

//Blink over i2c
void loop2() {
  uint8_t buf[] = { 'L', 13 };
  Wire.beginTransmission(8);
  Wire.write(buf, 2);
  Wire.endTransmission();
  delay(1000);
  uint8_t buf2[] = { 'H', 13 };
  Wire.beginTransmission(8);
  Wire.write(buf2, 2);
  Wire.endTransmission();
  delay(1000);
}

void manage_blinker()
{
  static uint32_t last_blink_time = 0;
  static bool led = 0;

  if( (millis() - last_blink_time) > 100 )
  {
    digitalWrite(LED_PIN,led);
    last_blink_time = millis();
    led = !(led);
  }
}
