/**
 * A bluetooth controlled sous-vide.
 * 
 * DS18B20 Thermal Probe
 * 1 Channel Relay
 * HC-06 Bluetooth Module
 * Immersion Heater
 * Submersion pump
 * 
 * Arduino Uno
 *
 */
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <stdlib.h>

#define MAX_BUFFER 4
#define RELAY_PIN 13
#define BT_TX_PIN 10
#define BT_RX_PIN 11
#define ONE_WIRE_PIN 3

SoftwareSerial bluetooth(BT_TX_PIN, BT_RX_PIN); // TX, RX
OneWire  ds(ONE_WIRE_PIN);  // on pin 3 (a 4.7K resistor is necessary)
 
int temp = 0;
char bt_data = 0;
char* buffer;
boolean receiving = false;
int pos = 0;
float currentTempCelsius = 0.0;
int desiredTempCelsius = 1;

void setup()  { 
  pinMode(RELAY_PIN, OUTPUT);
  bluetooth.begin(9600);
  // start serial for output
  Serial.begin(9600);         
  Serial.println("Ready!");
  bluetooth.println("Connected");
  buffer = new char[MAX_BUFFER];
}

void loop()  {   
  while (bluetooth.available()){
        bt_data=bluetooth.read();
        
         switch(bt_data) {
            //3: End of transmission
            case 3: 
              receiving = false;  
              temp = buffer2int(buffer);
                               
              Serial.print("Received: ");
              Serial.print(buffer);
              Serial.print(", Temp: ");
              Serial.println(temp);
              desiredTempCelsius = temp;
              break; //end message
            
            default: 
              if (receiving == false) 
                resetData();
               
               buffer[pos] = bt_data;
               pos++;
               receiving = true;          
          }
  }
  checkTemp();
  checkRelay();
  delay(2000);
}

void checkRelay()
{
  if(currentTempCelsius<=desiredTempCelsius)
  {
    Serial.println("Heating");
    digitalWrite(RELAY_PIN, HIGH;
  }
  else
  {
    Serial.println("Not Heating");
    digitalWrite(RELAY_PIN, LOW);   
  }
}

void resetData(){
   for (int i=0; i<=pos; i++) buffer[i] = 0; 
   pos = 0;
}
    
int buffer2int(char* buffer){
  int i;
  sscanf(buffer, "%d", &i);
  return i;
}

void checkTemp(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float fahrenheit;

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }

  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44);        // start conversion, use ds.write(0x44,1) with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  currentTempCelsius = (float)raw / 16.0;
  fahrenheit = currentTempCelsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(currentTempCelsius);
  char outstr[15];
  dtostrf(currentTempCelsius,7,3,outstr);
  bluetooth.write(outstr);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
}
