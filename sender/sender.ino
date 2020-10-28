#include <SPI.h>
#include <LoRa.h>


// LoRa Radio parameters
#define SCK 13
#define MISO 12
#define MOSI 11
#define SS 10
#define RST 9
#define DIO0 2
#define FRQ 915E6
#define implicitHeader false
int power = 20;
int signalBandwidth = 125e3;
int spreading_factor = 10;
int preamble_length = 8;
byte sync_word = 0x12;

// Initialize variables to get and save LoRa data
int rssi;
String LoRaMessage = "";
String LoRaData;
String loraOutMsg = "";


///SENSOR PARAMETERS
// OneWire DS18B20 Temperature Sensor
// Include the libraries we need
#include <OneWire.h>
OneWire  ds(6);
byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];
float celsius, fahrenheit;
float temp;

///LOW POWER PARAMETERS
#include <Adafruit_SleepyDog.h>
int max_sleep_iterations = 225; // 225*8 = 3600 second, 30 minutes
int sleep_iterations = max_sleep_iterations;
int sleepMS = 0;

void setup() {
  Serial.begin(115200);
  startLoRA();
  setup1W();
  Serial.println("Setup Complete");
}

void loop() {

  if (sleep_iterations >= max_sleep_iterations) {
    Serial.println("Running...");
    sleep_iterations = 0;

    temp = get1wTemp();
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
    }
    // CRC is OK.
    Serial.println(temp);
    LoRa.beginPacket();
    LoRa.print(temp);
    LoRa.endPacket();
    LoRa.sleep();

  }
  Serial.println("Awake. Number of naps: " + String(sleep_iterations) + ". I slept for " + String(sleepMS / 1000.0) + " seconds. Going to sleep...");

  digitalWrite(LED_BUILTIN, HIGH);
  delay(75);
  digitalWrite(LED_BUILTIN, LOW);

  sleepMS = Watchdog.sleep();

#if defined(USBCON) && !defined(USE_TINYUSB)
  USBDevice.attach();
#endif
  sleep_iterations += 1;

}

//Initialize LoRa module
void startLoRA() {
  int counter = 0;
  //SPI LoRa pins
  //  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  LoRa.setSpreadingFactor(spreading_factor);
  LoRa.setTxPower(power);
  LoRa.setPreambleLength(preamble_length);
  LoRa.setSignalBandwidth(signalBandwidth);
  LoRa.setSyncWord(sync_word);
  while (!LoRa.begin(FRQ) && counter < 10) {
    Serial.println("Attempt to initialize LoRa failed. Trying again up to 10 times.");
    counter++;
    delay(500);
  }
  if (counter == 10)
    Serial.println("Starting LoRa failed!");
  else
    Serial.println("LoRa Initialization OK!");
}


// Read LoRa packet and get the sensor readings
String getLoRaData() {
  String LoRaData = "";
  Serial.print("Lora packet received: ");
  // Read packet
  while (LoRa.available()) {
    LoRaData = LoRa.readString();
    Serial.print(LoRaData);
  }

  rssi = LoRa.packetRssi();
  Serial.print(" with RSSI ");
  Serial.println(rssi);
  return LoRaData;
}

void setup1W() {

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }

  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
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

}

float get1wTemp() {

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();

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
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  return fahrenheit;

}
