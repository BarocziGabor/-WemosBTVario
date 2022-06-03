#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

/*
R1 ~= 47480 Ohm
R2 ~= 99800 Ohm

R2/(R1+R2) = 0.676

Vbat = ADCin/820.02

Vbat% = (Vbat-3) /(4.2-3)*100
*/

#define SCALE_ADCBAT_TO_VBATFLOAT 820.02


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ledflasher.h>

#define FREQUENCY 20 // freq output in Hz
#define USE_LK8000 //comment this line to use PRS sentence
// #define USB_MODE // usb by default
#define BLUETOOTH_MODE // uncomment this line for bluetooth mode
#define BLUETOOTH_NAME "BG-Barometer"
// SDA = 19
// SCL = 23

BluetoothSerial SerialBT;
Adafruit_BME280 bme;
LEDFlasher lf;

void heartBeat();
void measureBatt();
void readSerialCommand();
void readBTSerialCommand();

int Vbatperc;
float Vbat;

void setup() {
  Serial.begin(115200);
  SerialBT.begin(BLUETOOTH_NAME); //Bluetooth device name
  Serial.println(F("The device started, now you can pair it with bluetooth!"));
  Serial.println(F(BLUETOOTH_NAME));

  //wdt_disable();
  delay(10);
  Serial.println("XCTRACK VARIO " + String(__DATE__));
  lf.begin(LED_BUILTIN);
  lf.setFlashingTIme(100);
  

  // if (bme.begin(0x76) == true) {
  //   Serial.println(F("0x76"));
  //   Serial.println(bme.sensorID());
  // } 
  // if (bme.begin(0x77) == true) {
  //   Serial.println(F("0x77"));
  //   Serial.println(bme.sensorID());
  // }
  // while(true);

  //wdt_enable(WDTO_1S); //enable the watchdog 1s without reset

  // while(true){
  //   heartBeat();
  //   measureBatt();
  // }

  Serial.println(F("Checking BMP280 sensor..."));
  if (bme.begin(0x76) == false) {  
    Serial.println(F("Error connecting to BMP280..."));
    Serial.println(F("Resetting in 3 seconds..."));
    SerialBT.println(F("Error connecting to BMP280..."));
    SerialBT.println(F("Resetting in 3 seconds..."));
    delay(3000);
    ESP.restart();
  }


  //}
  Serial.println(F("BMP280 initialized"));
  SerialBT.println(F("BMP280 initialized"));

  bme.setSampling(Adafruit_BME280::MODE_NORMAL, 
              Adafruit_BME280::SAMPLING_X1,
              Adafruit_BME280::SAMPLING_X8,
              Adafruit_BME280::SAMPLING_NONE,
              Adafruit_BME280::FILTER_X8,
              Adafruit_BME280::STANDBY_MS_0_5);
}

uint32_t get_time = millis();
uint32_t sum = 0;
uint8_t n = 0;

void loop() {
  //wdt_reset();
  uint32_t Pressure = (uint32_t)bme.readPressure();
  sum += Pressure;
  n += 1;

  if (millis() >= (get_time + (1000/FREQUENCY))) {
    get_time = millis();
    Pressure = sum / n ;
    sum=0; n=0;
    uint8_t Temp = bme.readTemperature()/100;

    //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
    //$LK8EX1,pressure,99999   ,9999 ,temp      ,999    ,*checksum

    #ifdef USE_LK8000
    String str_out = String("LK8EX1,") + String(Pressure)+String(",0,9999,") + String(Temp)+String(",") + String(1000+Vbatperc) + String(",");    
    uint16_t checksum = 0, bi;
    for (uint8_t ai = 0; ai < str_out.length(); ai++) {
      bi = (uint8_t)str_out[ai];
      checksum ^= bi;
    }    
    str_out = "$"+str_out+"*"+String(checksum, HEX);

    #else // use PRS sentence
    String str_out = String("PRS ")+ String(Pressure, HEX);
    #endif
    
    #ifdef USB_MODE      
    Serial.println(str_out);
    #endif
    #ifdef BLUETOOTH_MODE
    //SerialBT.println(str_out);
    SerialBT.println(String(millis()) + String("\t") + String(Vbat) + String("\t") + String(Vbatperc));
    #endif
  }

  readSerialCommand();
  readBTSerialCommand();
  measureBatt();
  heartBeat();
}

void inline heartBeat(){
  static unsigned long hb_timer;
  if (millis() > hb_timer + 3000){
      if(SerialBT.hasClient()){
        lf.flash(2);
      } else {
        lf.flash(1);
      }
      hb_timer = millis();
  }
  lf.update();  
}

void inline measureBatt(){
  static unsigned long bat_timer;
  if (millis() > bat_timer + 5000){
    Vbat = (float)analogRead(12)/SCALE_ADCBAT_TO_VBATFLOAT;
    Vbatperc = (Vbat - 3) / (4.2-3) * 100;
    // SerialBT.println(String(millis()) + String("\t") + String(Vbat) + String("\t") + String(Vbatperc));
    bat_timer = millis();
  }
}

void inline readSerialCommand() {
    if(Serial.available()){
    String s;
    while (Serial.available()){
      s += (char)Serial.read();
    }
    //Serial.println(s);
    // x,x
    int separator_index = s.indexOf(",");
    //Serial.println(separator_index);
    if (separator_index > 0){
      int pressure_sampling = s.substring(0,separator_index).toInt();
      int filter = s.substring(separator_index+1,s.length()).toInt();
      Serial.println(s);
      bme.setSampling(Adafruit_BME280::MODE_NORMAL, 
        Adafruit_BME280::SAMPLING_X1,
        static_cast<Adafruit_BME280::sensor_sampling>(pressure_sampling),
        Adafruit_BME280::SAMPLING_NONE,
        static_cast<Adafruit_BME280::sensor_filter>(filter),
        Adafruit_BME280::STANDBY_MS_0_5);
    }
    else {
      Serial.println(F("Invalid command. Use format: [0-5],[0-5]"));
    }
  }
}

void inline readBTSerialCommand() {
  if(SerialBT.available()){
    String s;
    while (SerialBT.available()){
      s += (char)SerialBT.read();
    }
    //SerialBT.println(s);
    // x,x
    int separator_index = s.indexOf(",");
    //SerialBT.println(separator_index);
    if (separator_index > 0){
      int pressure_sampling = s.substring(0,separator_index).toInt();
      int filter = s.substring(separator_index+1,s.length()).toInt();
      SerialBT.println(s);
      bme.setSampling(Adafruit_BME280::MODE_NORMAL, 
        Adafruit_BME280::SAMPLING_X1,
        static_cast<Adafruit_BME280::sensor_sampling>(pressure_sampling),
        Adafruit_BME280::SAMPLING_NONE,
        static_cast<Adafruit_BME280::sensor_filter>(filter),
        Adafruit_BME280::STANDBY_MS_0_5);
    }
    else {
      SerialBT.println(F("Invalid command. Use format: [0-5],[0-5]"));
    }
  }
}