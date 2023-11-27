/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleWrite.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            14
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}
#define m1    0.005294779116466
#define m2    0.005443435177539
#define v0_1  1.021101204819277
#define v0_2  1.038402146985962
#define Rsen 9.937
#define DELAYMS 30

int sen1,sen2; 
float v1 , v2;
int i = 0;
int n = 1000;
uint32_t t1;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
bool deviceConnect = false;
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;
BLEAdvertising *pAdvertising;

class MyCallbacks: public BLECharacteristicCallbacks {
  void onConnect(BLEServer *pServer){
    deviceConnect = true;
  }
  void onDisconnect(BLEServer *pServer){
    deviceConnect = false;
  }
//    void onWrite(BLECharacteristic *pCharacteristic) {
//      std::string value = pCharacteristic->getValue();
//
//      if (value.length() > 0) {
//        Serial.println("*********");
//        Serial.print("New value: ");
//        for (int i = 0; i < value.length(); i++)
//          Serial.print(value[i]);
//
//        Serial.println();
//        Serial.println("*********");
//      }
//    }
};

void setup() {
  //setCpuFrequencyMhz(80);
  Serial.begin(115200);
  Serial.println("BLE startting");
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
/*Serial.println("1- Download and install an BLE scanner app in your phone");
  Serial.println("2- Scan for BLE devices in the app");
  Serial.println("3- Connect to (reciever coil)");
  Serial.println("4- Go to CUSTOM CHARACTERISTIC in CUSTOM SERVICE and write something");
  Serial.println("5- See the magic =)");*/
  t1 = millis();
  BLEDevice::init("ESP32");
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Start");
  pService->start();

//  pAdvertising = pServer->getAdvertising();
//  pAdvertising->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  if( millis() - t1 > DELAYMS ){
     // set the brightness on LEDC channel 0
    ledcAnalogWrite(LEDC_CHANNEL_0, brightness);
  
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;
  
    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255) {
      fadeAmount = -fadeAmount;
    }
    // wait for 30 milliseconds to see the dimming effect
    //delay(1); 
    t1 = millis();
  }
  delayMicroseconds(50);
  sen1 +=  analogRead(34);
  sen2 +=  analogRead(35);
  i++;
  if( i == n ){
    sen1 /= n ;
    sen2 /= n ;
    v1 = sen1 * m1 + v0_1;
    v2 = sen2 * m2 + v0_2;
    //Serial.printf("vin=%5.3f , Iin = %5.3fmA\n",v1 , (v2 - v1)*1000 );
    /******/
    //if(deviceConnect){
    //char str[30];
    //sprintf(str,"vin=%5.3f , Iin = %5.3fmA\n",v1 , (v1 - v2)/Rsen*1000 );
    //pCharacteristic->setValue(str);
//    char buf[3];
//    buf[0] = (uint8_t)(( v1 - 3.0 ) * 255.0 / (17.0 - 3.0)) ;
//    float Iin = (v1 - v2)/Rsen*1000;
//    buf[1] = (uint8_t)(( Iin - 10.0 ) * 255.0 / (70.0 - 10.0)) ;
//    buf[2] = '\n';
char buf[13] = "Data_is:";
    buf[9] = (uint8_t)(( v1 - 3.0 ) * 255.0 / (17.0 - 3.0)) ;
    float Iin = (v1 - v2)/Rsen*1000;
    buf[10] = (uint8_t)(( Iin - 10.0 ) * 255.0 / (70.0 - 10.0)) ;
    buf[11] = '\n';
    buf[12] = 0;
    pCharacteristic->setValue(buf);
     pCharacteristic->notify();
     //delay(10);
    //}
    /*****/
    i=0;
    sen1 = sen2 = 0;
  }
}
