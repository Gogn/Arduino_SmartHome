#include <SoftwareSerial.h>;
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


////////////////////Setup//////////////////////
#define desiredHumidity 25
#define humidityGap 5 //humidity can change within this value (protect relay/humidifier from to much switching)


////////////////////SetUp//////////////////////




////////////////////MH-Z19B//////////////////////
SoftwareSerial SoftwareSerial(10, 11);
byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; 
unsigned char response[9];
////////////////////MH-Z19B//////////////////////

////////////////////BME-280/////////////////////
#define SEALEVELPRESSURE_HPA (1013.25) //760 mm hg (normal pressure on the water)
#define BME280RelayPin 4

Adafruit_BME280 bme;
////////////////////BME-280/////////////////////


////////////////////MQ-7///////////////////////
#define MQ7Pin 2    //the AOUT pin of the CO sensor goes into analog pin A0 of the arduino
#define MQ7RelayPin 3

unsigned long timingMQ7; //timing for millis()
unsigned long timingHeatingMQ7;
int flagMQ7Relay;
int MQ7CO;
int MQ7COLim;
////////////////////MQ-7///////////////////////

void setup() {
    Serial.begin(9600);
    SoftwareSerial.begin(9600);
    pinMode(MQ7RelayPin, OUTPUT); //relay MQ-7
    pinMode(BME280RelayPin, OUTPUT); //relay BME-280

    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
}

void loop() 
{
////////////////////MH-Z19B//////////////////я////
    SoftwareSerial.write(cmd, 9);
    memset(response, 0, 9);
    SoftwareSerial.readBytes(response, 9);
    int i;
    byte crc = 0;
    for (i = 1; i < 8; i++) crc+=response[i];
    crc = 255 - crc;
    crc++;

    if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
        Serial.println("MH-Z19 CRC error: " + String(crc) + " / "+ String(response[8]));
    } else {
        unsigned int responseHigh = (unsigned int) response[2];
        unsigned int responseLow = (unsigned int) response[3];
        unsigned int ppm = (256*responseHigh) + responseLow;
        Serial.println("ppm: " + String(ppm));
    }
////////////////////MH-Z19B//////////////////////

////////////////////BME-280/////////////////////
    int humidify = bme.readHumidity();
    Serial.println("Temperature = " + String(bme.readTemperature()) + "*C");
    Serial.println("Humidity = " + String(humidify) + "%");
    //Serial.print("Pressure = " + String(bme.readPressure() / 100.0F) + "hPa" + "\n");
    //Serial.print("Approx. Altitude = " + String(SEALEVELPRESSURE_HPA) + "m" + "\n");
////////////////////BME-280/////////////////////

////////////////////MQ-7///////////////////////
    if (millis() - timingMQ7 > 3600000) { //3600000 millis = 60mins

        if (millis() - timingHeatingMQ7 > 60000 && flagMQ7Relay) { //read CO after MQ-7 heated (60sec)
            //Serial.println(71);
            //Serial.println(millis());
            //Serial.println(timingHeatingMQ7);


            MQ7CO= analogRead(MQ7Pin);    //reads the analaog value from the CO sensor’s AOUT pin
            Serial.println("CO value: " + String(MQ7CO));
            digitalWrite(MQ7RelayPin, 0);
            timingHeatingMQ7 = 0;
            flagMQ7Relay = 0;
        } else {
            //Serial.println(77);
            //Serial.println(millis());
            //Serial.println(timingHeatingMQ7);

            if (!flagMQ7Relay) {
                digitalWrite(MQ7RelayPin, 1);
                flagMQ7Relay = 1;
                timingHeatingMQ7 = millis();
            }

        }

        timingMQ7 = 0;
    }
////////////////////MQ-7///////////////////////


////////////////////Relay///////////////////////
    if ( humidify < (desiredHumidity - humidityGap) ) {
        digitalWrite(BME280RelayPin, 1);
        Serial.println("Humidifier On");
    }
    if ( humidify > (desiredHumidity + humidityGap) ) {
        digitalWrite(BME280RelayPin, 0);
        Serial.println("Humidifier Off");
    }
////////////////////Relay///////////////////////

    delay(10000);
}