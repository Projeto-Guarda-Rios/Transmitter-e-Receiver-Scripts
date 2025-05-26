#include <ArduinoJson.h>  // Library for converting the values to JSON
#include <ArduinoJson.hpp>  // Library for converting the values to JSON
#include <ezButton.h>  // Library for the Limit Switch
#include "EBYTE.h"  // Library for the EBYTE E32
#include <ADS1X15.h>  // Library for the ADS
#include <Wire.h>  // Library for the ADS
#include <OneWire.h>  // Library for the DS18B20
#include <DallasTemperature.h>  // Library for the DS18B20

// ====== EBYTE E32 ====== \\

#define PIN_RX 16   // Serial2 RX (connect this to the EBYTE Tx pin)
#define PIN_TX 17   // Serial2 TX pin (connect this to the EBYTE Rx pin)
#define PIN_M0 5    // D4 on the board (possibly pin 24)
#define PIN_M1 18   // D2 on the board (possibly called pin 22)
#define PIN_AX 19   // D15 on the board (possibly called pin 21)

// ====== EBYTE ADC ====== \\

#define MY_SDA_PIN 21  // D19 on the board
#define MY_SCL_PIN 22  // D18 on the board

// ====== DS18B20 ====== \\

#define ONE_WIRE_BUS 4

// ====== Status LED ====== \\

#define PIN_STATUS_LED 15

// ====== Delay for the Sensors ====== \\

int SENSORES_DELAY = 0;

// ====== Structure de DATA ====== \\

struct DATA {
  unsigned long Count;
  char Message[20];
  int16_t ADC_Turbidez;
  int16_t ADC_TDS;
  int16_t ADC_PH;
  int16_t Temp;
};

DATA MyData;

// ====== Config for the EBYTE E32 ====== \\

EBYTE Transceiver(&Serial2, PIN_M0, PIN_M1, PIN_AX);

// ====== Config for the ADS ====== \\

ADS1115 ADS(0x48);

// ====== Config for the Temperature Sensor ====== \\

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// ====== Config for the Limit Switch ====== \\

ezButton limitSwitch(2);

void setup() {
  Serial.begin(9600);  // ESP32's Serial
  Serial2.begin(9600);  // EBYTE E32's Serial

  Serial.println("Starting Reader");

  // ====== Init the EBYTE E32 ====== \\

  Serial.println(Transceiver.init());
  Serial.println("Parameters: ");
  Transceiver.PrintParameters();
  Transceiver.SetChannel(0);
  Serial.println("GetChannel: ");
  Serial.print(Transceiver.GetAddress());

  tempSensor.begin();  // Init the Temperature Sensor

  // ====== Init the ADS ====== \\

  Wire.begin(MY_SDA_PIN, MY_SCL_PIN);

  if (!ADS.begin()) {  
    Serial.println("Failed to detect ADS1115 chip!");
    while (1);
  }

  limitSwitch.setDebounceTime(50); // Debounce Time for the Limit Switch

  pinMode(PIN_STATUS_LED, OUTPUT);  // Defines the Pin as Output
}

void loop() {
  limitSwitch.loop(); // Updates the state of the Limit Switch

  // ====== If the Limit Switch is released ====== \\

  if (limitSwitch.isReleased()) {
    Serial.println("Limit Switch Released!");
  }

  if (SENSORES_DELAY == 1000) {
    String userInput = Serial.readStringUntil('\n');
    userInput.trim();

    tempSensor.requestTemperatures();  // Get Temps from the Temperature Sensor

    StaticJsonDocument<200> data;
    data["temperatura"] = MyData.Temp;
    data["ph"] = MyData.ADC_PH;
    data["turbidez"] = MyData.ADC_Turbidez;
    data["tds"] = MyData.ADC_TDS;

    MyData.Count++;
    snprintf(MyData.Message, sizeof(MyData.Message), "RibaLab %lu", MyData.Count);
    Transceiver.SendStruct(&MyData, sizeof(MyData));
    Serial.print("Sending: "); Serial.println(MyData.Count);
    Serial.print("Turbidez: "); Serial.println(MyData.ADC_Turbidez);  
    Serial.print("TDS: "); Serial.println(MyData.ADC_TDS);
    Serial.print("PH: "); Serial.println(MyData.ADC_PH);
    Serial.print("Temperatura: "); Serial.println(MyData.Temp);
    Serial.print("Data: "); serializeJson(data, Serial); Serial.println();

    MyData.ADC_Turbidez = ADS.readADC(0);
    MyData.ADC_TDS = ADS.readADC(1);
    MyData.ADC_PH = ADS.readADC(2);
    MyData.Temp = tempSensor.getTempCByIndex(0);

    SENSORES_DELAY = 0;
  }

  digitalWrite(PIN_STATUS_LED, HIGH);
  SENSORES_DELAY = SENSORES_DELAY + 10;
  delay(10);
}