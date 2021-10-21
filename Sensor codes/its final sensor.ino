
//tem , ec , tds
#include <OneWire.h>
#include <DallasTemperature.h>


namespace pin {
  const byte tds_sensor = A0;
  const byte one_wire_bus = 2; // Dallas Temperature Sensor
}

namespace device {
  float aref = 4.3;
}

namespace sensor {
  float ec = 0;
  unsigned int tds = 0;
  float waterTemp = 0;
  float ecCalibration = 1;
}

OneWire oneWire(pin::one_wire_bus);
DallasTemperature dallasTemperature(&oneWire);
//esp
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
SoftwareSerial s(10, 11);

//turbidity
#define turbidAPin    A1    //for analog signal from turbidity sensor
float aRead = 0.0;
float ntu =0.0;
int sensorValue = 0;
//ph
const int analogInPin = A3;
unsigned long int avgValue;
float b;
int buf[10], temp;
//DO
#include <Arduino.h>

#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

//#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t DO;
float realDO;
int16_t readDO(uint16_t raw, uint8_t temperature)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature - (uint32_t)CAL1_T * 35;
  return (uint64_t(VREF) * DO_Table[temperature] * raw) / (uint32_t(ADC_RES) * V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T)+CAL2_V;
  return (uint64_t(VREF) * DO_Table[temperature] * raw) / (uint32_t(ADC_RES) * V_saturation);
#endif
}

void setup(void) {
  Serial.begin(115200);
  s.begin(115200);
  pinMode (10,INPUT);
  pinMode (11,OUTPUT);
//  myserial.begin(115200);
  dallasTemperature.begin();
  //Serial.print(char(169)); // Copyright Symbol
  //Serial.println(" Myengineeringstuffs.com");
  //pinMode(turbidAPin, INPUT);
  delay(2000);
}

StaticJsonBuffer<1000> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();

void readTdsQuick() {
  dallasTemperature.requestTemperatures();
  sensor::waterTemp = dallasTemperature.getTempCByIndex(0);
  float rawEc = analogRead(pin::tds_sensor) * device::aref / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float temperatureCoefficient = 1.0 + 0.02 * (sensor::waterTemp - 25.0); // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  sensor::ec = (rawEc / temperatureCoefficient) * sensor::ecCalibration; // temperature and calibration compensation
  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5; //convert voltage value to tds value
  Serial.print(F("TDS:")); Serial.println(sensor::tds);
  Serial.print(F("EC:")); Serial.println(sensor::ec, 2);
  Serial.print(F("Temperature:")); Serial.println(sensor::waterTemp, 2);
  //DO
  Temperaturet = (uint8_t)(sensor::waterTemp);
  ADC_Raw = analogRead(A2);

  Serial.print("Temperaturet:\t"+String(Temperaturet)+"\t");
  Serial.print("ADC RAW:\t"+String(ADC_Raw)+"\t");
  Serial.print("ADC Voltage:\t"+String((uint32_t)VREF*ADC_Raw/ADC_RES)+"\t");
  Serial.println("DO:\t"+String(readDO(ADC_Raw, Temperaturet))+"\t");
  realDO= (float)((readDO(ADC_Raw, Temperaturet))/400);
  Serial.println("realDO:\t"+String(realDO)+"\t");

  delay(1000);
  root["t"] = sensor::waterTemp;
  root["T"] = sensor::tds;
  root["e"] = sensor::ec; 
  root["D"] = realDO; 
//  s.print(root["t"]);
//  s.println("\n");
//  s.print(root["T"]);
//  s.println("\n");
//  s.print(["e"]);
//  s.println("\n");
//  s.print(root["D"]);
//  s.println("\n");
  delay(300);

}


void loop(void)
{

  //ph
  for (int i = 0; i < 10 ; i++)
  {
    buf[i] = analogRead(A3);
    delay(10);
  }
  for (int i = 0; i < 9 ; i++)
  {
    for (int j = i + 1 ; j < 10 ; j++)
    {
      if (buf[i] > buf[j])
      {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 0; i < 8; i++)
  {
    avgValue += buf[i];
  }
  float phValue = (float)avgValue * 5.0 * 1.689 / 1024 / 6;
  //  phValue=phValue;
  Serial.print("PH ");
  Serial.println(phValue);
  root["p"] = phValue;
//  s.print(root["p"]);
//  s.println("\n");

  //turbidity
  sensorValue = analogRead(turbidAPin);
  aRead = sensorValue * (1.14*5.0 / 1024.0);
  //float turbidityV = aRead/100;
  Serial.print("Turbidity level: ");
  Serial.print(aRead);
  Serial.println("V");
  if(aRead < 2.5){
      ntu = 3000;
    }else{
      ntu = -1120.4*square(aRead)+5742.3*aRead-4353.8; 
    }
    Serial.print("Turbidity(NTU): ");
    Serial.print(ntu);
    Serial.println("NTU");
    root["d"] = aRead;
    root["n"] =ntu;
//    s.print(root["d"]);
//    s.println("\n");
//    s.print(root["n"]);
//    s.println("\n");

  //temp , ec & tds
  readTdsQuick();
//  if (isnan(aread) || isnan(ntu) || isnan(phValue) || isnan(sensor::waterTemp) || isnan(sensor::tds) || isnan(sensor::ec) || isnan(realDO) ) {
    //return;
 // }
  delay(5000);
  if (s.available() > 0 )
  {
    root.printTo(s);
  }
delay(5000);
}