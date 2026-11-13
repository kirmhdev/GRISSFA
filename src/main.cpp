#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <DHT.h>

const float RL_VALUE = 10.0;

const float RO_CLEAN_AIR_FACTOR = 4.4;

float R0 = 10.0;

float calibration_value = 21.34 - 5.0 + 0.15;
unsigned long int avgval;
int buffer_arr[10], temp;

#define bleServerName "ESP32_Sensor"

#define STATUS_LED_PIN 2
#define RELAY_PIN 25
#define PH_SENSOR_PIN 26
#define MQ4_PIN 27
#define PRESSURE_SENSOR_PIN 32

#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define PRESSURE_CHAR_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define PH_CHAR_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"
#define METHANE_CHAR_UUID "19b10003-e8f2-537e-4f6c-d104768a1214"
#define HUMIDITY_CHAR_UUID "19b10004-e8f2-537e-4f6c-d104768a1214"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicPressure = NULL;
BLECharacteristic *pCharacteristicPh = NULL;
BLECharacteristic *pCharacteristicMethane = NULL;
BLECharacteristic *pCharacteristicHumidity = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;

int pressureValue = 0;
float phValue = 0;
int methaneValue = 0;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Perangkat Terhubung!");
    digitalWrite(STATUS_LED_PIN, HIGH);
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Perangkat Terputus!");
    digitalWrite(STATUS_LED_PIN, LOW);
  }
};

float ph_act()
{
  for (int i = 0; i < 10; i++)
  {
    buffer_arr[i] = analogRead(PH_SENSOR_PIN);
    delay(30);
  }

  for (int i = 0; i < 9; i++)
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buffer_arr[i] > buffer_arr[j])
      {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }

  avgval = 0;
  for (int i = 2; i < 8; i++)
    avgval += buffer_arr[i];

  // Konversi ke voltase ESP32
  float volt = (float)avgval * 3.3 / 4095.0 / 6;
  return (-5.70 * volt + calibration_value);
}

// MQ-4

float readMQ4Voltage(int pin)
{
  int ADCValue = analogRead(pin);
  float ADCVoltage = (ADCValue / 4095.0) * 3.3;

  float voltage = ADCVoltage * 1.5;

  return voltage;
}

float countRs(float voltage)
{
  if (voltage <= 0.0)
    return 99999.0;
  return ((5.0 - voltage) / voltage) * RL_VALUE;
}

// Fungsi untuk kalibrasi (mencari nilai R0)
float calibrateMQ4()
{
  float RsTotal = 0;
  int sampleCount = 75;

  for (int i = 0; i < sampleCount; i++)
  {
    float voltage = readMQ4Voltage(MQ4_PIN);
    RsTotal += countRs(voltage);
    delay(100);
  }

  float RsEverage = RsTotal / sampleCount;

  return RsEverage / RO_CLEAN_AIR_FACTOR;
}

float readMethanePPM()
{
  float voltage = readMQ4Voltage(MQ4_PIN);
  float Rs = countRs(voltage);
  float ratio = Rs / R0;

  float logPPM = (-2.73 * log10(ratio)) + 3.0;
  float ppm = pow(10, logPPM);

  return ppm;
}

void setup()
{
  Serial.begin(9600);

  Serial.println("Kalibrasi sensor MQ-4...");
  R0 = calibrateMQ4();
  Serial.print("Nilai R0 setelah kalibrasi: ");
  Serial.println(R0);

  Serial.println("Memulai BLE Server...");

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(PH_SENSOR_PIN, INPUT);
  pinMode(MQ4_PIN, INPUT);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(STATUS_LED_PIN, LOW);

  BLEDevice::init(bleServerName);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristicPressure = pService->createCharacteristic(
      PRESSURE_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicPh = pService->createCharacteristic(
      PH_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicMethane = pService->createCharacteristic(
      METHANE_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristicPressure->addDescriptor(new BLE2902());
  pCharacteristicPh->addDescriptor(new BLE2902());
  pCharacteristicMethane->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Menunggu koneksi dari aplikasi Flutter...");
}

void loop()
{
  pressureValue = (analogReadMilliVolts(PRESSURE_SENSOR_PIN) - 290) / 55;
  phValue = ph_act();
  methaneValue = readMethanePPM();

  String pressureStr = String(pressureValue);
  String phStr = String(phValue, 1);
  String methaneStr = String(methaneValue);

  Serial.println("Data dikirim -> P: " + pressureStr + " | pH: " + phStr + " | CH4: " + methaneStr + " | MQ4 volts : " + String(analogReadMilliVolts(MQ4_PIN)));

  if (pressureValue < 0)
  {
    pressureValue = 0;
  }

  if (pressureValue <= 3)
  {
    if (methaneValue >= 30)
    {
      digitalWrite(RELAY_PIN, LOW);
    }
    else
    {
      digitalWrite(RELAY_PIN, HIGH);
    }
  }
  else
  {
    digitalWrite(RELAY_PIN, HIGH);
  }

  if (deviceConnected)
  {
    pCharacteristicPressure->setValue(pressureStr.c_str());
    pCharacteristicPh->setValue(phStr.c_str());
    pCharacteristicMethane->setValue(methaneStr.c_str());
    pCharacteristicPressure->notify();
    pCharacteristicPh->notify();
    pCharacteristicMethane->notify();
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    pServer->startAdvertising();
    Serial.println("Mulai memancarkan sinyal Bluetooth lagi...");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }
  delay(100);
}