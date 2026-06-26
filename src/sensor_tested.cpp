#include "sensor_tested.h"

//PID Feed Data Extraction:
SensorData current_data = {-999.0f, -999.0f}; 


float ground_pressure_hpa = 1013.25f;


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SparkFunBME280.h> 
#include <Adafruit_BNO055.h>
#include <Adafruit_INA219.h>
#include <TinyGPS++.h>

// --- PORT VE PIN TANIMLAMALARI ---
#define LoRaSerial Serial1  
#define GPSSerial  Serial4  
#define I2C2_SDA 25         
#define I2C2_SCL 24         

BME280 bme; 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2); 
Adafruit_INA219 ina219;
TinyGPSPlus gps;

// --- TELEMETRİ SABİTLERİ ---
const int takimNo = 123456;      
unsigned long paketSayaci = 0;   

// Başlangıçta hepsi false, loop içinde sürekli denenecekler
bool bme_calisiyor = false;
bool bno_calisiyor = false;
bool ina_calisiyor = false;

unsigned long sonGonderimZamani = 0;
const unsigned long gonderimPeriyodu = 1000; 

void sensor_setup() {
  Serial.begin(115200);
  
  unsigned long start_time = millis();
  while (!Serial && (millis() - start_time < 3000)) { delay(1); }

  Serial1.setTX(1);
  Serial1.setRX(0);
  LoRaSerial.begin(9600);
  delay(100);

  Serial4.setTX(16);
  Serial4.setRX(17);
  GPSSerial.begin(9600); 
  delay(100);

  Wire2.setSDA(I2C2_SDA);
  Wire2.setSCL(I2C2_SCL);
  Wire2.begin();
  delay(200);

  Serial.println("--- DINAMIK OTO-BAGLANTI SISTEMI AKTIF ---");
}

void sensor_loop() {
  // 1. Background GPS Read (Runs as fast as possible)
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  // Shared variables between the 50Hz sensor read and 1Hz LoRa transmission
  static float basinc = -999.00, yukseklik = -999.00, inisHizi = -999.00, sicaklik = -999.00;
  static float pitch = -999.00, roll = -999.00, yaw = -999.00;
  static float pilGerilimi = -999.00; 
  static int hataKodu = 0;

  // 2. SENSOR READING BLOCK (50Hz / 20ms) - CRITICAL FIX #1
  static unsigned long sonSensorOkuma = 0;
  if (millis() - sonSensorOkuma >= 20) {
    sonSensorOkuma = millis();
    hataKodu = 0;

    // --- DYNAMIC SENSOR RECONNECTS ---
    static bool ground_pressure_captured = false;

    if (!bme_calisiyor) {
      bme.setI2CAddress(0x76);
      if (bme.beginI2C(Wire2) == true) { 
          bme_calisiyor = true; 
          if (!ground_pressure_captured) {
              ground_pressure_hpa = bme.readFloatPressure() / 100.0F;
              ground_pressure_captured = true;
          }
      } 
      else {
        bme.setI2CAddress(0x77);
        if (bme.beginI2C(Wire2) == true) { 
            bme_calisiyor = true; 
            if (!ground_pressure_captured) {
                ground_pressure_hpa = bme.readFloatPressure() / 100.0F;
                ground_pressure_captured = true;
            }
        }
      }
    }

    if (!bno_calisiyor) {
      if (bno.begin() == true) { bno.setExtCrystalUse(true); bno_calisiyor = true; }
    }

    if (!ina_calisiyor) {
      if (ina219.begin(&Wire2) == true) { ina_calisiyor = true; }
    }

    // --- VERİ OKUMA ALANI ---
    // --- VERİ OKUMA ALANI ---
    if (bme_calisiyor) {
      sicaklik = bme.readTempC();
      basinc = bme.readFloatPressure() / 100.0F; 
      
      if (sicaklik == 0.00 && basinc == 0.00) {
        bme_calisiyor = false; 
        sicaklik = -999.00; basinc = -999.00;
        
        // CRITICAL FIX: Propagate the sentinel value so main.cpp knows the sensor died
        yukseklik = -999.00; 
        inisHizi = -999.00;
      } else {
        // FIXED: Dividing by the actual pressure float, not the boolean flag
        yukseklik = 44330.0 * (1.0 - pow(basinc / ground_pressure_hpa, 0.1903)); 
        
        static float prev_altitude = yukseklik;
        static unsigned long prev_baro_time = millis();
        unsigned long now = millis();
        float baro_dt = (now - prev_baro_time) / 1000.0f;
        
        // FIXED: Lowered threshold to 0.015f for 50Hz velocity updates
        if (baro_dt > 0.015f) {
            inisHizi = (yukseklik - prev_altitude) / baro_dt; 
            prev_altitude = yukseklik;
            prev_baro_time = now;
        }
      }
    }
    if (!bme_calisiyor) { hataKodu = 1; }

    if (bno_calisiyor) {
      sensors_event_t event;
      if (bno.getEvent(&event) == true) {
        yaw = event.orientation.x;   
        pitch = event.orientation.y; 
        roll = event.orientation.z;  
      } else {
        bno_calisiyor = false; 
      }
    }
    if (!bno_calisiyor) { hataKodu = (hataKodu == 1) ? 3 : 2; }

    if (ina_calisiyor) {
      pilGerilimi = ina219.getBusVoltage_V();
    }

    // PID Feed Data Extraction (Updated 50 times a second)
    current_data.altitude = yukseklik;
    current_data.velocity = inisHizi;
  }

  // 3. TELEMETRY TRANSMISSION BLOCK (1Hz / 1000ms)
  if (millis() - sonGonderimZamani >= gonderimPeriyodu) {
    sonGonderimZamani = millis();
    paketSayaci++; 

    int uyduStatusu = gps.satellites.isValid() ? gps.satellites.value() : 0; 

    String gondermeSaati = "00:00:00"; 
    if (gps.time.isValid()) {
      char saatBuf[16];
      snprintf(saatBuf, sizeof(saatBuf), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      gondermeSaati = String(saatBuf);
    }
    double gpsLat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double gpsLong = gps.location.isValid() ? gps.location.lng() : 0.0;
    double gpsAlt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;

    // --- CSV FORMATLI PAKETLEME ---
    String veriPaketi = String(paketSayaci) + "," +
                        String(uyduStatusu) + "," +
                        String(hataKodu) + "," +
                        gondermeSaati + "," +
                        String(basinc, 2) + "," +
                        String(yukseklik, 2) + "," +
                        String(inisHizi, 2) + "," +
                        String(sicaklik, 2) + "," +
                        String(pilGerilimi, 2) + "," +
                        String(gpsLat, 6) + "," +
                        String(gpsLong, 6) + "," +
                        String(gpsAlt, 2) + "," +
                        String(pitch, 2) + "," +
                        String(roll, 2) + "," +
                        String(yaw, 2) + "," +
                        "RHRHRH*," +              
                        String(takimNo) + "\n";   

    LoRaSerial.print(veriPaketi);
    if (Serial) {
      Serial.print("Dinamik Paket: ");
      Serial.print(veriPaketi);
    }
  }
}


SensorData get_latest_sensor_data() {
    return current_data;
}