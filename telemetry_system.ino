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

void setup() {
  Serial.begin(9600);
  
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

void loop() {
  // Arka planda GPS karakter akışını dinle
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  if (millis() - sonGonderimZamani >= gonderimPeriyodu) {
    sonGonderimZamani = millis();
    paketSayaci++; 

    int uyduStatusu = 0; 
    int hataKodu = 0;    

    // --- DINAMIK SENSÖR KONTROLLERİ (Hatta Geri Dönme Mantığı) ---

    // 1. BME280 Kontrolü
    if (!bme_calisiyor) {
      bme.setI2CAddress(0x76);
      if (bme.beginI2C(Wire2) == true) { bme_calisiyor = true; } 
      else {
        bme.setI2CAddress(0x77);
        if (bme.beginI2C(Wire2) == true) { bme_calisiyor = true; }
      }
    }

    // 2. BNO055 Kontrolü
    if (!bno_calisiyor) {
      if (bno.begin() == true) {
        bno.setExtCrystalUse(true);
        bno_calisiyor = true;
      }
    }

    // 3. INA219 Kontrolü
    if (!ina_calisiyor) {
      if (ina219.begin(&Wire2) == true) {
        ina_calisiyor = true;
      }
    }

    // --- VERİ OKUMA ALANI ---
    float basinc = -999.00, yukseklik = -999.00, inisHizi = -999.00, sicaklik = -999.00;
    float pitch = -999.00, roll = -999.00, yaw = -999.00;
    float pilGerilimi = -999.00; 

    if (bme_calisiyor) {
      sicaklik = bme.readTempC();
      basinc = bme.readFloatPressure() / 100.0F; 
      // Eğer kablo o an koptuysa değer anlamsız gelir, sistemi korumaya alalım
      if (sicaklik == 0.00 && basinc == 0.00) {
        bme_calisiyor = false; // Hattan düştü olarak işaretle, bir sonraki saniye tekrar bağlanmayı deneyecek
        sicaklik = -999.00; basinc = -999.00;
      } else {
        yukseklik = 44330.0 * (1.0 - pow(basinc / 1013.25, 0.1903)); 
        inisHizi = 0.0;
      }
    }
    if (!bme_calisiyor) { hataKodu = 1; }

    if (bno_calisiyor) {
      sensors_event_t event;
      // BNO055 okuma hatası verirse koruma
      if (bno.getEvent(&event) == true) {
        yaw = event.orientation.x;   
        pitch = event.orientation.y; 
        roll = event.orientation.z;  
      } else {
        bno_calisiyor = false; // Kopma algılandı!
      }
    }
    if (!bno_calisiyor) { hataKodu = (hataKodu == 1) ? 3 : 2; }

    if (ina_calisiyor) {
      pilGerilimi = ina219.getBusVoltage_V();
      // INA219 kopma testi (genelde hat kopunca 0V veya absürt bir değer okur, ancak I2C kilitlenmesini engellemek için cihazı canlı tutuyoruz)
    }

    // GPS Verileri
    String gondermeSaati = "00:00:00"; 
    if (gps.time.isValid()) {
      char saatBuf[10];
      sprintf(saatBuf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
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

    // --- LORA VE SERİ PORT ÇIKIŞI ---
    LoRaSerial.print(veriPaketi);

    if (Serial) {
      Serial.print("Dinamik Paket: ");
      Serial.print(veriPaketi);
    }
  }
}