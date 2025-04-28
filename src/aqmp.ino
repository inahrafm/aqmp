#include <TinyGPS++.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Struct untuk data PM Sensor
struct PMData {
  int raw;
  float voltage;
  float density;
};

// Konfigurasi HardwareSerial untuk GPS
#define RXD2 16
#define TXD2 17
HardwareSerial SerialGPS(1); // UART 1
TinyGPSPlus gps;

// Konfigurasi PM Sensor
const int measurePin = 35;  // GPIO35 (Analog ADC1_CH7)
const int ledPower = 12;    // GPIO12 untuk kontrol LED

// Konfigurasi SD Card
#define SD_CS 5

// Konfigurasi WiFi
const char* ssid = "ssid";
const char* password = "password";

// Konfigurasi MQTT
const char* mqtt_server = "mqtt";
const int mqtt_port = 1883;
const char* mqtt_user = "xxx";
const char* mqtt_pass = "xxx";
const char* mqtt_topic = "xxx";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMillis = 0;          // Waktu terakhir operasi dilakukan
const unsigned long interval = 10000;  // Interval waktu 10 detik (10000 ms)
bool dataSent = false;                 // Flag untuk menandai apakah data sudah dikirim dalam interval

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Inisialisasi PM Sensor
  pinMode(ledPower, OUTPUT);
  digitalWrite(ledPower, HIGH); // Matikan LED awal
  
  // Inisialisasi SD Card
  if(!SD.begin(SD_CS)) {
    Serial.println("Gagal menginisialisasi SD Card");
    while(1);
  }
  Serial.println("SD Card terdeteksi");

  // Buat file header jika file belum ada
  if(!SD.exists("/sensor_data.csv")) {
    File file = SD.open("/sensor_data.csv", FILE_WRITE);
    if(file) {
      file.println("Timestamp,Latitude,Longitude,Altitude,Satellites,Speed,PM_Raw,PM_Voltage,PM_Density");
      file.close();
      Serial.println("File header dibuat");
    }
  }
  
  // Koneksi WiFi
  setup_wifi();
  
  // Konfigurasi MQTT
  client.setServer(mqtt_server, mqtt_port);
  
  Serial.println("Menunggu GPS mendapatkan sinyal...");
  
  // Tunggu sampai GPS memberikan informasi lengkap tentang lokasi, tanggal & waktu
  while (!gps.location.isValid() || !gps.time.isValid() || !gps.date.isValid()) {
    while (SerialGPS.available() > 0) {
      gps.encode(SerialGPS.read());
    }
    delay(1000); // cek setiap detik sampai semua valid
  }

  Serial.println("GPS sudah valid, sistem siap beroperasi!");
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Menghubungkan ke WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi terhubung");
  Serial.println("Alamat IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Mencoba koneksi MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("Terhubung");
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

PMData readPMSensor() {
  // 1. Nyalakan LED, baca sensor
  digitalWrite(ledPower, LOW);  // Aktifkan LED (LOW aktif)
  delayMicroseconds(280);       // Sampling time
  int raw = analogRead(measurePin); // Baca nilai analog (0-4095)
  delayMicroseconds(40);        // Delta time
  digitalWrite(ledPower, HIGH); // Matikan LED
  delayMicroseconds(9680);      // Sleep time

  // 2. Konversi ke tegangan (dengan pembagi tegangan 10k+20k)
  float voltage = raw * (3.3 / 4095.0) * (30.0 / 10.0); // Faktor pembagi (10k+20k)

  // 3. Hitung debu (rumus standar, bisa dikalibrasi ulang)
  float dustDensity = (0.17 * voltage - 0.1) * 1000; // μg/m³
  if (dustDensity < 0) dustDensity = 0; // Hindari nilai negatif

  // 4. Return semua data dalam struct
  return PMData{raw, voltage, dustDensity};
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis >= interval && !dataSent) {
    lastMillis = currentMillis;
    dataSent = true;

    // Baca data GPS
    while (SerialGPS.available() > 0) {
      if (gps.encode(SerialGPS.read())) {
        if (gps.location.isValid() && gps.time.isValid() && gps.date.isValid()) {
          PMData pm = readPMSensor();

          String savedData = saveSensorData(pm);
          displaySensorInfo(pm);
          sendToMQTT(savedData);
          break; // Hentikan loop setelah satu set data diproses
        }
      }
    }
  }

  // Reset flag setelah interval
  if (currentMillis - lastMillis >= interval) {
    dataSent = false;
  }
}

String saveSensorData(PMData pm) {
  File file = SD.open("/sensor_data.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Gagal membuka file untuk ditulis");
    return "";
  }

  // Format data lengkap
  String dataStr = "";
  
  // Timestamp
  if(gps.date.isValid() && gps.time.isValid()) {
    char timestamp[25];
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d", 
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    dataStr += String(timestamp) + ",";
  } else {
    dataStr += "NaN,";
  }
  
  // Lokasi
  dataStr += String(gps.location.lat(), 6) + ",";
  dataStr += String(gps.location.lng(), 6) + ",";
  
  // Altitude
  if(gps.altitude.isValid()) {
    dataStr += String(gps.altitude.meters()) + ",";
  } else {
    dataStr += "NaN,";
  }
  
  // Jumlah satelit
  if(gps.satellites.isValid()) {
    dataStr += String(gps.satellites.value()) + ",";
  } else {
    dataStr += "NaN,";
  }
  
  // Kecepatan
  if(gps.speed.isValid()) {
    dataStr += String(gps.speed.kmph()) + ",";
  } else {
    dataStr += "NaN,";
  }
  
  // Data PM Sensor
  dataStr += String(pm.raw) + ",";
  dataStr += String(pm.voltage, 3) + ",";
  dataStr += String(pm.density, 2);
  
  file.println(dataStr);
  file.close();
  Serial.println("Data disimpan ke SD Card");
  
  return dataStr;
}

void sendToMQTT(String csvData) {
  // Parsing CSV data
  String parts[9]; // Timestamp,Lat,Lon,Alt,Sat,Speed,Raw,Voltage,Density
  int lastIndex = 0;
  for(int i = 0; i < 8; i++) {
    int nextIndex = csvData.indexOf(',', lastIndex);
    parts[i] = csvData.substring(lastIndex, nextIndex);
    lastIndex = nextIndex + 1;
  }
  parts[8] = csvData.substring(lastIndex);

  // Buat payload JSON
  DynamicJsonDocument doc(512);
  doc["timestamp"] = parts[0];
  doc["latitude"] = parts[1].toFloat();
  doc["longitude"] = parts[2].toFloat();
  doc["altitude"] = parts[3].toFloat();
  doc["satellites"] = parts[4].toInt();
  doc["speed"] = parts[5].toFloat();
  doc["pm_raw"] = parts[6].toInt();
  doc["pm_voltage"] = parts[7].toFloat();
  doc["pm_density"] = parts[8].toFloat();

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  // Kirim via MQTT
  if (client.publish(mqtt_topic, jsonBuffer)) {
    Serial.println("Data terkirim via MQTT");
  } else {
    Serial.println("Gagal mengirim via MQTT");
  }
}

void displaySensorInfo(PMData pm) {
  Serial.println("\n--- Data Sensor Terbaru ---");
  
  // Tanggal dan waktu
  if(gps.date.isValid() && gps.time.isValid()) {
    Serial.printf("Tanggal/Waktu: %04d-%02d-%02d %02d:%02d:%02d\n", 
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    Serial.println("Waktu tidak valid");
  }
  
  // Lokasi
  Serial.print("Lokasi: ");
  Serial.print(gps.location.lat(), 6);
  Serial.print(", ");
  Serial.println(gps.location.lng(), 6);
  
  // Altitude
  if(gps.altitude.isValid()) {
    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");
  }
  
  // Satelit
  if(gps.satellites.isValid()) {
    Serial.print("Satelit: ");
    Serial.println(gps.satellites.value());
  }
  
  // Kecepatan
  if(gps.speed.isValid()) {
    Serial.print("Kecepatan: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");
  }
  
  // PM Sensor
  Serial.print("PM Raw: ");
  Serial.print(pm.raw);
  Serial.print(" | Voltage: ");
  Serial.print(pm.voltage, 3);
  Serial.print(" V | Density: ");
  Serial.print(pm.density, 2);
  Serial.println(" μg/m³");
  
  Serial.println("----------------------");
}
