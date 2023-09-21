#include <Wire.h>
#include <TinyGPSPlus.h>

#define GPS_TX_PIN 12
#define GPS_RX_PIN 34
#define DAC_PIN 25 //Sound output pin for TX

#define SAMPLE_RATE 9600

#define DISTANCE_THRESHOLD 1000 // 1km
#define TIME_THRESHOLD 1800000 // 30 λεπτά σε ms

TinyGPSPlus gps;

float prevLat = 0.0;
float prevLon = 0.0;
unsigned long prevTime = 0;

// Ορισμός των συχνοτήτων για το AFSK1200
const int MARK_FREQ = 1200;
const int SPACE_FREQ = 2200;

// Ορισμός των χαρακτήρων για το AFSK1200
const char MARK_CHAR = '1';
const char SPACE_CHAR = '0';

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);

  pinMode(DAC_PIN, OUTPUT);

  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated()) {
    float distance = TinyGPSPlus::distanceBetween(prevLat, prevLon, gps.location.lat(), gps.location.lng());
    unsigned long elapsedTime = millis() - prevTime;

    if (distance >= DISTANCE_THRESHOLD || elapsedTime >= TIME_THRESHOLD) {
      prevLat = gps.location.lat();
      prevLon = gps.location.lng();
      prevTime = millis();

      sendAPRS(gps.location.lat(), gps.location.lng());
    }
  }
}

void sendAPRS(float latitude, float longitude) {
  String latStr = convertToAPRSLatitude(latitude);
  String lonStr = convertToAPRSLongitude(longitude);

  String aprsMessage = "APRSMESSAGE";
  aprsMessage += ">APRS:";
  aprsMessage += latStr;
  aprsMessage += "/";
  aprsMessage += lonStr;
  aprsMessage += "<";

  //Trigger Vox
  generateTone(MARK_FREQ,2000);

  // Μετατροπή του μηνύματος APRS σε ήχο AFSK1200
  for (int i = 0; i < aprsMessage.length(); i++) {
    char c = aprsMessage.charAt(i);
    if (c == MARK_CHAR) {
      generateTone(MARK_FREQ,100);
    } else if (c == SPACE_CHAR) {
      generateTone(SPACE_FREQ,100);
    }
  }
}

void generateTone(int frequency, int duration) {
  int period = 1000000 / frequency; // Calculate the period of the tone in microseconds
  int halfPeriod = period / 2;       // Calculate half of the period
  
  for (long i = 0; i < duration * 1000; i += period) {
    digitalWrite(DAC_PIN, HIGH);    // Set the DAC pin to HIGH
    delayMicroseconds(halfPeriod);   // Wait for half of the period
    digitalWrite(DAC_PIN, LOW);     // Set the DAC pin to LOW
    delayMicroseconds(halfPeriod);   // Wait for the remaining half of the period
  }
  // Delay after tone
  delay(100);
}

String convertToAPRSLatitude(float latitude) {
  String latStr = String(abs(latitude), 6);

  if (latitude >= 0) {
    latStr += "N";
  } else {
    latStr += "S";
  }

  return latStr;
}

String convertToAPRSLongitude(float longitude) {
  String lonStr = String(abs(longitude), 6);

  if (longitude >= 0) {
    lonStr += "E";
  } else {
    lonStr += "W";
  }

  return lonStr;
}