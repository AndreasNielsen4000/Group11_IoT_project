#include <Adafruit_SH110X.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
//128x64 display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define PAYLOAD_SIZE 13
#define LDR A1
#define BATTERYPIN A0
int LDRvalue = 0;
uint8_t brightness = 0;
int x = 0;
bool boolDisplayData = false;
bool boolDisplayError = false;
/* void receiveEvent(int bytes) {
  x = Wire.read();    // read one character from the I2C
} */

// Variables to control the display
unsigned long previousMillisSuccessMessage = 0;
unsigned long previousMillisErrorMessage = 0;
unsigned long previousMillisBattery = 0;
const long intervalSuccessMessage = 1000; // interval at which to wait (milliseconds)
const long intervalErrorMessage = 2000; // interval at which to wait (milliseconds)
const long intervalBattery = 1000; // interval at which to wait (milliseconds)

struct MCU_Communication {
    float longitude;
    float latitude;
    char time[10]; // Use a fixed-size char array instead of String for simplicity
};

MCU_Communication receivedData;


void displaySetup() {
  // If the OLED display is present, we can use it to show some information
  if (!display.begin(0x3C, true)) {  // Address 0x3C for 128x32
    Serial.println(F("Adafruit SH1106G allocation failed"));
  } else {
    Serial.print("Display setup");
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextWrap(false);
  display.setTextColor(SH110X_WHITE, 0);
  display.setCursor(0, 25);
  display.print("Booting");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("CHECKER");
  display.drawFastHLine(0, 9, 42, SH110X_WHITE);
  display.display();
}

void displayBattery(float batteryVoltage) {
  display.setCursor(93, 0);
  display.print("Batt:");
  display.setCursor(93, 12);
  float batteryPercentage = (batteryVoltage - 1.0) / (4.2 - 1.0) * 100;
  display.print(String(batteryPercentage, 0));
  display.print("%");
  display.display();
}

void displayCurrentLocation(float latitude, float longitude, char* time) {
  displayClearMessage();
  char latBuffer[10];
  char longBuffer[10];

  dtostrf(latitude, 7, 3, latBuffer); // 7 is the width, 3 is the precision
  dtostrf(longitude, 7, 3, longBuffer); // 7 is the width, 3 is the precision

  display.setCursor(0, 24);
  display.print("Lat: ");
  display.print(latBuffer);
  display.setCursor(0, 36);
  display.print("Long: ");
  display.print(longBuffer);
  display.setCursor(0, 48);
  display.print("Time: ");
  display.print(time);
  display.display();
}

void displayMessageSuccess() {
  displayClearMessage();
  display.setCursor(0, 24);
  display.print("Location was sent");
  display.display();
}

void displayMessageErrorWarning(String errorMessage) {
  displayClearMessage();
  display.setCursor(0, 24);
  display.print("Error: ");
  display.print(errorMessage);
  display.display();
}

void displayStandbyMode() {
  display.clearDisplay();
  display.setCursor(0, 24);
  display.print("Standby mode");
  display.display();
}

void displayClearMessage() {
  display.setCursor(0, 24);
  display.print("                   ");
  display.setCursor(0, 36);
  display.print("                   ");
  display.setCursor(0, 48);
  display.print("                   ");
  display.display();
}

void displayControl() {
  /*
  Main control function to display messages on the OLED screen
  */
  if (boolDisplayData) {
    displayMessageSuccess();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisSuccessMessage >= intervalSuccessMessage) {
      previousMillisSuccessMessage = currentMillis;
      displayCurrentLocation(receivedData.latitude, receivedData.longitude, receivedData.time);
      boolDisplayData = false;
    }
  }
  if (boolDisplayError) {
    displayMessageErrorWarning("Invalid data");
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisErrorMessage >= intervalErrorMessage) {
      previousMillisErrorMessage = currentMillis;
      displayClearMessage();
      boolDisplayError = false;
    }
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisBattery >= intervalBattery) {
    previousMillisBattery = currentMillis;
    displayBattery(getBatteryVoltage());
  }
}

void measureBrightness() {
  LDRvalue = analogRead(LDR);
    brightness = map(LDRvalue, 0, 1023, 0, 255);
    display.setContrast(brightness);
}

float getBatteryVoltage() {
  int sensorValue = analogRead(BATTERYPIN);
  int value = constrain(sensorValue, 0, 1023);
  float voltage = value * (4.2 / 1023.0);
  return voltage;
}


void setup() {
  Serial.begin(9600);
  displaySetup();
  Wire.begin(0); 
  Wire.onReceive(receiveEvent);
  pinMode(LDR, INPUT);
  pinMode(BATTERYPIN, INPUT);
}

void loop() {
    measureBrightness();    
    displayControl();
    delay(100);
}

void receiveEvent(int howMany) {
  Wire.readBytes((char*)&receivedData, sizeof(receivedData));
  Serial.println("Received data");
  if (!isnan(receivedData.latitude) && !isnan(receivedData.longitude)) {
    boolDisplayData = true;
    boolDisplayError = false;
  }
  else if (isnan(receivedData.latitude) || isnan(receivedData.longitude)) {
    boolDisplayError = true;
    boolDisplayData = false;
  }
}