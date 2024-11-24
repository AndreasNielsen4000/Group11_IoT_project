#include <U8x8lib.h>
#include <rn2xx3_modified.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//128x64 display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

#define PAYLOAD_SIZE 13
#define LDR A1
#define BATTERYPIN A0
#define BTN_PIN 7
#define LEDPIN 13
#define BUZZERPIN 8
#define LORA_RX 10
#define LORA_TX 11
#define LORA_RESET 9
#define BUZZER_OFF 0
#define BUZZER_SUCCESS 2
#define BUZZER_ERROR 3
#define BUZZER_UPDATE 4

#define DEBUG

bool boolDisplayData = false;
bool boolDisplayError = false;
bool loraConnected = false;
unsigned long previousMillisLora = 0;
uint8_t buzzerStates = BUZZER_OFF;
//const long intervalLora = 60000; // interval at which to retry connection (milliseconds)

struct MCU_Communication {
    int32_t longitude;
    int32_t latitude;
    char date[8]; // YYYYMMDD
    char time[6]; // HHMMSS
};

SoftwareSerial loraSerial(LORA_RX, LORA_TX); // RX, TX
rn2xx3 myLora(loraSerial);

TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3); // RX, TX for GPS module

MCU_Communication receivedData;

unsigned long previousMillisSuccessMessage = 0;
unsigned long previousMillisErrorMessage = 0;
unsigned long previousMillisBattery = 0;
unsigned long previousMillisTransmittedMessage = 0;

int currentBtnState;
int previousBtnState;
unsigned int lastDebounceTime;
unsigned int debounceDelay = 50;
bool notHold = true;
volatile bool flag = false;
bool validLocation = false;


// Commands from https://ozzmaker.com/wp-content/uploads/2019/09/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221_Public.pdf
// Byte array for sleep command for GPS module
// byte GPSoff[] = {0xB5, 0x62, 0x06, 0x57, 0x08, 0x00, 0x01, 0x00, 0x00, 0x00, 0x50, 0x4F, 0x54, 0x53, 0xAC, 0x85};

byte CFG_PM2[] = {
    0xB5, 0x62,       // Sync
    0x06, 0x3B,       // Class and ID
    0x2C, 0x00,       // Length (44 bytes)
    0x01, 0x00, 0x00, 0x00, // Version and reserved
    0x04, 0x00, 0x00, 0x00, // Flags (RXD as wake-up source)
    0x00, 0x00, 0x00, 0x00, // Update period
    0x00, 0x00, 0x00, 0x00, // Search period
    0x00, 0x00, 0x00, 0x00, // Grid offset
    0x0A, 0x00,             // On-time
    0x05, 0x00,             // Min acquisition time
    0x00, 0x00,             // Reserved4
    0x00, 0x00,             // Reserved5
    0x00, 0x00, 0x00, 0x00, // Reserved6
    0x00, 0x00, 0x00, 0x00, // Reserved7
    0x00,                   // Reserved8
    0x00,                   // Reserved9
    0x00, 0x00,             // Reserved10
    0x00, 0x00, 0x00, 0x00, // Reserved11
    0x81, 0x07              // Checksum
};

byte GPSoff[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};

// Byte array for wake command for GPS module
byte GPSon[] = {0xFF, 0xFF, 0x00, 0x00};

// Send a byte array of UBX protocol to the GPS - //From https://forum.arduino.cc/t/gps-module-power-management/477859/26
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gpsSerial.write(MSG[i]);
  }
}


void displaySetup() {
  // Initialize the display
  u8x8.setI2CAddress(0x3C << 1);
  u8x8.begin();
  u8x8.setFont(u8x8_font_5x7_f);

  u8x8.drawString(0, 0, "Booting");

  delay(2000);
  u8x8.clearDisplay();

  u8x8.drawString(0, 0, "CHECKER");
  u8x8.setCursor(0, 1);
  u8x8.print("LoRa: N/A");
}

void displayBattery(float batteryVoltage) {
  u8x8.setCursor(11, 0);
  u8x8.print("Batt:");
  u8x8.setCursor(11, 1);
  float batteryPercentage = (batteryVoltage - 1.0) / (4.2 - 1.0) * 100;
  
  char batteryStr[4]; // Buffer to hold the formatted string
  sprintf(batteryStr, "%3d", (int)batteryPercentage); // Format the percentage to 3 digits
  
  u8x8.print(batteryStr);
  u8x8.print("%");
}

void displayCurrentLocation(int32_t latitude, int32_t longitude, char* date, char* time) {
  displayClearMessage();
  char latBuffer[15];
  char longBuffer[15];

  // Convert back to float and format with 7 digits after the decimal point
  float lat = latitude / 10000000.0;
  float lon = longitude / 10000000.0;

  dtostrf(lat, 10, 7, latBuffer); // 10 is the width, 7 is the precision
  dtostrf(lon, 11, 7, longBuffer); // 11 is the width, 7 is the precision

  u8x8.setCursor(0, 2);
  u8x8.print("Lat:  ");
  u8x8.print(latBuffer);
  u8x8.setCursor(0, 3);
  u8x8.print("Long:");
  u8x8.print(longBuffer);
  u8x8.setCursor(0, 4);
  u8x8.print("Date: ");
  u8x8.print(date[6]);
  u8x8.print(date[7]);
  u8x8.print("-");
  u8x8.print(date[4]);
  u8x8.print(date[5]);
  u8x8.print("-");
  u8x8.print(date[0]);
  u8x8.print(date[1]);
  u8x8.print(date[2]);
  u8x8.print(date[3]);
  u8x8.setCursor(0, 5);
  u8x8.print("Time: ");
  u8x8.print(time[0]);
  u8x8.print(time[1]);
  u8x8.print(":");
  u8x8.print(time[2]);
  u8x8.print(time[3]);
  // u8x8.print(":");
  // u8x8.print(time[4]);
  // u8x8.print(time[5]);
}

void displayMessageSuccess() {
  displayClearMessage();
  u8x8.setCursor(0, 2);
  u8x8.print("Location was ");
}

void displayMessageErrorWarning(String errorMessage) {
  displayClearMessage();
  u8x8.setCursor(0, 2);
  u8x8.print("Error: ");
  u8x8.print(errorMessage);
}

void displayClearMessage() {
  u8x8.clearLine(2);
  u8x8.clearLine(3);
  u8x8.clearLine(4);
  u8x8.clearLine(5);
}

void displayStatusMessage(const char* message) {
  u8x8.clearLine(7);
  u8x8.setCursor(0, 7);
  u8x8.print(message);
}

void displayLoRaStatus(bool status) {
  u8x8.setCursor(0, 1);
  if (status) {
    u8x8.print("LoRa: OK ");
  } else {
    u8x8.print("LoRa: N/A");
  }
}

void displayControl() {
  /*
  Main control function to display messages on the OLED screen
  */
  const int intervalSuccessMessage = 1000;
  const int intervalErrorMessage = 2000; // interval at which to wait (milliseconds)
  const int intervalBattery = 1000; // interval at which to wait (milliseconds)
  const int intervalTransmittedMessage = 2000; // interval to clear "Transmitted" message

  unsigned long currentMillis = millis();

  if (boolDisplayData) {
    displayMessageSuccess();
    if (currentMillis - previousMillisSuccessMessage >= intervalSuccessMessage) {
      previousMillisSuccessMessage = currentMillis;
      displayCurrentLocation(receivedData.latitude, receivedData.longitude, receivedData.date, receivedData.time);
      boolDisplayData = false;
    }
  }
  if (boolDisplayError) {
    displayMessageErrorWarning("Invalid data");
    if (currentMillis - previousMillisErrorMessage >= intervalErrorMessage) {
      previousMillisErrorMessage = currentMillis;
      displayClearMessage();
      boolDisplayError = false;
    }
  }
  if (currentMillis - previousMillisBattery >= intervalBattery) {
    previousMillisBattery = currentMillis;
    displayBattery(getBatteryVoltage());
  }
  if (currentMillis - previousMillisTransmittedMessage >= intervalTransmittedMessage) {
    displayStatusMessage("");
  }
}

void measureBrightness() {
  int LDRvalue = analogRead(LDR);
  uint8_t brightness = 0;
  brightness = map(LDRvalue, 0, 1023, 0, 255);
  u8x8.setContrast(brightness);
}

float getBatteryVoltage() {
  int sensorValue = analogRead(BATTERYPIN);
  int value = constrain(sensorValue, 0, 1023);
  float voltage = value * (4.2 / 1023.0);
  return voltage;
}

void processGPSData() {
  gpsSerial.listen(); // Make GPS serial active
  sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
  delay(500);
  sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
  delay(500);
  sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
  delay(500);
  char c = gpsSerial.read();
  Serial.write(c); // Print GPS data to serial monitor for debugging
  unsigned long start = millis();
  bool newData = false;

  #ifdef DEBUG
  Serial.println(F("Proc. GPS"));
  #endif
  displayStatusMessage("Getting location");
  while (millis() - start < 180000) { // Wait up to 180 seconds for valid GPS data
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      Serial.write(c); // Print GPS data to serial monitor for debugging
      if (gps.encode(c)) {
        newData = true;
      }
    }
    if (newData && gps.location.isUpdated() && gps.date.isValid() && gps.time.isValid()) {
      // Get the date and time, format it and enter it to the character array in the format, YYYYMMDD and HHMMSS
      sprintf(receivedData.date, "%04d%02d%02d", gps.date.year(), gps.date.month(), gps.date.day());
      sprintf(receivedData.time, "%02d%02d%02d", gps.time.hour(), gps.time.minute(), gps.time.second());

      // Convert latitude and longitude to signed integers with 7 digits after the decimal point
      receivedData.latitude = static_cast<int32_t>(gps.location.lat() * 10000000);
      receivedData.longitude = static_cast<int32_t>(gps.location.lng() * 10000000);

      // Serial.println("Date:");
      // Serial.println(receivedData.date);
      // Serial.println("Time:");
      // Serial.println(receivedData.time);
      // Serial.print("Formatted Latitude: ");
      // Serial.println(receivedData.latitude);
      // Serial.print("Formatted Longitude: ");
      // Serial.println(receivedData.longitude);

      validLocation = true;
      boolDisplayData = true;
      boolDisplayError = false;
      break;
    }
  }

  if (!newData || gps.charsProcessed() == 0 || !gps.location.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
    #ifdef DEBUG
    Serial.println("** No valid data received from GPS: check wiring **");
    #endif
    displayStatusMessage("GPS Error");
    boolDisplayError = true;
    boolDisplayData = false;
  } else {
    displayStatusMessage("Location valid");
  }
  sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
  delay(500);
  sendUBX(CFG_PM2, sizeof(CFG_PM2)/sizeof(uint8_t));
  delay(500);
  sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
  loraSerial.listen(); // Switch back to LoRa serial
}

void debounceButton() {
    int currentBtnState = digitalRead(BTN_PIN);
    if (currentBtnState != previousBtnState) {            
        // every time the button state changes, get the time of that change
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // if the button state has changed and the debounce delay has passed
            if (currentBtnState == HIGH && notHold == true) {
                #ifdef DEBUG
                Serial.println("Button Pressed");
                #endif
                processGPSData();
                // Serial.print("Longitude: ");
                // Serial.println(receivedData.longitude);
                // Serial.print("Latitude: ");
                // Serial.println(receivedData.latitude);
                // Serial.print("Date: ");
                // Serial.println(receivedData.date);
                // Serial.print("Time: ");
                // Serial.println(receivedData.time);
                notHold = false;
                flag = true;
                displayLoRaStatus(loraConnected); // Update LoRa status on button press
            }
            else if (currentBtnState == LOW) {
                notHold = true;
            }
    }
    previousBtnState = currentBtnState;
}

void wakeUpLora() {
  #ifdef DEBUG
  Serial.println("Wake up LoRa");
  #endif
  loraSerial.listen(); // Ensure LoRa serial is active
  digitalWrite(LORA_TX, LOW);
  pinMode(LORA_TX, OUTPUT);
  digitalWrite(LORA_TX, HIGH); 
  delay(20);
  loraSerial.write(0x55);
  myLora.autobaud();
  #ifdef DEBUG
  Serial.println("LoRa woke up");
  #endif
}

void sendData() {

    if (!flag || !validLocation) {
        return;
    }
    flag = false;
    validLocation = false; // Reset validLocation after transmission
    #ifdef DEBUG
    Serial.println("Transmitting");
    #endif
    displayStatusMessage("Transmitting");
    // delay(500);
    // Comment out the actual LoRa transmission for now
    // Serial.println((uint8_t*)&receivedData);
    wakeUpLora();
    uint8_t txResponse = myLora.txBytes((uint8_t*)&receivedData, sizeof(receivedData));
    if (txResponse == 0) {
        #ifdef DEBUG
        Serial.println(F("TX unsuccessful"));
        #endif
        displayStatusMessage("Transmit error");
        buzzerStates = BUZZER_ERROR;
        loraConnected = false;
    } else {
        #ifdef DEBUG
        Serial.println(F("TX successful"));
        #endif
        displayStatusMessage("Transmitted");
        buzzerStates = BUZZER_SUCCESS;
    }
    myLora.sleep(60000*10); // Sleep for 10 minutes
    previousMillisTransmittedMessage = millis();
}

void checkLoraConnection() {
  // Taking from https://github.com/jpmeijers/RN2483-Arduino-Library/blob/master/examples/ArduinoUnoNano-downlink/ArduinoUnoNano-downlink.ino
  loraSerial.listen(); // Ensure LoRa serial is active
  displayStatusMessage("Checking LoRa");
  switch(myLora.txCnf("!")) //one byte, blocking function
    {
        case TX_FAIL:
        {
            #ifdef DEBUG
            Serial.println(F("TX unsuccessful or not acknowledged"));
            #endif
            loraConnected = false;
            break;
        }
        case TX_SUCCESS:
        {
            #ifdef DEBUG
            Serial.println(F("TX successful and acknowledged"));
            #endif
            loraConnected = true;
            break;
        }
        case TX_WITH_RX:
        {
            String received = myLora.getRx();
            received = myLora.base16decode(received);
            loraConnected = true;
            #ifdef DEBUG
            Serial.println(F("Received downlink"));
            #endif
            displayStatusMessage("App updated");
            buzzerStates = BUZZER_UPDATE;
            break;
        }
        default:
        {
            #ifdef DEBUG
            Serial.println(F("Unknown response from TX function"));
            #endif
        }
    }

}

void initializeRadio() {
  displayStatusMessage("LoRa Starting");
  loraSerial.listen(); // Ensure LoRa serial is active
  //reset rn2483
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);
  delay(500);
  digitalWrite(LORA_RESET, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  loraSerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  // //check communication with radio
  // String hweui = myLora.hweui();
  // while(hweui.length() != 16) {
  //   Serial.println(("Communication with RN2xx3 unsuccessful. Power cycle the board."));
  //   Serial.println(hweui);
  //   delay(10000);
  //   hweui = myLora.hweui();
  // }

  //configure your keys and join the network
  #ifdef DEBUG
  Serial.println(F("Joining TTN"));
  #endif
  bool join_result = false;

  const char *appEui = "0000000000000000";
  const char *appKey = "511744DB68B8E1D4BCF88EA2E8F10887";

  join_result = myLora.initOTAA(appEui, appKey);
  // myLora.appeui(appEui);
  // myLora.appkey(appKey);
  unsigned long startAttemptTime = millis();
  while(!join_result && millis() - startAttemptTime < 185000) // Try for 3 minutes
  {
    #ifdef DEBUG
    Serial.println(F("Unable to join TTN"));
    #endif
    join_result = myLora.init();
    delay(5000); //delay 5 seconds before retry
  }

  if (join_result) {
    #ifdef DEBUG
    Serial.println(F("Success join TTN"));
    #endif
    myLora.setDR(0); //setting data-rate to 5, i.e. SF7
    loraConnected = true; // Set LoRa connection status
  } else {
    #ifdef DEBUG
    Serial.println(F("Fail join TTN"));
    #endif
    loraConnected = false;
  }
  displayStatusMessage("");
}

extern unsigned int __heap_start, *__brkval;
int freeMemory() {
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void buzzerSuccess(unsigned long startMillis) {
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis < 200) {
    digitalWrite(LEDPIN, HIGH);
    tone(BUZZERPIN, 1000, 200);
  } else if (currentMillis - startMillis < 400) {
    digitalWrite(LEDPIN, LOW);
    tone(BUZZERPIN, 1100, 200);
  } else if (currentMillis - startMillis < 600) {
    digitalWrite(LEDPIN, HIGH);
    tone(BUZZERPIN, 1200, 200);
  } else if (currentMillis - startMillis < 800) {
    digitalWrite(LEDPIN, LOW);
    tone(BUZZERPIN, 1300, 200);
  } else if (currentMillis - startMillis < 1000) {
    digitalWrite(LEDPIN, HIGH);
    tone(BUZZERPIN, 1400, 200);
  } else {
    noTone(BUZZERPIN);
    buzzerStates = BUZZER_OFF;
    digitalWrite(LEDPIN, LOW);
  }
}

void buzzerError(unsigned long startMillis) {
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis < 500) {
    digitalWrite(LEDPIN, HIGH);
    tone(BUZZERPIN, 930, 500);
  } else if (currentMillis - startMillis < 1000) {
    digitalWrite(LEDPIN, LOW);
    tone(BUZZERPIN, 880, 500);
  } else if (currentMillis - startMillis < 1800) {
    digitalWrite(LEDPIN, HIGH);
    tone(BUZZERPIN, 830, 800);
  } else {
    noTone(BUZZERPIN);
    buzzerStates = BUZZER_OFF;
    digitalWrite(LEDPIN, LOW);
  }
}

void buzzerUpdate(unsigned long startMillis) {
  unsigned long currentMillis = millis();
  if (currentMillis - startMillis < 300) {
    digitalWrite(LEDPIN, HIGH);
    tone(BUZZERPIN, 1500, 300);
  } else if (currentMillis - startMillis < 600) {
    digitalWrite(LEDPIN, LOW);
    tone(BUZZERPIN, 1600, 300);
  } else if (currentMillis - startMillis < 900) {
    digitalWrite(LEDPIN, HIGH);
    tone(BUZZERPIN, 1700, 300);
  } else {
    noTone(BUZZERPIN);
    buzzerStates = BUZZER_OFF;
    digitalWrite(LEDPIN, LOW);
  }
}

void buzzerControl() {
  static unsigned long startMillis = 0;

  if (buzzerStates != BUZZER_OFF) {
    if (startMillis == 0) {
      startMillis = millis();
    }
    switch (buzzerStates) {
      case BUZZER_SUCCESS:
        buzzerSuccess(startMillis);
        break;
      case BUZZER_ERROR:
        buzzerError(startMillis);
        break;
      case BUZZER_UPDATE:
        buzzerUpdate(startMillis);
        break;
    }
  } else {
    startMillis = 0; // Reset startMillis when buzzer is off
  }
}

void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  pinMode(BUZZERPIN, OUTPUT);
  tone(BUZZERPIN, 600, 300); // Buzzer startup sound
  displaySetup();
  loraSerial.begin(9600); //serial port to radio module
  gpsSerial.begin(9600); // Initialize GPS serial //Changed from 4800 to 9600 for testing
  gpsSerial.listen(); // Ensure GPS serial is active
  sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
  delay(500);
  sendUBX(CFG_PM2, sizeof(CFG_PM2)/sizeof(uint8_t));
  delay(500);
  sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t)); // Turn GPS Power save mode on
  delay(500);
  
  loraSerial.listen(); // Ensure LoRa serial is active
  initializeRadio();
  myLora.sleep(60000*10); // Sleep for 10 minutes
  pinMode(LDR, INPUT);
  pinMode(BATTERYPIN, INPUT);
  pinMode(BTN_PIN, INPUT);
  currentBtnState = LOW;            
  previousBtnState = LOW;
  lastDebounceTime = millis();  
  // #ifdef DEBUG
  // Serial.print(F("Free memory: "));
  // Serial.println(freeMemory());
  // #endif
  displayLoRaStatus(loraConnected); // Display initial LoRa status
}

void loop() {
    measureBrightness();    
    displayControl();
    debounceButton();
    sendData();
    delay(100);
    // #ifdef DEBUG
    // Serial.print(F("Free memory: "));
    // Serial.println(freeMemory());
    // #endif
    static unsigned long lastLoraUpdate = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastLoraUpdate >= 60000*10) { // Check RX message every ~10 minute
      wakeUpLora();
      lastLoraUpdate = currentMillis;
      checkLoraConnection();
      displayLoRaStatus(loraConnected);
      myLora.sleep(60000*10); // Sleep for 10 minutes
    }
    buzzerControl();
}