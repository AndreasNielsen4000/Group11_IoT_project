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
#define GPS_RX 4
#define GPS_TX 3

#define DEBUG 

bool displayDataFlag = false;
bool displayErrorFlag = false;
bool loraStatus = false;
uint8_t buzzerStates = BUZZER_OFF;

struct loraPayload_t {
    int32_t longitude;
    int32_t latitude;
    char date[8]; // YYYYMMDD
    char time[6]; // HHMMSS
};

SoftwareSerial loraSerial(LORA_RX, LORA_TX); // RX, TX
rn2xx3 myLora(loraSerial);

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX); // RX, TX for GPS module

loraPayload_t txData;

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

// Function to send a byte array of UBX protocol to the GPS
// This function sends a UBX command to the GPS module to configure its settings.
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gpsSerial.write(MSG[i]);
  }
}

// Function to initialize the OLED display
// This function sets up the OLED display with initial settings and displays a boot message.
void displaySetup() {
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

// Function to display battery voltage on the OLED display
// This function calculates the battery percentage from the voltage and displays it on the OLED.
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

// Function to display the current GPS location on the OLED display
// This function formats and displays the latitude, longitude, date, and time on the OLED.
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
  u8x8.print("Lat : ");
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
}

// Function to display a success message on the OLED display
// This function displays a success message indicating that the location was successfully obtained.
void displayMessageSuccess() {
  displayClearMessage();
  u8x8.setCursor(0, 2);
  u8x8.print("Location was ");
}

// Function to display an error message on the OLED display
// This function displays an error message with the provided error text.
void displayMessageErrorWarning(const char* errorMessage) {
  displayClearMessage();
  u8x8.setCursor(0, 2);
  u8x8.print("Error: ");
  u8x8.print(errorMessage);
}

// Function to clear specific lines on the OLED display
// This function clears lines 2 to 5 on the OLED display.
void displayClearMessage() {
  u8x8.clearLine(2);
  u8x8.clearLine(3);
  u8x8.clearLine(4);
  u8x8.clearLine(5);
}

// Function to display a status message on the OLED display
// This function displays a status message on the last line of the OLED display.
void displayStatusMessage(const char* message) {
  u8x8.clearLine(7);
  u8x8.setCursor(0, 7);
  u8x8.print(message);
}

// Function to display the LoRa connection status on the OLED display
// This function updates the OLED display with the current status of the LoRa connection.
void displayLoRaStatus(bool status) {
  u8x8.setCursor(0, 1);
  if (status) {
    u8x8.print("LoRa: OK ");
  } else {
    u8x8.print("LoRa: N/A");
  }
}

// Main control function to display messages on the OLED screen
// This function manages the display of various messages on the OLED based on different flags and intervals.
void displayControl() {
  const int intervalSuccessMessage = 1000;
  const int intervalErrorMessage = 2000; // interval at which to wait (milliseconds)
  const int intervalBattery = 30000; // interval at which to wait (milliseconds)
  const int intervalTransmittedMessage = 2000; // interval to clear "Transmitted" message

  unsigned long currentMillis = millis();

  if (displayDataFlag) {
    displayMessageSuccess();
    if (currentMillis - previousMillisSuccessMessage >= intervalSuccessMessage) {
      previousMillisSuccessMessage = currentMillis;
      displayCurrentLocation(txData.latitude, txData.longitude, txData.date, txData.time);
      displayDataFlag = false;
    }
  }
  if (displayErrorFlag) {
    displayMessageErrorWarning("Invalid data");
    if (currentMillis - previousMillisErrorMessage >= intervalErrorMessage) {
      previousMillisErrorMessage = currentMillis;
      displayClearMessage();
      displayErrorFlag = false;
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

// Function to measure the brightness using the LDR sensor and adjust the OLED display contrast
// This function reads the LDR sensor value and adjusts the OLED display contrast accordingly.
void measureBrightness() {
  int LDRvalue = analogRead(LDR);
  uint8_t brightness = 0;
  brightness = map(LDRvalue, 0, 1023, 0, 255);
  u8x8.setContrast(brightness);
}

// Function to get the battery voltage
// This function reads the battery voltage from the analog pin and returns it as a float.
float getBatteryVoltage() {
  int sensorValue = analogRead(BATTERYPIN);
  int value = constrain(sensorValue, 0, 1023);
  float voltage = value * (4.2 / 1023.0);
  return voltage;
}

// Function to get GPS data and update the txData structure
// This function retrieves GPS data, formats it, and updates the txData structure with the location, date, and time.
void getGPSData() {
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
      sprintf(txData.date, "%04d%02d%02d", gps.date.year(), gps.date.month(), gps.date.day());
      sprintf(txData.time, "%02d%02d%02d", gps.time.hour(), gps.time.minute(), gps.time.second());

      // Convert latitude and longitude to signed integers with 7 digits after the decimal point
      txData.latitude = static_cast<int32_t>(gps.location.lat() * 10000000);
      txData.longitude = static_cast<int32_t>(gps.location.lng() * 10000000);
      
      #ifdef DEBUG
      Serial.println("Date:");
      Serial.println(txData.date);
      Serial.println("Time:");
      Serial.println(txData.time);
      Serial.print("Formatted Latitude: ");
      Serial.println(txData.latitude);
      Serial.print("Formatted Longitude: ");
      Serial.println(txData.longitude);
      #endif

      validLocation = true;
      displayDataFlag = true;
      displayErrorFlag = false;
      break;
    }
  }

  if (!newData || gps.charsProcessed() == 0 || !gps.location.isValid() || !gps.date.isValid() || !gps.time.isValid()) {
    #ifdef DEBUG
    Serial.println("** No valid data received from GPS: check wiring **");
    #endif
    displayStatusMessage("GPS Error");
    displayErrorFlag = true;
    displayDataFlag = false;
    buzzerStates = BUZZER_ERROR;
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

// Function to debounce the button press
// This function debounces the button press to avoid multiple triggers and initiates GPS data retrieval on a valid press.
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
                getGPSData();
                #ifdef DEBUG
                Serial.print("Longitude: ");
                Serial.println(txData.longitude);
                Serial.print("Latitude: ");
                Serial.println(txData.latitude);
                Serial.print("Date: ");
                Serial.println(txData.date);
                Serial.print("Time: ");
                Serial.println(txData.time);
                #endif
                notHold = false;
                flag = true;
                displayLoRaStatus(loraStatus); // Update LoRa status on button press
            }
            else if (currentBtnState == LOW) {
                notHold = true;
            }
    }
    previousBtnState = currentBtnState;
}

// Function to wake up the LoRa module
// This function wakes up the LoRa module from sleep mode and sets it to the correct baud rate.
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

// Function to send data via LoRa
// This function sends the GPS data via LoRa and updates the display and buzzer based on the transmission result.
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
    // wakeUpLora();
    uint8_t txResponse = myLora.txBytes((uint8_t*)&txData, sizeof(txData));
    if (txResponse == 0) {
        #ifdef DEBUG
        Serial.println(F("TX unsuccessful"));
        #endif
        displayMessageErrorWarning("LoRa");
        buzzerStates = BUZZER_ERROR;
        loraStatus = false;
    } else {
        #ifdef DEBUG
        Serial.println(F("TX successful"));
        #endif
        displayStatusMessage("Transmitted");
        buzzerStates = BUZZER_SUCCESS;
    }
    displayLoRaStatus(loraStatus);
    // myLora.sleep(60000*10); // Sleep for 10 minutes
    previousMillisTransmittedMessage = millis();
}

// Function to check the LoRa connection
// This function checks the LoRa connection by sending a test message and updates the status based on the response.
void checkLoraConnection() {
  // From https://github.com/jpmeijers/RN2483-Arduino-Library/blob/master/examples/ArduinoUnoNano-downlink/ArduinoUnoNano-downlink.ino
  loraSerial.listen(); // Ensure LoRa serial is active
  displayStatusMessage("Checking LoRa");
  switch(myLora.txCnf("!")) //one byte, blocking function
    {
        case TX_FAIL:
        {
            #ifdef DEBUG
            Serial.println(F("TX unsuccessful or not acknowledged"));
            #endif
            loraStatus = false;
            break;
        }
        case TX_SUCCESS:
        {
            #ifdef DEBUG
            Serial.println(F("TX successful and acknowledged"));
            #endif
            loraStatus = true;
            break;
        }
        case TX_WITH_RX:
        {
            String received = myLora.getRx();
            received = myLora.base16decode(received);
            loraStatus = true;
            #ifdef DEBUG
            Serial.println("Received downlink: " + received);
            #endif
            displayStatusMessage("App update");
            buzzerStates = BUZZER_UPDATE;
            previousMillisTransmittedMessage = millis();
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

// Function to initialize the LoRa radio module
// This function initializes the LoRa module, attempts to join the network, and updates the connection status.
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

  //configure your keys and join the network
  #ifdef DEBUG
  Serial.println(F("Joining TTN"));
  #endif
  bool join_result = false;

  const char *appEui = "0000000000000000";
  const char *appKey = "511744DB68B8E1D4BCF88EA2E8F10887";

  join_result = myLora.initOTAA(appEui, appKey);

  unsigned long startAttemptTime = millis();
  while(!join_result && millis() - startAttemptTime < 185000) // Try for 3 minutes
  {
    #ifdef DEBUG
    Serial.println(F("Unable to join TTN"));
    #endif
    // myLora.setDR(0); //setting data-rate
    join_result = myLora.init();
    delay(5000); //delay 5 seconds before retry
  }

  if (join_result) {
    #ifdef DEBUG
    Serial.println(F("Success join TTN"));
    #endif
    // myLora.setDR(0); //setting data-rate
    loraStatus = true; // Set LoRa connection status
  } else {
    #ifdef DEBUG
    Serial.println(F("Fail join TTN"));
    #endif
    loraStatus = false;
  }
  displayStatusMessage("");
}

// Function to control the buzzer for success indication
// This function controls the buzzer to emit a success sound pattern.
void buzzerSuccess() {
  digitalWrite(LEDPIN, HIGH);
  tone(BUZZERPIN, 1000);
  delay(200);
  digitalWrite(LEDPIN, LOW);
  tone(BUZZERPIN, 1100);
  delay(200);
  digitalWrite(LEDPIN, HIGH);
  tone(BUZZERPIN, 1200);
  delay(200);
  digitalWrite(LEDPIN, LOW);
  tone(BUZZERPIN, 1300);
  delay(200);
  digitalWrite(LEDPIN, HIGH);
  tone(BUZZERPIN, 1400);
  delay(200);
  noTone(BUZZERPIN);
  buzzerStates = BUZZER_OFF;
  digitalWrite(LEDPIN, LOW);
}

// Function to control the buzzer for error indication
// This function controls the buzzer to emit an error sound pattern.
void buzzerError() {
  digitalWrite(LEDPIN, HIGH);
  tone(BUZZERPIN, 930);
  digitalWrite(LEDPIN, LOW);
  delay(500);
  tone(BUZZERPIN, 880);
  digitalWrite(LEDPIN, HIGH);
  delay(500);
  tone(BUZZERPIN, 830);
  delay(800);
  noTone(BUZZERPIN);
  buzzerStates = BUZZER_OFF;
  digitalWrite(LEDPIN, LOW);
}

// Function to control the buzzer for update indication
// This function controls the buzzer to emit an update sound pattern.
void buzzerUpdate() {
  digitalWrite(LEDPIN, HIGH);
  tone(BUZZERPIN, 1500); 
  delay(300);
  digitalWrite(LEDPIN, LOW);
  tone(BUZZERPIN, 1600);
  delay(300);
  digitalWrite(LEDPIN, HIGH);
  tone(BUZZERPIN, 1700);
  delay(300);
  noTone(BUZZERPIN);
  buzzerStates = BUZZER_OFF;
  digitalWrite(LEDPIN, LOW);
}

// Function to control the buzzer based on the current state
// This function manages the buzzer state and triggers the appropriate sound pattern based on the current state.
void buzzerControl() {
  switch (buzzerStates) {
    case BUZZER_SUCCESS:
      buzzerSuccess();
      break;
    case BUZZER_ERROR:
      buzzerError();
      break;
    case BUZZER_UPDATE:
      buzzerUpdate();
      break;
  }
}

// Setup function to initialize the hardware and software components
// This function sets up the initial state of the hardware and software components, including the OLED display, LoRa, and GPS modules.
void setup() {
  #ifdef DEBUG
  Serial.begin(9600);
  #endif
  pinMode(BUZZERPIN, OUTPUT);
  tone(BUZZERPIN, 600, 300); // Buzzer startup sound
  displaySetup();
  loraSerial.begin(9600); 
  gpsSerial.begin(9600);
  gpsSerial.listen(); // Ensure GPS serial is active
  // Turn GPS Power save mode on
  sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
  delay(500);
  sendUBX(CFG_PM2, sizeof(CFG_PM2)/sizeof(uint8_t));
  delay(500);
  sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
  delay(500);
  
  loraSerial.listen(); // Ensure LoRa serial is active
  initializeRadio();
  // myLora.sleep(60000*10);
  pinMode(LDR, INPUT);
  pinMode(BATTERYPIN, INPUT);
  pinMode(BTN_PIN, INPUT);
  currentBtnState = LOW;            
  previousBtnState = LOW;
  lastDebounceTime = millis();  
  displayLoRaStatus(loraStatus); // Display initial LoRa status
}

// Main loop function to handle the main logic of the program
// This function contains the main logic of the program, including measuring brightness, updating the display, debouncing the button, sending data, and controlling the buzzer.
void loop() {
    measureBrightness();    
    displayControl();
    debounceButton();
    sendData();
    delay(100);
    static unsigned long lastLoraUpdate = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastLoraUpdate >= 60000) { // Check RX message every ~10 minute
      // wakeUpLora();
      lastLoraUpdate = currentMillis;
      checkLoraConnection();
      displayLoRaStatus(loraStatus);
      // myLora.sleep(60000*10);
    }
    buzzerControl();
}