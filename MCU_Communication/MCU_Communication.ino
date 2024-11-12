// Include the required Wire library for I2C<br>#include <Wire.h>
#include <Wire.h>
int LED = 13;
int x = 0;


const int btnPin = 7; // the number of the pushbutton pin
int currentBtnState;  // Current and previous states of input Button pin
int previousBtnState;
unsigned int lastDebounceTime;      
unsigned int debounceDelay; 
volatile bool flag = false;
bool notHold = true;

struct MCU_Communication {
    float longitude;
    float latitude;
    char time[10]; // Use a fixed-size char array instead of String for simplicity
};

MCU_Communication mcu_communication;

byte payload[sizeof(MCU_Communication)]; // 4 bytes for longitude, 4 bytes for latitude, 5 bytes for time string


void getMCU_Communication(MCU_Communication *mcu_communication)
{
    mcu_communication->longitude = 37.7749;
    mcu_communication->latitude = -122.4194;
    strcpy(mcu_communication->time, "12:00");
}

void debounceButton() {
    int currentBtnState = digitalRead(btnPin);
    digitalWrite(LED_BUILTIN,currentBtnState);
    if (currentBtnState != previousBtnState) {            
        // every time the button state changes, get the time of that change
        lastDebounceTime = millis();
        Serial.print("Current button state: ");
        Serial.println(currentBtnState);
        Serial.print("Previous button state: ");
        Serial.println(previousBtnState);
    }
    
    if ((millis() - lastDebounceTime) > debounceDelay) {
        // if the button state has changed and the debounce delay has passed
            if (currentBtnState == HIGH && notHold == true) {
                Serial.println("Button Pressed");
                getMCU_Communication(&mcu_communication);
                Serial.print("Longitude: ");
                Serial.println(mcu_communication.longitude);
                Serial.print("Latitude: ");
                Serial.println(mcu_communication.latitude);
                Serial.print("Time: ");
                Serial.println(mcu_communication.time);
                notHold = false;
                flag = true;
            }
            else if (currentBtnState == LOW) {
                notHold = true;
            }
    }
    previousBtnState = currentBtnState;
}

void sentData() {
        if (!flag) {
            return;
        }
        memcpy(&payload[0], &mcu_communication.longitude, sizeof(mcu_communication.longitude));
        memcpy(&payload[4], &mcu_communication.latitude, sizeof(mcu_communication.latitude));
        memcpy(&payload[8], &mcu_communication.time, sizeof(mcu_communication.time));
        Wire.beginTransmission(0);
        Wire.write(payload, sizeof(payload));
        Wire.endTransmission();
        Serial.println("Data sent to Master");
        flag = false;
}

void setup() {
    Serial.begin(9600);
    // Define the LED pin as Output
    pinMode (LED_BUILTIN, OUTPUT);
    // Define the Button pin as Input
    pinMode(btnPin, INPUT);
    currentBtnState = LOW;            
    previousBtnState = LOW;
    lastDebounceTime = millis();  
    debounceDelay = 50;
    // Start the I2C Bus as Slave on address 9
    Wire.begin(9); 
    
}

void loop() {
    debounceButton();
    sentData();
    delay(10);
}
