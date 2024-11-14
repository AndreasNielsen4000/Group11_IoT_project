/* Arduino tutorial - Buzzer / Piezo Speaker
   More info and circuit: http://www.ardumotive.com/how-to-use-a-buzzer-en.html
   Dev: Michalis Vasilakis // Date: 9/6/2015 // www.ardumotive.com */

//const int pin = 8; //buzzer to arduino pin 9
#define buzzerPIN 8

void setup(){
  buzzer(buzzerPIN);
}

void loop(){
  start_stop_buzzer();
}


void buzzer(int pin) {
  pinMode(pin, OUTPUT);
  
}

void start_stop_buzzer(int number_buzz) {

  for (int i = 0; i < number_buzzz; i++) {
    tone(buzzerPIN, 1000); // Send 1KHz sound signal...
    delay(1000);        // ...for 1 sec
    noTone(buzzerPIN);     // Stop sound...
    delay(1000);        // ...for 1sec
  }
  
}
