#include <ESP32Servo.h>

Servo scanner;
// Led Pins
const int greenLED = 25;
const int yellowLED = 26;
const int redLED = 27;

// Buzzer
const int buzzer = 32;
// Servo 
const int servoPin = 13;

//FSM States
  enum State{
    IDLE,
    SURVEY,
    CAUTION,
    EMERGENCY,
    FAILSAFE
  };
  State currentState = IDLE;

void setup() {

  Serial.begin(115200);

  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  pinMode(buzzer, OUTPUT);

  scanner.attach(servoPin);

  Serial.println("Output system running");
}

void loop() {

  switch(currentState){

    case IDLE:
      idleOutputs();
      delay(3000);

      currentState = SURVEY;
      break;

    case SURVEY:
      surveyOutputs();
      delay(3000);

      currentState = CAUTION;
      break;

    case CAUTION:
      cautionOutputs();
      delay(3000);

      currentState = EMERGENCY;
      break;

    case EMERGENCY:
      emergencyOutputs();
      delay(3000);

      currentState = FAILSAFE;
      break;
    
    case FAILSAFE:
      failSafeOutputs();
      delay(3000);

      currentState = IDLE;
      break;
  }
}

void idleOutputs(){
 Serial.println("STATE: IDLE");

  // LEDs
  digitalWrite(greenLED, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(redLED, LOW);

  // buzzer off
  noTone(buzzer);

  // servo centered
  scanner.write(90);
}
void surveyOutputs(){

  Serial.println("STATE: SURVEY");

  digitalWrite(greenLED, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(redLED, LOW);

  // buzzer off
  noTone(buzzer);

  // servo scan
  for(int angle = 0; angle <= 180; angle += 10){

    scanner.write(angle);
    delay(40);
  }

  for(int angle = 180; angle >= 0; angle -= 10){

    scanner.write(angle);
    delay(40);
  }
}

void cautionOutputs(){

  Serial.println("STATE: CAUTION");

  // blink yellow LED
  for(int i = 0; i < 5; i++){

    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);

    digitalWrite(yellowLED, HIGH);

    tone(buzzer, 1000);

    delay(200);

    digitalWrite(yellowLED, LOW);

    noTone(buzzer);

    delay(200);
  }

  // faster servo scan
  for(int angle = 0; angle <= 180; angle += 15){

    scanner.write(angle);
    delay(20);
  }

  for(int angle = 180; angle >= 0; angle -= 15){

    scanner.write(angle);
    delay(20);
  }
}

void emergencyOutputs(){

  Serial.println("STATE: EMERGENCY");

  // rapid red flashing + alarm
  for(int i = 0; i < 10; i++){

    digitalWrite(greenLED, LOW);
    digitalWrite(yellowLED, LOW);

    digitalWrite(redLED, HIGH);

    tone(buzzer, 2000);

    delay(100);

    digitalWrite(redLED, LOW);

    noTone(buzzer);

    delay(100);
  }

  // aggressive servo scan
  for(int angle = 0; angle <= 180; angle += 20){

    scanner.write(angle);
    delay(10);
  }

  for(int angle = 180; angle >= 0; angle -= 20){

    scanner.write(angle);
    delay(10);
  }
}

void failSafeOutputs(){

  Serial.println("STATE: FAILSAFE");

  // stop servo
  scanner.write(90);

  // warning flash pattern
  for(int i = 0; i < 6; i++){

    digitalWrite(greenLED, HIGH);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(redLED, HIGH);

    tone(buzzer, 500);

    delay(250);

    digitalWrite(greenLED, LOW);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, LOW);

    noTone(buzzer);

    delay(250);
  }
}