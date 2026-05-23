#include <DHT.h>
#include <ESP32Servo.h>

// -------------------- PIN CONFIGURATION --------------------
#define DHT_PIN        15
#define DHT_TYPE       DHT22

#define TRIG_PIN       5
#define ECHO_PIN       18

#define GREEN_LED_PIN  25
#define YELLOW_LED_PIN 26
#define RED_LED_PIN    2

#define BUZZER_PIN     4
#define SERVO_PIN      13

// -------------------- THRESHOLDS --------------------
const float CAUTION_TEMP_THRESHOLD = 45.0;
const float EMERGENCY_TEMP_THRESHOLD = 60.0;
const float CAUTION_HUMIDITY_THRESHOLD = 20.0;
const float EMERGENCY_HUMIDITY_THRESHOLD = 15.0;
const int PRESENCE_DISTANCE_CM = 50;
const int MAX_VALID_DISTANCE_CM = 400;

// -------------------- TIMING --------------------
unsigned long lastSensorRead = 0;
unsigned long lastBlinkToggle = 0;
unsigned long lastServoMove = 0;
unsigned long sensorInterval = 2000;  // adaptive interval, updated by FSM

bool blinkState = false;
int servoAngle = 0;
int servoDirection = 1;

// -------------------- SENSOR VALUES --------------------
float temperature = 0.0;
float humidity = 0.0;
long distanceCm = 0;

// -------------------- OBJECTS --------------------
DHT dht(DHT_PIN, DHT_TYPE);
Servo scanServo;

// -------------------- FSM STATES --------------------
enum SystemState {
  IDLE,
  SURVEY,
  CAUTION,
  EMERGENCY,
  FAILSAFE
};

SystemState currentState = IDLE;
SystemState previousState = IDLE;

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  dht.begin();
  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2400);

  setAllOutputsOff();
  scanServo.write(90);

  Serial.println("Smart Hazard Detection System Started");
  Serial.println("FSM: IDLE | SURVEY | CAUTION | EMERGENCY | FAILSAFE");
}

// -------------------- MAIN CONTROL LOOP --------------------
void loop() {
  unsigned long now = millis();

  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now;

    readSensors();
    updateFSM();
    printStatus();
  }

  handleStateOutputs(now);
}

// -------------------- SENSOR READING --------------------
void readSensors() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  distanceCm = readDistanceCm();
}

long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return -1;  // timeout / invalid reading
  }

  long distance = duration * 0.034 / 2;
  return distance;
}

bool sensorsAreValid() {
  if (isnan(temperature) || isnan(humidity)) {
    return false;
  }

  if (distanceCm <= 0 || distanceCm > MAX_VALID_DISTANCE_CM) {
    return false;
  }

  return true;
}

// -------------------- FSM DECISION LOGIC --------------------
void updateFSM() {
  previousState = currentState;

  if (!sensorsAreValid()) {
    currentState = FAILSAFE;
    sensorInterval = 250;
    return;
  }

  bool cautionCondition = (temperature > CAUTION_TEMP_THRESHOLD || humidity < CAUTION_HUMIDITY_THRESHOLD);

  bool emergencyCondition =
    (temperature > EMERGENCY_TEMP_THRESHOLD && humidity < EMERGENCY_HUMIDITY_THRESHOLD) ||
    (temperature > EMERGENCY_TEMP_THRESHOLD && distanceCm < PRESENCE_DISTANCE_CM);

  if (emergencyCondition) {
    currentState = EMERGENCY;
    sensorInterval = 200;       // adaptive behaviour: monitor very quickly
  }
  else if (cautionCondition) {
    currentState = CAUTION;
    sensorInterval = 500;       // monitor faster than safe mode
  }
  else if (distanceCm < PRESENCE_DISTANCE_CM) {
    currentState = SURVEY;
    sensorInterval = 750;       // active sensing when presence is detected
  }
  else {
    currentState = IDLE;
    sensorInterval = 2000;      // low-frequency monitoring in safe conditions
  }
}

// -------------------- OUTPUT BEHAVIOUR --------------------
void handleStateOutputs(unsigned long now) {
  switch (currentState) {
    case IDLE:
      handleIdleState();
      break;

    case SURVEY:
      handleSurveyState(now);
      break;

    case CAUTION:
      handleCautionState(now);
      break;

    case EMERGENCY:
      handleEmergencyState(now);
      break;

    case FAILSAFE:
      handleFailsafeState(now);
      break;
  }
}

void handleIdleState() {
  Serial.println("STATE: IDLE");
  setAllOutputsOff();
  digitalWrite(GREEN_LED_PIN, HIGH);
  noTone(BUZZER_PIN);
  scanServo.write(90);
}

void handleSurveyState(unsigned long now) {
  Serial.println("STATE: SURVEY");
  setAllOutputsOff();
  digitalWrite(GREEN_LED_PIN, HIGH);
  noTone(BUZZER_PIN);
  sweepServo(now, 20);  // slow scanning
}

void handleCautionState(unsigned long now) {
  
  // Turn off unrelated LEDs
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  // Slow blinking yellow warning indicator
  if (now - lastBlinkToggle >= 500) {
    lastBlinkToggle = now;
    blinkState = !blinkState;

    digitalWrite(YELLOW_LED_PIN, blinkState);

    // Intermittent warning buzzer
    if (blinkState) {
      tone(BUZZER_PIN, 800);
    } else {
      noTone(BUZZER_PIN);
    }
  }

  // Servo centers toward monitoring position
  scanServo.write(90);
}

void handleEmergencyState(unsigned long now) {
  Serial.println("STATE: EMERGENCY");
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  if (now - lastBlinkToggle >= 150) {
    lastBlinkToggle = now;
    blinkState = !blinkState;
    digitalWrite(RED_LED_PIN, blinkState);
  }

  tone(BUZZER_PIN, 1200);
  scanServo.write(90);
}

void handleFailsafeState(unsigned long now) {
  Serial.println("STATE: FAILSAFE");
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);

  if (now - lastBlinkToggle >= 250) {
    lastBlinkToggle = now;
    blinkState = !blinkState;
    digitalWrite(RED_LED_PIN, blinkState);
  }

  tone(BUZZER_PIN, 400);
  scanServo.write(0);  // restricted safe position
}

void sweepServo(unsigned long now, int speedMs) {
  if (now - lastServoMove >= speedMs) {
    lastServoMove = now;

    servoAngle += servoDirection * 2;

    if (servoAngle >= 180) {
      servoAngle = 180;
      servoDirection = -1;
    }
    else if (servoAngle <= 0) {
      servoAngle = 0;
      servoDirection = 1;
    }

    scanServo.write(servoAngle);
  }
}

void setAllOutputsOff() {
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  noTone(BUZZER_PIN);
}

// -------------------- SERIAL DEBUGGING --------------------
void printStatus() {
  Serial.println("--------------------------------");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  Serial.print("State: ");
  Serial.println(stateToString(currentState));

  Serial.print("Monitoring interval: ");
  Serial.print(sensorInterval);
  Serial.println(" ms");
}

String stateToString(SystemState state) {
  switch (state) {
    case IDLE: return "IDLE";
    case SURVEY: return "SURVEY";
    case CAUTION: return "CAUTION";
    case EMERGENCY: return "EMERGENCY";
    case FAILSAFE: return "FAILSAFE";
    default: return "UNKNOWN";
  }
}

