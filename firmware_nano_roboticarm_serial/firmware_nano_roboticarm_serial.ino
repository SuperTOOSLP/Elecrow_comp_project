#include <Servo.h>
#include <SoftwareSerial.h>

// ================= UART ESP32 =================
#define ESP_RX 9
#define ESP_TX 8
SoftwareSerial espSerial(ESP_RX, ESP_TX);

// ================= PINS =================
const int irSensorPin = 4;
const int motorPin    = 11;
const int ledPin      = 7;
int ledPWM = 250;
int motorPWM = 110;

// ================= SERVOS =================
Servo base; /// Base
Servo hombro; /// Shoulder
Servo codo; /// Elbow
Servo mano; /// hand

// ================= SYSTEM =================
bool systemActive = false;
bool armBusy = false;

// ================= STATE MACHINE =================
enum State {
  IDLE,
  RUNNING,
  IR_DELAY,
  WAIT_ESP32
};

State currentState = IDLE;

// ================= TIMERS =================
unsigned long stateTimer = 0;

// ================= CONFIG =================int motorPWM = 100;
int velocidad = 8;

// ================= UART BUFFER =================
char rxBuffer[16];
uint8_t rxIndex = 0;

// ================= CLASSIFICATION STABILITY =================
int lastCmd = -1;
int stableCount = 0;

// ================= SERVO MOVE =================
// Smoothly move a servo from 'inicio' to 'fin' with delay t between steps
void moverServo(Servo &servo, int inicio, int fin, int t) {
  if (inicio < fin) {
    for (int p = inicio; p <= fin; p++) {
      servo.write(p);
      delay(t);
    }
  } else {
    for (int p = inicio; p >= fin; p--) {
      servo.write(p);
      delay(t);
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  espSerial.begin(115200);

  base.attach(3);
  hombro.attach(5);
  codo.attach(6);
  mano.attach(10);

  pinMode(irSensorPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  base.write(90);
  hombro.write(90);
  codo.write(180);
  mano.write(100);

  analogWrite(motorPin, 0);
  digitalWrite(ledPin, LOW); // Initially LED off

  Serial.println("Arduino Nano READY");
}

// ================= LOOP =================
void loop() {
  readUSB();
  readESP32();

  switch (currentState) {

    case IDLE:
      analogWrite(motorPin, 0);
      break;

    case RUNNING:
      analogWrite(motorPin, motorPWM);
      // LED is controlled only by numeric commands 2 (ON) and 3 (OFF).
      // Remove PWM control here to ensure LED persistent state.
      // analogWrite(ledPin, ledPWM); <-- removed on purpose

      if (digitalRead(irSensorPin) == LOW && !armBusy) {
        //Serial.println("IR DETECTED");
        stateTimer = millis();
        currentState = IR_DELAY;
      }
      break;

    case IR_DELAY:
      if (millis() - stateTimer >= 50) {   // 50 ms delay for stop motor
        analogWrite(motorPin, 0);
        // Do NOT change LED state here — LED must remain as commanded by serial 2/3.
        // digitalWrite(ledPin, HIGH); 
        stateTimer = millis();
        currentState = WAIT_ESP32;
        stableCount = 0;
        lastCmd = -1;
        //Serial.println("Motor stopped, waiting ESP32");
      }
      break;

    case WAIT_ESP32:
      // lectura ocurre en readESP32()
      break;
  }
}

// ================= READ ESP32 =================
void readESP32() {
  if (!systemActive) return;
  if (currentState != WAIT_ESP32) return;
  if (millis() - stateTimer < 1000) return; // 1 seg for verify the object

  while (espSerial.available()) {
    char c = espSerial.read();

    if (c == '\n' || c == '\r') {
      if (rxIndex == 0) return;

      rxBuffer[rxIndex] = '\0';
      int cmd = atoi(rxBuffer);
      rxIndex = 0;

      if (cmd == 400) {
        //Serial.println("ESP32 -> 400 (ignored)");
        stableCount = 0;
        lastCmd = -1;
        return;
      }

      if (cmd == lastCmd) {
        stableCount++;
      } else {
        lastCmd = cmd;
        stableCount = 1;
      }

      //Serial.print("ESP32 -> ");
      //Serial.print(cmd);
      //Serial.print(" | stable = ");
      //Serial.println(stableCount);

      if (stableCount >= 3 && (cmd == 100 || cmd == 200 || cmd == 300)) {
        Serial.println(cmd);
        executeClassification(cmd);
        stableCount = 0;
        lastCmd = -1;
      }
    }
    else if (rxIndex < 15) {
      rxBuffer[rxIndex++] = c;
    }
  }
}

// ================= READ USB =================
void readUSB() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxIndex == 0) return;

      rxBuffer[rxIndex] = '\0';
      rxIndex = 0;

      // ===== TEXT COMMANDS =====
      if (strncmp(rxBuffer, "Band:", 5) == 0) {
        // Parse Band command and map 0-100 to motor PWM 0-180
        int val = constrain(atoi(rxBuffer + 5), 0, 100);
        motorPWM = map(val, 0, 100, 0, 180);
        Serial.print("Band PWM = ");
        Serial.println(motorPWM);
        return;
      }

      if (strncmp(rxBuffer, "Arm:", 4) == 0) {
        // Parse Arm command and map inversely to velocidad
        int val = constrain(atoi(rxBuffer + 4), 0, 100);
        velocidad = map(val, 0, 100, 25, 5); // inverso
        Serial.print("Arm speed = ");
        Serial.println(velocidad);
        return;
      }

            

      // ===== NUMERIC COMMANDS =====
      int cmd = atoi(rxBuffer);

      Serial.print("RX: ");
      Serial.println(cmd);

      if (cmd == 1) {
        systemActive = true;
        currentState = RUNNING;
        //Serial.println("SYSTEM START");
      }
      else if (cmd == 0) {
        systemActive = false;
        currentState = IDLE;
        analogWrite(motorPin, 0);
        // Do NOT change LED on system stop — keep last commanded state (2/3).
        // digitalWrite(ledPin, LOW); 
      }
      else if (cmd == 2) {
        // Numeric command 2: turn LED ON and latch it ON
        digitalWrite(ledPin, HIGH);
      }
      else if (cmd == 3) {
        // Numeric command 3: turn LED OFF and latch it OFF
        digitalWrite(ledPin, LOW);
      }
    }
    else if (rxIndex < 15) {
      rxBuffer[rxIndex++] = c;
    }
  }
}


// ================= CLASSIFICATION =================
void executeClassification(int type) {
  armBusy = true;

  int baseTarget = 90;
  if (type == 100) baseTarget = 0;
  else if (type == 200) baseTarget = 30;
  else if (type == 300) baseTarget = 50;

  //Serial.print("EXECUTING TYPE ");
  //Serial.println(type);

  // ===== TOMAR OBJETO =====
  moverServo(codo, 180, 135, velocidad);
  moverServo(hombro, 90, 135, velocidad);
  moverServo(mano, 100, 60, velocidad);

  // ===== LEVANTAR =====
  moverServo(hombro, 135, 90, velocidad);
  moverServo(codo, 135, 180, velocidad);

  // ===== ROTAR BASE =====
  moverServo(base, 90, baseTarget, velocidad);

  // ===== SOLTAR =====
  moverServo(hombro, 90, 135, velocidad);
  moverServo(codo, 180, 135, velocidad);
  moverServo(mano, 60, 100, velocidad);

  // ===== HOME =====
  moverServo(codo, 135, 180, velocidad);
  moverServo(hombro, 135, 90, velocidad);
  moverServo(base, baseTarget, 90, velocidad);

  // Keep LED state as commanded by serial (do not force OFF here).
  // digitalWrite(ledPin, LOW); 
  armBusy = false;
  currentState = RUNNING;

  //Serial.println("READY FOR NEXT OBJECT");
}
