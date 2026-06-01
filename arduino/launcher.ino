#include <ESP32Servo.h>

// ── PINS ──────────────────────────────────────────────
#define PAN_PIN  13
#define TILT_PIN 12
#define PUSH_PIN 23
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 32
#define IN3 33
#define IN4 14

// ── OBJECTS ───────────────────────────────────────────
Servo panServo;
Servo tiltServo;
Servo pushServo;

// ── STATE ─────────────────────────────────────────────
int  panAngle   = 90;
int  tiltAngle  = 180;  // start horizontal
int  motorSpeed = 255;
int  spinUpTime = 5000;
bool motorsOn   = false;

// ── MOTOR FUNCTIONS ───────────────────────────────────
void spinUp() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  for(int i = 0; i <= motorSpeed; i += 5) {
    analogWrite(ENA, i);
    analogWrite(ENB, i);
    delay(10);
  }
  motorsOn = true;
  Serial.println("MOTORS_ON");
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  motorsOn = false;
  Serial.println("MOTORS_OFF");
}

void fireDart() {
  Serial.println("FIRING");
  if(!motorsOn) {
    spinUp();
    delay(spinUpTime);
  }
  pushServo.write(75);
  delay(300);
  pushServo.write(0);
  delay(200);
  stopMotors();
  Serial.println("FIRED");
}

// ── SETUP ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  pushServo.attach(PUSH_PIN);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  pushServo.write(0);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.println("READY");
  Serial.println("A=pan left  D=pan right");
  Serial.println("W=tilt up   S=tilt down");
  Serial.println("C=center    F=fire");
  Serial.println("X=spinup    Z=stop motors");
  Serial.println("V###=speed  U###=spinup time");
  Serial.println("P###=pan    T###=tilt");
}

// ── MAIN LOOP ─────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Pan left
    if (cmd == "A" || cmd == "a") {
      panAngle = constrain(panAngle + 1, 0, 180);
      panServo.write(panAngle);
      Serial.print("LEFT | Pan: ");
      Serial.println(panAngle);
    }
    // Pan right
    else if (cmd == "D" || cmd == "d") {
      panAngle = constrain(panAngle - 1, 0, 180);
      panServo.write(panAngle);
      Serial.print("RIGHT | Pan: ");
      Serial.println(panAngle);
    }
    // Tilt up (toward 90°)
    else if (cmd == "W" || cmd == "w") {
      tiltAngle = constrain(tiltAngle - 1, 90, 180);
      tiltServo.write(tiltAngle);
      Serial.print("UP | Tilt: ");
      Serial.println(tiltAngle);
    }
    // Tilt down (toward 180°)
    else if (cmd == "S" || cmd == "s") {
      tiltAngle = constrain(tiltAngle + 1, 90, 180);
      tiltServo.write(tiltAngle);
      Serial.print("DOWN | Tilt: ");
      Serial.println(tiltAngle);
    }
    // Center
    else if (cmd == "C" || cmd == "c") {
      panAngle  = 90;
      tiltAngle = 180;
      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
      pushServo.write(0);
      Serial.println("CENTERED");
    }
    // Fire
    else if (cmd == "F") {
      fireDart();
    }
    // Spin up
    else if (cmd == "X") {
      spinUp();
    }
    // Stop motors
    else if (cmd == "Z") {
      stopMotors();
    }
    // Pan to exact angle
    else if (cmd.charAt(0) == 'P') {
      int target = cmd.substring(1).toInt();
      panAngle = constrain(180 - target, 0, 180);
      panServo.write(panAngle);
      Serial.print("PAN: ");
      Serial.println(panAngle);
    }
    // Tilt to exact angle
    else if (cmd.charAt(0) == 'T') {
      tiltAngle = constrain(cmd.substring(1).toInt(), 90, 180);
      tiltServo.write(tiltAngle);
      Serial.print("TILT: ");
      Serial.println(tiltAngle);
    }
    // Set motor speed
    else if (cmd.charAt(0) == 'V') {
      motorSpeed = constrain(cmd.substring(1).toInt(), 0, 255);
      Serial.print("SPEED: ");
      Serial.println(motorSpeed);
    }
    // Set spinup time
    else if (cmd.charAt(0) == 'U') {
      spinUpTime = cmd.substring(1).toInt();
      Serial.print("SPINUP TIME: ");
      Serial.println(spinUpTime);
    }
    // Status
    else if (cmd == "?") {
      Serial.print("PAN: ");
      Serial.print(panAngle);
      Serial.print(" TILT: ");
      Serial.println(tiltAngle);
    }
  }
}
