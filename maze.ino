/*
 * Meshmerize Maze Solver — Arduino Nano + TB6612FNG + CD4051 (8 sensors) + FRONT PROBE
 * Additions:
 *  - 9th "front probe" analog sensor (ahead of the bar) on FRONT_PIN (default A6)
 *  - Per-sensor calibration now also calibrates the front probe
 *  - straightAvail() considers mid band OR front probe
 *  - Node detection & dead-end confirmation use the probe
 *  - Turn capture accepts new branch when mid OR probe reacquires line
 */

#include <EEPROM.h>
#include <Wire.h>

// ================== TB6612FNG MOTOR & IO PINS ==================
// Left motor (Channel A)
#define PWMA 11
#define AIN1 7
#define AIN2 8

// Right motor (Channel B)
#define PWMB 6
#define BIN1 9
#define BIN2 10

// Status LED
#define LED_RED 13

// Buttons
#define BTN_MODE 5
#define BTN_OK   2
#define BTN_BACK 4

// ================== CD4051 MULTIPLEXER PINS =====================
#define muxSig A0
#define muxS0  A1
#define muxS1  A2
#define muxS2  A3

// ================== FRONT PROBE SENSOR PIN ======================
#define FRONT_PIN A6   // <— change if you wired the forward sensor elsewhere

// ================== SENSOR / PID CONFIG =========================
const int sensor_count = 8;

// Physical wiring order on CD4051 (left->right logical index)
const int sensor_order[sensor_count] = {2,4,0,6,5,7,3,1};

// Calibrated min/max values per physical channel
float cal_min[sensor_count];
float cal_max[sensor_count];

// Live, mapped readings (logical 0..7 left->right), 0..1000 scale (WHITE high)
float sensor_reading[sensor_count];

// ===== FRONT PROBE calibration + reading =====
float cal_min_front = 0;
float cal_max_front = 1023;
float front_reading = 0;   // 0..1000 (WHITE high)

// Threshold to treat a sensor as “on line” (white tape on black mat)
const int line_threshold = 650;

// Lateral distances (center near 0). Tune for your bar geometry.
int distFromCenter[8] = { -400,-200,-70,-20,20,70,200,400 };

// PID constants
float Kp = 1.6;
float Ki = 0.0;
float Kd = 15.0;
double PIDcorr = 0, integral_error = 0, previous_error = 0;

// Base speeds
#define BASE_SPD 100
const int STALL_CLIP = 60;

// Lost-line memory (which side we last saw line)
int lastSeenSide = 0; // -1 = left, +1 = right, 0 = unknown

// Node handling timings
const int NODE_SETTLE_MS     = 70;
const int CONFIRM_DEADEND_MS = 120;
const int ALIGN_FWD_MS       = 110;



// ================== MAZE STATE / PATH STORAGE ===================
char path[200];
int  pathLength = 0;

enum Mode {CALIBRATION, DRY_RUN, ACTUAL_RUN};
Mode mode = CALIBRATION;

// ================== FORWARD DECLARATIONS ========================
void lineFollowPID();
void calibrateSensors();
void readSensors();
void setMotor(int leftSpeed, int rightSpeed);

// -------------------- helpers --------------------
bool midOnLine() {
  // mid band OR forward probe counts as straight “seen”
  return (sensor_reading[3] > line_threshold) ||
         (sensor_reading[4] > line_threshold) ||
         (front_reading      > line_threshold);
}

//////////////////// MUX & SENSOR ////////////////////
int muxRead(int channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  delayMicroseconds(5);
  return analogRead(muxSig);
}

void sensor_read() {
  for (int i = 0; i < sensor_count; i++) {
    int phys = sensor_order[i];
    int raw  = muxRead(phys);
    raw = constrain(raw, (long)cal_min[phys], (long)cal_max[phys]);
    // Map raw -> 0..1000 with WHITE = high (white tape brighter)
    sensor_reading[i] = map(raw, cal_min[phys], cal_max[phys], 0, 1000);
  }

  // ---- FRONT PROBE ----
  int fraw = analogRead(FRONT_PIN);
  fraw = constrain(fraw, (long)cal_min_front, (long)cal_max_front);
  front_reading = map(fraw, cal_min_front, cal_max_front, 0, 1000);
}

void readSensors() { sensor_read(); }

//////////////////// PID FOLLOWER /////////////////////
void updateLastSeenSide() {
  int leftSum = 0, rightSum = 0;
  for (int i = 0; i < sensor_count; i++) {
    if (sensor_reading[i] > 350) {
      if (i < sensor_count / 2) leftSum  += sensor_reading[i];
      else                      rightSum += sensor_reading[i];
    }
  }
  if (leftSum > 0 || rightSum > 0) {
    if (leftSum > rightSum)      lastSeenSide = -1;
    else if (rightSum > leftSum) lastSeenSide = +1;
  }
}

void pid_follower() {
  sensor_read();
  updateLastSeenSide();

  double totalSum = 0, weightedSum = 0;
  for (int i = 0; i < sensor_count; i++) {
    totalSum += sensor_reading[i];
    if (sensor_reading[i] < 350) continue;
    weightedSum += distFromCenter[i] * sensor_reading[i];
  }

  float error = 0;
  if (totalSum > 0) {
    error = (float)weightedSum / totalSum;
  } else {
    // Lost line -> recover toward last seen side
    int recover = 120;
    if (lastSeenSide == -1) setMotor(-recover, recover);
    else if (lastSeenSide == +1) setMotor(recover, -recover);
    else setMotor(0,0);
    return;
  }

  // Edge “snap” if outer hits strongly
  if (sensor_reading[0] > line_threshold)       error = -300;
  else if (sensor_reading[7] > line_threshold)  error =  300;

  // Anti-windup
  if (abs(error) > 120) integral_error = 0;
  else {
    integral_error += error;
    integral_error = constrain(integral_error, -80, 80);
  }

  float derivative = error - previous_error;
  float lookAheadError = 0.7f * error + 0.3f * previous_error;

  PIDcorr = (Kp * lookAheadError) + (Ki * integral_error) + (Kd * derivative);
  previous_error = error;

  int baseSpeed = BASE_SPD;
  if (abs(error) > 100) baseSpeed = (int)(BASE_SPD * 0.7);
  if (abs(error) > 180) baseSpeed = (int)(BASE_SPD * 0.5);

  int L = baseSpeed - (int)PIDcorr;
  int R = baseSpeed + (int)PIDcorr;

  if (L > 0 && L < STALL_CLIP) L = STALL_CLIP;
  if (R > 0 && R < STALL_CLIP) R = STALL_CLIP;

  setMotor(constrain(L,-255,255), constrain(R,-255,255));
}

void lineFollowPID() { pid_follower(); }

//////////////////// NODE DETECTION ///////////////////
bool leftAvail()    { return (sensor_reading[0] > line_threshold) || (sensor_reading[1] > line_threshold); }
bool rightAvail()   { return (sensor_reading[6] > line_threshold) || (sensor_reading[7] > line_threshold); }
bool straightAvail(){ return ((sensor_reading[3] > line_threshold) ||
                             (sensor_reading[4] > line_threshold) )||
                             (front_reading      > line_threshold); }

bool detectDeadEnd() {
  // If nothing on the bar AND probe also doesn't see straight, call dead-end
  for (int i = 0; i < sensor_count; i++)
    if (sensor_reading[i] > line_threshold) return false;
  if (front_reading > line_threshold) return false;
  return true;
}

bool detectEndAllWhite() { // end pad = all white strongly (bar + probe)
  for (int i = 0; i < sensor_count; i++)
    if (sensor_reading[i] < line_threshold) return false;
  if (front_reading < line_threshold) return false;
  return true;
}

// Conservative node predicate
bool probableNode() {
  bool L = leftAvail(), S = straightAvail(), R = rightAvail();
  // Node if straight exists and any lateral exists, OR mid disappeared but probe sees something (approach), OR mid disappeared totally.
  return (S && (L || R)) || (!S);
}

//////////////////// TURNS / MOTION ///////////////////
void moveForwardSmall() {
  setMotor(BASE_SPD, BASE_SPD);
  delay(100);
  setMotor(0,0);
}

// Turn capture: accept when mid band OR probe reacquires line
void turnLeft() {
    // Start turning
    setMotor(-BASE_SPD, BASE_SPD);
    
    // Wait to lose line (with timeout)
    unsigned long t = millis();
    sensor_read();
    while(midOnLine() && (millis() - t < 500)) {
        sensor_read();
    }
    
    // Wait to find line again (with timeout)
    t = millis();
    while(!midOnLine() && (millis() - t < 800)) {
        sensor_read();
    }
    
    // Align forward a bit
    setMotor(BASE_SPD, BASE_SPD);
    delay(20);  // Move forward to center on line
    
    // Stop
    setMotor(0, 0);
    
    // Reset PID
    integral_error = 0;
    previous_error = 0;
}

void turnRight() {
  setMotor(BASE_SPD,-BASE_SPD);
    
    // Wait to lose line (with timeout)
    unsigned long t = millis();
    sensor_read();
    while(midOnLine() && (millis() - t < 500)) {
        sensor_read();
    }
    
    // Wait to find line again (with timeout)
    t = millis();
    while(!midOnLine() && (millis() - t < 800)) {
        sensor_read();
    }
    
    // Align forward a bit
    setMotor(BASE_SPD, BASE_SPD);
    delay(20);  // Move forward to center on line
    
    // Stop
    setMotor(0, 0);
    
    // Reset PID
    integral_error = 0;
    previous_error = 0;
  
  }
  
  


void turnBack() {
  setMotor(BASE_SPD, -BASE_SPD);
    
    // Phase 1: Wait to lose the current line
    unsigned long t = millis();
    sensor_read();
    while(midOnLine() && (millis() - t < 600)) {
        sensor_read();
        delay(5);
    }
    
    // Phase 2: Keep spinning until we find the line from opposite direction
    t = millis();
    while(!midOnLine() && (millis() - t < 1000)) {
        sensor_read();
        delay(5);
    }
    
    // Stop turning
    setMotor(0, 0);
    
  
  
    
  
}

//////////////////// LSRB DECISION /////////////////////
char chooseLSRB(bool L, bool S, bool R) {
  if (!L && !S && !R) return 'B';
  if (L) return 'L';
  if (S) return 'S';
  if (R) return 'R';
  return 'B';
}

//////////////////// PATH SIMPLIFIER //////////////////
int moveAngle(char m) {
  if (m=='L') return -90;
  if (m=='R') return  90;
  if (m=='B') return 180;
  return 0; // 'S'
}
char angleToMove(int a) {
  a%=360; if (a<0) a+=360;
  if (a==0)   return 'S';
  if (a==90)  return 'R';
  if (a==180) return 'B';
  if (a==270) return 'L';
  if (a<45 || a>=315) return 'S';
  if (a<135) return 'R';
  if (a<225) return 'B';
  return 'L';
}
void simplifyPathOnce() {
  if (pathLength < 3) return;
  char Y = path[pathLength-1];
  char B = path[pathLength-2];
  char X = path[pathLength-3];
  if (B != 'B') return;
  int sum = moveAngle(X) + 180 + moveAngle(Y);
  char Z = angleToMove(sum);
  path[pathLength-3] = Z;
  pathLength -= 2;
}
void simplifyPathFull() {
  bool changed = true;
  while (changed) {
    int before = pathLength;
    simplifyPathOnce();
    changed = (pathLength != before);
  }
}

//////////////////// UI / MODES ///////////////////////
void selectMode() {
  int current = 0;
  const char* modeNames[3] = {"CALIBRATION", "DRY RUN", "ACTUAL RUN"};
  delay(300);
  while (1) {
    if (digitalRead(BTN_MODE) == LOW) {
      current = (current + 1) % 3;
      Serial.print("Selected Mode: ");
      Serial.println(modeNames[current]);
      delay(400);
    }
    if (digitalRead(BTN_OK) == LOW) {
      mode = (Mode)current;
      Serial.print("Mode Confirmed: ");
      Serial.println(modeNames[current]);
      delay(400);
      break;
    }
  }
}

//////////////////// CALIBRATION //////////////////////
void calibrateSensors() {
  for (int i = 0; i < sensor_count; i++) { cal_min[i] = 1023.0; cal_max[i] = 0.0; }
  cal_min_front = 1023.0; cal_max_front = 0.0;

  // spin in place ~8s to see both colors
  setMotor(180, -180);
  digitalWrite(LED_RED, HIGH);

  unsigned long startTime = millis();
  while (millis() - startTime < 8000) {
    // bar channels
    for (int ch = 0; ch < sensor_count; ch++) {
      int raw = muxRead(ch);
      if (raw < cal_min[ch]) cal_min[ch] = raw;
      if (raw > cal_max[ch]) cal_max[ch] = raw;
    }
    // front probe
    int fraw = analogRead(FRONT_PIN);
    if (fraw < cal_min_front) cal_min_front = fraw;
    if (fraw > cal_max_front) cal_max_front = fraw;

    delay(10);
  }
  setMotor(0,0);
  digitalWrite(LED_RED, LOW);

  // Guard for low spread (widen a bit)
  for (int i = 0; i < sensor_count; i++) {
    if (cal_max[i] - cal_min[i] < 60) {
      cal_min[i] = max(0.f,      cal_min[i]-30);
      cal_max[i] = min(1023.f,   cal_max[i]+30);
    }
  }
  if (cal_max_front - cal_min_front < 60) {
    cal_min_front = max(0.f,     cal_min_front-30);
    cal_max_front = min(1023.f,  cal_max_front+30);
  }

  Serial.println("Calibration Done (min/max in RAM).");
}

//////////////////// DRY RUN (EXPLORE) ////////////////
void runDry() {
  pathLength = 0;

  while (1) {
    readSensors();

    // END: all-white pad (bar + probe)
    if (detectEndAllWhite()) {
      setMotor(0,0);
      digitalWrite(LED_RED, HIGH);
      Serial.println("END DETECTED - Press OK to Save Path");
      while (digitalRead(BTN_OK) == HIGH) { /* wait */ }
      digitalWrite(LED_RED, LOW);
      break;
    }

    // Probable node? Settle & classify
    if (probableNode()) {
      // slow & settle
      setMotor(BASE_SPD, BASE_SPD);
      delay(NODE_SETTLE_MS);
      setMotor(0,0);
      readSensors();

      // confirm dead-end by a tiny creep
      bool L = leftAvail(), S = straightAvail(), R = rightAvail();
      if (!L && !S && !R) {
        setMotor(BASE_SPD, BASE_SPD);
        delay(CONFIRM_DEADEND_MS);
        setMotor(0,0);
        readSensors();
        L = leftAvail(); S = straightAvail(); R = rightAvail();
      }

      char mv = chooseLSRB(L,S,R);
      if (pathLength < (int)sizeof(path)-1) {
        path[pathLength++] = mv;
        simplifyPathOnce();        // on-the-fly simplification
      }

      if (mv=='L') turnLeft();
      else if (mv=='R') turnRight();
      else if (mv=='B') turnBack();
      else { // 'S'
        setMotor(BASE_SPD, BASE_SPD);
        delay(ALIGN_FWD_MS);
        setMotor(0,0);
      }
      continue;
    }

    // Otherwise, just follow line
    lineFollowPID();
  }
}

//////////////////// ACTUAL RUN (REPLAY) //////////////
void runActual() {
  for (int i = 0; i < pathLength; i++) {
    char mv = path[i];
    Serial.print("Executing: "); Serial.println(mv);
    if (mv=='L') turnLeft();
    else if (mv=='R') turnRight();
    else if (mv=='B') turnBack();
    else { // 'S'
      // short push, then PID for stability
      setMotor(BASE_SPD+40, BASE_SPD+40);
      delay(140);
      setMotor(0,0);
      for (int t=0; t<6; t++) { lineFollowPID(); delay(20); }
    }
  }
  setMotor(0,0);
  digitalWrite(LED_RED, HIGH);
  Serial.println("Reached End of Actual Run");
}

//////////////////// EEPROM PATH SAVE/LOAD ////////////
void savePathToEEPROM() {
  EEPROM.write(0, pathLength);
  for (int i = 0; i < pathLength; i++) EEPROM.write(i+1, path[i]);
  Serial.println("Path stored in EEPROM.");
}

void loadPathFromEEPROM() {
  pathLength = EEPROM.read(0);
  for (int i = 0; i < pathLength; i++) path[i] = EEPROM.read(i+1);
  Serial.print("Loaded Path: ");
  for (int k = 0; k < pathLength; k++) { Serial.print(path[k]); Serial.print(' '); }
  Serial.println();
}

//////////////////// MOTOR CONTROL ////////////////////
void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left A
  if (leftSpeed > 0)      { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else if (leftSpeed < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  else                    { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  }
  analogWrite(PWMA, abs(leftSpeed));

  // Right B
  if (rightSpeed > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else if (rightSpeed < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else                     { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  }
  analogWrite(PWMB, abs(rightSpeed));
}

//////////////////// SETUP / LOOP /////////////////////
void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_OK,   INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  pinMode(muxS0, OUTPUT); pinMode(muxS1, OUTPUT); pinMode(muxS2, OUTPUT);
  pinMode(muxSig, INPUT);
  pinMode(FRONT_PIN, INPUT);

  // Default calibration range (if you skip CAL)
  for (int i = 0; i < sensor_count; i++) { cal_min[i] = 0; cal_max[i] = 1023; }
  cal_min_front = 0; cal_max_front = 1023;

 
  selectMode();
}

void loop() {
  if (mode == CALIBRATION) {
    Serial.println("Calibrating sensors...");
    delay(300);
    calibrateSensors();
    Serial.println("Calibration done. Press MODE to switch.");
    selectMode();
  }
  else if (mode == DRY_RUN) {
    Serial.println("=== DRY RUN START ===");
    runDry();
    Serial.println("=== DRY RUN DONE ===");
    simplifyPathFull();
    savePathToEEPROM();
    Serial.println("Path saved to EEPROM!");
    selectMode();
  }
  else if (mode == ACTUAL_RUN) {
    Serial.println("=== ACTUAL RUN START ===");
    loadPathFromEEPROM();
    runActual();
    Serial.println("=== ACTUAL RUN DONE ===");
    selectMode();
  }
}