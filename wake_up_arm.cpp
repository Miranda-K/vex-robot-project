#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

//----------------------------------------------------------------------------
//
//    Module:       main.cpp
//    Project:      Wake-Up Arm
//    Description:  Waits for an alarm, finds the sleeper's head with the
//                  distance sensor, nudges until they win a crossy-road game
//                  on the brain, then goes home.
//
//----------------------------------------------------------------------------

// Include the IQ Library
#include "iq_cpp.h"

// Allows for easier use of the VEX Library
using namespace vex;

// ===========================================================================
// CONSTANTS (tune these, all in one place)
// ===========================================================================

// Gear ratios
const double BASE_RATIO = 112.0 / 12.0;  // motor deg per base deg
const double ELBOW_RATIO = 48.0;         // motor deg per arm deg
const double CM_PER_DEG = 0.0;           // MEASURE: spin carriage motor 360 deg, measure travel (cm), divide by 360

// Motor ranges (MEASURE every one). Each is in the joint's own units, measured
// from the home pose, where every encoder is zeroed. Every move is clamped to these.
const double BASE_MIN_DEG = -90.0;       // base degrees, 0 = straight ahead (cables wrap past this)
const double BASE_MAX_DEG = 90.0;
const double ELBOW_MIN_DEG = 0.0;        // arm degrees, 0 = home (arm up), + = down toward the bed
const double ELBOW_MAX_DEG = 60.0;
const double CARRIAGE_MIN_CM = 0.0;      // cm along the arm, 0 = fully pulled in
const double CARRIAGE_MAX_CM = 25.0;
const double NUDGER_MIN_DEG = 0.0;       // nudger motor degrees, 0 = retracted
const double NUDGER_MAX_DEG = 120.0;
const double RANGE_OVERRUN_DEG = 10.0;   // base past its range by this much = stop (motor direction wrong)

// Geometry and tuning (must sit inside the ranges above)
const double SENSOR_TO_TIP_CM = 10.0;    // nudger tip ahead of the sensor when retracted
const double NUDGE_ANGLE = 40.0;         // elbow angle used for nudging (MEASURE)
const double NUDGE_DEG = 90.0;           // nudger motor stroke per poke (MEASURE)

// Base control
const bool USE_GYRO_FOR_BASE = true;     // false = use base motor encoder instead of the gyro
const double BASE_TOLERANCE_DEG = 2.0;
const double BASE_TIMEOUT_SEC = 5.0;
const double BASE_MIN_POWER = 8.0;       // percent, so it never stalls just short of target

// Head scan
const int SCAN_DEG = 90;                 // sweep from -SCAN_DEG to +SCAN_DEG
const int SCAN_STEP_DEG = 5;
const int SCAN_STEPS = 2 * SCAN_DEG / SCAN_STEP_DEG + 1;  // 37 readings
const int MIN_HEAD_MM = 50;              // ignore readings outside the bed range (MEASURE)
const int MAX_HEAD_MM = 400;
const int NOT_FOUND = -999;

// Recovery after a push (stretch goal)
const double PUSH_HOLD_SEC = 3.0;        // bumper held this long after a poke = arm is being pushed
const double PUSH_ANGLE_TOL_DEG = 5.0;
const double PUSH_ELBOW_TOL_DEG = 2.0;
const int PUSH_RESCAN_MM = 50;           // rescan if the head moved more than 5 cm

// Approximate motor speed at 100% (IQ smart motor ~120 rpm = 720 deg/s), used for timeouts
const double MOTOR_DEG_PER_SEC_AT_100 = 720.0;

// Game
const int SCREEN_W = 160;
const int SCREEN_H = 108;
const int NUM_LANES = 6;                 // grass, 4 road lanes, grass
const int LANE_W = 26;
const int LANE_X0 = 2;                   // left margin so 6 lanes are centered
const int PLAYER_SIZE = 10;
const int PLAYER_Y = 49;
const int CAR_W = 18;
const int CAR_H = 20;
const int NUM_CARS = 8;                  // 2 per road lane
const int FRAME_MS = 40;
const double IDLE_LIMIT_SEC = 20.0;
const double SPEEDUP = 1.15;             // cars get 15% faster after each crossing

// ===========================================================================
// DEVICES (ports are placeholders until measured)
// ===========================================================================

// BrainInertial is declared in the generated section above. It only measures
// the base if the brain turns with the base. If it doesn't, either add a
// separate gyro sensor on the turntable or set USE_GYRO_FOR_BASE = false.
motor BaseMotor     = motor(PORT1, false);  // true = reversed
motor ElbowMotor    = motor(PORT2, false);
motor CarriageMotor = motor(PORT3, false);
motor NudgerMotor   = motor(PORT4, false);
distance HeadSensor = distance(PORT5);
bumper NudgeBumper  = bumper(PORT6);
timer AlarmTimer    = timer();

// ===========================================================================
// GLOBAL STATE
// ===========================================================================

bool jammed = false;             // set by any move that times out
char jamPart[16] = "";           // which part jammed, for the screen
bool buttonRequested = false;    // a brain button was seen during nudging
bool pushDetected = false;       // nudge() saw the bumper held after retracting

// Game data
struct Car {
  double y;       // top edge in pixels
  int lane;       // 1 to 4 (road lanes)
  int height;     // pixels
  double speed;   // pixels per frame, + is down, - is up
};

// ===========================================================================
// SMALL HELPERS
// ===========================================================================

double clampD(double value, double low, double high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

bool anyButtonPressed() {
  return Brain.buttonLeft.pressing() || Brain.buttonRight.pressing() ||
         Brain.buttonCheck.pressing();
}

// Waits until no brain button is held (max 3 s), so one press isn't read twice.
void waitForRelease() {
  timer releaseTimer = timer();
  releaseTimer.clear();
  while (anyButtonPressed() && releaseTimer.time(seconds) < 3) {
    wait(20, msec);
  }
}

// Every screen update ends with render() because double buffering is on.
void showMessage(const char *line1, const char *line2) {
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(4, 35, "%s", line1);
  Brain.Screen.printAt(4, 65, "%s", line2);
  Brain.Screen.render();
}

void reportJam(const char *part) {
  jammed = true;
  strncpy(jamPart, part, sizeof(jamPart) - 1);
  jamPart[sizeof(jamPart) - 1] = '\0';
  showMessage(part, "jammed!");
}

// Base angle in degrees, 0 = straight ahead.
double baseAngle() {
  if (USE_GYRO_FOR_BASE) {
    return BrainInertial.rotation(degrees);  // rotation(), not heading(): no 359->0 jump
  }
  return BaseMotor.position(degrees) / BASE_RATIO;
}

// Starts a move to an absolute encoder position and watches it with a timeout.
// Returns false (and reports a jam) if the motor doesn't get there in time.
// The motor is passed by reference so the real device object is used.
bool moveMotorTo(motor &m, double targetMotorDeg, int speed, const char *name) {
  double travel = fabs(targetMotorDeg - m.position(degrees));
  double expectedSec = travel / (MOTOR_DEG_PER_SEC_AT_100 * speed / 100.0);
  double timeoutSec = 2.0 * expectedSec + 1.5;

  m.spinToPosition(targetMotorDeg, degrees, speed, percent, false);
  timer moveTimer = timer();
  moveTimer.clear();
  wait(20, msec);  // give the move time to start before checking isDone()

  while (!m.isDone()) {
    if (moveTimer.time(seconds) > timeoutSec) {
      m.stop(hold);
      reportJam(name);
      return false;
    }
    wait(20, msec);
  }
  return true;
}

// ===========================================================================
// THE SIX MAIN FUNCTIONS
// ===========================================================================

// Turns the base to an angle measured by the gyro (0 = straight ahead).
void rotateBase(double targetDeg, int speed) {
  targetDeg = clampD(targetDeg, BASE_MIN_DEG, BASE_MAX_DEG);

  timer moveTimer = timer();
  moveTimer.clear();

  while (true) {
    double current = baseAngle();
    double error = targetDeg - current;

    if (fabs(error) < BASE_TOLERANCE_DEG) {
      break;
    }
    // Safety: if the base runs past its range, the motor direction is probably
    // backwards (flip BaseMotor's reversed setting).
    if (current < BASE_MIN_DEG - RANGE_OVERRUN_DEG || current > BASE_MAX_DEG + RANGE_OVERRUN_DEG) {
      BaseMotor.stop(hold);
      reportJam("Base dir?");
      return;
    }
    if (moveTimer.time(seconds) > BASE_TIMEOUT_SEC) {
      BaseMotor.stop(hold);
      reportJam("Base");
      return;
    }

    // Slow down near the target so it doesn't overshoot
    double power = fmin(speed, 3.0 * fabs(error));
    if (power < BASE_MIN_POWER) {
      power = BASE_MIN_POWER;
    }
    if (error > 0) {
      BaseMotor.spin(forward, power, percent);
    } else {
      BaseMotor.spin(reverse, power, percent);
    }
    wait(20, msec);
  }
  BaseMotor.stop(hold);
}

// Moves the arm to an angle (0 = home, arm up; positive = down toward the bed).
void tiltElbow(double targetDeg, int speed) {
  targetDeg = clampD(targetDeg, ELBOW_MIN_DEG, ELBOW_MAX_DEG);
  if (moveMotorTo(ElbowMotor, targetDeg * ELBOW_RATIO, speed, "Elbow")) {
    ElbowMotor.stop(hold);  // the worm holds anyway, but hold to be sure
  }
}

// Slides the carriage to an absolute position (0 = fully pulled in).
void extendCarriage(double cm, int speed) {
  if (CM_PER_DEG <= 0) {
    showMessage("Set CM_PER_DEG", "in the code");
    jammed = true;
    strncpy(jamPart, "Carriage cfg", sizeof(jamPart) - 1);
    return;
  }
  cm = clampD(cm, CARRIAGE_MIN_CM, CARRIAGE_MAX_CM);
  moveMotorTo(CarriageMotor, cm / CM_PER_DEG, speed, "Carriage");
}

// Pokes the person `times` times and returns how many pokes touched (bumper pressed).
int nudge(int times, int speed) {
  int hits = 0;
  double stroke = clampD(NUDGE_DEG, NUDGER_MIN_DEG, NUDGER_MAX_DEG);

  for (int i = 0; i < times; i++) {
    if (anyButtonPressed()) {
      buttonRequested = true;
      return hits;
    }

    // Poke out while watching the bumper
    NudgerMotor.spinToPosition(stroke, degrees, speed, percent, false);
    timer pokeTimer = timer();
    pokeTimer.clear();
    wait(20, msec);
    bool hit = false;

    while (!NudgerMotor.isDone()) {
      if (NudgeBumper.pressing()) {
        hit = true;
        NudgerMotor.stop(brake);  // stop pushing right away
        break;
      }
      if (anyButtonPressed()) {
        buttonRequested = true;
      }
      if (pokeTimer.time(seconds) > 2) {
        NudgerMotor.stop(brake);
        break;
      }
      wait(10, msec);
    }
    if (hit) {
      hits++;
    }

    // Pull back (setTimeout in startUp stops this from hanging)
    NudgerMotor.spinToPosition(0, degrees, speed, percent, true);

    // Stretch goal: bumper still pressed after retracting = someone is pushing the arm
    timer holdTimer = timer();
    holdTimer.clear();
    while (NudgeBumper.pressing()) {
      if (holdTimer.time(seconds) > PUSH_HOLD_SEC) {
        pushDetected = true;
        return hits;
      }
      wait(20, msec);
    }

    // Pause about half a second, checking buttons so the person can stop it quickly
    for (int t = 0; t < 25; t++) {
      if (anyButtonPressed()) {
        buttonRequested = true;
        return hits;
      }
      wait(20, msec);
    }
  }
  return hits;
}

// Sweeps the base and finds the closest object in bed range.
// Returns its angle and sets closestMm (by reference); NOT_FOUND and -1 if nothing.
int scanBed(int &closestMm) {
  int scanDist[SCAN_STEPS];

  showMessage("Scanning...", "");
  for (int i = 0; i < SCAN_STEPS; i++) {
    int angle = -SCAN_DEG + i * SCAN_STEP_DEG;
    rotateBase(angle, 30);
    if (jammed) {
      closestMm = -1;
      return NOT_FOUND;
    }
    wait(100, msec);  // let the sensor settle

    if (HeadSensor.isObjectDetected()) {
      scanDist[i] = (int)HeadSensor.objectDistance(mm);
    } else {
      scanDist[i] = -1;
    }
  }

  // Search the array for the closest reading inside the bed range
  int bestIndex = -1;
  for (int i = 0; i < SCAN_STEPS; i++) {
    int d = scanDist[i];
    if (d >= MIN_HEAD_MM && d <= MAX_HEAD_MM) {
      if (bestIndex == -1 || d < scanDist[bestIndex]) {
        bestIndex = i;
      }
    }
  }

  if (bestIndex == -1) {
    closestMm = -1;
    return NOT_FOUND;
  }
  closestMm = scanDist[bestIndex];
  return -SCAN_DEG + bestIndex * SCAN_STEP_DEG;
}

// --- game helpers ---

int laneX(int lane) {
  return LANE_X0 + lane * LANE_W;
}

void setCarColor(int lane) {
  if (lane == 1) {
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(red);
  } else if (lane == 2) {
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.setPenColor(yellow);
  } else if (lane == 3) {
    Brain.Screen.setFillColor(orange);
    Brain.Screen.setPenColor(orange);
  } else {
    Brain.Screen.setFillColor(purple);
    Brain.Screen.setPenColor(purple);
  }
}

void drawGame(Car cars[], int playerLane, int score, int rounds) {
  Brain.Screen.setFillColor(black);
  Brain.Screen.clearScreen();

  // Grass on both sides
  Brain.Screen.setFillColor(green);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawRectangle(0, 0, laneX(1), SCREEN_H);
  Brain.Screen.drawRectangle(laneX(NUM_LANES - 1), 0, SCREEN_W - laneX(NUM_LANES - 1), SCREEN_H);

  // Lane dividers between road lanes
  Brain.Screen.setPenColor(white);
  for (int lane = 2; lane < NUM_LANES - 1; lane++) {
    Brain.Screen.drawLine(laneX(lane), 0, laneX(lane), SCREEN_H);
  }

  // Cars
  for (int i = 0; i < NUM_CARS; i++) {
    setCarColor(cars[i].lane);
    Brain.Screen.drawRectangle(laneX(cars[i].lane) + (LANE_W - CAR_W) / 2,
                               (int)cars[i].y, CAR_W, cars[i].height);
  }

  // Player
  Brain.Screen.setFillColor(cyan);
  Brain.Screen.setPenColor(cyan);
  Brain.Screen.drawRectangle(laneX(playerLane) + (LANE_W - PLAYER_SIZE) / 2, PLAYER_Y,
                             PLAYER_SIZE, PLAYER_SIZE);

  // Score on the left grass, goal on the right grass
  Brain.Screen.setFillColor(green);
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(laneX(0) + 4, 18, "%d", score);
  Brain.Screen.printAt(laneX(NUM_LANES - 1) + 4, 18, "%d", rounds);

  Brain.Screen.render();
}

// Crossy road: cross the road `rounds` times without being hit.
// Returns false on a crash or 20 s with no input.
bool awakeCheck(int rounds) {
  waitForRelease();  // the press that started the game shouldn't move the player
  srand((unsigned int)AlarmTimer.time(msec));

  // 3, 2, 1 so a sleepy person isn't hit instantly
  for (int n = 3; n >= 1; n--) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(20, 30, "Cross the road!");
    Brain.Screen.printAt(75, 70, "%d", n);
    Brain.Screen.render();
    wait(1, seconds);
  }

  // Set up the cars: 2 per road lane, neighbouring lanes go opposite ways
  Car cars[NUM_CARS];
  for (int lane = 1; lane <= 4; lane++) {
    double dir = (lane % 2 == 0) ? 1.0 : -1.0;
    double laneSpeed = dir * (0.8 + (rand() % 9) / 10.0);  // 0.8 to 1.6 px per frame
    int firstY = rand() % (SCREEN_H / 2);
    for (int k = 0; k < 2; k++) {
      int i = (lane - 1) * 2 + k;
      cars[i].lane = lane;
      cars[i].height = CAR_H;
      cars[i].speed = laneSpeed;
      cars[i].y = firstY + k * (SCREEN_H / 2 + CAR_H / 2);
    }
  }

  int playerLane = 0;
  int score = 0;
  bool prevLeft = Brain.buttonLeft.pressing();
  bool prevRight = Brain.buttonRight.pressing();
  timer idleTimer = timer();
  idleTimer.clear();

  while (true) {
    // Buttons: only a NEW press moves the player
    bool left = Brain.buttonLeft.pressing();
    bool right = Brain.buttonRight.pressing();
    if (right && !prevRight) {
      playerLane++;
      idleTimer.clear();
    }
    if (left && !prevLeft) {
      if (playerLane > 0) {
        playerLane--;
      }
      idleTimer.clear();
    }
    prevLeft = left;
    prevRight = right;

    // Reached the far grass
    if (playerLane >= NUM_LANES - 1) {
      score++;
      if (score >= rounds) {
        drawGame(cars, NUM_LANES - 1, score, rounds);
        wait(500, msec);
        return true;
      }
      playerLane = 0;
      for (int i = 0; i < NUM_CARS; i++) {
        cars[i].speed *= SPEEDUP;
      }
    }

    // Move the cars and wrap them around the screen
    for (int i = 0; i < NUM_CARS; i++) {
      cars[i].y += cars[i].speed;
      if (cars[i].y > SCREEN_H) {
        cars[i].y = -cars[i].height;
      } else if (cars[i].y < -cars[i].height) {
        cars[i].y = SCREEN_H;
      }
    }

    // Collisions: same lane and the boxes overlap vertically
    for (int i = 0; i < NUM_CARS; i++) {
      if (cars[i].lane == playerLane &&
          cars[i].y < PLAYER_Y + PLAYER_SIZE &&
          cars[i].y + cars[i].height > PLAYER_Y) {
        drawGame(cars, playerLane, score, rounds);
        wait(500, msec);
        return false;
      }
    }

    // Fell back asleep?
    if (idleTimer.time(seconds) > IDLE_LIMIT_SEC) {
      return false;
    }

    drawGame(cars, playerLane, score, rounds);
    wait(FRAME_MS, msec);
  }
}

// ===========================================================================
// OTHER HELPERS
// ===========================================================================

void startUp() {
  Brain.Screen.render();  // turn on double buffering from the start
  showMessage("Keep robot still", "Calibrating...");

  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {
    wait(50, msec);
  }
  BrainInertial.setRotation(0, degrees);

  BaseMotor.setPosition(0, degrees);
  ElbowMotor.setPosition(0, degrees);
  CarriageMotor.setPosition(0, degrees);
  NudgerMotor.setPosition(0, degrees);

  BaseMotor.setStopping(hold);
  ElbowMotor.setStopping(hold);
  CarriageMotor.setStopping(hold);
  NudgerMotor.setStopping(brake);

  BaseMotor.setMaxTorque(50, percent);
  ElbowMotor.setMaxTorque(50, percent);
  CarriageMotor.setMaxTorque(50, percent);
  NudgerMotor.setMaxTorque(40, percent);

  // Only applies to waiting (blocking) moves, like the nudger pulling back
  NudgerMotor.setTimeout(2, seconds);
}

// Left adds an hour, right adds a minute, check confirms.
// Returns seconds until the alarm (the brain has no time-of-day clock).
int setAlarm() {
  int hours = 0;
  int minutes = 1;  // 1 minute default for the demo

  waitForRelease();
  bool prevLeft = false;
  bool prevRight = false;
  bool prevCheck = false;

  while (true) {
    bool left = Brain.buttonLeft.pressing();
    bool right = Brain.buttonRight.pressing();
    bool check = Brain.buttonCheck.pressing();

    if (left && !prevLeft) {
      hours = (hours + 1) % 24;
    }
    if (right && !prevRight) {
      minutes = (minutes + 1) % 60;
    }
    if (check && !prevCheck) {
      break;
    }
    prevLeft = left;
    prevRight = right;
    prevCheck = check;

    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(4, 20, "Set alarm");
    Brain.Screen.printAt(4, 50, "in %02d h %02d min", hours, minutes);
    Brain.Screen.printAt(4, 80, "< h  m >  check");
    Brain.Screen.render();
    wait(20, msec);
  }
  waitForRelease();
  return hours * 3600 + minutes * 60;
}

// Clears the timer and shows a countdown until the alarm.
void waitForAlarm(int seconds) {
  AlarmTimer.clear();
  while (true) {
    int elapsed = (int)(AlarmTimer.time(msec) / 1000);
    int left = seconds - elapsed;
    if (left <= 0) {
      break;
    }
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(4, 35, "Alarm in");
    Brain.Screen.printAt(4, 65, "%02d:%02d:%02d", left / 3600, (left / 60) % 60, left % 60);
    Brain.Screen.render();
    wait(200, msec);
  }
  showMessage("WAKE UP!", "");
}

// How far to extend the carriage for a head measured at distanceMm.
// The 2 cm margin stops it just short, since tilting the elbow moves the tip further.
double reachFor(int distanceMm) {
  double cm = distanceMm / 10.0 - SENSOR_TO_TIP_CM - 2;
  return clampD(cm, CARRIAGE_MIN_CM, CARRIAGE_MAX_CM);
}

// Carriage in, then elbow up, then base to 0, so it never swings while extended.
void homeAll() {
  extendCarriage(0, 40);
  tiltElbow(0, 50);
  rotateBase(0, 30);
}

// After a jam: pull the carriage in and lift the elbow, but leave the base alone.
void stopForJam() {
  char part[16];
  strncpy(part, jamPart, sizeof(part));
  part[sizeof(part) - 1] = '\0';

  jammed = false;  // let the retract moves run
  NudgerMotor.stop(brake);
  extendCarriage(0, 30);
  tiltElbow(0, 40);
  showMessage(part, "jammed. Stopped.");
}

// Moves to the head: base, then carriage, then elbow.
void reachHead(int headAngle, int headMm, double &reachCm) {
  rotateBase(headAngle, 30);
  if (jammed) return;
  reachCm = reachFor(headMm);
  extendCarriage(reachCm, 40);
  if (jammed) return;
  tiltElbow(NUDGE_ANGLE, 50);
}

// Stretch goal: the person held the arm away. Back off, return to the saved
// head angle with the gyro, and rescan only if the person moved.
// Returns false if the head can't be found again.
bool recoverFromPush(int &headAngle, int &headMm, double &reachCm) {
  showMessage("Pushed away", "Backing off...");
  pushDetected = false;

  extendCarriage(reachCm - 3, 30);
  if (jammed) return true;

  // Wait for the person to let go (max 10 s)
  timer letGo = timer();
  letGo.clear();
  while (NudgeBumper.pressing() && letGo.time(seconds) < 10) {
    wait(50, msec);
  }

  // Base: use the gyro, since a push can back-drive the base gears
  if (fabs(baseAngle() - headAngle) > PUSH_ANGLE_TOL_DEG) {
    rotateBase(headAngle, 20);
    if (jammed) return true;
  }

  // Elbow: the worm shouldn't move, but check
  double elbowDeg = ElbowMotor.position(degrees) / ELBOW_RATIO;
  if (fabs(elbowDeg - NUDGE_ANGLE) > PUSH_ELBOW_TOL_DEG) {
    tiltElbow(NUDGE_ANGLE, 40);
    if (jammed) return true;
  }

  // Rescan only if the head looks different from before
  bool seen = HeadSensor.isObjectDetected();
  int nowMm = seen ? (int)HeadSensor.objectDistance(mm) : -1;
  if (!seen || abs(nowMm - headMm) > PUSH_RESCAN_MM) {
    extendCarriage(0, 40);
    tiltElbow(0, 50);
    if (jammed) return true;
    int newMm = 0;
    int newAngle = scanBed(newMm);
    if (jammed) return true;
    if (newAngle == NOT_FOUND) {
      return false;
    }
    headAngle = newAngle;
    headMm = newMm;
    reachHead(headAngle, headMm, reachCm);
  } else {
    extendCarriage(reachCm, 30);
  }
  showMessage("WAKE UP!", "");
  return true;
}

// ===========================================================================
// MAIN
// ===========================================================================

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  startUp();
  int alarmSec = setAlarm();
  waitForAlarm(alarmSec);

  // Find the head (scan once more if nothing found)
  int headMm = 0;
  int headAngle = scanBed(headMm);  // headMm filled in by reference
  if (headAngle == NOT_FOUND && !jammed) {
    headAngle = scanBed(headMm);
  }
  if (jammed) { stopForJam(); return 0; }
  if (headAngle == NOT_FOUND) {
    homeAll();
    showMessage("No head found", "Going home");
    return 0;
  }

  // Reach
  double reachCm = 0;
  reachHead(headAngle, headMm, reachCm);
  if (jammed) { stopForJam(); return 0; }
  showMessage("WAKE UP!", "");

  // Wake loop: nudge until a button is pressed and the game is won
  bool awake = false;
  while (!awake) {
    int hits = nudge(3, 60);

    if (pushDetected) {
      if (!recoverFromPush(headAngle, headMm, reachCm)) {
        homeAll();
        showMessage("Lost the head", "Going home");
        return 0;
      }
      if (jammed) { stopForJam(); return 0; }
      continue;
    }

    if (hits == 0 && !buttonRequested && reachCm < CARRIAGE_MAX_CM) {
      reachCm += 1;
      extendCarriage(reachCm, 20);
    }
    if (jammed) { stopForJam(); return 0; }

    if (buttonRequested || anyButtonPressed()) {
      buttonRequested = false;
      awake = awakeCheck(5);
      if (!awake) {
        showMessage("Not awake yet!", "");
        wait(1500, msec);
        showMessage("WAKE UP!", "");
      }
    }
  }

  // Shut down
  homeAll();
  if (jammed) { stopForJam(); return 0; }
  showMessage("Good morning!", "");
  wait(3, seconds);
  return 0;
}
