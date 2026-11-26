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
const double CM_PER_DEG = 0.0144;           // MEASURE: spin carriage motor 360 deg, measure travel (cm), divide by 360

// Motor ranges (MEASURE every one). Each is in the joint's own units, measured
// from the home pose, where every encoder is zeroed. Every move is clamped to these.
const double BASE_MIN_DEG = -90.0;       // base degrees, 0 = straight ahead (cables wrap past this)
const double BASE_MAX_DEG = 90.0;
const double ELBOW_MIN_DEG = -7.5;        // arm degrees, 0 = home (arm up), + = down toward the bed
const double ELBOW_MAX_DEG = 27.0;
const double CARRIAGE_MIN_CM = 0.0;      // cm along the arm, 0 = fully pulled in
const double CARRIAGE_MAX_CM = 18.0;
const double NUDGER_MIN_DEG = 0.0;       // nudger motor degrees, 0 = retracted
const double NUDGER_MAX_DEG = 104.0;
const double RANGE_OVERRUN_DEG = 10.0;   // base past its range by this much = stop (motor direction wrong)

// Where the distance sensor sits relative to the nudger, seen from above
const double SENSOR_TO_TIP_CM = 6.0;     // nudger tip is this far ahead of the sensor, along the arm (carriage in)
const double SENSOR_SIDE_CM = 8.0;       // sensor is this far to the side of the nudger's line.
                                         // + = sensor on the robot's right (the way positive base angles turn).
                                         // If the nudger ends up FURTHER from the head, flip the sign.
const double SENSOR_FROM_PIVOT_CM = 12.0; // base's centre of rotation to the sensor's front, along the arm
const double RAD_TO_DEG = 180.0 / 3.14159265;

// Tuning (must sit inside the ranges above)
const double NUDGE_ANGLE = 40.0;         // elbow angle used for nudging (MEASURE)
const double NUDGE_EXTRA_DEG = 20.0;     // keep pushing this far past first contact so the nudge is felt (TUNE)

// Alarm: true = demo timing, where the clock-style "07:30" means 7 min 30 s
// instead of 7 h 30 min. Set to false for real hours and minutes.
const bool DEMO_ALARM = true;

// Motor safety: a jammed motor draws more current (class rule: keep it under 1 A)
const double MAX_CURRENT_AMP = 2.0;
const int CURRENT_IGNORE_MS = 200;       // ignore the current spike when a motor first starts

// Base control
const bool USE_GYRO_FOR_BASE = true;     // false = use base motor encoder instead of the gyro
const double BASE_TOLERANCE_DEG = 2.0;
const double BASE_TIMEOUT_SEC = 12.0;
const double BASE_SLOW_ZONE_DEG = 10.0;  // slow down this far from the target so it doesn't overshoot
const int BASE_SLOW_SPEED = 10;          // percent

// Head scan
const int SCAN_DEG = 90;                 // sweep from -SCAN_DEG to +SCAN_DEG
const int SCAN_STEP_DEG = 5;
const int SCAN_STEPS = 2 * SCAN_DEG / SCAN_STEP_DEG + 1;  // 37 readings
const int MIN_HEAD_MM = 150;              // ignore readings outside the bed range (MEASURE)
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
const int PLAYER_START_Y = 49;           // player starts in the middle of the screen
const int PLAYER_STEP = 12;              // pixels moved per up/down press (TUNE)
const int CAR_W = 18;
const int CAR_H = 20;
const int NUM_CARS = 8;                  // 2 per road lane
const int FRAME_MS = 40;
const double IDLE_LIMIT_SEC = 20.0;
const double SPEEDUP = 1.15;             // cars get 15% faster after each crossing
const int GRASS_RGB = 0x2E7D32;          // darker green for the grass (hex RGB, used as color(GRASS_RGB))

// ===========================================================================
// DEVICES (ports are placeholders until measured)
// ===========================================================================

// BrainInertial is declared in the generated section above. It only measures
// the base if the brain turns with the base. If it doesn't, either add a
// separate gyro sensor on the turntable or set USE_GYRO_FOR_BASE = false.
motor BaseMotor(PORT1, bool(forward));      // bool(reverse) flips the direction
motor ElbowMotor(PORT2, bool(forward));
motor CarriageMotor(PORT3, bool(forward));
motor NudgerMotor(PORT4, bool(forward));
distance HeadSensor(PORT5);
bumper NudgeBumper(PORT6);

// No global variables: main() owns the program's state (jammed, buttonRequested,
// pushDetected) and passes it to functions by reference.

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
  timer releaseTimer;
  releaseTimer.clear();
  while (anyButtonPressed() && releaseTimer.time(seconds) < 3)
  {}
}

// Every screen update ends with render() because double buffering is on.
void showMessage(const char *line1, const char *line2) {
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.clearScreen();
  // The false (no text background) picks the right printAt; without it the
  // call is ambiguous because "%s" could also be read as a bool
  Brain.Screen.printAt(4, 35, false, "%s", line1);
  Brain.Screen.printAt(4, 65, false, "%s", line2);
  Brain.Screen.render();
}

// Marks a jam (jammed is main's flag, passed by reference) and shows which part
// jammed. The message stays on screen through the shutdown.
void reportJam(const char *part, bool &jammed) {
  jammed = true;
  showMessage(part, "jammed!");
}

// Base angle in degrees, 0 = straight ahead.
double baseAngle() {
  if (USE_GYRO_FOR_BASE) {
    return BrainInertial.rotation(degrees);  // rotation(), not heading(): no 359->0 jump
  }
  return BaseMotor.position(degrees) / BASE_RATIO;
}

// True while a moving motor looks healthy: not past its timeout, and not drawing
// jam-level current (the start-up spike in the first CURRENT_IGNORE_MS is ignored).
bool motorOk(motor &m, timer &moveTimer, double timeoutSec) {
  if (moveTimer.time(seconds) > timeoutSec) return false;
  if (moveTimer.time(msec) > CURRENT_IGNORE_MS && m.current(amp) >= MAX_CURRENT_AMP) return false;
  return true;
}

// True if the base has run well past its range (motor direction is probably wrong).
bool baseOutOfRange() {
  double angle = baseAngle();
  return angle < BASE_MIN_DEG - RANGE_OVERRUN_DEG || angle > BASE_MAX_DEG + RANGE_OVERRUN_DEG;
}

// True while the base move looks healthy: in range, not timed out, not jammed.
bool baseOk(timer &moveTimer) {
  return !baseOutOfRange() && motorOk(BaseMotor, moveTimer, BASE_TIMEOUT_SEC);
}

// Spins a motor until its encoder reaches targetMotorDeg, then holds.
// Returns false (and reports a jam) on a timeout or a current spike.
// The motor is passed by reference so the real device object is used.
bool moveMotorTo(motor &m, double targetMotorDeg, int speed, const char *name, bool &jammed) {
  double start = m.position(degrees);
  if (fabs(targetMotorDeg - start) < 1) {
    return true;  // already there
  }
  double expectedSec = fabs(targetMotorDeg - start) / (MOTOR_DEG_PER_SEC_AT_100 * speed / 100.0);
  double timeoutSec = 2.0 * expectedSec + 1.5;

  timer moveTimer;
  moveTimer.clear();
  bool reached = false;

  if (targetMotorDeg > start) {
    m.spin(forward, speed, percent);
    while (m.position(degrees) < targetMotorDeg && motorOk(m, moveTimer, timeoutSec))
    {}
    reached = m.position(degrees) >= targetMotorDeg;
  } else {
    m.spin(reverse, speed, percent);
    while (m.position(degrees) > targetMotorDeg && motorOk(m, moveTimer, timeoutSec))
    {}
    reached = m.position(degrees) <= targetMotorDeg;
  }
  m.stop(hold);

  if (!reached) {
    reportJam(name, jammed);
  }
  return reached;
}

// ===========================================================================
// THE SIX MAIN FUNCTIONS
// ===========================================================================

// Turns the base to an angle measured by the gyro (0 = straight ahead).
void rotateBase(double targetDeg, int speed, bool &jammed) {
  targetDeg = clampD(targetDeg, BASE_MIN_DEG, BASE_MAX_DEG);
  double start = baseAngle();
  if (fabs(targetDeg - start) < BASE_TOLERANCE_DEG) {
    return;  // already there
  }

  timer moveTimer;
  moveTimer.clear();
  bool reached = false;

  if (targetDeg > start) {
    // Fast until close, then slow so it doesn't overshoot
    BaseMotor.spin(forward, speed, percent);
    while (baseAngle() < targetDeg - BASE_SLOW_ZONE_DEG && baseOk(moveTimer))
    {}
    BaseMotor.spin(forward, BASE_SLOW_SPEED, percent);
    while (baseAngle() < targetDeg && baseOk(moveTimer))
    {}
    reached = baseAngle() >= targetDeg;
  } else {
    BaseMotor.spin(reverse, speed, percent);
    while (baseAngle() > targetDeg + BASE_SLOW_ZONE_DEG && baseOk(moveTimer))
    {}
    BaseMotor.spin(reverse, BASE_SLOW_SPEED, percent);
    while (baseAngle() > targetDeg && baseOk(moveTimer))
    {}
    reached = baseAngle() <= targetDeg;
  }
  BaseMotor.stop(hold);

  if (!reached) {
    if (baseOutOfRange()) {
      reportJam("Base dir?", jammed);  // ran past its range: flip BaseMotor's direction
    } else {
      reportJam("Base", jammed);
    }
  }
}

// Moves the arm to an angle (0 = home, arm up; positive = down toward the bed).
// The worm gear holds the arm when the motor stops.
void tiltElbow(double targetDeg, int speed, bool &jammed) {
  targetDeg = clampD(targetDeg, ELBOW_MIN_DEG, ELBOW_MAX_DEG);
  moveMotorTo(ElbowMotor, targetDeg * ELBOW_RATIO, speed, "Elbow", jammed);
}

// Slides the carriage to an absolute position (0 = fully pulled in).
void extendCarriage(double cm, int speed, bool &jammed) {
  if (CM_PER_DEG <= 0) {
    showMessage("Set", "CM_PER_DEG");
    jammed = true;  // stop safely: can't convert cm to motor degrees yet
    return;
  }
  cm = clampD(cm, CARRIAGE_MIN_CM, CARRIAGE_MAX_CM);
  moveMotorTo(CarriageMotor, cm / CM_PER_DEG, speed, "Carriage", jammed);
}

// Pokes the person up to `times` times and returns how many pokes touched.
// Each poke goes out until the bumper touches, pushes NUDGE_EXTRA_DEG further so
// the nudge is felt, then pulls back. If a poke reaches the end of the nudger's
// range without touching, it stops early so main() can move the carriage 1 cm
// closer and try again. So a full round of 3 returns 3; anything less is a miss.
// buttonRequested and pushDetected are main's flags, set here by reference.
int nudge(int times, int speed, bool &buttonRequested, bool &pushDetected) {
  int hits = 0;
  int pokes = 0;
  bool stopRound = false;  // set when the round should end early

  if (anyButtonPressed()) {
    buttonRequested = true;
    stopRound = true;
  }

  while (pokes < times && !stopRound) {
    pokes++;

    // Poke out until the bumper touches, the nudger reaches the end of its
    // range, a button is pressed, or the motor jams
    timer pokeTimer;
    pokeTimer.clear();
    NudgerMotor.spin(forward, speed, percent);
    while (!NudgeBumper.pressing() && NudgerMotor.position(degrees) < NUDGER_MAX_DEG &&
           !anyButtonPressed() && motorOk(NudgerMotor, pokeTimer, 2))
    {}

    bool touched = NudgeBumper.pressing();
    if (touched) {
      // Push a little more past contact (never past the range) so it's noticeable
      double pushTo = clampD(NudgerMotor.position(degrees) + NUDGE_EXTRA_DEG,
                             NUDGER_MIN_DEG, NUDGER_MAX_DEG);
      pokeTimer.clear();
      while (NudgerMotor.position(degrees) < pushTo && motorOk(NudgerMotor, pokeTimer, 1))
      {}
      hits++;
    }
    NudgerMotor.stop(brake);

    if (anyButtonPressed()) {
      buttonRequested = true;
    }

    // Pull back to 0
    pokeTimer.clear();
    NudgerMotor.spin(reverse, speed, percent);
    while (NudgerMotor.position(degrees) > NUDGER_MIN_DEG && motorOk(NudgerMotor, pokeTimer, 2))
    {}
    NudgerMotor.stop(brake);

    // Stretch goal: bumper still pressed after retracting = someone is pushing the arm
    timer holdTimer;
    holdTimer.clear();
    while (NudgeBumper.pressing() && holdTimer.time(seconds) < PUSH_HOLD_SEC)
    {}
    if (NudgeBumper.pressing()) {
      pushDetected = true;
    }

    // End the round early if: pushed away, a button was pressed, or the poke
    // missed (reached the end of the range without touching, so the carriage
    // needs to move closer)
    stopRound = pushDetected || buttonRequested || !touched;

    // Pause half a second before the next poke, but stop early if a button is pressed
    if (!stopRound && pokes < times) {
      timer pauseTimer;
      pauseTimer.clear();
      while (pauseTimer.time(msec) < 500 && !anyButtonPressed())
      {}
      if (anyButtonPressed()) {
        buttonRequested = true;
        stopRound = true;
      }
    }
  }
  return hits;
}

// Sweeps the base from -SCAN_DEG to +SCAN_DEG in SCAN_STEP_DEG steps and stops
// at the FIRST object inside the bed range (MIN_HEAD_MM to MAX_HEAD_MM), which
// is assumed to be the head. The base is left pointing at it.
// Returns its angle and sets headMm (by reference); NOT_FOUND and -1 if nothing
// was in range anywhere in the sweep.
int scanBed(int &headMm, bool &jammed) {
  int headAngle = NOT_FOUND;
  headMm = -1;
  bool found = false;

  // Keep sweeping until something in range is seen, the sweep ends, or the base jams
  showMessage("Scanning", "");
  int step = 0;
  while (step < SCAN_STEPS && !found && !jammed) {
    int angle = -SCAN_DEG + step * SCAN_STEP_DEG;
    rotateBase(angle, 30, jammed);
    if (!jammed) {
      wait(100, msec);  // let the sensor settle
      if (HeadSensor.isObjectDetected()) {
        int d = (int)HeadSensor.objectDistance(mm);
        if (d >= MIN_HEAD_MM && d <= MAX_HEAD_MM) {
          found = true;  // ends the loop: the base stays pointing at the head
          headAngle = angle;
          headMm = d;
        }
      }
    }
    step++;
  }
  return headAngle;
}

// --- game helpers ---

int laneX(int lane) {
  return LANE_X0 + lane * LANE_W;
}

void drawGame(Car cars[], int playerLane, int playerY, int score, int rounds) {
  Brain.Screen.setFillColor(black);
  Brain.Screen.clearScreen();

  // Grass on both sides (a custom colour needs color(...); a bare number would be read as a hue)
  Brain.Screen.setFillColor(color(GRASS_RGB));
  Brain.Screen.setPenColor(color(GRASS_RGB));
  Brain.Screen.drawRectangle(0, 0, laneX(1), SCREEN_H);
  Brain.Screen.drawRectangle(laneX(NUM_LANES - 1), 0, SCREEN_W - laneX(NUM_LANES - 1), SCREEN_H);

  // Lane dividers between road lanes
  Brain.Screen.setPenColor(white);
  for (int lane = 2; lane < NUM_LANES - 1; lane++) {
    Brain.Screen.drawLine(laneX(lane), 0, laneX(lane), SCREEN_H);
  }

  // Cars (all red)
  Brain.Screen.setFillColor(red);
  Brain.Screen.setPenColor(red);
  for (int i = 0; i < NUM_CARS; i++) {
    Brain.Screen.drawRectangle(laneX(cars[i].lane) + (LANE_W - CAR_W) / 2,
                               (int)cars[i].y, CAR_W, cars[i].height);
  }

  // Player
  Brain.Screen.setFillColor(cyan);
  Brain.Screen.setPenColor(cyan);
  Brain.Screen.drawRectangle(laneX(playerLane) + (LANE_W - PLAYER_SIZE) / 2, playerY,
                             PLAYER_SIZE, PLAYER_SIZE);

  // Score on the left grass, goal on the right grass
  Brain.Screen.setFillColor(color(GRASS_RGB));
  Brain.Screen.setPenColor(black);
  Brain.Screen.printAt(laneX(0) + 4, 18, "%d", score);
  Brain.Screen.printAt(laneX(NUM_LANES - 1) + 4, 18, "%d", rounds);

  Brain.Screen.render();
}

// Crossy road: cross the road `rounds` times without being hit.
// Controls: check = step forward one lane, left = move up, right = move down.
// There's no moving backwards: dodge cars by moving up or down in the lane.
// Returns false on a crash or 20 s with no input.
bool awakeCheck(int rounds) {
  waitForRelease();  // the press that started the game shouldn't move the player
  // (random numbers are already seeded by vexcodeInit() at the start of main)

  // 3, 2, 1 so a sleepy person isn't hit instantly
  for (int n = 3; n >= 1; n--) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(4, 25, "Cross the");
    Brain.Screen.printAt(4, 45, "road!");
    Brain.Screen.printAt(75, 85, "%d", n);
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
  int playerY = PLAYER_START_Y;
  int score = 0;
  bool prevLeft = Brain.buttonLeft.pressing();
  bool prevRight = Brain.buttonRight.pressing();
  bool prevCheck = Brain.buttonCheck.pressing();
  timer idleTimer;
  idleTimer.clear();

  bool gameOver = false;  // set on a win, a crash, or no input
  bool won = false;

  while (!gameOver) {
    // Buttons: only a NEW press moves the player
    bool left = Brain.buttonLeft.pressing();
    bool right = Brain.buttonRight.pressing();
    bool check = Brain.buttonCheck.pressing();
    if (check && !prevCheck) {
      playerLane++;  // step forward one lane
      idleTimer.clear();
    }
    if (left && !prevLeft) {
      // up, but not off the top of the screen
      playerY = (int)clampD(playerY - PLAYER_STEP, 0, SCREEN_H - PLAYER_SIZE);
      idleTimer.clear();
    }
    if (right && !prevRight) {
      // down, but not off the bottom of the screen
      playerY = (int)clampD(playerY + PLAYER_STEP, 0, SCREEN_H - PLAYER_SIZE);
      idleTimer.clear();
    }
    prevLeft = left;
    prevRight = right;
    prevCheck = check;

    // Reached the far grass
    if (playerLane >= NUM_LANES - 1) {
      score++;
      if (score >= rounds) {
        won = true;
        gameOver = true;
      } else {
        playerLane = 0;
        for (int i = 0; i < NUM_CARS; i++) {
          cars[i].speed *= SPEEDUP;
        }
      }
    }

    // Move the cars and wrap them around the screen
    if (!gameOver) {
      for (int i = 0; i < NUM_CARS; i++) {
        cars[i].y += cars[i].speed;
        if (cars[i].y > SCREEN_H) {
          cars[i].y = -cars[i].height;
        } else if (cars[i].y < -cars[i].height) {
          cars[i].y = SCREEN_H;
        }
      }
    }

    // Collisions: same lane and the boxes overlap vertically
    bool hit = false;
    for (int i = 0; i < NUM_CARS; i++) {
      if (cars[i].lane == playerLane &&
          cars[i].y < playerY + PLAYER_SIZE &&
          cars[i].y + cars[i].height > playerY) {
        hit = true;
      }
    }
    if (hit && !won) {
      gameOver = true;
    }

    // Fell back asleep?
    if (!gameOver && idleTimer.time(seconds) > IDLE_LIMIT_SEC) {
      gameOver = true;
    }

    drawGame(cars, playerLane, playerY, score, rounds);
    if (gameOver) {
      wait(500, msec);  // show the final frame for a moment
    } else {
      wait(FRAME_MS, msec);
    }
  }
  return won;
}

// ===========================================================================
// OTHER HELPERS
// ===========================================================================

void startUp() {
  Brain.Screen.render();  // turn on double buffering from the start
  showMessage("Keep still", "Calibrate");

  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating())
  {}
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
}

// Left adds to the first number, right adds to the second, check confirms.
// The alarm shows like a clock, "07:30". With DEMO_ALARM that means 7 min 30 s,
// otherwise 7 h 30 min. Returns seconds until the alarm (the brain has no
// time-of-day clock).
int setAlarm() {
  int hours = 0;
  int minutes = DEMO_ALARM ? 30 : 1;  // default: 30 s in demo mode, 1 min otherwise
  bool confirmed = false;

  waitForRelease();
  while (!confirmed) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(4, 20, "Set alarm");
    Brain.Screen.printAt(4, 50, "%02d:%02d", hours, minutes);
    Brain.Screen.printAt(4, 80, "<   > chk");
    Brain.Screen.render();

    // Wait for a press, act on it, then wait for the release (so it counts once)
    while (!anyButtonPressed())
    {}
    if (Brain.buttonLeft.pressing()) {
      hours = (hours + 1) % 24;
    } else if (Brain.buttonRight.pressing()) {
      minutes = (minutes + 1) % 60;
    } else if (Brain.buttonCheck.pressing()) {
      confirmed = true;
    }
    while (anyButtonPressed())
    {}
  }
  if (DEMO_ALARM) {
    return hours * 60 + minutes;  // "07:30" = 7 min 30 s
  }
  return hours * 3600 + minutes * 60;
}

// Clears the timer and shows a countdown until the alarm.
void waitForAlarm(int seconds) {
  timer alarmTimer;
  alarmTimer.clear();
  int left = seconds;
  while (left > 0) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(4, 35, "Alarm in");
    if (DEMO_ALARM) {
      Brain.Screen.printAt(4, 65, "%02d:%02d", left / 60, left % 60);  // same style as setAlarm
    } else {
      Brain.Screen.printAt(4, 65, "%02d:%02d:%02d", left / 3600, (left / 60) % 60, left % 60);
    }
    Brain.Screen.render();
    wait(200, msec);
    left = seconds - (int)(alarmTimer.time(msec) / 1000);
  }
  showMessage("WAKE UP!", "");
}

// The sensor is SENSOR_SIDE_CM to the side of the nudger, so when the sensor
// points at the head, the nudger points beside it. This is the extra base turn
// (degrees) that lines the NUDGER up with the head instead.
// Seen from above, with the base's centre of rotation at (0, 0) and the arm
// pointing along x: the head is at (SENSOR_FROM_PIVOT_CM + distance, SENSOR_SIDE_CM).
double aimOffsetDeg(int headMm) {
  double headAlongArm = SENSOR_FROM_PIVOT_CM + headMm / 10.0;
  return atan2(SENSOR_SIDE_CM, headAlongArm) * RAD_TO_DEG;
}

// How far to extend the carriage for a head the sensor measured at distanceMm,
// once the base has turned so the nudger points at it (see aimOffsetDeg).
// The 2 cm margin stops it just short, since tilting the elbow moves the tip further.
double reachFor(int distanceMm) {
  double headAlongArm = SENSOR_FROM_PIVOT_CM + distanceMm / 10.0;
  double pivotToHead = sqrt(headAlongArm * headAlongArm + SENSOR_SIDE_CM * SENSOR_SIDE_CM);
  double pivotToTip = SENSOR_FROM_PIVOT_CM + SENSOR_TO_TIP_CM;  // tip with the carriage pulled in
  double cm = pivotToHead - pivotToTip - 2;
  return clampD(cm, CARRIAGE_MIN_CM, CARRIAGE_MAX_CM);
}

// Carriage in, then elbow up, then base to 0, so it never swings while extended.
void homeAll(bool &jammed) {
  extendCarriage(0, 40, jammed);
  tiltElbow(0, 50, jammed);
  rotateBase(0, 30, jammed);
}

// After a jam: pull the carriage in and lift the elbow, but leave the base alone.
// The "<part> jammed!" message from reportJam() stays on the screen.
void stopForJam() {
  bool retractJammed = false;  // separate flag so the retract moves still run
  NudgerMotor.stop(brake);
  extendCarriage(0, 30, retractJammed);
  tiltElbow(0, 40, retractJammed);
}

// Moves to the head: base, then carriage, then elbow.
// headAngle is where the SENSOR saw the head; the base turns a little further
// (aimOffsetDeg) so the NUDGER points at it.
void reachHead(int headAngle, int headMm, double &reachCm, bool &jammed) {
  rotateBase(headAngle + aimOffsetDeg(headMm), 30, jammed);
  if (jammed) return;
  reachCm = reachFor(headMm);
  extendCarriage(reachCm, 25, jammed);
  if (jammed) return;
  tiltElbow(NUDGE_ANGLE, 50, jammed);
}

// Stretch goal: the person held the arm away. Back off, return to the saved
// head angle with the gyro, and rescan only if the person moved.
// Returns false if the head can't be found again.
bool recoverFromPush(int &headAngle, int &headMm, double &reachCm,
                     bool &pushDetected, bool &jammed) {
  showMessage("Pushed!", "Backing up");
  pushDetected = false;

  extendCarriage(reachCm - 3, 30, jammed);
  if (jammed) return true;

  // Wait for the person to let go (max 10 s)
  timer letGo;
  letGo.clear();
  while (NudgeBumper.pressing() && letGo.time(seconds) < 10)
  {}

  // Base: turn so the SENSOR points at the saved head angle again, to check
  // whether the person moved. Use the gyro, since a push can back-drive the gears.
  if (fabs(baseAngle() - headAngle) > PUSH_ANGLE_TOL_DEG) {
    rotateBase(headAngle, 20, jammed);
    if (jammed) return true;
  }

  // Elbow: the worm shouldn't move, but check
  double elbowDeg = ElbowMotor.position(degrees) / ELBOW_RATIO;
  if (fabs(elbowDeg - NUDGE_ANGLE) > PUSH_ELBOW_TOL_DEG) {
    tiltElbow(NUDGE_ANGLE, 40, jammed);
    if (jammed) return true;
  }

  // Rescan only if the head looks different from before
  bool seen = HeadSensor.isObjectDetected();
  int nowMm = seen ? (int)HeadSensor.objectDistance(mm) : -1;
  if (!seen || abs(nowMm - headMm) > PUSH_RESCAN_MM) {
    extendCarriage(0, 40, jammed);
    tiltElbow(0, 50, jammed);
    if (jammed) return true;
    int newMm = 0;
    int newAngle = scanBed(newMm, jammed);
    if (jammed) return true;
    if (newAngle == NOT_FOUND) {
      return false;
    }
    headAngle = newAngle;
    headMm = newMm;
    reachHead(headAngle, headMm, reachCm, jammed);
  } else {
    // Same place: turn the nudger (not the sensor) back onto the head, then reach
    rotateBase(headAngle + aimOffsetDeg(headMm), 20, jammed);
    if (!jammed) {
      extendCarriage(reachCm, 30, jammed);
    }
  }
  showMessage("WAKE UP!", "");
  return true;
}

// ===========================================================================
// TEST 4: ALARM  (tests setAlarm and waitForAlarm)
// ===========================================================================
// Set an alarm, see how many seconds setAlarm() returned, press check, then
// watch the countdown. Run it once per test case (restart the program).
// DEMO_ALARM is true, so "07:30" means 7 min 30 s.
//
// Test cases                              Pass when
//  a. Press check straight away            Shows 00:30, returns "30 sec", countdown from 00:30
//                                          ends with WAKE UP! after 30 s (check with a stopwatch,
//                                          within 1 s)
//  b. Left once, then check                Shows 01:30, returns "90 sec"
//  c. Hold left for 2 s                    First number goes up by only 1
//  d. Right 30 times                       Second number goes 30 -> 59 -> 00 (wraps, doesn't hit 60)
//  e. Left 24 times                        First number goes 23 -> 00 (wraps)
//  f. Set 00:05, then check                Countdown 00:05 -> 00:01, then WAKE UP!
// ===========================================================================

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.render();  // turn on double buffering, like startUp() does

  int alarmSec = setAlarm();

  // Show what setAlarm returned, then wait for check
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(4, 35, "%d sec", alarmSec);
  Brain.Screen.printAt(4, 65, "chk: start");
  Brain.Screen.render();
  while (!Brain.buttonCheck.pressing())
  {}
  while (Brain.buttonCheck.pressing())
  {}

  waitForAlarm(alarmSec);  // shows WAKE UP! at the end
  wait(3, seconds);
  Brain.programStop();
  return EXIT_SUCCESS;
}
