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
//    Module:       range_test.cpp
//    Project:      Wake-Up Arm (test program)
//    Description:  Finds the range of motion of each joint. Hold the arrows
//                  to move one joint, read the numbers on the brain at each
//                  end of its travel, and copy them into the range constants
//                  in wake_up_arm.cpp.
//
//    How to use:
//      1. Put the robot in its home pose (base straight, elbow up, carriage
//         pulled in, nudger retracted). All encoders are zeroed there.
//      2. Run the program. For each joint:
//           right arrow (hold) = move forward, left arrow (hold) = move back
//           check = done with this joint (it then returns to 0)
//      3. Move to each end of the safe travel. The screen keeps the lowest
//         and highest value reached. If a motor hits a hard stop, it stops
//         and shows "LIMIT" (current over 1 A).
//      4. Write down min and max for each joint, back them off a little from
//         any hard stop, and set them as constants:
//           Carriage -> CARRIAGE_MIN_CM / CARRIAGE_MAX_CM (see CM_PER_DEG below)
//           Nudger   -> NUDGER_MIN_DEG / NUDGER_MAX_DEG
//           Elbow    -> ELBOW_MIN_DEG / ELBOW_MAX_DEG
//           Base     -> BASE_MIN_DEG / BASE_MAX_DEG
//
//    CM_PER_DEG: the carriage is shown in motor degrees. While testing it,
//    measure with a ruler how far the carriage moved between min and max.
//    CM_PER_DEG = that distance in cm / (max deg - min deg).
//
//----------------------------------------------------------------------------

#include "iq_cpp.h"

using namespace vex;

// ===========================================================================
// CONSTANTS (same as wake_up_arm.cpp)
// ===========================================================================

const double BASE_RATIO = 112.0 / 12.0;  // motor deg per base deg
const double ELBOW_RATIO = 48.0;         // motor deg per arm deg
const double MAX_CURRENT_AMP = 1.0;      // class rule: keep it under 1 A
const int CURRENT_IGNORE_MS = 200;       // ignore the start-up current spike

// ===========================================================================
// DEVICES (same ports as wake_up_arm.cpp)
// ===========================================================================

motor BaseMotor(PORT1, bool(forward));
motor ElbowMotor(PORT2, bool(forward));
motor CarriageMotor(PORT3, bool(forward));
motor NudgerMotor(PORT4, bool(forward));

// ===========================================================================
// FUNCTIONS
// ===========================================================================

// Shows one joint's current position and the range found so far.
void showRange(const char *name, double now, double minSeen, double maxSeen,
               const char *unit, bool atLimit) {
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(4, 18, "%s (%s)", name, unit);
  Brain.Screen.printAt(4, 42, "now %.1f", now);
  Brain.Screen.printAt(4, 66, "min %.1f", minSeen);
  Brain.Screen.printAt(4, 90, "max %.1f", maxSeen);
  if (atLimit) {
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(110, 18, "LIMIT");
  }
  Brain.Screen.render();
}

// Lets the user move one motor with the arrows and records the lowest and
// highest position reached, in the joint's own units (motor deg / ratio).
// minSeen and maxSeen are passed back by reference.
void findRange(motor &m, const char *name, double ratio, const char *unit,
               int speed, double &minSeen, double &maxSeen) {
  minSeen = 0;
  maxSeen = 0;
  bool atLimit = false;
  timer spinTimer;

  // Wait for the check press that ended the last joint to be released
  while (Brain.buttonCheck.pressing())
  {}

  while (!Brain.buttonCheck.pressing()) {
    bool right = Brain.buttonRight.pressing();
    bool left = Brain.buttonLeft.pressing();

    if (right || left) {
      if (!m.isSpinning() && !atLimit) {
        spinTimer.clear();
        if (right) {
          m.spin(forward, speed, percent);
        } else {
          m.spin(reverse, speed, percent);
        }
      }
      // Hard stop: current climbs when the joint can't move any further
      if (spinTimer.time(msec) > CURRENT_IGNORE_MS && m.current(amp) >= MAX_CURRENT_AMP) {
        m.stop(hold);
        atLimit = true;
      }
    } else {
      m.stop(hold);
      atLimit = false;  // released: the user can try the other way
    }

    double now = m.position(degrees) / ratio;
    if (now < minSeen) minSeen = now;
    if (now > maxSeen) maxSeen = now;

    showRange(name, now, minSeen, maxSeen, unit, atLimit);
    wait(50, msec);
  }

  m.stop(hold);
  printf("%s: min %.1f  max %.1f %s\n", name, minSeen, maxSeen, unit);

  // Back to the home position before testing the next joint
  m.spinToPosition(0, degrees, speed, percent, true);
}

// ===========================================================================
// MAIN
// ===========================================================================

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Brain.Screen.render();  // double buffering, same as the main program

  BaseMotor.setPosition(0, degrees);
  ElbowMotor.setPosition(0, degrees);
  CarriageMotor.setPosition(0, degrees);
  NudgerMotor.setPosition(0, degrees);

  BaseMotor.setStopping(hold);
  ElbowMotor.setStopping(hold);
  CarriageMotor.setStopping(hold);
  NudgerMotor.setStopping(hold);

  double carriageMin, carriageMax;
  double nudgerMin, nudgerMax;
  double elbowMin, elbowMax;
  double baseMin, baseMax;

  // Carriage and nudger first, so the arm is pulled in before it tilts or turns
  findRange(CarriageMotor, "Carriage", 1.0, "motor deg", 30, carriageMin, carriageMax);
  findRange(NudgerMotor, "Nudger", 1.0, "motor deg", 20, nudgerMin, nudgerMax);
  findRange(ElbowMotor, "Elbow", ELBOW_RATIO, "arm deg", 100, elbowMin, elbowMax);
  findRange(BaseMotor, "Base", BASE_RATIO, "base deg", 30, baseMin, baseMax);

  // Summary: copy these into the range constants in wake_up_arm.cpp
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(mono12);
  Brain.Screen.printAt(2, 16, "Carriage %.0f / %.0f mdeg", carriageMin, carriageMax);
  Brain.Screen.printAt(2, 40, "Nudger %.0f / %.0f deg", nudgerMin, nudgerMax);
  Brain.Screen.printAt(2, 64, "Elbow %.1f / %.1f deg", elbowMin, elbowMax);
  Brain.Screen.printAt(2, 88, "Base %.1f / %.1f deg", baseMin, baseMax);
  Brain.Screen.render();

  while (!Brain.buttonCheck.pressing())
  {}
  Brain.programStop();
  return EXIT_SUCCESS;
}
