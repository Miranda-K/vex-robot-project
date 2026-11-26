# Wake-Up Arm
 
A VEX IQ robot arm that wakes a person up without sound. At the alarm time it turns to find the sleeper's head, reaches over and nudges them, and it keeps going until they prove they're awake by winning a short crossy road game on the brain. Then the arm goes home by itself.
 
MTE 100 / MTE 121 design project, Group 20, University of Waterloo: Ridhikka Arivalagan, Cynthia Li, Miranda Kong, Rinko Koyama.
 
<!-- IMAGE: final robot -->
![Final robot](images/final_bot.jpg)
 
## The problem
 
Deaf and hard-of-hearing people don't have a reliable way to wake up without sound. Current options each leave a gap:
 
- **Pillow shakers** only work when the head is on the pillow.
- **Light strobes** are easy to miss with closed eyes, under a blanket, or facing away.
- **Smartwatch vibration** is often too weak to wake a deep sleeper.
- **Louder alarms** don't help at all.
The same need applies to students with roommates who want a silent, physical alarm. The Wake-Up Arm sits on a bedside table and physically nudges the user at the time they set.
 
## What it does
 
1. **Start up.** The robot starts in its home pose (base straight, elbow up, carriage pulled in). It calibrates the gyro and zeroes every motor encoder.
2. **Set the alarm.** On the brain, left adds to the first number, right adds to the second, and check confirms. In demo mode "07:30" means 7 min 30 s instead of 7 h 30 min.
3. **Wait.** A countdown runs on the screen using a timer.
4. **Find the head.** The base sweeps from −90° to +90° in 5° steps, taking a distance reading at each one. It stops at the first object inside the bed range, which it takes to be the head.
5. **Reach.** The distance sensor sits 8 cm to the side of the nudger, so the base turns a little past the angle where the sensor saw the head, until the nudger lines up with it. The carriage then slides out to match the distance, and the elbow tilts down.
6. **Nudge.** The nudger pokes until the bumper confirms contact, pushes a little further so the nudge is felt, then pulls back. If a poke misses, the carriage moves 1 cm closer and tries again.
7. **Prove you're awake.** Pressing any brain button starts the crossy road game. Losing, or not pressing anything for 20 s, goes back to nudging.
8. **Shut down.** After a win, the carriage pulls in, the elbow lifts, and the base turns back to 0, in that order, so the arm never swings across the bed while extended. The program ends itself, with no need for the stop button.
**Unexpected events it handles**
 
- **Jammed motor:** every move has a timeout and a current check (over 1 A). On a jam, the robot stops, shows which part jammed, and retracts.
- **No head found:** it rescans once, then goes home with a message.
- **Arm pushed away** (stretch goal): it backs off and returns to the saved angle using the gyro. It only rescans if the person moved.
- **Wrong buttons or no input:** the person isn't really awake, so nudging continues.
## Physical system
 
The arm has four motion systems, one VEX IQ smart motor each.
 
| System | How it moves | Built from |
| --- | --- | --- |
| Base yaw | 12T pinion driving a 112T gear (about 9.33 : 1) | 3D printed turntable and gear, 3D printed motor housing |
| Elbow pitch | Worm gear into 48T gears (48 : 1, self-locking) | Two 48T gears sandwiching the aluminium channel, between doubled laser-cut side plates |
| Linear extension | Chain and sprocket pulling a carriage along the channel | Aluminium channel rail, 3D printed carriage on bearings |
| Nudger | Motor directly on the carriage | VEX IQ parts, bumper on the tip |
 
### Base yaw
 
<!-- IMAGE: base yaw 3D design -->
![Base yaw 3D design](images/base_yaw_cad.png)
 
- The kit turntable was too small, so we designed and 3D printed a larger turntable. It sits inside a large printed gear.
- The gear is driven by 12T gears on the base motor. The motor sits in a printed housing.
- Supports on the turntable hold the tower's side boards.
- A centre bolt holds the top down, so the turntable doesn't lift when the arm is extended.
### Tower and elbow pitch
 
- The side plates are laser cut, two layers glued together on each side, with VEX H-frames bolted between them.
- An H-frame across the top carries the elbow mechanism.
- The worm drive gives a large reduction and can't be back-driven, so the arm holds its angle even with the motor off.
- A counterweight (about 3 lb) on the back end of the channel balances the arm.
### Linear extension and carriage
 
<!-- IMAGE: carriage 3D design -->
![Carriage 3D design](images/carriage_cad.png)
 
- The arm is an aluminium channel. The carriage rides along it on bearings that touch every side, so it slides without rocking.
- A single chain loop, driven directly by a motor on a VEX IQ plate, pulls the carriage in and out.
- The carriage is printed in PETG and has a built-in mount for the nudger motor.
- Custom printed bearings (4.22 mm bore) go through the plywood so the axles spin smoothly.
### Sensors and inputs
 
| Input | Used for |
| --- | --- |
| Distance sensor | Finding the head during the scan, and checking whether the person moved |
| Bumper (on the nudger) | Confirming each poke touched the person |
| Gyro (brain's built-in inertial sensor) | Measuring the base angle |
| Motor encoders | Elbow angle, carriage position, nudger stroke |
| Brain buttons | Setting the alarm and playing the game |
| Timer | Alarm countdown, move timeouts, game idle limit |
 
### CAD files
 
<!-- Update these paths to match the folders you upload -->
- [`cad/base_yaw/`](cad/base_yaw/): turntable, gear and motor housing
- [`cad/carriage/`](cad/carriage/): carriage and nudger mount
- [`cad/bearings/`](cad/bearings/): printed bearings and spacers
- [`cad/laser_cut/`](cad/laser_cut/): tower side plates
All printed parts were PETG on Bambu Lab P1S/P2S printers, with 4 walls and at least 40% infill. Holes were modelled about 0.1 mm oversize, because printed holes come out small.
 
## Code
 
Written in VEXcode IQ C++ for the 2nd generation IQ brain.
 
| File | What it is |
| --- | --- |
| `wake_up_arm.cpp` | The full robot program |
| `Wake_Up_Arm.iqcpp` | The same program as a VEXcode IQ project file |
| `rotateBase_test.cpp`, `tiltElbow_test.cpp`, `scanBed_test.cpp`, `scanBed_test2.cpp` | Unit tests for the motion and scan functions |
| `tests/01` to `tests/07` | Tests for buttons, screen messages, the game, the alarm, and the wake-up flow |
| `nudge&extend test.iqcpp`, `awakeCheck function.iqcpp` | VEXcode projects used for testing |
| `elbowconstants` | Program used to measure the elbow's range |
 
### The six main functions
 
| Function | What it does | Returns | Author |
| --- | --- | --- | --- |
| `rotateBase(double targetDeg, int speed, bool &jammed)` | Turns the base to an angle using the gyro, fast then slow near the target | nothing (jam passed back by reference) | Cynthia |
| `scanBed(int &headMm, bool &jammed)` | Sweeps the base and stops at the first object in bed range | angle of the head (distance by reference) | Cynthia |
| `tiltElbow(double targetDeg, int speed, bool &jammed)` | Tilts the arm to an angle using the motor encoder | nothing | Rinko |
| `extendCarriage(double cm, int speed, bool &jammed)` | Slides the carriage to a position in cm along the arm | nothing | Rinko |
| `nudge(int times, int speed, bool &buttonRequested, bool &pushDetected)` | Pokes until the bumper touches, pushes a bit further, pulls back | number of pokes that touched | Ridhikka |
| `awakeCheck(int rounds)` | Crossy road game on the brain screen | `true` if the player wins | Miranda |
 
### How the code is organized
 
- **Constants:** gear ratios, measured motion limits, speeds and timeouts sit at the top of the file, so tuning means changing one number.
- **No global variables:** `main()` owns the program's state (jam flag, head angle and distance, carriage reach) and passes it to functions by reference.
- **Struct and array:** each car in the game is a `Car` with a position, lane, height and speed. The 8 cars are kept in an array, `Car cars[8]`.
- **Safety:** every move checks a timeout and the motor current, and every target is clamped to the measured range of that joint.
### Crossy road game
 
The player crosses 4 lanes of traffic on the brain screen. Check steps forward one lane, and left and right dodge up and down. The cars wrap around the screen and speed up after each crossing. The game draws each frame off screen and shows it with `Brain.Screen.render()`, so it doesn't flicker.
 
## Running it
 
1. Open `Wake_Up_Arm.iqcpp` in VEXcode IQ and download it to the brain.
2. Put the robot in its home pose before starting: base straight, elbow up, carriage pulled in, nudger retracted.
3. Keep the robot still while the gyro calibrates.
4. Set `DEMO_ALARM = false` at the top of the code for real hours and minutes.
 