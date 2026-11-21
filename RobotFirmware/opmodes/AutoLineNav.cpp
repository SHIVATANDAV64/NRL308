#include "NRL.h"
#include "LineActions.h"

// ============================================================
// PART 1: DEFINE HARDWARE FOR TEAM 308
// ============================================================

// Left drive motor
static HexaDCMotor leftMotor {
    {
        .dirPin = MOTOR_L_DIR,
        .pwmPin = MOTOR_L_PWM,
        .flipped = true
    }
};

// Right drive motor
static HexaDCMotor rightMotor {
    {
        .dirPin = MOTOR_R_DIR,
        .pwmPin = MOTOR_R_PWM
    }
};

// Arm servo (SERVO_1 / GPIO 39) - Starts at ARM_UP_ANGLE (25 deg)
static HexaServo armServo {
    {
        .signalPin = SERVO_1,
        .startAngle = 25.0f,
        .settleMs = 0
    }
};

// Gripper claw servo (SERVO_2 / GPIO 40) - Starts at GRIPPER_CLOSE_ANGLE (95 deg)
static HexaServo gripperServo {
    {
        .signalPin = SERVO_2,
        .startAngle = 95.0f,
        .settleMs = 0
    }
};

// Third servo (SERVO_3 / GPIO 41) - Starts at SERVO3_CLOSE_ANGLE (50 deg)
static HexaServo servo3 {
    {
        .signalPin = SERVO_3,
        .startAngle = 50.0f,
        .settleMs = 0
    }
};

// TankDrive drivetrain
static TankDrive drive { leftMotor, rightMotor };

// 8-channel Arc IR Line Array on DIGITAL_PORT_1 & 2
static HexaLineArray line { hexaLineConfig() };

// Tuned line parameters for Team 308 on 2026 HYPERDRIVE Arena (12x12 ft)
static LineTuning team308LineTuning() {
    LineTuning t;
    t.pid.kP          = 0.55f;   // Responsive steering to stay centered on 25 mm tape
    t.pid.kI          = 0.00f;   // Always 0 for line following
    t.pid.kD          = 0.05f;   // Damping to prevent wobble
    t.followSpeed     = 0.32f;   // Smooth, controlled forward cruise
    t.junctionSpeed   = 0.25f;   // Safe crossing speed at branches
    t.speedDrop       = 0.35f;   // Reduced speed drop so robot maintains smooth momentum
    t.minSpeedFrac    = 0.50f;   // Never drops below 50% cruise speed
    t.pivotMs         = 70;      // Pivot duration to center chassis over 500 mm terminal spurs
    t.pivotSpeed      = 0.25f;   // Pivot speed for 90-deg and 180-deg turns
    t.straightMs      = 250;     // Forward drive to cross junction tapes cleanly
    t.eventThreshold  = 400;     // 400 = robust branch detection even with elevated Ch 0
    t.startTimeoutMs  = 6000;    // 6 seconds to clear start box
    t.boxLeaveTicks   = 2;       // Confirmed ticks for box exit
    t.endConfirmTicks = 3;       // Confirmed ticks for end of line
    return t;
}

static LineActions lineActions { drive, line, team308LineTuning() };


// ============================================================
// PART 2: CALIBRATED SERVO ANGLES (Team 308)
// ============================================================
constexpr float ARM_UP_ANGLE        = 25.0f;   // Raised arm limit angle (carrying CORE)
constexpr float ARM_DOWN_ANGLE      = 95.0f;   // Lowered arm limit angle (at floor)
constexpr float GRIPPER_OPEN_ANGLE  = 60.0f;   // Gripper open limit angle
constexpr float GRIPPER_CLOSE_ANGLE = 95.0f;   // Gripper closed limit angle
constexpr float SERVO3_OPEN_ANGLE   = 120.0f;  // Open auxiliary servo
constexpr float SERVO3_CLOSE_ANGLE  = 50.0f;   // Closed auxiliary servo


// ============================================================
// PART 3: MECHANISM ACTION HELPERS (Pick-Up & Drop)
// ============================================================

// Collects a CORE ring at the 150 mm end bar of a Pick-Up terminal
static ActionPtr pickUpCoreAction() {
    return sequential({
        instant([]{
            gripperServo.setPosition(GRIPPER_OPEN_ANGLE);   // 60 deg open
            armServo.setPosition(ARM_DOWN_ANGLE);           // 95 deg lower
        }),
        sleep_ms(350),
        instant([]{
            gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);  // 95 deg clamp
        }),
        sleep_ms(400),
        instant([]{
            armServo.setPosition(ARM_UP_ANGLE);             // 25 deg lift off floor
        }),
        sleep_ms(350),
    });
}

// Drops the carried CORE into the central DROP ZONE from a Drop terminal
static ActionPtr dropCoreAction() {
    return sequential({
        instant([]{
            armServo.setPosition(ARM_DOWN_ANGLE);           // 95 deg lower
        }),
        sleep_ms(350),
        instant([]{
            gripperServo.setPosition(GRIPPER_OPEN_ANGLE);   // 60 deg release
        }),
        sleep_ms(400),
        instant([]{
            armServo.setPosition(ARM_UP_ANGLE);             // 25 deg lift
            gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);  // 95 deg close
        }),
        sleep_ms(250),
    });
}


// ============================================================
// ROUTINE 1: OFFICIAL MATCH RUN — GOLD MISSION FROM START ZONE (65 Pts)
// Arena Setup: Place robot completely inside the START ZONE (bottom-right 500x500 box),
// facing EAST (pointing toward the East line).
// Strategy:
//   1. Exits Start Zone along 300 mm spur (5 pts)
//   2. Turns LEFT onto East main line (heading North)
//   3. Passes P1 (Red), D1 (Drop), P2 (Red)
//   4. Turns LEFT at top-right corner onto North line (heading West)
//   5. Turns RIGHT onto P3, follows 500 mm, grabs Gold CORE (5 pts)
//   6. 180 U-turn back to North line, resumes West
//   7. Turns LEFT onto D2, follows 500 mm into central DROP ZONE, releases Gold CORE (25 pts)
//   8. 180 U-turn back to North line, resumes West
//   9. Passes P4 (Gold)
//  10. Turns LEFT at top-left corner onto West main line (heading South)
//  11. Passes P5 (Blue), D3 (Drop), P6 (Blue)
//  12. Turns LEFT at bottom-left corner into 300 mm PARK spur
//  13. Arrives in 500x500 PARK ZONE, stops completely (30 pts)
// Total Score: 65 Points
// ============================================================

class AutoGoldFromBox : public NRLOpMode {
public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        armServo.begin();
        gripperServo.begin();
        servo3.begin();
        line.begin();
        line.autoFixCalibration();

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        runAction(lineActions.auto_()
            .leaveBox()                                   // 1. Exit Start Box along 300 mm spur (5 pts)
            .followToJunction(LineSide::LEFT, 1)          // 2. Reach junction with East line
            .left()                                       // 3. Turn LEFT 90 deg onto East line (heading North)
            .followToJunction(LineSide::RIGHT, 1)         // 4. Pass P1 (Red Pick on right)
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 5. Pass D1 (Drop on left)
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 6. Pass P2 (Red Pick on right)
            .straight()
            // Top-right corner: automatically turns 90 deg LEFT onto North line (heading West)
            .followToJunction(LineSide::RIGHT, 1)         // 7. Stop at P3 (Gold CORE terminal)
            .right()                                      // Turn onto P3
            .followToEnd()                                // Follow 500 mm to 150 mm end bar
            .then(pickUpCoreAction())                     // 8. Grab Gold CORE! (5 pts)
            .uTurn()                                      // 180 U-turn back to North line
            .followToJunction()                           // Back to junction
            .right()                                      // Facing South, turn right onto North line (heading West)
            .followToJunction(LineSide::LEFT, 1)          // 9. Stop at D2 (Drop terminal)
            .left()                                       // Turn left into Drop Zone
            .followToEnd()                                // Follow 500 mm to end bar
            .then(dropCoreAction())                       // 10. Release Gold CORE into DROP ZONE! (25 pts)
            .uTurn()                                      // 180 U-turn back to North line
            .followToJunction()                           // Back to junction
            .left()                                       // Facing North, turn left onto North line (heading West)
            .followToJunction(LineSide::RIGHT, 1)         // 11. Pass P4 (Gold Pick 2 on right)
            .straight()
            // Top-left corner: automatically turns 90 deg LEFT onto West line (heading South)
            .followToJunction(LineSide::RIGHT, 1)         // 12. Pass P5 (Blue Pick on right)
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 13. Pass D3 (Drop on left)
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 14. Pass P6 (Blue Pick on right)
            .straight()
            // Bottom-left corner: line turns LEFT (East) into 300 mm spur leading to Park Zone
            .followToBox()                                // 15. Arrive at 500x500 Park Zone
            .park(350)                                    // 16. Stop completely on black square (30 pts)!
            .build());
    }

    void loop() override {
        telemetry.addData("Mission", "GOLD BOX START (65P)");
        const char* st = isActionRunning() ? "RUNNING" :
                         (lineActions.fault() != LineFault::NONE ? "FAULT" : "FINISHED/PARKED");
        telemetry.addData("Status", st);
        telemetry.addData("Calibrated", line.isCalibrated() ? "YES" : "NO! CALIBRATE");
        telemetry.addData("Line Pos", line.position());
        telemetry.addData("Pattern", linePatternName(line.pattern()));
        telemetry.addData("Junctions", lineActions.junctionCount());
        telemetry.addData("Fault", lineActions.faultName());
    }

    void stop() override {
        cancelActions();
        drive.stop();
        armServo.detach();
        gripperServo.detach();
        servo3.detach();
    }
};


// ============================================================
// ROUTINE 2: PRACTICE RUN — GOLD MISSION FROM LINE (65 Pts)
// Arena Setup: Place robot directly on the East line facing NORTH
// (just ahead of the Start Box spur).
// ============================================================

class AutoGoldFromLine : public NRLOpMode {
public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        armServo.begin();
        gripperServo.begin();
        servo3.begin();
        line.begin();
        line.autoFixCalibration();

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        runAction(lineActions.auto_()
            .followToJunction(LineSide::RIGHT, 1)         // 1. Pass P1
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 2. Pass D1
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 3. Pass P2
            .straight()
            // Top-right corner: turns 90 deg LEFT onto North line
            .followToJunction(LineSide::RIGHT, 1)         // 4. Stop at P3
            .right()
            .followToEnd()
            .then(pickUpCoreAction())                     // 5. Grab Gold CORE!
            .uTurn()
            .followToJunction()
            .right()                                      // Heading West on North line
            .followToJunction(LineSide::LEFT, 1)          // 6. Stop at D2
            .left()
            .followToEnd()
            .then(dropCoreAction())                       // 7. Deposit Gold CORE into Drop Zone!
            .uTurn()
            .followToJunction()
            .left()                                       // Heading West on North line
            .followToJunction(LineSide::RIGHT, 1)         // 8. Pass P4
            .straight()
            // Top-left corner: turns 90 deg LEFT onto West line
            .followToJunction(LineSide::RIGHT, 1)         // 9. Pass P5
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 10. Pass D3
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 11. Pass P6
            .straight()
            // Bottom-left corner: turns into Park Zone
            .followToBox()
            .park(350)                                    // 12. Park safely (30 pts)!
            .build());
    }

    void loop() override {
        telemetry.addData("Mission", "GOLD LINE START (65P)");
        const char* st = isActionRunning() ? "RUNNING" :
                         (lineActions.fault() != LineFault::NONE ? "FAULT" : "FINISHED/PARKED");
        telemetry.addData("Status", st);
        telemetry.addData("Calibrated", line.isCalibrated() ? "YES" : "NO! CALIBRATE");
        telemetry.addData("Line Pos", line.position());
        telemetry.addData("Pattern", linePatternName(line.pattern()));
        telemetry.addData("Junctions", lineActions.junctionCount());
        telemetry.addData("Fault", lineActions.faultName());
    }

    void stop() override {
        cancelActions();
        drive.stop();
        armServo.detach();
        gripperServo.detach();
        servo3.detach();
    }
};


// ============================================================
// ROUTINE 3: SAFE TOUR & PARK (Guaranteed Baseline - 35 Points)
// Starts inside START ZONE facing East, tours perimeter, and parks.
// ============================================================

class AutoTourPark : public NRLOpMode {
public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        armServo.begin();
        gripperServo.begin();
        servo3.begin();
        line.begin();
        line.autoFixCalibration();

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        runAction(lineActions.auto_()
            .leaveBox()                                   // Exit Start Zone (5 pts)
            .followToJunction(LineSide::LEFT, 1)          // Reach East line
            .left()                                       // Turn Left onto East line (North)
            .followToBox()                                // Follow perimeter to Park Zone
            .park(350)                                    // Stop on Park Zone (30 pts)
            .build());
    }

    void loop() override {
        telemetry.addData("Mission", "TOUR & PARK (35 PTS)");
        const char* st = isActionRunning() ? "RUNNING" :
                         (lineActions.fault() != LineFault::NONE ? "FAULT" : "FINISHED/PARKED");
        telemetry.addData("Status", st);
        telemetry.addData("Calibrated", line.isCalibrated() ? "YES" : "NO! CALIBRATE");
        telemetry.addData("Line Pos", line.position());
        telemetry.addData("Pattern", linePatternName(line.pattern()));
        telemetry.addData("Junctions", lineActions.junctionCount());
        telemetry.addData("Fault", lineActions.faultName());
    }

    void stop() override {
        cancelActions();
        drive.stop();
        armServo.detach();
        gripperServo.detach();
        servo3.detach();
    }
};


// ============================================================
// PART 4: REGISTER THE AUTONOMOUS OPMODES ON CONTROLLER
// ============================================================
REGISTER_OPMODE(AutoGoldFromBox,  "AUTO: GOLD (BOX START)",  AUTO);
REGISTER_OPMODE(AutoGoldFromLine, "AUTO: GOLD (LINE START)", AUTO);
REGISTER_OPMODE(AutoTourPark,     "AUTO: TOUR & PARK",       AUTO);
