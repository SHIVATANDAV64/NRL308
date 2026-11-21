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

// Tuned line parameters for Team 308 on 2026 HYPERDRIVE Arena
static LineTuning team308LineTuning() {
    LineTuning t;
    t.pid.kP          = 0.45f;   // Steering strength on 25 mm tape
    t.pid.kI          = 0.00f;   // Always 0 for line following
    t.pid.kD          = 0.04f;   // Damping to eliminate wobble
    t.followSpeed     = 0.35f;   // Base forward cruise speed
    t.junctionSpeed   = 0.25f;   // Safe crossing speed at branches
    t.pivotMs         = 60;      // Pivot duration to center chassis over junction
    t.pivotSpeed      = 0.25f;   // Turn speed during 90-deg and 180-deg turns
    t.eventThreshold  = 400;     // Lowered to 400 so weak channels (Ch0) reliably trigger events
    t.startTimeoutMs  = 6000;    // 6 seconds to clear start box without early timeout
    t.boxLeaveTicks   = 2;       // Confirmed ticks for box exit
    t.endConfirmTicks = 3;       // Confirmed ticks for end of line
    return t;
}

static LineActions lineActions { drive, line, team308LineTuning() };

// Fallback calibration: if controller LINE TEST wasn't run, load empirical baseline
// so the robot NEVER aborts with NOT_CALIBRATED
static void ensureLineCalibration() {
    if (!line.isCalibrated()) {
        // Measured baseline for elevated/tilted Arc-8 on Team 308
        // White floor readings: Ch0=3850, Ch1=3720, Ch2=3640, Ch3=3560, Ch4=3480, Ch5=3410, Ch6=3360, Ch7=3360
        // Black tape readings: 4095
        static const uint16_t whiteCal[8] = { 3850, 3720, 3640, 3560, 3480, 3410, 3360, 3360 };
        static const uint16_t blackCal[8] = { 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095 };
        line.setCalibration(whiteCal, blackCal);
    }
}


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

// Collects a CORE ring at the end bar of a Pick-Up terminal
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
// ROUTINE 1: GOLD MISSION FROM LINE (RECOMMENDED - 65 Points)
// Placement: Place robot directly on the East line facing NORTH
// (just in front of the Start Box spur).
// Collects P3 Gold CORE (25 pts drop + 5 pts pick) -> Drops D2 -> Parks (30 pts)
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
        ensureLineCalibration();

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        // Navigation starts on East line heading NORTH:
        runAction(lineActions.auto_()
            .followToJunction(LineSide::RIGHT, 1)         // 1. Pass P1 (Red Pick on East line)
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 2. Pass D1 (Drop on East line)
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 3. Pass P2 (Red Pick on East line)
            .straight()
            // Top-right corner: automatically turns 90 deg LEFT onto North line (heading West)
            .followToJunction(LineSide::RIGHT, 1)         // 4. Stop at P3 (Gold CORE terminal)
            .right()                                      // Turn onto P3 spur
            .followToEnd()                                // Follow ~500mm to end bar
            .then(pickUpCoreAction())                     // 5. Grab Gold CORE!
            .uTurn()                                      // Turn 180 back toward North line
            .followToJunction()                           // Back to junction
            .right()                                      // Facing South, turn right onto North line (heading West)
            .followToJunction(LineSide::LEFT, 1)          // 6. Stop at D2 (Drop terminal)
            .left()                                       // Turn inward to D2
            .followToEnd()                                // Follow ~500mm into central Drop Zone
            .then(dropCoreAction())                       // 7. Release Gold CORE into Drop Zone!
            .uTurn()                                      // Turn 180 back toward North line
            .followToJunction()                           // Back to junction
            .left()                                       // Facing North, turn left onto North line (heading West)
            .followToJunction(LineSide::RIGHT, 1)         // 8. Pass P4 (Gold Pick 2 on North line)
            .straight()
            // Top-left corner: automatically turns 90 deg LEFT onto West line (heading South)
            .followToJunction(LineSide::RIGHT, 1)         // 9. Pass P5 (Blue Pick 1 on West line)
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 10. Pass D3 (Drop on West line)
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 11. Pass P6 (Blue Pick 2 on West line)
            .straight()
            // Bottom-left corner: line turns LEFT (East) toward Park Zone
            .followToBox()                                // 12. Arrive at Park Zone (all 8 channels dark)
            .park(350)                                    // 13. Drive 350ms into box and brake (30 pts)!
            .build());
    }

    void loop() override {
        telemetry.addData("Mission", "GOLD FROM LINE (65P)");
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
// ROUTINE 2: GOLD MISSION FROM START BOX (65 Points)
// Placement: Place robot inside the Right Black Box facing EAST.
// The robot drives East out of the box, turns LEFT onto East line,
// and executes the full Gold mission.
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
        ensureLineCalibration();

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        runAction(lineActions.auto_()
            .leaveBox()                                   // 1. Exit Start Box onto East horizontal spur
            .followToJunction(LineSide::LEFT, 1)          // 2. Reach junction with East vertical line
            .left()                                       // 3. Turn LEFT 90 deg onto East line (heading North)
            .followToJunction(LineSide::RIGHT, 1)         // 4. Pass P1
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 5. Pass D1
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 6. Pass P2
            .straight()
            // Top-right corner: turns 90 deg LEFT onto North line (heading West)
            .followToJunction(LineSide::RIGHT, 1)         // 7. Stop at P3 (Gold CORE terminal)
            .right()                                      // Turn onto P3
            .followToEnd()
            .then(pickUpCoreAction())                     // 8. Grab Gold CORE!
            .uTurn()
            .followToJunction()
            .right()                                      // Turn right onto North line (heading West)
            .followToJunction(LineSide::LEFT, 1)          // 9. Stop at D2
            .left()
            .followToEnd()
            .then(dropCoreAction())                       // 10. Deposit Gold CORE!
            .uTurn()
            .followToJunction()
            .left()                                       // Turn left onto North line (heading West)
            .followToJunction(LineSide::RIGHT, 1)         // 11. Pass P4
            .straight()
            // Top-left corner: turns 90 deg LEFT onto West line (heading South)
            .followToJunction(LineSide::RIGHT, 1)         // 12. Pass P5
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // 13. Pass D3
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // 14. Pass P6
            .straight()
            // Bottom-left corner: turns LEFT into Park Zone
            .followToBox()                                // 15. Arrive at Park Zone
            .park(350)                                    // 16. Park safely (30 pts)!
            .build());
    }

    void loop() override {
        telemetry.addData("Mission", "GOLD FROM BOX (65P)");
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
// Starts from East line facing North, tours perimeter, and parks.
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
        ensureLineCalibration();

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        runAction(lineActions.auto_()
            // Follow full perimeter loop passing all junctions to Park Zone
            .followToBox()                                // Follow around arena to Park Zone
            .park(350)                                    // Stop on Park Zone (30 pts + tour)
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
REGISTER_OPMODE(AutoGoldFromLine, "AUTO: GOLD LINE START", AUTO);
REGISTER_OPMODE(AutoGoldFromBox,  "AUTO: GOLD BOX START",  AUTO);
REGISTER_OPMODE(AutoTourPark,     "AUTO: TOUR & PARK",     AUTO);
