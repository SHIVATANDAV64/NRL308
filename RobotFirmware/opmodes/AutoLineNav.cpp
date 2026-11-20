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
    t.pid.kP        = 0.50f;   // Steering strength on 25 mm tape
    t.pid.kI        = 0.00f;   // Always 0 for line following
    t.pid.kD        = 0.04f;   // Damping to eliminate wobble
    t.followSpeed   = 0.35f;   // Base forward cruise speed
    t.junctionSpeed = 0.25f;   // Safe crossing speed at branches
    t.pivotMs       = 60;      // Pivot duration to center chassis over junction
    t.pivotSpeed    = 0.25f;   // Turn speed during 90-deg and 180-deg turns
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
// ROUTINE 1: GOLD CORE MISSION (Recommended Primary Strategy)
// Collects P3 Gold CORE (25 pts drop + 5 pts pick) -> Drops D2 -> Parks (30 pts)
// Total Score: 65 Points
// ============================================================

class AutoGoldMission : public NRLOpMode {
public:
    void init() override {
        leftMotor.begin();
        rightMotor.begin();
        armServo.begin();
        gripperServo.begin();
        servo3.begin();
        line.begin();

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        // Arena layout navigation:
        // Start Zone (SE) -> East Leg (North) -> North Leg (West) ->
        // P3 Gold Pick -> D2 Drop Zone -> West Leg (South) -> Park Zone (SW)
        runAction(lineActions.auto_()
            .leaveBox()                                   // 1. Exit Start Zone onto East line
            .followToJunction(LineSide::RIGHT, 1)         // Pass P1
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // Pass D1
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // Pass P2
            .straight()
            // Automatic 90-deg left corner onto North line (heading West)
            .followToJunction(LineSide::RIGHT, 1)         // 2. Stop at P3 (Gold CORE terminal)
            .right()                                      // Turn onto P3
            .followToEnd()                                // Follow 500mm to end bar
            .then(pickUpCoreAction())                     // 3. Grab Gold CORE!
            .uTurn()                                      // Turn 180 back to North main line
            .followToJunction()                           // Back to junction
            .right()                                      // Turn right onto North main line (heading West)
            .followToJunction(LineSide::LEFT, 1)          // 4. Stop at D2 (Drop terminal)
            .left()                                       // Turn inward to D2
            .followToEnd()                                // Follow 500mm to central Drop Zone
            .then(dropCoreAction())                       // 5. Release Gold CORE into Drop Zone!
            .uTurn()                                      // Turn 180 back to North main line
            .followToJunction()                           // Back to junction
            .left()                                       // Turn left onto North line (heading West)
            // Passes through top-left corner onto West line (heading South)
            .followToJunction(LineSide::RIGHT, 1)         // Pass P5
            .straight()
            .followToJunction(LineSide::LEFT, 1)          // Pass D3
            .straight()
            .followToJunction(LineSide::RIGHT, 1)         // Pass P6
            .straight()
            // Corner turns onto bottom line toward Park Zone
            .followToBox()                                // 6. Arrive at Park Zone (solid black box)
            .park()                                       // 7. Park safely on square (30 pts)!
            .build());
    }

    void loop() override {
        telemetry.addData("Mission", "GOLD CORE (65 PTS)");
        telemetry.addData("Status", isActionRunning() ? "RUNNING" : "PARKED");
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
// ROUTINE 2: SAFE TOUR & PARK (Guaranteed Baseline - 35 Points)
// Exits Start -> Full Perimeter Tour -> Parks in Park Zone
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

        armServo.setPosition(ARM_UP_ANGLE);
        gripperServo.setPosition(GRIPPER_CLOSE_ANGLE);
        servo3.setPosition(SERVO3_CLOSE_ANGLE);
    }

    void start() override {
        runAction(lineActions.auto_()
            .leaveBox()                                   // 1. Exit Start Zone (5 pts)
            // Follow full perimeter loop passing all junctions
            .followToBox()                                // 2. Follow all the way to Park Zone
            .park()                                       // 3. Stop on Park Zone (30 pts)
            .build());
    }

    void loop() override {
        telemetry.addData("Mission", "TOUR & PARK (35 PTS)");
        telemetry.addData("Status", isActionRunning() ? "RUNNING" : "PARKED");
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
REGISTER_OPMODE(AutoGoldMission, "AUTO: GOLD MISSION", AUTO);
REGISTER_OPMODE(AutoTourPark,    "AUTO: TOUR & PARK",  AUTO);
