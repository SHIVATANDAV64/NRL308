#include "NRL.h"  // Includes the NRL library containing motor, servo, gamepad, and OpMode definitions

// ============================================================
// PART 1: DEFINE THE ROBOT HARDWARE
// These objects represent the physical components of the robot.
// ============================================================

// Create the left DC motor object.
static HexaDCMotor leftMotor {
    {
        .dirPin = MOTOR_L_DIR,   // Pin used to control the motor's direction.
        .pwmPin = MOTOR_L_PWM,   // Pin used to control the motor's speed.
        .flipped = true          // Reverse the motor direction in the library.
    }
};

// Create the right DC motor object.
static HexaDCMotor rightMotor {
    {
        .dirPin = MOTOR_R_DIR,   // Pin used to control the motor's direction.
        .pwmPin = MOTOR_R_PWM    // Pin used to control the motor's speed.
    }
};

// Create the arm servo object (SERVO_1 / GPIO 39).
static HexaServo armServo {
    {
        .signalPin = SERVO_1,    // Pin connected to the arm servo's signal wire.
        .startAngle = 60.0f,     // Initial configured angle of the arm servo.
        .settleMs = 0            // Configured settling delay in milliseconds (active holding).
    }
};

// Create the gripper servo object (SERVO_2 / GPIO 40).
static HexaServo gripperServo {
    {
        .signalPin = SERVO_2,    // Pin connected to the gripper servo's signal wire.
        .startAngle = 95.0f,     // Initial configured angle of the gripper servo (closed position).
        .settleMs = 0            // Configured settling delay in milliseconds (active holding).
    }
};

// Create the third servo object (SERVO_3 / GPIO 41).
static HexaServo servo3 {
    {
        .signalPin = SERVO_3,    // Pin connected to servo 3 signal wire.
        .startAngle = 50.0f,     // Initial configured angle (starts closed at 50 deg).
        .settleMs = 0            // Configured settling delay in milliseconds (active holding).
    }
};

// Create a tank-drive object using the left and right motors.
static TankDrive drive { leftMotor, rightMotor };


// ============================================================
// PART 2: DEFINE SERVO TARGET ANGLES & RAMPING SPEEDS
// ============================================================

// Present limits for Team 308 (active when RT is OFF)
constexpr float ARM_UP_ANGLE        = 25.0f;   // Raised arm limit angle
constexpr float ARM_DOWN_ANGLE      = 95.0f;   // Lowered arm limit angle
constexpr float GRIPPER_OPEN_ANGLE  = 60.0f;   // Gripper open limit angle
constexpr float GRIPPER_CLOSE_ANGLE = 95.0f;   // Gripper closed limit angle

// Servo 3 limits (active when RT is ON)
constexpr float SERVO3_OPEN_ANGLE   = 120.0f;  // D-pad UP target angle
constexpr float SERVO3_CLOSE_ANGLE  = 50.0f;   // D-pad DOWN target angle

// Timing-based speed ramping parameters (deg/sec) - high responsiveness
constexpr float ARM_MIN_SPEED       = 200.0f;  // Starting speed on button tap
constexpr float ARM_MAX_SPEED       = 450.0f;  // Full servo physical transit speed on hold
constexpr float ARM_RAMP_SEC        = 0.50f;   // Quick 0.25s ramp to full speed

constexpr float GRIPPER_MIN_SPEED   = 120.0f;  // Starting speed for gripper
constexpr float GRIPPER_MAX_SPEED   = 400.0f;  // Fast grip actuation speed
constexpr float GRIPPER_RAMP_SEC    = 0.20f;   // Quick 0.20s ramp to full speed

constexpr float SERVO3_MIN_SPEED    = 100.0f;  // Starting speed for servo 3
constexpr float SERVO3_MAX_SPEED    = 450.0f;  // Fast transit speed for servo 3
constexpr float SERVO3_RAMP_SEC     = 0.25f;   // Quick 0.25s ramp to full speed


// ============================================================
// PART 3: CREATE THE OPMODE CLASS
// An OpMode defines how the robot behaves during a session.
// ============================================================

class fullcode : public NRLOpMode {  // Create a class named fullcode based on NRLOpMode.

private:
    // Tracks current angles for servos
    float currentArmAngle = 60.0f;
    float currentGripperAngle = 95.0f;
    float currentServo3Angle = 50.0f;

    // Loop delta-time tracking
    uint32_t lastLoopMs = 0;

    // Button hold start timestamps (0 = not pressed)
    uint32_t armUpHoldStartMs = 0;
    uint32_t armDownHoldStartMs = 0;
    uint32_t gripperOpenHoldStartMs = 0;
    uint32_t gripperCloseHoldStartMs = 0;
    uint32_t servo3UpHoldStartMs = 0;
    uint32_t servo3DownHoldStartMs = 0;

    // Helper: moves currentAngle towards targetAngle with hold-duration acceleration
    void updateServoWithHoldRamp(HexaServo& servo, float& currentAngle,
                                 float targetAngle, uint32_t& holdStartMs,
                                 uint32_t nowMs, float dt,
                                 float minSpeed, float maxSpeed, float rampSec) {
        if (holdStartMs == 0) {
            holdStartMs = nowMs;
        }
        float holdTimeSec = (float)(nowMs - holdStartMs) / 1000.0f;
        float progress = holdTimeSec / rampSec;
        if (progress > 1.0f) progress = 1.0f;

        // Speed increases based on how long the button is held
        float speed = minSpeed + (maxSpeed - minSpeed) * progress;
        float step = speed * dt;

        // Move towards targetAngle without overshooting
        if (currentAngle < targetAngle) {
            currentAngle += step;
            if (currentAngle > targetAngle) currentAngle = targetAngle;
        } else if (currentAngle > targetAngle) {
            currentAngle -= step;
            if (currentAngle < targetAngle) currentAngle = targetAngle;
        }

        servo.setPosition(currentAngle);
    }

public:  // The following functions can be accessed by the framework.

    // --------------------------------------------------------
    // PART 4: INITIALIZATION FUNCTION
    // This function prepares the hardware before control begins.
    // --------------------------------------------------------

    void init() override {  // Override the initialization function provided by NRLOpMode.

        leftMotor.begin();      // Initialize the left motor.
        rightMotor.begin();     // Initialize the right motor.

        armServo.begin();       // Initialize the arm servo.
        gripperServo.begin();   // Initialize the gripper servo.
        servo3.begin();         // Initialize the third servo.

        // Sync tracked angles to servo start positions
        currentArmAngle = armServo.getPosition();
        currentGripperAngle = gripperServo.getPosition();
        currentServo3Angle = servo3.getPosition();

        lastLoopMs = millis();
        armUpHoldStartMs = 0;
        armDownHoldStartMs = 0;
        gripperOpenHoldStartMs = 0;
        gripperCloseHoldStartMs = 0;
        servo3UpHoldStartMs = 0;
        servo3DownHoldStartMs = 0;
    }


    // --------------------------------------------------------
    // PART 5: MAIN CONTROL LOOP
    // This function repeatedly checks inputs while the OpMode runs.
    // --------------------------------------------------------

    void loop() override {  // Override the repeated control-loop function.

        // Calculate delta time (dt) in seconds
        uint32_t nowMs = millis();
        float dt = (lastLoopMs == 0) ? 0.01f : (float)(nowMs - lastLoopMs) / 1000.0f;
        if (dt > 0.05f) dt = 0.01f;  // Clamp to prevent jump if stalled
        lastLoopMs = nowMs;

        // Read the controller joysticks and send their values to the drive system.
        drive.drive(gamepad1.leftY(), gamepad1.rightX());

        // ----------------------------------------------------
        // SERVO SELECTION VIA RT TOGGLE SWITCH (GP41 on Controller)
        // ----------------------------------------------------
        bool rtMode = gamepad1.pressed(BTN_RT);

        if (rtMode) {
            // ================================================
            // RT IS ON: SERVO 3 CONTROL ONLY (SERVO_3 / GPIO 41)
            // ================================================
            armUpHoldStartMs = 0;
            armDownHoldStartMs = 0;
            gripperOpenHoldStartMs = 0;
            gripperCloseHoldStartMs = 0;

            bool upPressed = gamepad1.pressed(BTN_DPAD_UP);
            bool downPressed = gamepad1.pressed(BTN_DPAD_DOWN);

            if (upPressed && !downPressed) {
                servo3DownHoldStartMs = 0;
                updateServoWithHoldRamp(servo3, currentServo3Angle, SERVO3_OPEN_ANGLE,
                                        servo3UpHoldStartMs, nowMs, dt,
                                        SERVO3_MIN_SPEED, SERVO3_MAX_SPEED, SERVO3_RAMP_SEC);
            } else if (downPressed && !upPressed) {
                servo3UpHoldStartMs = 0;
                updateServoWithHoldRamp(servo3, currentServo3Angle, SERVO3_CLOSE_ANGLE,
                                        servo3DownHoldStartMs, nowMs, dt,
                                        SERVO3_MIN_SPEED, SERVO3_MAX_SPEED, SERVO3_RAMP_SEC);
            } else {
                servo3UpHoldStartMs = 0;
                servo3DownHoldStartMs = 0;
            }

            telemetry.addData("Mode", "SERVO 3");
            telemetry.addData("S3 Deg", currentServo3Angle);
        } else {
            // ================================================
            // RT IS OFF: SERVO 1 (ARM) & SERVO 2 (GRIPPER)
            // ================================================
            servo3UpHoldStartMs = 0;
            servo3DownHoldStartMs = 0;

            // Arm link control (D-Pad UP to raise, D-Pad DOWN to lower)
            bool armUpPressed = gamepad1.pressed(BTN_DPAD_UP);
            bool armDownPressed = gamepad1.pressed(BTN_DPAD_DOWN);

            if (armUpPressed && !armDownPressed) {
                armDownHoldStartMs = 0;
                updateServoWithHoldRamp(armServo, currentArmAngle, ARM_UP_ANGLE,
                                        armUpHoldStartMs, nowMs, dt,
                                        ARM_MIN_SPEED, ARM_MAX_SPEED, ARM_RAMP_SEC);
            } else if (armDownPressed && !armUpPressed) {
                armUpHoldStartMs = 0;
                updateServoWithHoldRamp(armServo, currentArmAngle, ARM_DOWN_ANGLE,
                                        armDownHoldStartMs, nowMs, dt,
                                        ARM_MIN_SPEED, ARM_MAX_SPEED, ARM_RAMP_SEC);
            } else {
                armUpHoldStartMs = 0;
                armDownHoldStartMs = 0;
            }

            // Gripper claw control (D-Pad LEFT to open, D-Pad RIGHT to close)
            bool gripperOpenPressed = gamepad1.pressed(BTN_DPAD_LEFT);
            bool gripperClosePressed = gamepad1.pressed(BTN_DPAD_RIGHT);

            if (gripperOpenPressed && !gripperClosePressed) {
                gripperCloseHoldStartMs = 0;
                updateServoWithHoldRamp(gripperServo, currentGripperAngle, GRIPPER_OPEN_ANGLE,
                                        gripperOpenHoldStartMs, nowMs, dt,
                                        GRIPPER_MIN_SPEED, GRIPPER_MAX_SPEED, GRIPPER_RAMP_SEC);
            } else if (gripperClosePressed && !gripperOpenPressed) {
                gripperOpenHoldStartMs = 0;
                updateServoWithHoldRamp(gripperServo, currentGripperAngle, GRIPPER_CLOSE_ANGLE,
                                        gripperCloseHoldStartMs, nowMs, dt,
                                        GRIPPER_MIN_SPEED, GRIPPER_MAX_SPEED, GRIPPER_RAMP_SEC);
            } else {
                gripperOpenHoldStartMs = 0;
                gripperCloseHoldStartMs = 0;
            }

            telemetry.addData("Mode", "ARM/GRIP");
            telemetry.addData("Arm", currentArmAngle);
            telemetry.addData("Gripper", currentGripperAngle);
        }
    }


    // --------------------------------------------------------
    // PART 8: STOP FUNCTION
    // Called when the OpMode is stopped by the framework.
    // --------------------------------------------------------

    void stop() override {  // Override the stop function provided by NRLOpMode.

        leftMotor.stop();   // Stop the left motor.
        rightMotor.stop();  // Stop the right motor.

        armServo.detach();      // Release control of the arm servo.
        gripperServo.detach();  // Release control of the gripper servo.
        servo3.detach();        // Release control of the third servo.
    }
};


// ============================================================
// PART 9: REGISTER THE OPMODE
// This makes the program available in the NRL OpMode system.
// ============================================================

REGISTER_OPMODE(fullcode, "RUNBOT", TELEOP);
// fullcode = Name of the class containing the robot program.
// "RUNBOT" = Name displayed for this OpMode.
// TELEOP = Identifies the program as driver-controlled operation.
