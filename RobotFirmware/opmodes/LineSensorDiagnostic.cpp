#include "NRL.h"

// ============================================================
// LINE SENSOR HARDWARE DIAGNOSTIC OPMODE
// Displays real-time raw ADC counts, normalized (0..1000) levels,
// visual sensor bar, line position, error, and calibration health.
// ============================================================

static HexaLineArray line { hexaLineConfig() };

class LineSensorDiagnostic : public NRLOpMode {
public:
    void init() override {
        line.begin();
        // Auto-fix any channel that missed black or was uncalibrated
        line.autoFixCalibration();
    }

    void loop() override {
        // Read raw ADC counts (0..4095)
        uint16_t r[8];
        uint16_t n[8];
        char bar[11] = "[........]";

        uint8_t m = line.mask();
        for (int i = 0; i < 8; ++i) {
            r[i] = line.readRaw(i);
            n[i] = line.read(i);
            if (m & (1 << i)) {
                bar[i + 1] = '#';  // Detected line
            }
        }

        // Live visual bar and pattern:
        telemetry.addData("Bar", bar);
        telemetry.addData("Pattern", linePatternName(line.pattern()));
        telemetry.addData("Position", line.position());
        telemetry.addData("Error", line.error());

        // Normalized readings (0 = pure white, 1000 = pure black):
        char normStr1[32];
        char normStr2[32];
        snprintf(normStr1, sizeof(normStr1), "N 0-3: %3u %3u %3u %3u", n[0], n[1], n[2], n[3]);
        snprintf(normStr2, sizeof(normStr2), "N 4-7: %3u %3u %3u %3u", n[4], n[5], n[6], n[7]);
        telemetry.addData("Norm 0-3", normStr1);
        telemetry.addData("Norm 4-7", normStr2);

        // Raw ADC counts (0..4095):
        char rawStr1[32];
        char rawStr2[32];
        snprintf(rawStr1, sizeof(rawStr1), "%4u|%4u|%4u|%4u", r[0], r[1], r[2], r[3]);
        snprintf(rawStr2, sizeof(rawStr2), "%4u|%4u|%4u|%4u", r[4], r[5], r[6], r[7]);
        telemetry.addData("Raw 0-3", rawStr1);
        telemetry.addData("Raw 4-7", rawStr2);

        // Button A: force recalibration baseline
        if (gamepad1.justPressed(BTN_A)) {
            line.autoFixCalibration();
        }

        // Serial logging at 4 Hz
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 250) {
            lastPrint = millis();
            Serial.printf("[LINE] Bar:%s Pos:%-5d Err:%-5d | Raw: %4u %4u %4u %4u %4u %4u %4u %4u\n",
                          bar, line.position(), line.error(),
                          r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
        }
    }

    void stop() override {}
};

// Registered on Controller TELEOP menu
REGISTER_OPMODE(LineSensorDiagnostic, "DIAG: LINE SENSOR", TELEOP);
