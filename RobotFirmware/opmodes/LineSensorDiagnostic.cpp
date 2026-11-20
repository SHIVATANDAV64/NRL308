#include "NRL.h"

// ============================================================
// LINE SENSOR HARDWARE DIAGNOSTIC OPMODE (Team 308)
// Use this to check raw ADC counts, wiring, and emitters.
// ============================================================

static HexaLineArray line { hexaLineConfig() };

class LineSensorDiagnostic : public NRLOpMode {

public:
    void init() override {
        line.begin();
    }

    void loop() override {
        // Read raw ADC counts (0..4095) across all 8 channels:
        // Healthy White floor: ~300 - 1500
        // Healthy Black tape:  ~3500 - 4095
        uint16_t r0 = line.readRaw(0);
        uint16_t r1 = line.readRaw(1);
        uint16_t r2 = line.readRaw(2);
        uint16_t r3 = line.readRaw(3);
        uint16_t r4 = line.readRaw(4);
        uint16_t r5 = line.readRaw(5);
        uint16_t r6 = line.readRaw(6);
        uint16_t r7 = line.readRaw(7);

        String s01 = String(r0) + " | " + String(r1);
        String s23 = String(r2) + " | " + String(r3);
        String s45 = String(r4) + " | " + String(r5);
        String s67 = String(r6) + " | " + String(r7);

        // Display on Controller Telemetry:
        telemetry.addData("Calibrated", line.isCalibrated() ? "YES" : "NO");
        telemetry.addData("Ch 0-1", s01.c_str());
        telemetry.addData("Ch 2-3", s23.c_str());
        telemetry.addData("Ch 4-5", s45.c_str());
        telemetry.addData("Ch 6-7", s67.c_str());

        // Diagnose common hardware issues:
        bool allZero = (r0 < 50 && r3 < 50 && r7 < 50);
        bool allMax  = (r0 > 4050 && r3 > 4050 && r7 > 4050);

        if (allZero) {
            telemetry.addData("DIAG", "NO SIGNAL / POWER OFF (Check 5V & SIG)");
        } else if (allMax) {
            telemetry.addData("DIAG", "ADC FLOATING / BLACK (Check height or SIG)");
        } else {
            telemetry.addData("DIAG", "ADC ACTIVE (Slide over black line to test)");
        }

        // Print to Serial Monitor at 115200 baud:
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 250) {
            lastPrint = millis();
            Serial.printf("[LINE RAW 308] C0:%-4u C1:%-4u C2:%-4u C3:%-4u C4:%-4u C5:%-4u C6:%-4u C7:%-4u\n",
                          r0, r1, r2, r3, r4, r5, r6, r7);
        }
    }

    void stop() override {
    }
};

// Registered on Controller TELEOP menu for easy access
REGISTER_OPMODE(LineSensorDiagnostic, "DIAG: LINE SENSOR", TELEOP);
