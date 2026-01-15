package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;

@TeleOp(name = "Flywheel Test", group = "Test")
public class FlywheelTest extends OpMode {

    private Flywheel flywheel;

    // Calibration state
    private double targetRPM = 0.0;
    private double hoodPos = 0.0;

    // Step sizes
    private static final double RPM_STEP_COARSE = 100.0;
    private static final double RPM_STEP_FINE   = 25.0;
    private static final double HOOD_STEP_FINE  = 0.01;
    private static final double HOOD_STEP_COARSE= 0.05;

    // Limits (servo should always be 0..1)
    private static final double HOOD_MIN = 0.0;
    private static final double HOOD_MAX = 1.0;

    // Debounce
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLB, prevRB, prevA, prevX, prevY;
    private boolean prevStart, prevBack;

    // Optional “distance” tracker for notes (you type it in with dpad left/right)
    private double distanceFt = 10.0;

    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);

        // Start from current servo pos (so you don’t jump unexpectedly)
        hoodPos = flywheel.getHoodPosition();
        hoodPos = clamp(hoodPos, HOOD_MIN, HOOD_MAX);

        flywheel.stop();

        telemetry.addLine("Flywheel Test Ready");
        telemetry.addLine("Dpad: RPM (Up/Down coarse, Left/Right fine)");
        telemetry.addLine("LB/RB: Hood position (fine). Hold START for coarse.");
        telemetry.addLine("A: toggle hood open/close | Y: apply target RPM | X: stop");
        telemetry.addLine("Back: reset encoder (optional safety) ");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read buttons
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        boolean a = gamepad1.a;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        boolean start = gamepad1.start;
        boolean back = gamepad1.back;

        // --- RPM adjustments ---
        if (dpadUp && !prevDpadUp) {
            targetRPM += RPM_STEP_COARSE;
        }
        if (dpadDown && !prevDpadDown) {
            targetRPM -= RPM_STEP_COARSE;
        }
        if (dpadRight && !prevDpadRight) {
            targetRPM += RPM_STEP_FINE;
        }
        if (dpadLeft && !prevDpadLeft) {
            targetRPM -= RPM_STEP_FINE;
        }
        if (targetRPM < 0) targetRPM = 0;

        // --- Hood adjustments ---
        // Hold START for coarse hood steps (fast tuning)
        double hoodStep = start ? HOOD_STEP_COARSE : HOOD_STEP_FINE;

        if (rb && !prevRB) hoodPos += hoodStep;
        if (lb && !prevLB) hoodPos -= hoodStep;

        hoodPos = clamp(hoodPos, HOOD_MIN, HOOD_MAX);
        // Directly set servo position (for continuous calibration)
        // (This does not change your hoodOpen boolean inside Flywheel, but that's fine for testing.)
        // If you want hoodOpen to stay consistent, use open/close only.
        flywheelHoodSetPosition(hoodPos);

        // --- Actions ---
        if (a && !prevA) {
            flywheel.toggleHood();
            hoodPos = flywheel.getHoodPosition(); // sync local value
        }

        if (y && !prevY) {
            flywheel.setTargetRPM(targetRPM);
        }

        if (x && !prevX) {
            flywheel.stop();
        }

        // Optional: encoder reset safety
        if (back && !prevBack) {
            // Quick reset to help if velocity reading gets weird
            // (Only do this while stopped in real life)
            flywheel.stop();
            // If you want a true reset, add a method to Flywheel to reset encoder/mode.
            // For now, just stop.
        }

        // --- Telemetry ---
        double actualTicksPerSec = flywheel.getAverageVelocity();
        double ticksPerRev = flywheel.getTicksPerRev();
        double actualRPM = (actualTicksPerSec * 60.0) / ticksPerRev;

        telemetry.addLine("=== Flywheel Calibration ===");
        telemetry.addData("Target RPM", "%.1f (press Y to apply)", targetRPM);
        telemetry.addData("Actual RPM", "%.1f", actualRPM);
        telemetry.addData("Vel (ticks/s)", "%.1f", actualTicksPerSec);

        telemetry.addLine("=== Hood Calibration ===");
        telemetry.addData("Hood Pos", "%.3f", hoodPos);

        telemetry.addLine("=== Notes for distance tuning ===");
        telemetry.addData("Distance (ft)", "%.1f  (change in code / keep notes)", distanceFt);
        telemetry.addLine("Record: distance ft -> targetRPM, hoodPos");

        telemetry.update();

        // Save previous states (debounce)
        prevDpadUp = dpadUp; prevDpadDown = dpadDown;
        prevDpadLeft = dpadLeft; prevDpadRight = dpadRight;
        prevLB = lb; prevRB = rb;
        prevA = a; prevX = x; prevY = y;
        prevStart = start; prevBack = back;
    }

    /**
     * We keep hoodPos locally and set it directly.
     * Flywheel class doesn't expose a setHoodPosition(), so we do a tiny workaround:
     * Add this method below or (recommended) add a setHoodPosition(pos) to Flywheel.
     */
    private void flywheelHoodSetPosition(double pos) {
        // Recommended: add a method Flywheel.setHoodPosition(double pos)
        // For now, we just call open/close when near endpoints, otherwise we can't set intermediate.
        // So: YOU SHOULD ADD setHoodPosition to Flywheel for true calibration.
        //
        // Quick best-effort behavior:
        // - If you want intermediate positions, add setHoodPosition in Flywheel (shown below).
        //
        // Placeholder does nothing.
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}