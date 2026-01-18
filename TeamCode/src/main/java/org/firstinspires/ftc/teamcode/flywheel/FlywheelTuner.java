package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Transfer;

@TeleOp(name = "Flywheel Tuner & Calibrator", group = "Test")
public class FlywheelTuner extends OpMode {

    // Subsystems
    private DcMotorEx flywheelMotor;
    private Intake intakeTest;
    private Transfer transferTest;
    private Distance distanceSensor;

    // State Variables
    private double targetVelocity = 1600;

    // PIDF values
    private double P = 0.0;
    private double F = 13.5;

    // Tuning/Calibration State
    private enum TunerMode {
        PID_TUNING,
        SPEED_CALIBRATION
    }
    private TunerMode currentMode = TunerMode.SPEED_CALIBRATION; // Default to speed

    // Tuning step sizes (used for both PID and Speed)
    private final double[] stepSizes = {50, 10, 1, 0.1, 0.01, 0.001};
    private int stepIndex = 0;

    // Edge detection
    private boolean lastUp, lastDown, lastLeft, lastRight, lastLB, lastRB, lastX, lastA;

    @Override
    public void init() {
        // Hardware Init
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize subsystems (if they don't require loop updates immediately)
        distanceSensor = new Distance(hardwareMap);

        // Initial PIDF
        updatePIDF();

        telemetry.addLine("Ready. Press X to switch modes.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update Subsystems
        if (intakeTest == null) intakeTest = new Intake(hardwareMap);
        if (transferTest == null) transferTest = new Transfer(hardwareMap);

        intakeTest.update(gamepad1);
        transferTest.update(0.2); // Constant speed for test
        distanceSensor.update();

        // Read buttons
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean x = gamepad1.x; // Mode Switch
        boolean a = gamepad1.a; // Quick reset

        // ---------------- CONTROL LOGIC ----------------

        // 1. Switch Modes (X Button)
        if (x && !lastX) {
            if (currentMode == TunerMode.PID_TUNING) {
                currentMode = TunerMode.SPEED_CALIBRATION;
            } else {
                currentMode = TunerMode.PID_TUNING;
            }
        }

        // 2. Adjust Step Size (Left Bumper)
        if (lb && !lastLB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // 3. Mode Specific Controls
        double currentStep = stepSizes[stepIndex];

        if (currentMode == TunerMode.PID_TUNING) {
            // --- PID MODE ---
            // Up/Down = P gain
            if (up && !lastUp)    P += currentStep;
            if (down && !lastDown) P -= currentStep;

            // Left/Right = F gain
            if (right && !lastRight) F += currentStep;
            if (left && !lastLeft)   F -= currentStep;

        } else {
            // --- SPEED CALIBRATION MODE ---
            // Up/Down = Target Velocity
            if (up && !lastUp)    targetVelocity += currentStep;
            if (down && !lastDown) targetVelocity -= currentStep;

            // Right Bumper = Toggle presets (still useful)
            if (rb && !lastRB) {
                targetVelocity = (targetVelocity == 1600) ? 900 : 1600;
            }

            // A Button = Emergency Stop / Zero
            if (a && !lastA) {
                targetVelocity = 0;
            }
        }

        // Save button states
        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastLB = lb; lastRB = rb; lastX = x; lastA = a;

        // Apply Motor Updates
        updatePIDF();
        flywheelMotor.setVelocity(targetVelocity);

        // ---------------- TELEMETRY ----------------
        telemetry.addLine("=== FLYWHEEL CALIBRATION ===");
        telemetry.addData("MODE", currentMode == TunerMode.PID_TUNING ? "PID TUNING" : "SPEED ADJUST");
        telemetry.addData("Step Size", currentStep);
        telemetry.addLine("----------------------------");

        if (distanceSensor.hasValidTag()) {
            telemetry.addData("Distance (m)", "%.3f", distanceSensor.getDistanceMeters());
            telemetry.addData("Tag ID", distanceSensor.getTagId());
        } else {
            telemetry.addData("Distance", "NO TAG");
        }

        telemetry.addLine("----------------------------");
        telemetry.addData("Target Vel", "%.0f", targetVelocity);
        telemetry.addData("Actual Vel", "%.0f", flywheelMotor.getVelocity());
        telemetry.addData("PIDF (P/F)", "%.4f / %.4f", P, F);

        telemetry.update();
    }

    private void updatePIDF() {
        // Only update if changed to save bus bandwidth
        PIDFCoefficients current = flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (current.p != P || current.f != F) {
            flywheelMotor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, 0, 0, F)
            );
        }
    }
}