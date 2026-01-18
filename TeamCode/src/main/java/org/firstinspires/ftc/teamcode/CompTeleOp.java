package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.flywheel.Distance;
import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gate.Gate;
import org.firstinspires.ftc.teamcode.limelight.AprilTagTracking;

@TeleOp(name = "Competition TeleOp (Hybrid)", group = "Competition")
public class CompTeleOp extends LinearOpMode {

    // Tuning increments (Same as your Test Class)
    private static final double VEL_STEP_FINE = 25.0; // ticks/sec

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Subsystems =====
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Gate gate = new Gate(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        Distance distanceSensor = new Distance(hardwareMap);

        // ===== State Variables =====
        double targetRPM = 0;
        boolean shooterActive = false;

        // Auto vs Manual State
        boolean isManualRPM = false;
        double manualRPMOffset = 0; // Allows you to "nudge" the auto-aim if it's slightly off

        // Button Debouncing
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;
        boolean prevA = false;

        boolean aprilTagMode = false;

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // -----------------------------
            // 1. DRIVE
            // -----------------------------
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,
                    false, false, gamepad1.dpad_right, gamepad1.dpad_left);

            // -----------------------------
            // 2. INPUTS & OVERRIDES
            // -----------------------------
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;   // Manual Decrease
            boolean dpadRight = gamepad1.dpad_right; // Manual Increase

            // Shooter Toggle (Up/Down)
            if (dpadUp && !prevDpadUp) {
                shooterActive = true;
                isManualRPM = false; // Reset to Auto when turned on
            }
            if (dpadDown && !prevDpadDown) {
                shooterActive = false;
                targetRPM = 0;
            }

            // Manual Adjustment (Left/Right) - Same feel as Test Class
            if (shooterActive) {
                if (dpadRight && !prevDpadRight) {
                    manualRPMOffset += manualRPMOffset += VEL_STEP_FINE;
                    manualRPMOffset -= VEL_STEP_FINE;
                    ;
                    isManualRPM = true; // Switch to manual/offset mode
                }
                if (dpadLeft && !prevDpadLeft) {
                    manualRPMOffset -= manualRPMOffset += VEL_STEP_FINE;
                    manualRPMOffset -= VEL_STEP_FINE;
                    ;
                    isManualRPM = true;
                }
            }

            // -----------------------------
            // 3. SHOOTER LOGIC (Hybrid)
            // -----------------------------
            distanceSensor.update();
            double distMeters = distanceSensor.getDistanceMeters();

            if (shooterActive) {
                // Base calculation from distance
                double calculatedVel = (162 * distMeters) + 1309; // ticks/sec (LIKE TUNER)
                if (distMeters > 4.0 || distMeters < 0.2) {
                    calculatedVel = 1500; // safe default velocity (ticks/sec)
                }

// manualRPMOffset is now a "velocity offset" (still fine to keep same variable name, but better rename)
                double targetVel = calculatedVel + manualRPMOffset;

                flywheel.setTargetVelocity(targetVel);


                // Safety: Clamp reasonable values (e.g., don't spin 5000 RPM if sensor sees infinity)
                if (distMeters > 4.0 || distMeters < 0.2) {
                    // Sensor likely reading garbage, default to a "Safe Shot" speed (e.g. Launch Line)
                    calculatedVel = 1500;
                }

                // Final Target = Auto Calc + Manual Nudge
                targetRPM = calculatedVel + manualRPMOffset;

                flywheel.setTargetRPM(targetRPM);
            } else {
                flywheel.stop();
                manualRPMOffset = 0; // Reset offset when stopped
            }

            // -----------------------------
            // 4. TELEMETRY (Copied from Test Class)
            // -----------------------------
            // Using the exact math that you verified works:
            double actualTicksPerSec = flywheel.getAverageVelocity();
            double ticksPerRev = flywheel.getTicksPerRev();
            double currentRPM = (actualTicksPerSec * 60.0) / ticksPerRev;

            // -----------------------------
            // 5. OTHER SUBSYSTEMS
            // -----------------------------

            // Turret Logic
            if (gamepad1.a && !prevA) {
                aprilTagMode = !aprilTagMode;
            }
            prevA = gamepad1.a;

            if (gamepad1.right_bumper) turret.setManualPower(-0.4);
            else if (gamepad1.left_bumper) turret.setManualPower(0.4);
            else if (aprilTagMode) turret.update();
            else turret.stop();

            // Intake/Gate
            intake.update(gamepad1);
            gate.update(gamepad1);
            transfer.update(gamepad1.right_trigger);

            // -----------------------------
            // 6. DRIVER FEEDBACK
            // -----------------------------
            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData("State", shooterActive ? "ACTIVE" : "OFF");
            telemetry.addData("Mode", isManualRPM ? "MANUAL OFFSET" : "AUTO");
            telemetry.addData("Dist (m)", "%.2f", distMeters);
            telemetry.addData("Target RPM", "%.0f (Offset: %.0f)", targetRPM, manualRPMOffset);

            // This is the value you trust from the test class:
            telemetry.addData("ACTUAL RPM", "%.1f", currentRPM);

            telemetry.update();

            // Store previous button states
            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;
            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;
        }
    }
}