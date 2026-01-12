package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gate.Gate;
import org.firstinspires.ftc.teamcode.limelight.AprilTagTracking;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Subsystems =====
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Gate gate = new Gate(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        double targetRPM = 0;
        boolean aprilTagMode = false;
        boolean aPressedLast = false;

        // Triangle edge detect
        boolean flywheelToggleLast = false;

        // Dpad-down edge detect for RPM step
        boolean dpadDownLast = false;

        double RPM_STEP = 10;

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // -----------------------------
            // DRIVE CONTROL
            // -----------------------------
            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false,
                    false,
                    gamepad1.dpad_right,
                    gamepad1.dpad_left
            );

            // -----------------------------
            // TRANSFER (RIGHT TRIGGER)
            // -----------------------------
            transfer.update(gamepad1.right_trigger);

            // -----------------------------
            // FLYWHEEL CONTROL
            // -----------------------------
            // RPM decrement moved off right_trigger to avoid conflict
            boolean dpadDownNow = gamepad1.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                targetRPM -= RPM_STEP;
            }
            dpadDownLast = dpadDownNow;

            if (gamepad1.left_stick_button) {
                targetRPM = 0;
            }

            flywheel.setTargetRPM(targetRPM);

            // Hood toggle on TRIANGLE
            boolean trianglePressed = gamepad1.triangle;
            if (trianglePressed && !flywheelToggleLast) {
                flywheel.toggleHood();
            }
            flywheelToggleLast = trianglePressed;

            double currentRPM = (flywheel.getAverageVelocity() / flywheel.getTicksPerRev()) * 60.0;

            // -----------------------------
            // APRILTAG TURRET CONTROL
            // -----------------------------
            boolean aPressedNow = gamepad1.a;
            if (aPressedNow && !aPressedLast) {
                aprilTagMode = !aprilTagMode;
            }
            aPressedLast = aPressedNow;

            double manualPower = 0;
            if (gamepad1.right_bumper) manualPower = -0.4; // right
            if (gamepad1.left_bumper)  manualPower = 0.4;  // left

            if (manualPower != 0) {
                turret.setManualPower(manualPower);     // manual override
            } else if (aprilTagMode) {
                turret.update();                        // AprilTag tracking
            } else {
                turret.stop();                          // idle hold
            }

            // -----------------------------
            // INTAKE & GATE
            // -----------------------------
            intake.update(gamepad1);
            gate.update(gamepad1); // LEFT TRIGGER controls gate

            // -----------------------------
            // TELEMETRY
            // -----------------------------
            telemetry.addLine("--- TELEOP STATUS ---");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", String.format("%.1f", currentRPM));
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.addData("Transfer Power", transfer.getPower());
            telemetry.addData("Intake Running", intake.isRunning());
            telemetry.addData("Intake Motor Power", intake.getMotorPower());
            telemetry.addData("AprilTag Mode", aprilTagMode ? "Tracking" : "Manual");
            telemetry.addData("Gate Position", gate.getPosition());
            telemetry.update();
        }

        // -----------------------------
        // STOP EVERYTHING
        // -----------------------------
        drive.stop();
        turret.stop();
        flywheel.stop();
        intake.stop();
        transfer.stop();
        gate.close();
    }
}
