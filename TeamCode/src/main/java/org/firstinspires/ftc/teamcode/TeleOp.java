package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Subsystems =====
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Kicker kicker = new Kicker(hardwareMap);

        double targetRPM = 0;
        boolean aprilTagMode = false;
        boolean aPressedLast = false;
        boolean flywheelToggleLast = false;
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
            // FLYWHEEL CONTROL
            // -----------------------------
            if (gamepad1.right_trigger > 0.5) {
                targetRPM -= RPM_STEP;
                sleep(150);
            }
            if (gamepad1.left_trigger > 0.5) {
                targetRPM += RPM_STEP;
                if (targetRPM < 0) targetRPM = 0;
                sleep(150);
            }
            if (gamepad1.left_stick_button) {
                targetRPM = 0;
            }
            flywheel.setTargetRPM(targetRPM);

            boolean flywheelToggle = gamepad1.right_stick_button;
            if (flywheelToggle && !flywheelToggleLast) {
                flywheel.toggleHood();
            }
            flywheelToggleLast = flywheelToggle;

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
                turret.manual(manualPower);             // manual override
            } else if (aprilTagMode) {
                turret.update();                         // AprilTag tracking
            } else {
                turret.stop();                           // idle hold
            }

            // -----------------------------
            // INTAKE & KICKER
            // -----------------------------
            intake.update(gamepad1);
            kicker.update(gamepad1);

            // -----------------------------
            // TELEMETRY
            // -----------------------------
            telemetry.addLine("--- TELEOP STATUS ---");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", String.format("%.1f", currentRPM));
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.addData("Intake Running", intake.isRunning());
            telemetry.addData("Intake Motor Power", intake.getMotorPower());
            telemetry.addData("AprilTag Mode", aprilTagMode ? "Tracking" : "Manual");
            telemetry.update();
        }

        // -----------------------------
        // STOP EVERYTHING
        // -----------------------------
        drive.stop();
        turret.stop();
        flywheel.stop();
        intake.stop();
    }
}