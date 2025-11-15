package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CombinedTeleOp", group = "Competition")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Initialize subsystems =====
        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Kicker kicker = new Kicker(hardwareMap);

        double targetRPM = 0;
        // double hoodPos = flywheel.getHoodPosition();
        boolean aprilTagMode = false;
        boolean aPressedLast = false;
        boolean rbPressedLast = false;

        double RPM_STEP = 10;

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ===== DRIVE CONTROL =====

            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false, // dpad_up (disabled)
                    false, // dpad_down (disabled)
                    gamepad1.dpad_left,
                    gamepad1.dpad_right
            );

            // ===== FLYWHEEL CONTROL (uses D-pad up/down) =====
            // Increase RPM
            if (gamepad1.right_trigger > 0.5) {
                targetRPM += RPM_STEP;
                sleep(150);
            }

            // Decrease RPM
            if (gamepad1.left_trigger > 0.5) {
                targetRPM -= RPM_STEP;
                if (targetRPM < 0) targetRPM = 0;
                sleep(150);
            }

            // LB resets RPM instantly
            if (gamepad1.left_bumper) {
                targetRPM = 0;
            }

            // Apply new RPM
            flywheel.setTargetRPM(targetRPM);

            // RB toggles hood
            boolean rb = gamepad1.right_bumper;
            if (rb && !rbPressedLast) {
                flywheel.toggleHood();
            }
            rbPressedLast = rb;

            // Display current RPM
            double currentRPM =
                    (flywheel.getAverageVelocity() / flywheel.getTicksPerRev()) * 60.0;

            // ===== APRILTAG TURRET CONTROL =====
            boolean aPressedNow = gamepad1.a;

            // Toggle AprilTag mode only once per press (edge detection)
            if (aPressedNow && !aPressedLast) {
                aprilTagMode = !aprilTagMode;
            }
            aPressedLast = aPressedNow;

            if (aprilTagMode) {
                turret.update();  // auto-tracking or return to 0°
            } else {
                double manualPower = gamepad1.right_stick_x * 0.4;
                turret.setManualPower(manualPower);
            }

            // ===== INTAKE CONTROL =====
            intake.update(gamepad1);


            //  ===== KICKER CONTROL =====
            kicker.update(gamepad1);

            // ===== TELEMETRY =====
            telemetry.addLine("--- TELEOP STATUS ---");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel Velocity", flywheel.getAverageVelocity());
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.addData("Intake Running", intake.isRunning());
            telemetry.addData("Intake Motor Power", intake.getMotorPower());
            telemetry.addData("AprilTag Mode", aprilTagMode ? "Tracking" : "Manual");
            // telemetry.addData("Servo Position", kickerServo.getPosition());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", String.format("%.1f", currentRPM));
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.update();
        }

        // ===== STOP EVERYTHING =====
        drive.stop();
        turret.stop();
        flywheel.stop();
        intake.stop();
    }
}
