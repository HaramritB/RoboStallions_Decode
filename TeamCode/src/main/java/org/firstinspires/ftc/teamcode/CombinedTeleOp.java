package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "CombinedTeleOp", group = "Competition")
public class CombinedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Initialize subsystems =====
        MecanumTeleOpAxisLocked drive = new MecanumTeleOpAxisLocked(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);   // ✅ Intake subsystem

        double targetRPM = 0;
        double hoodPos = flywheel.getHoodPosition();
        boolean aprilTagMode = false;

        telemetry.addLine("Combined TeleOp Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ===== DRIVE CONTROL =====
            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    gamepad1.dpad_left,
                    gamepad1.dpad_right
            );

            // ===== FLYWHEEL CONTROL =====
            if (gamepad1.dpad_up) {
                targetRPM += 25;
                sleep(200);
            } else if (gamepad1.dpad_down) {
                targetRPM = Math.max(0, targetRPM - 25);
                sleep(200);
            }

            if (gamepad1.right_trigger > 0.1) {
                flywheel.setTargetRPM(targetRPM);
            } else if (gamepad1.left_trigger > 0.1) {
                flywheel.stop();
            }

            // Hood control
            if (gamepad1.y) flywheel.openHood();
            if (gamepad1.x) flywheel.closeHood();
            if (gamepad1.right_bumper) {
                hoodPos = Math.min(1.0, flywheel.getHoodPosition() + 0.02);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            } else if (gamepad1.left_bumper) {
                hoodPos = Math.max(0.0, flywheel.getHoodPosition() - 0.02);
                flywheel.setHoodPosition(hoodPos);
                sleep(150);
            }

            // ===== APRILTAG TURRET CONTROL =====
            if (gamepad1.a) {
                aprilTagMode = !aprilTagMode; // toggle tracking
                sleep(300);
            }

            if (aprilTagMode) {
                turret.update(); // track tag
            } else {
                turret.stop();
            }

            if (gamepad2.right_trigger > 0.1) {
                intake.startIntake();
            } else if (gamepad2.left_trigger > 0.1) {
                intake.eject();
            } else if (gamepad2.b) {
                intake.retract();
            } else if (gamepad2.a) {
                intake.deploy();
            } else {
                intake.stop();
            }

            // ===== TELEMETRY =====
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel Velocity", flywheel.getAverageVelocity());
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.addData("Intake Power", intake.getMotorPower());
            telemetry.addData("Intake Servo", intake.getServoPosition());
            telemetry.addData("Has Artifact", intake.hasArtifact());
            telemetry.update();
        }

        // ===== STOP EVERYTHING =====
        drive.stop();
        turret.stop();
        flywheel.stop();
        intake.stop();
    }
}