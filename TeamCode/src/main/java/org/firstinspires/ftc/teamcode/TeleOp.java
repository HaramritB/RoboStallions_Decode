package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Kicker kicker = new Kicker(hardwareMap);

        double targetRPM = 0;
        boolean aprilTagMode = false;
        boolean aPressedLast = false;
        boolean rbPressedLast = false;

        double RPM_STEP = 10;

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();
        waitForStart();

        turret.resetAngle();
        turret.resetPID();

        while (opModeIsActive()) {

            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false,
                    false,
                    gamepad1.dpad_left,
                    gamepad1.dpad_right
            );


            if (gamepad1.right_trigger > 0.5) {
                targetRPM += RPM_STEP;
                sleep(150);
            }

            if (gamepad1.left_trigger > 0.5) {
                targetRPM -= RPM_STEP;
                if (targetRPM < 0) targetRPM = 0;
                sleep(150);
            }

            if (gamepad1.left_bumper) {
                targetRPM = 0;
            }

            flywheel.setTargetRPM(targetRPM);

            boolean rb = gamepad1.right_bumper;
            if (rb && !rbPressedLast) {
                flywheel.toggleHood();
            }
            rbPressedLast = rb;

            double currentRPM =
                    (flywheel.getAverageVelocity() / flywheel.getTicksPerRev()) * 60.0;

            boolean aPressedNow = gamepad1.a;

            if (aPressedNow && !aPressedLast) {
                aprilTagMode = !aprilTagMode;
                turret.resetPID();
                turret.resetAngle();
            }
            aPressedLast = aPressedNow;

            if (aprilTagMode) {
                turret.update();
            } else {
                double manualPower = gamepad1.right_stick_x * 0.4;
                turret.setManualPower(manualPower);
            }

            intake.update(gamepad1);

            kicker.update(gamepad1);

            // ===== TELEMETRY =====
            telemetry.addLine("--- TELEOP STATUS ---");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel Velocity", flywheel.getAverageVelocity());
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.addData("Intake Running", intake.isRunning());
            telemetry.addData("Intake Motor Power", intake.getMotorPower());
            telemetry.addData("AprilTag Mode", aprilTagMode ? "Tracking" : "Manual");
            telemetry.addData("Current RPM", String.format("%.1f", currentRPM));
            telemetry.update();
        }

        drive.stop();
        turret.stop();
        flywheel.stop();
        intake.stop();
    }
}