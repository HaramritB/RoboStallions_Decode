package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "CombinedTeleOp", group = "Competition")
public class CombinedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ======== DRIVE SYSTEM ========
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");

        // ✅ FIXED DIRECTIONS
        // Swap depending on your wiring.
        // Try this configuration if your robot's directions are flipped:
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // ======== LIMELIGHT + ROTATION ========
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        DcMotor rotationMotor = hardwareMap.get(DcMotor.class, "rotation");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight.pipelineSwitch(7);
        limelight.start();

        double kP = 0.04;
        double maxPower = 0.5;

        // ======== FLYWHEEL SYSTEM ========
        Flywheel flywheel = new Flywheel(hardwareMap);
        double targetRPM = 0;
        double hoodPos = flywheel.getHoodPosition();

        telemetry.addLine("Combined TeleOp initialized.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ===== DRIVE CONTROL =====
            double y = -gamepad1.left_stick_y; // forward is negative on joystick
            double x = gamepad1.left_stick_x * 1.1; // strafe
            double rx = gamepad1.right_stick_x; // rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Optional: slow mode (hold left trigger)
            if (gamepad1.left_trigger > 0.2) {
                frontLeftPower *= 0.4;
                backLeftPower *= 0.4;
                frontRightPower *= 0.4;
                backRightPower *= 0.4;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ===== LIMELIGHT ROTATION CONTROL =====
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double power = -kP * tx;
                power = Math.max(-maxPower, Math.min(maxPower, power));
                rotationMotor.setPower(power);
            } else {
                rotationMotor.setPower(0);
            }

            // ===== FLYWHEEL CONTROL =====
            if (gamepad1.dpad_up) {
                targetRPM += 25;
                sleep(200);
            } else if (gamepad1.dpad_down) {
                targetRPM -= 25;
                if (targetRPM < 0) targetRPM = 0;
                sleep(200);
            }

            if (gamepad1.right_trigger > 0.1) {
                flywheel.setTargetRPM(targetRPM);
            } else if (gamepad1.left_trigger > 0.1) {
                flywheel.stop();
            }

            if (gamepad1.x) flywheel.openHood();
            if (gamepad1.y) flywheel.closeHood();
            if (gamepad1.right_bumper) flywheel.setHoodPosition(flywheel.getHoodPosition() + 0.02);
            if (gamepad1.left_bumper) flywheel.setHoodPosition(flywheel.getHoodPosition() - 0.02);

            // ===== TELEMETRY =====
            telemetry.addData("Drive", "Y: %.2f  X: %.2f  RX: %.2f", y, x, rx);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel Velocity", flywheel.getAverageVelocity());
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.update();
        }

        // Stop all motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        rotationMotor.setPower(0);
        flywheel.stop();
    }
}
