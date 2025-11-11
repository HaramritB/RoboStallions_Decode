package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "CombinedTeleOp", group = "Competition")
public class CombinedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== DRIVE SYSTEM =====
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // ===== LIMELIGHT + TURRET =====
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        DcMotor rotationMotor = hardwareMap.dcMotor.get("rotation");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight.pipelineSwitch(7);
        limelight.start();

        double kP = 0.04;
        double maxPower = 0.5;

        // ===== FLYWHEEL SYSTEM =====
        Flywheel flywheel = new Flywheel(hardwareMap);
        double targetRPM = 0;

        telemetry.addLine("Combined TeleOp initialized.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ===== DRIVE CONTROL =====
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Slow mode
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

            // ===== LIMELIGHT TURRET LOCK =====
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double power = -kP * tx;
                rotationMotor.setPower(Math.max(-maxPower, Math.min(maxPower, power)));
            } else {
                rotationMotor.setPower(0);
            }

            // ===== FLYWHEEL CONTROL (RPM ONLY) =====
            if (gamepad1.right_trigger > 0.1) {
                flywheel.setRPM(targetRPM); // ← use setRPM() instead of setTargetRPM()
            } else if (gamepad1.left_trigger > 0.1) {
                flywheel.stop();
            }


            // ===== HOOD CONTROL =====
            if (gamepad1.x) flywheel.openHood();
            if (gamepad1.y) flywheel.closeHood();
            if (gamepad1.right_bumper) flywheel.setHoodPosition(flywheel.getHoodPosition() + 0.02);
            if (gamepad1.left_bumper) flywheel.setHoodPosition(flywheel.getHoodPosition() - 0.02);

            // ===== TELEMETRY =====
            telemetry.addData("Drive", "Y: %.2f  X: %.2f  RX: %.2f", y, x, rx);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", flywheel.getAverageVelocity()); // optional feedback
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.update();
        }

        // Stop everything safely
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        rotationMotor.setPower(0);
        flywheel.stop();
    }
}
