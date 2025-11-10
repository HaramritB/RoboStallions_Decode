package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RotationControl", group = "Test")
public class RotationControl extends LinearOpMode {

    // 5202 Series Yellow Jacket 50.9:1
    private static final double TICKS_PER_MOTOR_REV = 537.7; // after gearbox

    // External gear ratio (motor gear : big piece gear)
    private static final double MOTOR_GEAR = 5.0;
    private static final double BIG_GEAR = 12.0;

    // The big piece rotates slower than motor by 5/12 ratio
    private static final double DEGREES_PER_MOTOR_REV = 360.0 * (MOTOR_GEAR / BIG_GEAR);
    private static final double TICKS_PER_BIG_PIECE_DEGREE = TICKS_PER_MOTOR_REV / DEGREES_PER_MOTOR_REV;

    private static final int NINETY_DEGREES_TICKS = (int) Math.round(90.0 * TICKS_PER_BIG_PIECE_DEGREE);

    private DcMotor rotationMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        rotationMotor = hardwareMap.get(DcMotor.class, "rotation");
        rotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready — press Play");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // 1. Rotate 90° left (negative)
            moveToPosition(-NINETY_DEGREES_TICKS, 0.4);

            sleep(500);

            // 2. Return to center (0°)
            moveToPosition(0, 0.4);

            sleep(500);

            // 3. Rotate 90° right (positive)
            moveToPosition(NINETY_DEGREES_TICKS, 0.4);

            rotationMotor.setPower(0);
            rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addLine("Rotation sequence complete");
            telemetry.update();
        }
    }

    private void moveToPosition(int targetTicks, double power) {
        rotationMotor.setTargetPosition(targetTicks);
        rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotationMotor.setPower(power);

        while (opModeIsActive() && rotationMotor.isBusy()) {
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Current", rotationMotor.getCurrentPosition());
            telemetry.update();
        }

        rotationMotor.setPower(0);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}