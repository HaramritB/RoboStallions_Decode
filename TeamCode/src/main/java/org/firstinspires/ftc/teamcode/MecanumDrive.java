package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {

    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    private final Telemetry telemetry;

    // ===== Direction Fix Toggles =====
    private static final boolean INVERT_Y  = false;  // forward/backward
    private static final boolean INVERT_X  = true;   // FIXED: strafing reversed
    private static final boolean INVERT_RX = true;   // rotation

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Motor mapping
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontleft");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backleft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontright");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backright");

        // Reverse right side motors for mecanum
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor braking behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drive method for mecanum control.
     */
    public void drive(double y, double x, double rx,
                      boolean dpadUp, boolean dpadDown,
                      boolean dpadLeft, boolean dpadRight) {

        // ===== Apply direction toggles =====
        if (INVERT_Y)  y  = -y;
        if (INVERT_X)  x  = -x;     // << FIX: Reversed strafing
        if (INVERT_RX) rx = -rx;

        // Slight compensation for imperfect strafing
        x *= 1.1;

        // ===== D-pad precision override =====
        if (dpadUp)    { y = 0.5;  x = 0;   rx = 0; }
        if (dpadDown)  { y = -0.5; x = 0;   rx = 0; }
        if (dpadLeft)  { x = -0.5; y = 0;   rx = 0; }
        if (dpadRight) { x = 0.5;  y = 0;   rx = 0; }

        // ===== Calculating wheel powers =====
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        // ===== Set motor powers =====
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}