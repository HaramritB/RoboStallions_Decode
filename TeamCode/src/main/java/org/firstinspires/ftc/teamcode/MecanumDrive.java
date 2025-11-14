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

    // Toggle these if directions are inverted
    private static final boolean INVERT_Y  = false;  // Forward/backward
    private static final boolean INVERT_X  = false;  // Strafing (left/right)
    private static final boolean INVERT_RX = true;   // Rotation (turning) — set true to fix reversed turning

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors (use your configured names)
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontleft");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backleft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontright");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backright");

        // Reverse the right side if your mecanum setup requires it
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: better stopping behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drive method for mecanum control.
     *
     * @param y         forward/backward input (-1 to 1)
     * @param x         strafe left/right input (-1 to 1)
     * @param rx        rotation input (-1 to 1)
     * @param dpadUp    D-pad forward (precision)
     * @param dpadDown  D-pad backward (precision)
     * @param dpadLeft  D-pad strafe left (precision)
     * @param dpadRight D-pad strafe right (precision)
     */
    public void drive(double y, double x, double rx,
                      boolean dpadUp, boolean dpadDown,
                      boolean dpadLeft, boolean dpadRight) {

        // Apply optional inversion toggles
        if (INVERT_Y)  y  = -y;
        if (INVERT_X)  x  = -x;
        if (INVERT_RX) rx = -rx;  // <-- fixes reversed turning when true

        // Slight scale for imperfect strafing
        x *= 1.1;

        // D-pad for slow precision driving (overrides sticks)
        if (dpadUp)    { y = 0.5; x = 0;   rx = 0; }
        if (dpadDown)  { y = -0.5; x = 0;  rx = 0; }
        if (dpadLeft)  { x = -0.5; y = 0;  rx = 0; }
        if (dpadRight) { x = 0.5; y = 0;   rx = 0; }

        // Calculate wheel powers
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Telemetry (single update)
        telemetry.addData("FL", frontLeftPower);
        telemetry.addData("FR", frontRightPower);
        telemetry.addData("BL", backLeftPower);
        telemetry.addData("BR", backRightPower);
        telemetry.update();
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
