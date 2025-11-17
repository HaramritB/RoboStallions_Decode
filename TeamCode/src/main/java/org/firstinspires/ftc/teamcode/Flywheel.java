package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flywheel {

    private final DcMotorEx low;
    private final DcMotorEx high;
    private final Servo hoodServo;

    private static final double HOOD_CLOSED = 0.0;
    private static final double HOOD_OPEN = 0.8;
    private static final double TICKS_PER_REV = 8192;      // GoBILDA 6374 encoder

    private boolean hoodOpen = false;

    public Flywheel(HardwareMap hardwareMap) {
        low = hardwareMap.get(DcMotorEx.class, "low");
        high = hardwareMap.get(DcMotorEx.class, "high");

        // Directions: motors must spin in SAME direction for flywheel
        low.setDirection(DcMotorSimple.Direction.REVERSE);
        high.setDirection(DcMotorSimple.Direction.FORWARD);  // Swapped from previous

        low.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        high.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Start in velocity mode
        low.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        high.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        low.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        high.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // PIDF tuning for 12V GoBILDA 6374
        // These values may need adjustment based on your specific setup
        double kP = 10.0;   // Increased from 0.05
        double kI = 0.1;    // Small integral term
        double kD = 0.0;
        double kF = 1.0;    // Increased from 0.02 - adjust based on testing

        low.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        high.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.scaleRange(0.0, 1.0);
    }

    public void setTargetRPM(double rpm) {
        if (rpm == 0) {
            // Full stop
            stop();
            return;
        }

        // Always use velocity mode for consistent control
        low.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        high.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Convert RPM to ticks per second
        double velocity = (rpm * TICKS_PER_REV) / 60.0;

        low.setVelocity(velocity);
        high.setVelocity(velocity);
    }

    public void stop() {
        low.setVelocity(0);
        high.setVelocity(0);
    }

    public double getAverageVelocity() {
        return (low.getVelocity() + high.getVelocity()) / 2.0;
    }

    public double getLowVelocity() {
        return low.getVelocity();
    }

    public double getHighVelocity() {
        return high.getVelocity();
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    public void openHood() {
        hoodServo.setPosition(HOOD_OPEN);
        hoodOpen = true;
    }

    public void closeHood() {
        hoodServo.setPosition(HOOD_CLOSED);
        hoodOpen = false;
    }

    public void toggleHood() {
        if (hoodOpen) closeHood();
        else openHood();
    }

    public double getTicksPerRev() {
        return TICKS_PER_REV;
    }

    // Diagnostic method - bypasses velocity control to test basic motor function
    public void setRawPower(double power) {
        low.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        high.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        low.setPower(power);
        high.setPower(power);
    }
}