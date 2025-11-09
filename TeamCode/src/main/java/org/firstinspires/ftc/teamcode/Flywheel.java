package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Flywheel {
    private DcMotorEx low;
    private DcMotorEx high;
    private Servo hoodServo;

    private static final PIDFCoefficients PIDF_COEFFS = new PIDFCoefficients(10.0, 3.0, 0.0, 12.0);
    private static final double HOOD_CLOSED = 0.0;
    private static final double HOOD_OPEN = 0.8;

    // Adjust this for your specific motor (GoBilda 1150 = 28 ticks/rev, 435 RPM = 28, etc.)
    private static final double TICKS_PER_REV = 28.0;

    public Flywheel(HardwareMap hardwareMap) {
        low = hardwareMap.get(DcMotorEx.class, "low");
        high = hardwareMap.get(DcMotorEx.class, "high");

        low.setDirection(DcMotorSimple.Direction.FORWARD);
        high.setDirection(DcMotorSimple.Direction.REVERSE);

        low.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        high.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        low.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        high.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        low.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        high.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        low.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF_COEFFS);
        high.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, PIDF_COEFFS);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.scaleRange(0.0, 1.0);
    }

    /** Flywheel velocity control (ticks/sec) */
    public void setVelocity(double velocity) {
        low.setVelocity(velocity);
        high.setVelocity(velocity);
    }

    /** NEW: Flywheel control in RPM */
    public void setTargetRPM(double rpm) {
        double ticksPerRev = low.getMotorType().getTicksPerRev();
        double targetVelocity = (rpm * ticksPerRev) / 60.0; // ticks per second
        setVelocity(targetVelocity);
    }

    public void setPower(double power) {
        low.setPower(power);
        high.setPower(power);
    }

    public void stop() {
        setPower(0);
    }

    public double getAverageVelocity() {
        return (low.getVelocity() + high.getVelocity()) / 2.0;
    }

    /** Hood control */
    public void openHood() {
        hoodServo.setPosition(HOOD_OPEN);
    }

    public void closeHood() {
        hoodServo.setPosition(HOOD_CLOSED);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }
}
