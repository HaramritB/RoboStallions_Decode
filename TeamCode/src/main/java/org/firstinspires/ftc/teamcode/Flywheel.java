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

    public Flywheel(HardwareMap hardwareMap) {
        low = hardwareMap.get(DcMotorEx.class, "low");
        high = hardwareMap.get(DcMotorEx.class, "high");

        low.setDirection(DcMotorSimple.Direction.FORWARD);
        high.setDirection(DcMotorSimple.Direction.REVERSE);

        low.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        high.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        low.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        high.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.scaleRange(0.0, 1.0);
    }

    /** Spin flywheel to a target RPM */
    public void setTargetRPM(double rpm) {
        // Convert RPM to ticks/sec
        double ticksPerRev = low.getMotorType().getTicksPerRev();
        double velocity = (rpm * ticksPerRev) / 60.0;
        low.setVelocity(velocity);
        high.setVelocity(velocity);
    }

    public void stop() {
        low.setPower(0);
        high.setPower(0);
    }

    public double getAverageVelocity() {
        return (low.getVelocity() + high.getVelocity()) / 2.0;
    }

    /** Hood controls */
    public void openHood() {
        hoodServo.setPosition(HOOD_OPEN);
    }

    public void closeHood() {
        hoodServo.setPosition(HOOD_CLOSED);
    }

    public void setHoodPosition(double pos) {
        hoodServo.setPosition(pos);
    }

    public double getHoodPosition() {
        return hoodServo.getPosition();
    }
}
