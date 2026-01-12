package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flywheel {

    private final DcMotorEx flywheel;
    private final Servo hoodServo;

    private static final double HOOD_CLOSED = 0.0;
    private static final double HOOD_OPEN = 0.8;
    private static final double TICKS_PER_REV = 8192; // GoBILDA 6374

    private boolean hoodOpen = false;

    public Flywheel(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // PIDF tuned for single GoBILDA 6374
        double kP = 10.0;
        double kI = 0.1;
        double kD = 0.0;
        double kF = 1.0;

        flywheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.scaleRange(0.0, 1.0);
    }

    public void setTargetRPM(double rpm) {
        if (rpm <= 0) {
            stop();
            return;
        }

        double velocity = (rpm * TICKS_PER_REV) / 60.0;
        flywheel.setVelocity(velocity);
    }

    public void stop() {
        flywheel.setVelocity(0);
    }

    public double getAverageVelocity() {
        return flywheel.getVelocity();
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

    public void setRawPower(double power) {
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(power);
    }
}
