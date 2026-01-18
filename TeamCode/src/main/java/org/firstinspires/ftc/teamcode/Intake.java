package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


public class Intake {

    private final DcMotor intakeMotor;

    private final double defaultPower = 0.75;
    // private final double ejectPower = -0.9;

    private boolean isRunning = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD); // reverse if needed
    }

    // Call this every loop and pass gamepad1
    public void update(Gamepad gamepad) {
        if (gamepad.x) {
            intakeMotor.setPower(defaultPower);
            isRunning = true;
        } else if (gamepad.b) {
            intakeMotor.setPower(0);
            isRunning = false;
        }
    }

    // Stop motor safely
    public void stop() {
        intakeMotor.setPower(0);
        isRunning = false;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public double getMotorPower() {
        return intakeMotor.getPower();
    }
}
