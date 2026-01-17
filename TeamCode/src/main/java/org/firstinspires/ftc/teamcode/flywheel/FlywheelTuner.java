package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Transfer;

@TeleOp(name = "Flywheel Test For Real", group = "Test")
public class FlywheelTuner extends OpMode {

    // Motors / subsystems
    private DcMotorEx flywheelMotor;
    private Intake intake;

    // Intake toggle
    private boolean intakeOn = false;
    private boolean lastX = false;

    // Flywheel velocities
    private double highVelocity = 2000;
    private double lowVelocity = 900;
    private double targetVelocity = highVelocity;

    // PIDF values
    private double P = 0.0;
    private double F = 13.5;

    // Tuning step sizes
    private final double[] stepSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 0;

    // Edge detection
    private boolean lastUp, lastDown, lastLeft, lastRight, lastLB, lastRB;

    @Override
    public void init() {

        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        updatePIDF();
    }

    @Override
    public void loop() {
        Intake intakeTest = new Intake(hardwareMap);

        intakeTest.update(gamepad1);

        Transfer transferTest = new Transfer(hardwareMap);

        transferTest.update(0.2);

        // Read buttons
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean x = gamepad1.x;

        // ---------------- PIDF TUNING ----------------
        if (up && !lastUp)    P += stepSizes[stepIndex];
        if (down && !lastDown) P -= stepSizes[stepIndex];

        if (right && !lastRight) F += stepSizes[stepIndex];
        if (left && !lastLeft)   F -= stepSizes[stepIndex];

        if (lb && !lastLB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (rb && !lastRB) {
            targetVelocity = (targetVelocity == highVelocity)
                    ? lowVelocity
                    : highVelocity;
        }

        // Save button states
        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
        lastLB = lb;
        lastRB = rb;

        // Apply PIDF + velocity
        updatePIDF();
        flywheelMotor.setVelocity(targetVelocity);

        // ---------------- TELEMETRY ----------------
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", "%.1f", flywheelMotor.getVelocity());
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.update();
    }

    private void updatePIDF() {
        flywheelMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );
    }
}
