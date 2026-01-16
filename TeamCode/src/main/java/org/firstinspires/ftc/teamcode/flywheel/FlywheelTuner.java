package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuner extends OpMode {

    DcMotorEx flywheelMotor;

    double highVelocity = 2300;
    double lowVelocity = 900;
    double targetVelocity = highVelocity;

    double P = 0.00;
    double F = 0.00;

    double[] stepSizes = {0.01, 0.001, 0.0001};
    int stepIndex = 1;

    // button state tracking (edge detection)
    boolean lastUp, lastDown, lastLeft, lastRight, lastLB, lastRB;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        updatePIDF();
    }

    @Override
    public void loop() {

        // ---- EDGE DETECTION ----
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        // Adjust P
        if (up && !lastUp)    P += stepSizes[stepIndex];
        if (down && !lastDown) P -= stepSizes[stepIndex];

        // Adjust F
        if (right && !lastRight) F += stepSizes[stepIndex];
        if (left && !lastLeft)  F -= stepSizes[stepIndex];

        // Change step size
        if (lb && !lastLB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Toggle velocity
        if (rb && !lastRB) {
            targetVelocity =
                    (targetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        // Save states
        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
        lastLB = lb;
        lastRB = rb;

        // Apply PIDF + velocity
        updatePIDF();
        flywheelMotor.setVelocity(targetVelocity);

        // Telemetry
        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Velocity", "%.1f", flywheelMotor.getVelocity());
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Step", stepSizes[stepIndex]);
        telemetry.update();
    }

    void updatePIDF() {
        flywheelMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );
    }
}