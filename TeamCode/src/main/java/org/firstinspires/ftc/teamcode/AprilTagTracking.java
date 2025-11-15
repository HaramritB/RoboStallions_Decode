package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AprilTagTracking {

    private final Limelight3A limelight;
    private final DcMotor rotationMotor;
    private final Telemetry telemetry;

    private final double kP = 0.015;
    private final double kI = 0.000;
    private final double kD = 0.0015;

    private double integral = 0;
    private double lastError = 0;

    private final double maxPower = 0.45;
    private final double maxAngle = 180;
    private final double minAngle = -180;

    private final double ticksPerDegree = 7.11;
    private int lastEncoderPos = 0;
    private double turretAngle = 0;     // filtered, stable angle

    private final double alpha = 0.5;   // higher = sharper
    private double filteredTx = 0;

    public AprilTagTracking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotation");

        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure this is an AprilTag pipeline
        limelight.pipelineSwitch(0);
        limelight.start();

        lastEncoderPos = rotationMotor.getCurrentPosition();
    }

    public void update() {
        updateTurretAngle();

        LLResult result = limelight.getLatestResult();
        double power;

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            // Low-pass filter
            filteredTx = alpha * tx + (1 - alpha) * filteredTx;

            double error = filteredTx;

            power = computePID(error);
            power = clamp(power, -maxPower, maxPower);
            power *= softLimitFactor();

            telemetry.addLine("Tracking Tag");
            telemetry.addData("Filtered Tx", filteredTx);

        } else {
            double error = -turretAngle;   // Target = 0°

            power = computePID(error);
            power = clamp(power, -maxPower * 0.8, maxPower * 0.8); // gentler return
            power *= softLimitFactor();

            // Stop if close to center
            if (Math.abs(turretAngle) < 1.0) {
                power = 0;
                resetPID();
            }

            telemetry.addLine("Returning to Zero (No Tag)");
        }

        rotationMotor.setPower(power);

        telemetry.addData("Power", power);
        telemetry.addData("Turret Angle", turretAngle);
        telemetry.update();
    }

    private double computePID(double error) {
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    private void updateTurretAngle() {
        int encoder = rotationMotor.getCurrentPosition();
        int delta = encoder - lastEncoderPos;
        lastEncoderPos = encoder;

        turretAngle += delta / ticksPerDegree;

        turretAngle = clamp(turretAngle, minAngle, maxAngle);
    }

    private double softLimitFactor() {
        double buffer = 20;
        if (turretAngle > maxAngle - buffer)
            return Math.max(0, (maxAngle - turretAngle) / buffer);
        if (turretAngle < minAngle + buffer)
            return Math.max(0, (turretAngle - minAngle) / buffer);
        return 1.0;
    }

    public void resetAngle() {
        lastEncoderPos = rotationMotor.getCurrentPosition();
        turretAngle = 0;
    }

    public void resetPID() {
        integral = 0;
        lastError = 0;
        filteredTx = 0;
    }

    public void setManualPower(double power) {
        updateTurretAngle();
        power = clamp(power * softLimitFactor(), -maxPower, maxPower);
        rotationMotor.setPower(power);
    }

    public void stop() {
        rotationMotor.setPower(0);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}