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

    // PID constants
    private final double kP = 0.03;
    private final double kI = 0.0;
    private final double kD = 0.002;

    private double integral = 0;
    private double lastError = 0;

    private final double maxPower = 0.5;

    // Turret rotation limits
    private final double maxAngle = 180;
    private final double minAngle = -180;
    private double cumulativeAngle = 0;
    private int lastEncoderPos = 0;

    private final double ticksPerDegree = 7.11;

    // Low-pass filter for tx
    private double filteredTx = 0;
    private final double alpha = 0.3;

    // Faster return factor
    private final double returnMultiplier = 1.5;

    public AprilTagTracking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotation");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(7);
        limelight.start();

        lastEncoderPos = rotationMotor.getCurrentPosition();
    }

    public void update() {
        // Update cumulative angle
        int currentEncoder = rotationMotor.getCurrentPosition();
        double deltaDegrees = (currentEncoder - lastEncoderPos) / ticksPerDegree;
        cumulativeAngle += deltaDegrees;
        lastEncoderPos = currentEncoder;

        // Clamp cumulative angle
        cumulativeAngle = Math.max(minAngle, Math.min(maxAngle, cumulativeAngle));

        LLResult result = limelight.getLatestResult();
        double power;

        if (result != null && result.isValid()) {
            // Track AprilTag
            double tx = result.getTx();
            filteredTx = alpha * tx + (1 - alpha) * filteredTx;

            double error = filteredTx;
            integral += error;
            double derivative = error - lastError;
            power = kP * error + kI * integral + kD * derivative;
            lastError = error;

            // Clamp power and apply soft stop near limits
            double softLimitFactor = getSoftLimitFactor();
            power = Math.max(-maxPower, Math.min(maxPower, power)) * softLimitFactor;

            telemetry.addLine("Tracking AprilTag");
            telemetry.addData("tx", tx);

        } else {
            // No tag → return to 0° smoothly
            double error = -cumulativeAngle; // target = 0°
            integral += error;
            double derivative = error - lastError;
            power = kP * error + kI * integral + kD * derivative;
            lastError = error;

            // Apply faster return multiplier
            power *= returnMultiplier;

            // Soft stop near limits
            double softLimitFactor = getSoftLimitFactor();
            power *= softLimitFactor;

            // Stop if within threshold
            if (Math.abs(cumulativeAngle) < 1.0) power = 0;

            telemetry.addLine("No valid AprilTag → returning to 0°");
        }

        rotationMotor.setPower(power);

        telemetry.addData("Motor Power", power);
        telemetry.addData("Cumulative Angle", cumulativeAngle);
        telemetry.update();
    }

    // Soft stop factor: reduces power near ±180° for smoother motion
    private double getSoftLimitFactor() {
        double buffer = 20; // degrees before hard stop to start reducing power
        double factor = 1.0;

        if (cumulativeAngle > maxAngle - buffer) {
            factor = Math.max(0, (maxAngle - cumulativeAngle) / buffer);
        } else if (cumulativeAngle < minAngle + buffer) {
            factor = Math.max(0, (cumulativeAngle - minAngle) / buffer);
        }

        return factor;
    }

    public void stop() {
        rotationMotor.setPower(0);
    }

    // Manual control when AprilTag mode is off
    public void setManualPower(double power) {
        double softLimitFactor = getSoftLimitFactor();
        power *= softLimitFactor;
        rotationMotor.setPower(power);
    }
}