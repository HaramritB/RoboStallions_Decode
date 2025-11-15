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

    // PID / proportional control
    private final double kP = 0.03;
    private final double maxPower = 0.75;

    // Low-pass filter
    private final double alpha = 0.4;
    private double filteredTx = 0;
    private boolean firstReading = true;

    // Deadband to ignore small errors
    private final double deadband = 0.5;

    public AprilTagTracking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotation");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure pipeline for AprilTag detection
        limelight.pipelineSwitch(7);
        limelight.start();
    }

    /** Call this in your TeleOp loop when auto-tracking is enabled */
    public void update() {
        LLResult result = limelight.getLatestResult();

        double tx = 0;
        boolean valid = false;

        if (result != null && result.isValid()) {
            tx = result.getTx();
            valid = true;

            // Low-pass filter
            if (firstReading) {
                filteredTx = tx;
                firstReading = false;
            } else {
                filteredTx = alpha * tx + (1 - alpha) * filteredTx;
            }

            // Deadband
            if (Math.abs(filteredTx) < deadband) filteredTx = 0;

            // Compute power
            double power = -kP * filteredTx;
            power = clamp(power, -maxPower, maxPower);

            rotationMotor.setPower(power);

            telemetry.addLine("Auto Tracking");
            telemetry.addData("Raw tx", tx);
            telemetry.addData("Filtered tx", filteredTx);
            telemetry.addData("Motor Power", power);

        } else {
            rotationMotor.setPower(0);
            firstReading = true;
            telemetry.addLine("No Tag Detected");
        }

        telemetry.update();
    }

    /** Manual control for turret (e.g., RB/LB) */
    public void setManualPower(double power) {
        rotationMotor.setPower(clamp(power, -maxPower, maxPower));
        firstReading = true; // reset filtered value when manual control is used
    }

    /** Stop turret immediately */
    public void stop() {
        rotationMotor.setPower(0);
        firstReading = true;
    }

    /** Utility clamp method */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}