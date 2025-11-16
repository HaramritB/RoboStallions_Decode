package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AprilTagTracking {

    private final Limelight3A limelight;
    private final DcMotor turretMotor;
    private final Telemetry telemetry;

    // PID (Proportional only is fine for turret steering)
    private final double kP = 0.05;
    private final double maxPower = 0.6;

    // Low-Pass Filter settings
    private final double alpha = 0.15;   // smooth, decrease if oscillation happens
    private double filteredTx = 0;
    private boolean firstReading = true;

    // Ignore very small noise
    private final double deadband = 0.4;

    // If no tag is seen for this long, turret stops
    private long lastValidTime = 0;
    private final long tagTimeout = 250; // ms

    public AprilTagTracking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        turretMotor = hardwareMap.get(DcMotor.class, "rotation");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run AprilTag pipeline
        limelight.pipelineSwitch(7);
        limelight.start();
    }

    /** Call this every loop */
    public void update() {
        LLResult result = limelight.getLatestResult();

        boolean hasTag = result != null && result.isValid();

        if (hasTag) {
            lastValidTime = System.currentTimeMillis();

            double tx = result.getTx();

            // Low-pass filter
            if (firstReading) {
                filteredTx = tx;
                firstReading = false;
            } else {
                filteredTx = alpha * tx + (1 - alpha) * filteredTx;
            }

            // Deadband small errors
            if (Math.abs(filteredTx) < deadband) filteredTx = 0;

            // Proportional control
            double power = -kP * filteredTx;
            power = clamp(power, -maxPower, maxPower);

            turretMotor.setPower(power);

            telemetry.addLine("TAG LOCK");
            telemetry.addData("Raw tx", tx);
            telemetry.addData("Filtered tx", filteredTx);
            telemetry.addData("Power", power);

        } else {
            // If tag recently seen, hold last error to prevent twitching
            long dt = System.currentTimeMillis() - lastValidTime;

            if (dt < tagTimeout) {
                // keep last power, do nothing
                telemetry.addLine("Tag Lost (Holding)");
            } else {
                // stop turret after timeout
                turretMotor.setPower(0);
                firstReading = true;
                telemetry.addLine("No Tag");
            }
        }

        telemetry.update();
    }

    /** Manual override from driver */
    public void manual(double power) {
        turretMotor.setPower(clamp(power, -maxPower, maxPower));
        firstReading = true; // reset smoothing
    }

    /** Full stop */
    public void stop() {
        turretMotor.setPower(0);
        firstReading = true;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}