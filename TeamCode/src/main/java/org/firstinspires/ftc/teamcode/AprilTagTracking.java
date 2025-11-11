package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class AprilTagTracking {

    private final Limelight3A limelight;
    private final DcMotor rotationMotor;
    private final Telemetry telemetry;

    private final double kP = 0.03;
    private final double maxPower = 0.5;

    public AprilTagTracking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        rotationMotor = hardwareMap.get(DcMotor.class, "rotation");

        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(7);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D resultBotPose = result.getBotpose();
            double tx = result.getTx();

            double power = -kP * tx;
            power = Math.max(-maxPower, Math.min(maxPower, power));

            rotationMotor.setPower(power);

            telemetry.addData("tx", tx);
            telemetry.addData("Pose", resultBotPose.toString());
            telemetry.addData("Motor Power", power);
        } else {
            rotationMotor.setPower(0);
            telemetry.addLine("No valid AprilTag detected");
        }

        telemetry.update();
    }

    public void stop() {
        rotationMotor.setPower(0);
    }
}
