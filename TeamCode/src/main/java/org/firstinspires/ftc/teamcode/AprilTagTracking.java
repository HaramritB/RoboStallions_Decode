package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "AprilTag Rotation Align", group = "Testing")
public class AprilTagTracking extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        DcMotor rotationMotor = hardwareMap.get(DcMotor.class, "rotation");
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(10);

        limelight.pipelineSwitch(7);

        limelight.start();

        waitForStart();

        double kP = 0.03;
        double maxPower = 0.5;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D resultBotPose = result.getBotpose();
                    double tx = result.getTx();

                    double power = -kP * tx;

                    if (power > maxPower) power = maxPower;
                    if (power < -maxPower) power = -maxPower;

                    rotationMotor.setPower(power);

                    telemetry.addData("tx", tx);
                    telemetry.addData("Pose", resultBotPose.toString());
                    telemetry.addData("Motor Power", power);
                }
            } else {
                rotationMotor.setPower(0);
            }

            telemetry.update();
        }
    }
}
