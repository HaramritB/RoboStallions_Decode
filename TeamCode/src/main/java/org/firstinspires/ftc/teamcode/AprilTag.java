package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "AprilTag LimeLight Test", group = "Testing")
public class AprilTag extends OpMode {

    private Limelight3A limelight;
    private IMU imu;
    private DcMotorEx rotation;
    private Flywheel flywheel;

    // Tunables
    private static final double kP_turret = 0.02;   // rotation proportional gain
    private static final double kMinPower = 0.05;   // deadband for turret motion

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        rotation = hardwareMap.get(DcMotorEx.class, "rotation");
        flywheel = new Flywheel(hardwareMap);

        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight.pipelineSwitch(8); // Make sure pipeline 8 is AprilTag detection
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2(); // <-- define botPose here

            double distance = Math.hypot(botPose.getPosition().x, botPose.getPosition().y); // meters

            final double SHOOTER_HEIGHT_M = 18.0 * 0.0254; // 18 inches -> meters
            final double TARGET_HEIGHT_M = 42.0 * 0.0254;  // 42 inches -> meters
            final double HOOD_ANGLE_DEG = 30.0;            // example hood angle; tune
            final double WHEEL_RADIUS_M = 0.048;          // 1" radius
            final double EFFICIENCY = 0.95;                // estimate for slip

            double targetRPM = ShooterMath.computeTargetWheelRPM(
                    distance,
                    SHOOTER_HEIGHT_M,
                    TARGET_HEIGHT_M,
                    HOOD_ANGLE_DEG,
                    WHEEL_RADIUS_M,
                    EFFICIENCY
            );

            if (Double.isNaN(targetRPM)) {
                telemetry.addLine("No valid solution for hood angle " + HOOD_ANGLE_DEG + "° at distance " + String.format("%.2f", distance) + " m");
                flywheel.stop();
            } else {
                flywheel.setTargetRPM(targetRPM);
                telemetry.addData("Target RPM", "%.0f", targetRPM);
            }

            // telemetry
            telemetry.addData("Bot X (m)", botPose.getPosition().x);
            telemetry.addData("Bot Y (m)", botPose.getPosition().y);
            telemetry.addData("Distance (m)", distance);

        } else {
            telemetry.addLine("No valid AprilTag detected");
            flywheel.stop();
        }

        telemetry.update();
    }
}
