package org.firstinspires.ftc.teamcode.limelight;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

public class TurretCameraLocalizer {
    private final Limelight3A limelight;

    // Physical distance from turret center to camera center (in inches)
    // 153mm converted to inches
    private static final double CAMERA_OFFSET_INCHES = 153.0 / 25.4;

    public TurretCameraLocalizer(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        // Ensure Limelight is on the AprilTag pipeline (usually pipeline 7)
        limelight.pipelineSwitch(7);
        limelight.start();
    }

    public Pose getPoseFromApriltag(double currentTurretAngleRadians) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Using MegaTag (Botpose) is the easiest way if you have
            // the field map uploaded to the Limelight.
            // Note: Use getBotpose() for blue alliance origin or getBotpose_wpired() for red.
            Pose3D botpose = result.getBotpose();

            if (botpose != null) {
                double camX = botpose.getPosition().x;
                double camY = botpose.getPosition().y;
                double camHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                // If the Limelight is on a TURRET, the 'Botpose' it returns is
                // actually the Camera's Field Pose. We must shift it to the Robot Center.
                double robotX = camX - (CAMERA_OFFSET_INCHES * Math.cos(camHeading));
                double robotY = camY - (CAMERA_OFFSET_INCHES * Math.sin(camHeading));

                // Calculate Robot Heading: Field Heading minus Turret Offset
                double robotHeading = camHeading - currentTurretAngleRadians;

                // Return as a Pedro Pathing Pose (assuming inches)
                // Note: Limelight returns meters by default, convert to inches (* 39.37)
                return new Pose(robotX * 39.37, robotY * 39.37, robotHeading);
            }
        }
        return null;
    }

    public void stop() {
        limelight.stop();
    }
}