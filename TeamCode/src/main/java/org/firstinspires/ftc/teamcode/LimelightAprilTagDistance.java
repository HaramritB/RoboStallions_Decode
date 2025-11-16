package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Limelight AprilTag Distance", group = "Sensor")
public class LimelightAprilTagDistance extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize the Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(7); // AprilTag pipeline
        limelight.start(); // Begin polling

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                telemetry.addData("Pipeline", result.getPipelineIndex());
                telemetry.addData("# AprilTags Detected", fiducials.size());
                telemetry.addData("Capture Latency", "%.2f ms", result.getCaptureLatency());
                telemetry.addData("Target Latency", "%.2f ms", result.getTargetingLatency());
                telemetry.addData("Parse Latency", "%.2f ms", result.getParseLatency());

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    telemetry.addLine(String.format("\n==== Tag ID %d ====", fiducial.getFiducialId()));

                    Pose3D pose = fiducial.getTargetPoseCameraSpace();
                    Pose3D robotPose = fiducial.getTargetPoseRobotSpace();

                    // Define x,y,z ahead of time
                    double x = 0, y = 0, z = 0;
                    boolean poseValid = false;

                    if (pose != null) {
                        poseValid = true;

                        x = pose.getPosition().x;
                        y = pose.getPosition().y;
                        z = pose.getPosition().z;

                        double distance = Math.sqrt(x * x + y * y + z * z);

                        telemetry.addData("Distance", "%.2f meters", distance);
                        telemetry.addData("X (Right)", "%.2f meters", x);
                        telemetry.addData("Y (Down)", "%.2f meters", y);
                        telemetry.addData("Z (Forward)", "%.2f meters", z);

                        telemetry.addData("Pitch", "%.1f degrees", Math.toDegrees(pose.getOrientation().getPitch()));
                        telemetry.addData("Yaw", "%.1f degrees", Math.toDegrees(pose.getOrientation().getYaw()));
                        telemetry.addData("Roll", "%.1f degrees", Math.toDegrees(pose.getOrientation().getRoll()));
                    }

                    if (robotPose != null) {
                        telemetry.addData("Robot X", "%.2f meters", robotPose.getPosition().x);
                        telemetry.addData("Robot Y", "%.2f meters", robotPose.getPosition().y);
                        telemetry.addData("Robot Z", "%.2f meters", robotPose.getPosition().z);
                    }

                    // Now x,y,z exist here safely
                    if (poseValid) {
                        double horizontalAngle = Math.toDegrees(Math.atan2(x, z));
                        double verticalAngle = Math.toDegrees(Math.atan2(y, z));

                        telemetry.addData("Horizontal Angle", "%.2f degrees", horizontalAngle);
                        telemetry.addData("Vertical Angle", "%.2f degrees", verticalAngle);
                    }
                }

            } else {
                telemetry.addData("Status", "No valid data");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
