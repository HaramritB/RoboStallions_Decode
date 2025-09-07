
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode. LinearOpMode; import com.qualcomm.robotcore.eventloop.opmode. TeleOp;
@TeleOp
public class Vision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // camera possibly Logitech 1
                        .setCameraResolution(new Size(640,480))
                        .build();

        waitForStart();
        while (!isStopRequested () && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor().getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);

            }

            telemetry.update();


            }
        }
}