package org.firstinspires.ftc.teamcode.limelight;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Localization Test", group = "Test")
public class TurretLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        TurretCameraLocalizer localizer = new TurretCameraLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // 1. Run Turret Tracking
            turret.update();

            // 2. Get Field Position
            double turretAngle = turret.getTurretAngleRadians();
            Pose robotPose = localizer.getPoseFromApriltag(turretAngle);

            // 3. Telemetry Feedback
            telemetry.addData("Turret Angle (Deg)", Math.toDegrees(turretAngle));
            if (robotPose != null) {
                telemetry.addLine("--- ROBOT POSITION ---");
                telemetry.addData("X (Inches)", "%.2f", robotPose.getX());
                telemetry.addData("Y (Inches)", "%.2f", robotPose.getY());
                telemetry.addData("Heading (Deg)", "%.2f", Math.toDegrees(robotPose.getHeading()));
            } else {
                telemetry.addLine("NO APRIL TAG DETECTED");
            }
            telemetry.update();
        }
    }
}