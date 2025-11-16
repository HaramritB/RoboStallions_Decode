package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Flywheel RPM + Hood Test", group = "Testing")
public class FlywheelTest extends LinearOpMode {

    private Flywheel flywheel;

    private static final double RPM_STEP = 10;
    private double targetRPM = 0;

    private boolean rbPressedLast = false;

    @Override
    public void runOpMode() {
        flywheel = new Flywheel(hardwareMap);

        telemetry.addLine("Flywheel Test Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Increase RPM
            if (gamepad1.right_trigger > 0.5) {
                targetRPM += RPM_STEP;
                sleep(150);
            }

            // LB resets RPM instantly
            if (gamepad1.left_bumper) {
                targetRPM = 0;
            }

            // Apply new RPM
            flywheel.setTargetRPM(targetRPM);

            // RB toggles hood
            boolean rb = gamepad1.right_bumper;
            if (rb && !rbPressedLast) {
                flywheel.toggleHood();
            }
            rbPressedLast = rb;

            // Display current RPM
            double currentRPM =
                    (flywheel.getAverageVelocity() / flywheel.getTicksPerRev()) * 60.0;

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Current RPM", String.format("%.1f", currentRPM));
            telemetry.addData("Hood Position", flywheel.getHoodPosition());
            telemetry.update();
        }
    }
}
