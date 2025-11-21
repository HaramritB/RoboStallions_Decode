package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Flywheel RPM + Hood + Distance Test", group = "Testing")
public class FlywheelTest extends LinearOpMode {

    private Flywheel flywheel;
    private Distance distanceSensor;

    private static final double RPM_STEP = 1;  // Increased step for easier testing
    private double targetRPM = 0;

    private boolean rbPressedLast = false;
    private boolean rtPressedLast = false;
    private boolean ltPressedLast = false;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing flywheel...");
        telemetry.update();

        try {
            flywheel = new Flywheel(hardwareMap);
            telemetry.addLine("✓ Flywheel initialized");
        } catch (Exception e) {
            telemetry.addLine("✗ FLYWHEEL INIT FAILED:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            while (opModeIsActive()) {
                sleep(100);
            }
            return;
        }

        try {
            distanceSensor = new Distance(hardwareMap);
            telemetry.addLine("✓ Distance sensor initialized");
        } catch (Exception e) {
            telemetry.addLine("⚠ Distance sensor failed (non-critical)");
            distanceSensor = null;
        }

        telemetry.addLine();
        telemetry.addLine("Flywheel Test Ready");
        telemetry.addLine("RT = +100 RPM | LT = -100 RPM");
        telemetry.addLine("LB = Reset | RB = Toggle Hood");
        telemetry.addLine("A = Raw Power Test (30%)");
        telemetry.addLine("Y = Raw Power Test (60%)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --------------------------
            // Raw power diagnostic OVERRIDES everything else
            // --------------------------
            boolean diagnosticMode = gamepad1.a || gamepad1.y;

            if (gamepad1.a) {
                flywheel.setRawPower(0.5);  // Increased to 50%
                telemetry.addLine("╔═══════════════════════════╗");
                telemetry.addLine("║ RAW POWER MODE: 50%       ║");
                telemetry.addLine("║ Release A to exit         ║");
                telemetry.addLine("╚═══════════════════════════╝");
                telemetry.update();
                continue;  // Skip all other controls
            } else if (gamepad1.y) {
                flywheel.setRawPower(0.8);  // Increased to 80%
                telemetry.addLine("╔═══════════════════════════╗");
                telemetry.addLine("║ RAW POWER MODE: 80%       ║");
                telemetry.addLine("║ Release Y to exit         ║");
                telemetry.addLine("╚═══════════════════════════╝");
                telemetry.update();
                continue;  // Skip all other controls
            }

            // --------------------------
            // RPM control
            // --------------------------
            /*
            boolean rtPressed = gamepad1.right_trigger > 0.5;
            boolean ltPressed = gamepad1.left_trigger > 0.5;

            if (rtPressed && !rtPressedLast) {
                targetRPM += RPM_STEP;
            }
            rtPressedLast = rtPressed;

            if (ltPressed && !ltPressedLast) {
                targetRPM -= RPM_STEP;
            }
            ltPressedLast = ltPressed;

            // LB resets RPM
            if (gamepad1.left_bumper) {
                targetRPM = 0;
            }

            // Clamp RPM to reasonable range
            if (targetRPM < 0) targetRPM = 0;
            if (targetRPM > 6000) targetRPM = 6000;  // Safety limit

            flywheel.setTargetRPM(targetRPM);

             */
            boolean upHeld = false;
            boolean downHeld = false;

            if (gamepad1.dpad_up && !upHeld) {
                targetRPM += 5;
                upHeld = true;
            }
            if (gamepad1.dpadUpWasReleased()) {
                upHeld = false;
            }

            if (gamepad1.dpad_down && !downHeld) {
                targetRPM -= 5;
                downHeld = true;
            }
            if (gamepad1.dpadDownWasReleased()) {
                downHeld = false;
            }

            if (targetRPM<0) {
                targetRPM=0;
            }
            if (targetRPM>100) {
                targetRPM=100;
            }
            //flywheel.setTargetRPM(targetRPM);
            flywheel.setRawPower(targetRPM/100);


            // --------------------------
            // Hood toggle via RB
            // --------------------------
            boolean rb = gamepad1.right_bumper;
            if (rb && !rbPressedLast) {
                flywheel.toggleHood();
            }
            rbPressedLast = rb;

            // --------------------------
            // Raw power diagnostic (A button)
            // --------------------------
            if (gamepad1.a) {
                flywheel.setRawPower(0.3);
                telemetry.addLine(">>> RAW POWER MODE: 30% <<<");
            } else if (gamepad1.y) {
                flywheel.setRawPower(0.6);
                telemetry.addLine(">>> RAW POWER MODE: 60% <<<");
            }

            // --------------------------
            // Distance sensor
            // --------------------------
            if (distanceSensor != null) {
                distanceSensor.update();
            }

            // --------------------------
            // Telemetry
            // --------------------------
            double currentRPM = (flywheel.getAverageVelocity() / flywheel.getTicksPerRev()) * 60.0;
            double lowRPM = (flywheel.getLowVelocity() / flywheel.getTicksPerRev()) * 60.0;
            double highRPM = (flywheel.getHighVelocity() / flywheel.getTicksPerRev()) * 60.0;

            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Average RPM", "%.1f", currentRPM);
            telemetry.addData("Low Motor RPM", "%.1f", lowRPM);
            telemetry.addData("High Motor RPM", "%.1f", highRPM);
            telemetry.addData("Hood Position", "%.2f", flywheel.getHoodPosition());
            telemetry.addLine();

            // Show button states for debugging
            telemetry.addData("A Button", gamepad1.a ? "PRESSED" : "not pressed");
            telemetry.addData("Y Button", gamepad1.y ? "PRESSED" : "not pressed");
            telemetry.addLine();

            if (distanceSensor != null && distanceSensor.hasValidTag()) {
                telemetry.addData("Distance (m)", "%.2f", distanceSensor.getDistanceMeters());
            } else {
                telemetry.addData("Distance", "No Tag");
            }

            telemetry.update();
        }
    }
}