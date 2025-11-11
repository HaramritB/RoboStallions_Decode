package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "CombinedTeleOp", group = "Competition")
public class CombinedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize subsystems ---
        MecanumTeleOpAxisLocked drive = new MecanumTeleOpAxisLocked(hardwareMap, telemetry);
        AprilTagTracking turret = new AprilTagTracking(hardwareMap, telemetry);
        FlywheelTest flywheelSystem = new FlywheelTest(hardwareMap, telemetry);

        telemetry.addLine("Combined TeleOp Initialized");
        telemetry.update();

        waitForStart();

        boolean aprilTagMode = false;

        while (opModeIsActive()) {

            // --- DRIVING CONTROL ---
            drive.drive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    gamepad1.dpad_left,
                    gamepad1.dpad_right
            );

            // --- FLYWHEEL CONTROL ---
            flywheelSystem.update(gamepad1);

            // --- APRILTAG ROTATION CONTROL ---
            if (gamepad1.a) {
                aprilTagMode = !aprilTagMode;  // toggle tracking
                sleep(300); // debounce
            }

            if (aprilTagMode) {
                turret.update();  // turn turret to track tag
            } else {
                turret.stop();    // stop turret rotation
            }
        }

        // --- Stop everything on exit ---
        drive.stop();
        turret.stop();
        flywheelSystem.stop();
    }
}
