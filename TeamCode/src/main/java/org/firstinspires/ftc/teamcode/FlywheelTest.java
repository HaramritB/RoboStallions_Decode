package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Flywheel;

@TeleOp(name = "Flywheel + Hood Test", group = "Testing")
public class FlywheelTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Flywheel flywheel = new Flywheel(hardwareMap);

        telemetry.addLine("Flywheel & Hood initialized.");
        telemetry.addLine("Press A: Spin flywheel (velocity mode)");
        telemetry.addLine("Press B: Spin flywheel (power mode)");
        telemetry.addLine("Press X: Open hood");
        telemetry.addLine("Press Y: Close hood");
        telemetry.update();

        waitForStart();

        double velocity = 0.0;
        double maxVelocity = 10;
        double rampRate = 0.5;
        double loopDelay = 50;

        while (opModeIsActive()) {
            // Flywheel control
            if (gamepad1.a) {
                if (velocity < maxVelocity) {
                    velocity += rampRate * (maxVelocity - velocity);
                }
            } else {
                velocity += rampRate * (0 - velocity);
            }
            flywheel.setVelocity(velocity);


            if (gamepad1.x) {
                flywheel.openHood();
            } else if (gamepad1.y) {
                flywheel.closeHood();
            }

            telemetry.addData("Flywheel Vel", velocity);
            telemetry.addData("Hood Pos", flywheel.getAverageVelocity());
            telemetry.update();

            sleep((long) loopDelay);
        }

        flywheel.stop();
    }
}
