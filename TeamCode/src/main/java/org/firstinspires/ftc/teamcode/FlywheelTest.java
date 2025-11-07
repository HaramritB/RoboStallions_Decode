package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        double maxVelocity = 1000; // encoder ticks/sec
        double rampRate = 0.1;     // smooth acceleration factor
        double loopDelay = 50;

        while (opModeIsActive()) {

            // Velocity ramping
            if (gamepad1.a) {
                if (velocity < maxVelocity) {
                    velocity += rampRate * (maxVelocity - velocity);
                }
            } else {
                velocity += rampRate * (0 - velocity);
            }

            flywheel.setVelocity(velocity);

            // Hood control
            if (gamepad1.x) {
                flywheel.openHood();
            } else if (gamepad1.y) {
                flywheel.closeHood();
            }

            // Telemetry
            telemetry.addData("Flywheel Target Vel", velocity);
            telemetry.addData("Flywheel Avg Vel", flywheel.getAverageVelocity());
            telemetry.addData("Hood Pos", flywheel.getHoodPosition());
            telemetry.update();

            sleep((long) loopDelay);
        }

        flywheel.stop();
    }


}
