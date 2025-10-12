package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

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

        while (opModeIsActive()) {
            // Flywheel control
            if (gamepad1.a) {
                flywheel.setVelocity(2000); // tune ticks/sec
            } else if (gamepad1.b) {
                flywheel.setPower(0.8);
            } else {
                flywheel.stop();
            }

            // Hood control
            if (gamepad1.x) {
                flywheel.openHood();
            } else if (gamepad1.y) {
                flywheel.closeHood();
            }

            telemetry.addData("Flywheel Vel", flywheel.getAverageVelocity());
            telemetry.addData("Hood Pos", flywheel.getHoodPosition());
            telemetry.update();
        }

        flywheel.stop();
    }
}
