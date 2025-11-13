package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Kicker Control (Hold A)", group = "Testing")
public class Kicker extends LinearOpMode {

    private Servo kickerServo;

    // Adjust these for your kicker’s motion range
    private static final double REST_POSITION = 0.275;  // resting angle
    private static final double KICK_POSITION = 0.0;    // kicking angle

    @Override
    public void runOpMode() {
        // Initialize servo
        kickerServo = hardwareMap.get(Servo.class, "kicker");

        // Set initial position
        kickerServo.setPosition(REST_POSITION);

        telemetry.addLine("Kicker ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                // Hold A → stay in kick position
                kickerServo.setPosition(KICK_POSITION);
            } else {
                // Release A → go back to rest
                kickerServo.setPosition(REST_POSITION);
            }

            telemetry.addData("Servo Position", kickerServo.getPosition());
            telemetry.update();
        }
    }
}
