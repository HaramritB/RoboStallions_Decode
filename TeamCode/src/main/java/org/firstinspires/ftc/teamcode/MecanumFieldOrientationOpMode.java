package org.firstinspires.ftc.teamcode;

public class MecanumFieldOrientationOpMode {

    MecanumDrive drive = new MecanumDrive();

    double forward,strafe,rotate;
    @Override
    public void init() {

    }
    @Override
    public void loop(){
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);

    }


}
