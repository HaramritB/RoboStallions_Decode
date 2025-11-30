package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import java.nio.file.Path;


// ROBOT MUST BE FULLY TUNED

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class ZainabBlueDepot extend OpMode{

    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState {

    }

    PathState pathState;
    public int headingVal, xVal, yVal;
    // start position
    private final Pose startPose = new Pose(xVal, yVal, Math.toRadians(headingVal));

    // position for shooting
    public int shootX, shootY, shootHeading;
    private final Pose shootPose = new Pose(shootX, shootY, shootHeading);
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
