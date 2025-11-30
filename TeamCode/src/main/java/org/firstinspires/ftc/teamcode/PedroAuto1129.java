package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.TeleOp;
import com.pedropathing.util.Timer;


// ROBOT MUST BE FULLY TUNED before doing this

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class PedroAuto1129 extend OpMode{

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

    public int endX, endY, endHeading;
    private final Pose endPose = new Pose(endX, endY, endHeading);


    private PathChain driveStartPosShootPos; // single path
    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.setHeading())
                .build;

        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVER_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                pathState = PathState.SHOOT_RELOAD;
                break;

            case SHOOT_PRELOAD:
                if(!follower.isBusy()) {
                    telemetry.addLine("Done Path 1");
                }

                break;

            case DRIVE_SHOOT_ENDPOS:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1")
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }


    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getX());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());


    }
}
