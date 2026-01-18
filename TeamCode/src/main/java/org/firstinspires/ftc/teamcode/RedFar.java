package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;

@Autonomous
public class RedFar extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    //private DcMotor autoIntake;
    //private DcMotor autoTransfer;

    public enum PathState {
        // Start Position_End Position
        // Drive State > Movement State
        // Shoot > Scoring
        // 18 ball auto
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        SHOOT_PRELOADFIRST_INTAKE,
        FIRST_INTAKE,
        FIRST_INTAKE_SHOOT,

    }

    PathState pathState;

    private final Pose startPose = new Pose(82.6030989272944, 27.909415971394516, Math.toRadians(50));
    private final Pose midPose = new Pose(116.79022646007151, 1.3241954707985695, Math.toRadians(45));
    private final Pose firstIntake = new Pose(140.38617401668648, 11.491060786650783, Math.toRadians(60));

    // control point for first intake
    private final Pose c2 = new Pose(96.37604290822408, 5.150774731823589);


    private PathChain driveStartPosShootPos, shootPreloadPosFirstIntakePos, firstIntakePosShootPos, secondIntakePosShootPos,
            shootPosThirdIntake, thirdIntakeShootPos;


    public void buildPaths() {
        // coordinates for starting pose then ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
                .build();
        shootPreloadPosFirstIntakePos = follower.pathBuilder()
                .addPath(new BezierCurve(midPose, c2, firstIntake))
                .setLinearHeadingInterpolation(midPose.getHeading(), firstIntake.getHeading())
                .build();
        firstIntakePosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(firstIntake, startPose))
                .setLinearHeadingInterpolation(firstIntake.getHeading(), startPose.getHeading())
                .build();

    }


    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                // shoot preload
                // add shooting code here
                if (pathTimer.getElapsedTime() > 2.0) { // wait 2 seconds to shoot
                    setPathState(PathState.SHOOT_PRELOADFIRST_INTAKE);
                }
                break;
            case SHOOT_PRELOADFIRST_INTAKE:
                follower.followPath(shootPreloadPosFirstIntakePos);
                if (!follower.isBusy()) {
                    setPathState(PathState.FIRST_INTAKE);
                }
                break;
            case FIRST_INTAKE:
                // activate intake motors
                //autoIntake.setPower(1.0);
                follower.followPath(firstIntakePosShootPos);
                if (!follower.isBusy()) {
                    //autoIntake.setPower(0.0);
                    setPathState(PathState.FIRST_INTAKE_SHOOT);
                }
                break;
            case FIRST_INTAKE_SHOOT:
                // shoot intake balls
                // add shooting code here
                if (pathTimer.getElapsedTime() > 2.0) { // wait 2 seconds to shoot
                    // finished auto
                }
                break;
            default:
                break;

        }
    }

    // helper function
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        // opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // add mechs
        //autoIntake = hardwareMap.get(DcMotor.class, "intake");
        //autoTransfer = hardwareMap.get(DcMotor.class, "transfer");

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
        // add telemetry data here

        telemetry.addData("Path State:", pathState.toString());
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Time: ", pathTimer.getElapsedTime());
    }
}