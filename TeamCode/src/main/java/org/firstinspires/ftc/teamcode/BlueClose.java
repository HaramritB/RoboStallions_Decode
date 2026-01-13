package org.firstinspires.ftc.teamcode;


import android.graphics.Point;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
@Autonomous
public class BlueClose extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public enum PathState {
        // Start Position_End Position
        // Drive State > Movement State
        // Shoot > Scoring
        // 18 ball auto
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        SHOOT_PRELOADFIRST_INTAKE,
        FIRST_INTAKE,
        FIRST_INTAKESHOOT_POS,
        SHOOT_POS_FIRST,
        SHOOT_POSFIRST_GATE,
        SECOND_INTAKE,
        SECOND_INTAKESHOOT_POS,
        SHOOT_POS_SECOND,
        SHOOT_POSSECOND_GATE,
        THIRD_INTAKE,
        THIRD_INTAKESHOOT_POS,
        SHOOT_POS_THIRD,
        SHOOT_POSTHIRD_GATE,
        FOURTH_INTAKE,
        FOURTH_INTAKESHOOT_POS,
        SHOOT_POS_FOURTH,
        SHOOT_POSFIFTH_INTAKE,
        FIFTH_INTAKE,
        FIFTH_INTAKESHOOT_POS,
        SHOOT_POS_FIFTH,

       /*
       21 ball auto
       SHOOT_POSSIXTH_INTAKE,
       SIXTH_INTAKE,
       SIXTH_INTAKESHOOT_POS,
       SHOOT_POS_SIXTH
       */
    }

    PathState pathState;

    private final Pose startPose = new Pose(20.532974427994617, 122.43876177658142, Math.toRadians(138));
    private final Pose shootPose = new Pose(59.294751009421255, 83.87079407806189, Math.toRadians(124));
    private final Pose firstIntake = new Pose(15.48183041722746, 58.41184387617765, Math.toRadians(205));
    // control points for first intake
    private final Pose c1 = new Pose(46.62113055181696, 61.15612382234185);
    private final Pose gateIntake = new Pose(13.35531628532974, 62.58681022880214, Math.toRadians(170));
    private final Pose secondIntake = new Pose(14.830417227456266, 84.76446837146705, Math.toRadians(180));


    private PathChain driveStartPosShootPos, shootPreloadPosFirstIntakePos, firstIntakePosShootPos,
            shootPosGateIntakePos, gateIntakePosShootPos, shootPos2GateIntakePos, gateIntakePosShootPos2,
            shootPos3GateIntakePos, gateIntakePosShootPos3, shootPosSecondIntakePos, secondIntakePosShootPos;


    public void buildPaths() {
        // coordinates for starting pose then ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootPreloadPosFirstIntakePos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c1, firstIntake))
                .setLinearHeadingInterpolation(firstIntake.getHeading(), firstIntake.getHeading())
                .build();
        firstIntakePosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(firstIntake, c1, shootPose))
                .setLinearHeadingInterpolation(firstIntake.getHeading(), shootPose.getHeading())
                .build();
        shootPosGateIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gateIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gateIntake.getHeading())
                .build();
        gateIntakePosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gateIntake, shootPose))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), shootPose.getHeading())
                .build();
        shootPos2GateIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gateIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gateIntake.getHeading())
                .build();
        gateIntakePosShootPos2 = follower.pathBuilder()
                .addPath(new BezierLine(gateIntake, shootPose))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), shootPose.getHeading())
                .build();
        shootPos3GateIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gateIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gateIntake.getHeading())
                .build();
        gateIntakePosShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(gateIntake, shootPose))
                .setLinearHeadingInterpolation(gateIntake.getHeading(), shootPose.getHeading())
                .build();
        shootPosSecondIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondIntake.getHeading())
                .build();
        secondIntakePosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(secondIntake, shootPose))
                .setLinearHeadingInterpolation(secondIntake.getHeading(), shootPose.getHeading())
                .build();
    }


    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // timer
                break;
            case SHOOT_PRELOAD:
                // flywheel logic
                // check if follower is done its path

                if (!follower.isBusy()) {

                    setPathState(PathState.SHOOT_PRELOADFIRST_INTAKE);
                }

                break;
            case SHOOT_PRELOADFIRST_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPreloadPosFirstIntakePos, true);
                    setPathState(PathState.FIRST_INTAKE);
                }


                break;
            case FIRST_INTAKE:
                // intake logic

                if (!follower.isBusy()) {
                    setPathState(PathState.FIRST_INTAKESHOOT_POS);
                }


                break;
            case FIRST_INTAKESHOOT_POS:

                if (!follower.isBusy()) {
                    follower.followPath(firstIntakePosShootPos, true);
                    setPathState(PathState.SHOOT_POS_FIRST);
                }


                break;
            case SHOOT_POS_FIRST:
                // flywheel logic

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_POSFIRST_GATE);
                }

                break;
            case SHOOT_POSFIRST_GATE:

                if (!follower.isBusy()) {
                    follower.followPath(shootPosGateIntakePos, true);
                    setPathState(PathState.SECOND_INTAKE);
                }


                break;
            case SECOND_INTAKE:
                //intake logic

                if (!follower.isBusy()) {
                    setPathState(PathState.SECOND_INTAKESHOOT_POS);
                }

                break;
            case SECOND_INTAKESHOOT_POS:

                if (!follower.isBusy()) {
                    follower.followPath(gateIntakePosShootPos, true);
                    setPathState(PathState.SHOOT_POS_SECOND);
                }


                break;
            case SHOOT_POS_SECOND:
                // flywheel logic

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_POSSECOND_GATE);
                }


                break;
            case SHOOT_POSSECOND_GATE:

                if (!follower.isBusy()) {
                    follower.followPath(shootPos2GateIntakePos, true);
                    setPathState(PathState.THIRD_INTAKE);
                }


                break;
            case THIRD_INTAKE:
                // intake logic

                if (!follower.isBusy()) {
                    setPathState(PathState.THIRD_INTAKESHOOT_POS);
                }


                break;
            case THIRD_INTAKESHOOT_POS:

                if (!follower.isBusy()) {
                    follower.followPath(gateIntakePosShootPos2, true);
                    setPathState(PathState.SHOOT_POS_THIRD);
                }


                break;
            case SHOOT_POS_THIRD:
                // flywheel logic

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_POSTHIRD_GATE);
                }


                break;
            case SHOOT_POSTHIRD_GATE:

                if (!follower.isBusy()) {
                    follower.followPath(shootPos3GateIntakePos, true);
                    setPathState(PathState.FOURTH_INTAKE);
                }


                break;
            case FOURTH_INTAKE:
                // intake logic
                if (!follower.isBusy()) {
                    setPathState(PathState.FOURTH_INTAKESHOOT_POS);
                }

                break;
            case FOURTH_INTAKESHOOT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(gateIntakePosShootPos3, true);
                    setPathState(PathState.SHOOT_POS_FOURTH);
                }


                break;
            case SHOOT_POS_FOURTH:
                // flywheel logic
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_POSFIFTH_INTAKE);
                }


                break;
            case SHOOT_POSFIFTH_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPosSecondIntakePos, true);
                    setPathState(PathState.FIFTH_INTAKE);
                }


                break;
            case FIFTH_INTAKE:
                // intake logic
                if (!follower.isBusy()) {
                    setPathState(PathState.FIFTH_INTAKESHOOT_POS);
                }

                break;
            case FIFTH_INTAKESHOOT_POS:

                if (!follower.isBusy()) {
                    follower.followPath(secondIntakePosShootPos, true);
                    setPathState(PathState.SHOOT_POS_FIFTH);
                }
                break;
            case SHOOT_POS_FIFTH:
                // flywheel logic

                if (!follower.isBusy()) {
                    telemetry.addLine("Done All Paths");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
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



    }
}

