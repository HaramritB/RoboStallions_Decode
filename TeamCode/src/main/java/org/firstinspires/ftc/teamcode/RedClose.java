package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class RedClose extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotor autoIntake;
    private final double intakeDefaultPower = 0.75;
    private final double intakeShootPower = 0.95;
    private DcMotor autoTransfer;

    private final double transferDefaultPower = 0.25;

    private final double transferShootPower = 0.75;

    private DcMotorEx autoFlywheel;

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

        // 21 ball auto
        SHOOT_POSSIXTH_INTAKE,
        SIXTH_INTAKE,
        SIXTH_INTAKESHOOT_POS,
        SHOOT_POS_SIXTH
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.0578734858681, 121.76581426648723, Math.toRadians(45));
    private final Pose shootPose = new Pose(84.6837146702557, 83.8707940780619, Math.toRadians(45));
    private final Pose firstIntake = new Pose(125.32839838492598, 59.39703903095557, Math.toRadians(0));
    // control point for first intake
    private final Pose c1 = new Pose(95.90107671601615, 57.68371467025571);
    private final Pose gateIntake = new Pose(130.6204576043068, 64.19246298788697, Math.toRadians(5));
    private final Pose secondIntake = new Pose(125.23822341857336, 83.46702557200538, Math.toRadians(0));
    private final Pose thirdIntake = new Pose(125.58815612382233, 35.27321668909825, Math.toRadians(0));

    // control point for third intake
    private final Pose c2 = new Pose(86.2664872139973, 32.663526244952884);


    private PathChain driveStartPosShootPos, shootPreloadPosFirstIntakePos, firstIntakePosShootPos,
            shootPosGateIntakePos, gateIntakePosShootPos, shootPos2GateIntakePos, gateIntakePosShootPos2,
            shootPos3GateIntakePos, gateIntakePosShootPos3, shootPosSecondIntakePos, secondIntakePosShootPos,
            shootPosThirdIntake, thirdIntakeShootPos;


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
        shootPosThirdIntake = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, c2, thirdIntake))
                .setLinearHeadingInterpolation(thirdIntake.getHeading(), thirdIntake.getHeading())
                .build();
        thirdIntakeShootPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdIntake, shootPose))
                .setLinearHeadingInterpolation(thirdIntake.getHeading(), shootPose.getHeading())
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
                //autoIntake.setPower(0.95);
                //autoTransfer.setPower(1.0);

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
                //autoIntake.setPower(0.95);
                //autoTransfer.setPower(1.0);

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
                //autoIntake.setPower(0.95);
                //autoTransfer.setPower(1.0);


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
                //autoIntake.setPower(0.95);
                //autoTransfer.setPower(1.0);

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
                //autoIntake.setPower(0.95);
                //autoTransfer.setPower(1.0);

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


                    setPathState(PathState.SHOOT_POSSIXTH_INTAKE);
                }
                break;
            case SHOOT_POSSIXTH_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPosThirdIntake, true);
                    setPathState(PathState.SIXTH_INTAKE);
                }
                break;
            case SIXTH_INTAKE:
                // intake logic
                //autoIntake.setPower(0.95);
                //autoTransfer.setPower(1.0);

                if (!follower.isBusy()) {

                    setPathState(PathState.SIXTH_INTAKESHOOT_POS);
                }

                break;
            case SIXTH_INTAKESHOOT_POS:
                if (!follower.isBusy()) {
                    follower.followPath(thirdIntakeShootPos, true);
                    setPathState(PathState.SHOOT_POS_SIXTH);
                }
                break;
            case SHOOT_POS_SIXTH:
                // flywheel logic

                if (!follower.isBusy()) {


                    telemetry.addLine("All Paths Completed");
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

