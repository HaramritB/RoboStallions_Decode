package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.gate.AutoGate;
import org.firstinspires.ftc.teamcode.gate.Gate;

@Autonomous
public class BlueClose extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotor autoIntake;
    private final double intakeDefaultPower = 0.75;
    private final double intakeShootPower = 0.95;
    private DcMotor autoTransfer;

    private final double transferDefaultPower = 0.25;

    private final double transferShootPower = 0.75;

    private DcMotorEx autoFlywheel;

    private AutoGate gate;

    private final double FLYWHEEL_TARGET_VELOCITY = 1600.0; // ticks/sec (tuned)
    private final double VELOCITY_READY_TOLERANCE = 100.0;  // when considered "at speed"
    private final double VELOCITY_DROP_THRESHOLD  = 50.0;  // drop that indicates a shot
    private boolean feeding = false; // true while we are feeding a ball into the flywheel
    private long feedStartTime = 0; // used for failsafe timing of ball feed

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

    private final Pose startPose = new Pose(20.532974427994617, 122.43876177658142, Math.toRadians(138));
    private final Pose shootPose = new Pose(59.294751009421255, 83.87079407806189, Math.toRadians(138));
    private final Pose firstIntake = new Pose(17.419919246298797, 59.768506056527585, Math.toRadians(195));
    // control point for first intake
    private final Pose c1 = new Pose(46.62113055181696, 58.63660834454912);
    private final Pose gateIntake = new Pose(13.35531628532974, 62.58681022880214, Math.toRadians(175));
    private final Pose secondIntake = new Pose(20.257065948856, 83.60161507402425, Math.toRadians(180));
    private final Pose thirdIntake = new Pose(20.03903095558547, 35.675639300134584, Math.toRadians(180));

    // control point for third intake
    private final Pose c2 = new Pose(70.16083445491253, 36.22139973082097);


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

    private boolean flywheelAtSpeed() {
        return Math.abs(autoFlywheel.getVelocity() - FLYWHEEL_TARGET_VELOCITY) <= VELOCITY_READY_TOLERANCE;
    }

    private boolean flywheelDropped() {
        return autoFlywheel.getVelocity() < (FLYWHEEL_TARGET_VELOCITY - VELOCITY_DROP_THRESHOLD);
    }

    private void startFeed() {
        autoIntake.setPower(intakeShootPower);
        autoTransfer.setPower(transferShootPower);
        feeding = true;
    }

    private void stopFeed() {
        autoIntake.setPower(intakeDefaultPower);
        autoTransfer.setPower(transferDefaultPower);
        feeding = false;
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // immediately go to shoot preload after path
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    // start feeding once at speed and settled
                    if (!feeding && flywheelAtSpeed() && pathTimer.getElapsedTime() >= 0.15) {
                        startFeed();
                        gate.open();
                        feedStartTime = System.currentTimeMillis(); // start failsafe timer
                    }

                    // stop feeding once shot detected or failsafe timeout
                    if (feeding && (flywheelDropped() || System.currentTimeMillis() - feedStartTime >= 1200)) {
                        stopFeed();
                        gate.close();
                        setPathState(PathState.SHOOT_PRELOADFIRST_INTAKE);
                    }
                }
                break;

            case SHOOT_PRELOADFIRST_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPreloadPosFirstIntakePos, true);
                    setPathState(PathState.FIRST_INTAKE);
                }
                break;

            case FIRST_INTAKE:
                autoIntake.setPower(intakeDefaultPower);
                autoTransfer.setPower(transferDefaultPower);
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
                if (!follower.isBusy()) {
                    if (!feeding && flywheelAtSpeed() && pathTimer.getElapsedTime() >= 0.15) {
                        startFeed();
                        gate.open();
                        feedStartTime = System.currentTimeMillis();
                    }

                    if (feeding && (flywheelDropped() || System.currentTimeMillis() - feedStartTime >= 1200)) {
                        stopFeed();
                        gate.close();
                        setPathState(PathState.SHOOT_POSFIRST_GATE);
                    }
                }
                break;

            case SHOOT_POSFIRST_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPosGateIntakePos, true);
                    setPathState(PathState.SECOND_INTAKE);
                }
                break;

            case SECOND_INTAKE:
                autoIntake.setPower(intakeDefaultPower);
                autoTransfer.setPower(transferDefaultPower);
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
                if (!follower.isBusy()) {
                    if (!feeding && flywheelAtSpeed() && pathTimer.getElapsedTime() >= 0.15) {
                        startFeed();
                        gate.open();
                        feedStartTime = System.currentTimeMillis();
                    }

                    if (feeding && (flywheelDropped() || System.currentTimeMillis() - feedStartTime >= 1200)) {
                        stopFeed();
                        gate.close();
                        setPathState(PathState.SHOOT_POSSECOND_GATE);
                    }
                }
                break;

            case SHOOT_POSSECOND_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPos2GateIntakePos, true);
                    setPathState(PathState.THIRD_INTAKE);
                }
                break;

            case THIRD_INTAKE:
                autoIntake.setPower(intakeDefaultPower);
                autoTransfer.setPower(transferDefaultPower);
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
                if (!follower.isBusy()) {
                    if (!feeding && flywheelAtSpeed() && pathTimer.getElapsedTime() >= 0.15) {
                        startFeed();
                        gate.open();
                        feedStartTime = System.currentTimeMillis();
                    }

                    if (feeding && (flywheelDropped() || System.currentTimeMillis() - feedStartTime >= 1200)) {
                        stopFeed();
                        gate.close();
                        setPathState(PathState.SHOOT_POSTHIRD_GATE);
                    }
                }
                break;

            case SHOOT_POSTHIRD_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPos3GateIntakePos, true);
                    setPathState(PathState.FOURTH_INTAKE);
                }
                break;

            case FOURTH_INTAKE:
                autoIntake.setPower(intakeDefaultPower);
                autoTransfer.setPower(transferDefaultPower);
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
                if (!follower.isBusy()) {
                    if (!feeding && flywheelAtSpeed() && pathTimer.getElapsedTime() >= 0.15) {
                        startFeed();
                        gate.open();
                        feedStartTime = System.currentTimeMillis();
                    }

                    if (feeding && (flywheelDropped() || System.currentTimeMillis() - feedStartTime >= 1200)) {
                        stopFeed();
                        gate.close();
                        setPathState(PathState.SHOOT_POSFIFTH_INTAKE);
                    }
                }
                break;

            case SHOOT_POSFIFTH_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPosSecondIntakePos, true);
                    setPathState(PathState.FIFTH_INTAKE);
                }
                break;

            case FIFTH_INTAKE:
                autoIntake.setPower(intakeDefaultPower);
                autoTransfer.setPower(transferDefaultPower);
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
                if (!follower.isBusy()) {
                    if (!feeding && flywheelAtSpeed() && pathTimer.getElapsedTime() >= 0.15) {
                        startFeed();
                        gate.open();
                        feedStartTime = System.currentTimeMillis();
                    }

                    if (feeding && (flywheelDropped() || System.currentTimeMillis() - feedStartTime >= 1200)) {
                        stopFeed();
                        gate.close();
                        setPathState(PathState.SHOOT_POSSIXTH_INTAKE);
                    }
                }
                break;

            case SHOOT_POSSIXTH_INTAKE:
                if (!follower.isBusy()) {
                    follower.followPath(shootPosThirdIntake, true);
                    setPathState(PathState.SIXTH_INTAKE);
                }
                break;

            case SIXTH_INTAKE:
                autoIntake.setPower(intakeDefaultPower);
                autoTransfer.setPower(transferDefaultPower);
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
                if (!follower.isBusy()) {
                    if (!feeding && flywheelAtSpeed() && pathTimer.getElapsedTime() >= 0.15) {
                        startFeed();
                        gate.open();
                        feedStartTime = System.currentTimeMillis();
                    }

                    if (feeding && (flywheelDropped() || System.currentTimeMillis() - feedStartTime >= 1200)) {
                        stopFeed();
                        gate.close();
                        telemetry.addLine("All Paths Completed");
                    }
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
        feeding = false; // EDIT: reset feeding on state change
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        // opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        // add mechs
        autoIntake = hardwareMap.get(DcMotor.class, "intake");
        autoTransfer = hardwareMap.get(DcMotor.class, "transfer");
        autoFlywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        gate = new AutoGate(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        autoFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        // autoFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                // new PIDFCoefficients(23.0, 0, 0, 13.5));
        autoFlywheel.setVelocity(FLYWHEEL_TARGET_VELOCITY);
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
        telemetry.addData("FlywheelVel", "%.1f", autoFlywheel.getVelocity());
        telemetry.addData("Feeding", feeding);
        telemetry.update();
    }
}
