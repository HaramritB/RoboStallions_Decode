package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;

import com.pedropathing.util.Timer;
@Autonomous
public class BlueClose extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // Start Position_End Position
        // Drive State > Movement State
        // Shoot > Scoring

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        FIRST_INTAKE,

        FIRST_INTAKE_SHOOT,


    }

    PathState pathState;

    private final Pose startPose = new Pose(20.920592193808872, 121.95962314939437, Math.toRadians(138));
    private final Pose shootPose = new Pose(53.86810228802154, 89.49125168236878, Math.toRadians(138));
    private final Pose firstIntakePose = new Pose(12.519515477792734, 82.63257065948856, Math.toRadians(190));

    private final Pose secondIntakePose = new Pose(17.033647375504707, 52.29205921938089, Math.toRadians(230));

    private final Pose thirdIntakePose = new Pose(17.30512514898689, 36.55899880810489, Math.toRadians(190));

    private PathChain driveStartPosShootPos, shootPosFirstIntake, firstIntakeShootPos, shootPosSecondIntake, shootPosThirdPoseIntake, shootThirdIntakeReturn ;
    public void buildPaths() {
        // coordinates for starting pose then ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootPosFirstIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstIntakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstIntakePose.getHeading())
                .build();
        firstIntakeShootPos = follower.pathBuilder()
                .addPath(new BezierLine(firstIntakePose, shootPose))
                .setLinearHeadingInterpolation(firstIntakePose.getHeading(), shootPose.getHeading())
                .build();
        shootPosSecondIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondIntakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondIntakePose.getHeading())
                .build();

        shootPosThirdPoseIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdIntakePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdIntakePose.getHeading())
                .build();

        shootThirdIntakeReturn = follower.pathBuilder()
                .addPath(new BezierLine(thirdIntakePose, shootPose))
                .setLinearHeadingInterpolation(thirdIntakePose.getHeading(), shootPose.getHeading())
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
                    // flywheel logic
                }

                setPathState(PathState.FIRST_INTAKE);
                break;
            case FIRST_INTAKE:
                // intake logic

                setPathState(PathState.FIRST_INTAKE_SHOOT);
                break;
            case FIRST_INTAKE_SHOOT:
                // flywheel logic
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

        Flywheel flywheel = new Flywheel(hardwareMap);

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
    }
}
