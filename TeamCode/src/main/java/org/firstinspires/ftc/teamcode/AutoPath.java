package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.limelight.AprilTagTracking;

import com.pedropathing.util.Timer;
@Autonomous
public class AutoPath extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    Flywheel flywheel = new Flywheel(hardwareMap);

    public enum PathState {
        // Start Position_End Position
        // Drive State > Movement State
        // Shoot > Scoring

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }

    PathState pathState;

    private final Pose startPose = new Pose(20.920592193808872, 121.95962314939437, Math.toRadians(138));
    private final Pose shootPose = new Pose(46.6971736204576, 97.15208613728132, Math.toRadians(138));

    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        // coordinates for starting pose then ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                // flywheel logic
                // check if follower is done its path
                if (!follower.isBusy()) {
                    flywheel.setTargetRPM(500);
                }
                break;
        }
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
