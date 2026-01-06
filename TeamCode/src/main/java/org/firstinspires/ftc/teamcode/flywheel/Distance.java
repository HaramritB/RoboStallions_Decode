package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Distance {

    private final Limelight3A limelight;

    // store last known good distance
    private double lastDistance = -1;
    private boolean lastValid = false;
    private int lastTagId = -1;

    private double lastX = 0;
    private double lastY = 0;
    private double lastZ = 0;

    public Distance(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(7); // AprilTag pipeline
        limelight.start();
    }

    /** Call this every loop */
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            lastValid = false;
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags.size() == 0) {
            lastValid = false;
            return;
        }

        // For now: use the FIRST detected tag
        LLResultTypes.FiducialResult tag = tags.get(0);

        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null) {
            lastValid = false;
            return;
        }

        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        double z = pose.getPosition().z;

        lastX = x;
        lastY = y;
        lastZ = z;

        lastTagId = tag.getFiducialId();

        lastDistance = Math.sqrt(x * x + y * y + z * z);
        lastValid = true;
    }

    public boolean hasValidTag() {
        return lastValid;
    }

    public int getTagId() {
        return lastTagId;
    }

    public double getDistanceMeters() {
        return lastDistance;
    }

    public double getX() {
        return lastX;
    }

    public double getY() {
        return lastY;
    }

    public double getZ() {
        return lastZ;
    }
}
