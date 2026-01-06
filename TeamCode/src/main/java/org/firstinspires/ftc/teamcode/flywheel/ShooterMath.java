package org.firstinspires.ftc.teamcode.flywheel;

/**
 * ShooterMath - compute muzzle speed and wheel RPM for fixed-angle hooded flywheel shooter.
 *
 * Units:
 *  - distances in meters
 *  - angles in degrees (method parameter)
 *  - wheel radius in meters
 */
public class ShooterMath {

    private static final double G = 9.81; // m/s^2

    /**
     * Compute required linear muzzle speed (m/s) for a desired horizontal distance and vertical offset,
     * given a fixed launch angle (degrees).
     *
     * @param distanceHorizMeters horizontal distance from shooter to target (m)
     * @param deltaYMeters (targetHeight - shooterHeight) in meters
     * @param launchAngleDeg launch angle in degrees (hood angle)
     * @return required muzzle speed in m/s, or Double.NaN if no valid solution for this angle
     */
    public static double computeMuzzleSpeedForFixedAngle(double distanceHorizMeters, double deltaYMeters, double launchAngleDeg) {
        double theta = Math.toRadians(launchAngleDeg);
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denom = 2.0 * cos * cos * (distanceHorizMeters * tan - deltaYMeters);
        if (denom <= 0.0) {
            return Double.NaN; // no solution for this angle (denominator nonpositive)
        }

        double v2 = (G * distanceHorizMeters * distanceHorizMeters) / denom;
        if (v2 <= 0.0) return Double.NaN;
        return Math.sqrt(v2);
    }

    /**
     * Convert linear muzzle speed (m/s) into wheel RPM, given wheel radius and efficiency.
     *
     * @param muzzleSpeed_mps muzzle speed in m/s
     * @param wheelRadiusMeters wheel radius in meters (r)
     * @param efficiency fraction (<=1) to account for slip; e.g. 0.95
     * @return wheel RPM (rotations per minute)
     */
    public static double muzzleSpeedToWheelRPM(double muzzleSpeed_mps, double wheelRadiusMeters, double efficiency) {
        if (Double.isNaN(muzzleSpeed_mps) || muzzleSpeed_mps <= 0 || wheelRadiusMeters <= 0) return Double.NaN;
        // account for slip by dividing by efficiency (i.e., require wheel to spin slightly faster)
        double linearAtWheel = muzzleSpeed_mps / Math.max(1e-9, efficiency);
        double wheelCircum = 2.0 * Math.PI * wheelRadiusMeters; // meters per revolution
        double revsPerSec = linearAtWheel / wheelCircum;
        return revsPerSec * 60.0; // RPM
    }

    /**
     * Convenience: from distance+heights+angle -> wheel RPM
     */
    public static double computeTargetWheelRPM(double distanceMeters,
                                               double shooterHeightMeters,
                                               double targetHeightMeters,
                                               double launchAngleDeg,
                                               double wheelRadiusMeters,
                                               double efficiency) {
        double dy = targetHeightMeters - shooterHeightMeters;
        double v = computeMuzzleSpeedForFixedAngle(distanceMeters, dy, launchAngleDeg);
        if (Double.isNaN(v)) return Double.NaN;
        return muzzleSpeedToWheelRPM(v, wheelRadiusMeters, efficiency);
    }
}
