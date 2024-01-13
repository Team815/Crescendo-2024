package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AccelerationModulator {
    private final double maxForwardAcceleration;
    private final double maxSidewaysAcceleration;
    private final double maxAngularAcceleration;
    private ChassisSpeeds previousSpeeds = new ChassisSpeeds();

    public AccelerationModulator(
            double maxForwardAcceleration,
            double maxSidewaysAcceleration,
            double maxAngularAcceleration) {
        this.maxForwardAcceleration = maxForwardAcceleration;
        this.maxSidewaysAcceleration = maxSidewaysAcceleration;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    public ChassisSpeeds modulate(ChassisSpeeds targetSpeeds) {
        var speeds = new ChassisSpeeds(
                limit(previousSpeeds.vxMetersPerSecond, targetSpeeds.vxMetersPerSecond, maxForwardAcceleration),
                limit(previousSpeeds.vyMetersPerSecond, targetSpeeds.vyMetersPerSecond, maxSidewaysAcceleration),
                limit(previousSpeeds.omegaRadiansPerSecond, targetSpeeds.omegaRadiansPerSecond, maxAngularAcceleration)
        );
        previousSpeeds = speeds;
        return speeds;
    }

    private static double limit(double fromVelocity, double toVelocity, double maxAcceleration) {
        var targetDifference = toVelocity - fromVelocity;
        return fromVelocity + Math.min(maxAcceleration, Math.abs(targetDifference)) * Math.signum(targetDifference);
    }
}
