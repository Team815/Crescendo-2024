package frc.robot.subsystems.speedsmodulator;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AccelerationLimiter implements SpeedsModulator {
    private double maxForwardAcceleration;
    private double maxSidewaysAcceleration;
    private double maxAngularAcceleration;
    private ChassisSpeeds previousSpeeds = new ChassisSpeeds();

    public AccelerationLimiter(double maxLinearAcceleration, double maxAngularAcceleration) {
        this(maxLinearAcceleration, maxLinearAcceleration, maxAngularAcceleration);
    }

    public AccelerationLimiter(
        double maxForwardAcceleration,
        double maxSidewaysAcceleration,
        double maxAngularAcceleration) {
        this.maxForwardAcceleration = maxForwardAcceleration;
        this.maxSidewaysAcceleration = maxSidewaysAcceleration;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    public ChassisSpeeds modulate(ChassisSpeeds targetSpeeds) {
        System.out.println(maxForwardAcceleration);
        var speeds = new ChassisSpeeds(
            limit(previousSpeeds.vxMetersPerSecond, targetSpeeds.vxMetersPerSecond, maxForwardAcceleration),
            limit(previousSpeeds.vyMetersPerSecond, targetSpeeds.vyMetersPerSecond, maxSidewaysAcceleration),
            limit(previousSpeeds.omegaRadiansPerSecond, targetSpeeds.omegaRadiansPerSecond, maxAngularAcceleration)
        );
        previousSpeeds = speeds;
        return speeds;
    }

    public void setMaxLinearAcceleration(double maxLinearAcceleration) {
        maxForwardAcceleration = maxLinearAcceleration;
        maxSidewaysAcceleration = maxLinearAcceleration;
    }

    private static double limit(double fromVelocity, double toVelocity, double maxAcceleration) {
        var targetDifference = toVelocity - fromVelocity;
        return fromVelocity + Math.min(maxAcceleration, Math.abs(targetDifference)) * Math.signum(targetDifference);
    }
}
