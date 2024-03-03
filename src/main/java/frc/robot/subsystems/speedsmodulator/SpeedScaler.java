package frc.robot.subsystems.speedsmodulator;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SpeedScaler implements SpeedsModulator {
    private double maxForwardScale;
    private double maxSidewaysScale;
    private double maxAngularScale;

    public SpeedScaler() {
        this(1, 1, 1);
    }

    public SpeedScaler(double maxForwardScale, double maxSidewaysScale, double maxAngularScale) {
        setScales(maxForwardScale, maxSidewaysScale, maxAngularScale);
    }

    public void setScales(double maxScale) {
        setScales(maxScale, maxScale, maxScale);
    }

    public void setScales(double maxForwardScale, double maxSidewaysScale, double maxAngularScale) {
        this.maxForwardScale = Math.abs(maxForwardScale);
        this.maxSidewaysScale = Math.abs(maxSidewaysScale);
        this.maxAngularScale = Math.abs(maxAngularScale);
    }

    @Override
    public ChassisSpeeds modulate(ChassisSpeeds speeds) {
        return new ChassisSpeeds(
            speeds.vxMetersPerSecond * maxForwardScale,
            speeds.vyMetersPerSecond * maxSidewaysScale,
            speeds.omegaRadiansPerSecond * maxAngularScale);
    }
}
