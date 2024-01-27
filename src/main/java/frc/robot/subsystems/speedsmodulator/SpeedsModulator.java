package frc.robot.subsystems.speedsmodulator;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SpeedsModulator {
    ChassisSpeeds modulate(ChassisSpeeds speeds);
}
