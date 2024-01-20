package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;

import java.util.function.DoubleSupplier;

public class AngleCorrectionModulator {
    private DoubleSupplier robotAngle;
    private PIDController pid = new PIDController(0.01d, 0d, 0d);

    public AngleCorrectionModulator(DoubleSupplier robotAngle) {
        this.robotAngle = robotAngle;
    }

    public ChassisSpeeds modulate(ChassisSpeeds speeds) {
        var angularCorrection = speeds.omegaRadiansPerSecond;
        if (angularCorrection != 0) {
            pid.setSetpoint(robotAngle.getAsDouble());
        } else {
            angularCorrection = pid.calculate(robotAngle.getAsDouble());
        }
        var correctedSpeeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                angularCorrection
        );
        return correctedSpeeds;
    }
}
