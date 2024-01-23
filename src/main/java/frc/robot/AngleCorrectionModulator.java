package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.DoubleSupplier;

public class AngleCorrectionModulator {
    private final DoubleSupplier robotAngle;
    private final PIDController pid = new PIDController(0.01d, 0d, 0d);
    private double previousAngle = 0d;
    private double previousAngularVelocity = 0d;
    private boolean settling = false;

    public AngleCorrectionModulator(DoubleSupplier robotAngle) {
        this.robotAngle = robotAngle;
    }

    public ChassisSpeeds modulate(ChassisSpeeds speeds) {
        var currentAngle = robotAngle.getAsDouble();
        var angularCorrection = speeds.omegaRadiansPerSecond;
        if (angularCorrection != 0) {
            pid.setSetpoint(currentAngle);
        } else if (previousAngularVelocity != 0) {
            settling = true;
            pid.setSetpoint(currentAngle);
        } else if (settling) {
            pid.setSetpoint(currentAngle);
            if (Math.abs(currentAngle - previousAngle) < 1d) {
                settling = false;
            }
        } else {
            angularCorrection = pid.calculate(currentAngle);
        }
        previousAngle = currentAngle;
        previousAngularVelocity = speeds.omegaRadiansPerSecond;
        return new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                angularCorrection
        );
    }

    public void reset() {
        pid.setSetpoint(0d);
    }
}
