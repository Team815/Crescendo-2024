package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private SwerveModule module;

    public SwerveDrive(SwerveModule module) {
        this.module = module;
    }

    public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
        System.out.printf("F: %.2f, S: %.2f, A: %.2f\n", forwardVelocity, sidewaysVelocity, angularVelocity);
        module.drive(forwardVelocity, angularVelocity);
    }
}
