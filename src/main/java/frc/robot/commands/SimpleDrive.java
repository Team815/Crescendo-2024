package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AccelerationModulator;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class SimpleDrive extends Command {
    private final SwerveDrive drive;
    private final DoubleSupplier forwardVelocity;
    private final DoubleSupplier sidewaysVelocity;
    private final DoubleSupplier angularVelocity;
    private final AccelerationModulator accelerator = new AccelerationModulator(0.01d, 0.01d, 0.01d);

    public SimpleDrive(
        SwerveDrive drive,
        DoubleSupplier forwardVelocity,
        DoubleSupplier sidewaysVelocity,
        DoubleSupplier angularVelocity) {
        this.drive = drive;
        this.forwardVelocity = forwardVelocity;
        this.sidewaysVelocity = sidewaysVelocity;
        this.angularVelocity = angularVelocity;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var speeds =  new ChassisSpeeds(
                forwardVelocity.getAsDouble(),
                sidewaysVelocity.getAsDouble(),
                angularVelocity.getAsDouble());
        speeds = accelerator.modulate(speeds);
        drive.drive(speeds);
    }
}
