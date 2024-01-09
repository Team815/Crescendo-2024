package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class SimpleDrive extends Command {
    private SwerveDrive drive;
    private DoubleSupplier forwardVelocity;
    private DoubleSupplier sidewaysVelocity;
    private DoubleSupplier angularVelocity;

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
        drive.drive(
            forwardVelocity.getAsDouble(),
            sidewaysVelocity.getAsDouble(),
            angularVelocity.getAsDouble());
    }
}
