package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AccelerationModulator;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class CenterOnTarget extends Command {
    private final SwerveDrive drive;
    private final DoubleSupplier forwardVelocity;
    private final DoubleSupplier sidewaysVelocity;
    private final DoubleSupplier targetOffset;
    private final PIDController pid = new PIDController(0.02d, 0d, 0d);

    public CenterOnTarget(
            SwerveDrive drive,
            DoubleSupplier forwardVelocity,
            DoubleSupplier sidewaysVelocity,
            DoubleSupplier targetOffset) {
        this.drive = drive;
        this.forwardVelocity = forwardVelocity;
        this.sidewaysVelocity = sidewaysVelocity;
        this.targetOffset = targetOffset;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var response = pid.calculate(targetOffset.getAsDouble());
        var speeds = new ChassisSpeeds(
                forwardVelocity.getAsDouble(),
                sidewaysVelocity.getAsDouble(),
                response);
        drive.drive(speeds);
    }
}
