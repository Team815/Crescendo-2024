package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AccelerationModulator;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class CenterOnNote extends Command {
    private final SwerveDrive drive;
    private final DoubleSupplier forwardVelocity;
    private final DoubleSupplier sidewaysVelocity;
    private final DoubleSupplier offset;
    private final PIDController pid = new PIDController(0.02d, 0d, 0d);
    private final AccelerationModulator accelerator = new AccelerationModulator(0.01d, 0.01d, 0.01d);

    public CenterOnNote(
            SwerveDrive drive,
            DoubleSupplier forwardVelocity,
            DoubleSupplier sidewaysVelocity,
            DoubleSupplier offset) {
        this.drive = drive;
        this.forwardVelocity = forwardVelocity;
        this.sidewaysVelocity = sidewaysVelocity;
        this.offset = offset;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        var response = pid.calculate(offset.getAsDouble());
        var speeds = new ChassisSpeeds(
                forwardVelocity.getAsDouble(),
                sidewaysVelocity.getAsDouble(),
                response);
        //speeds = accelerator.modulate(speeds);
        drive.drive(speeds);
    }
}
