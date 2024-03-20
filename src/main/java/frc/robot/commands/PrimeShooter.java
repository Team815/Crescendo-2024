package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class PrimeShooter extends Command {
    private final Shooter shooter;
    private final Arm arm;
    private final double angle;
    private final double speed;

    public PrimeShooter(double angle, double speed, Shooter shooter, Arm arm) {
        this.angle = angle;
        this.speed = speed;
        this.shooter = shooter;
        this.arm = arm;
        addRequirements(shooter, arm);
    }

    @Override
    public void initialize() {
        arm.setPosition(angle);
        shooter.run(speed);
    }

    @Override
    public boolean isFinished() {
        return arm.getController().atGoal() && shooter.atSetpoint();
    }
}
