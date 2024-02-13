package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private final Shooter shooter;
    private final Pickup pickup;
    private final double speed;

    public Shoot(Shooter shooter, Pickup pickup, double speed) {
        this.shooter = shooter;
        this.pickup = pickup;
        this.speed = speed;
        addRequirements(shooter, pickup);
    }

    @Override
    public void initialize() {
        shooter.run(speed);
    }

    @Override
    public void execute() {
        if (MathUtil.isNear(speed, shooter.getVelocity(), 100)) {
            pickup.run(0.4d);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.run(0d);
        pickup.run(0d);
    }
}
