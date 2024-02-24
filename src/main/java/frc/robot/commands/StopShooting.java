package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

public class StopShooting extends Command {
    private final Arm arm;
    private final Shooter shooter;
    private final Pickup pickup;

    public StopShooting(Arm arm, Shooter shooter, Pickup pickup) {
        this.arm = arm;
        this.shooter = shooter;
        this.pickup = pickup;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        shooter.stop();
        pickup.stop();
        arm.setPosition(Math.toRadians(0d));
    }

    @Override
    public boolean isFinished() {
        return arm.getController().atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
