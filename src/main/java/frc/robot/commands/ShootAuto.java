package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

import java.util.Set;

public class ShootAuto extends ProxyCommand {

    private final Command endCommand;

    public ShootAuto(double angle, double speed, Commander commander) {
        super(commander.startShootingAuto(angle, speed));
        endCommand = commander.stopShooting();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        endCommand.schedule();
    }
}
