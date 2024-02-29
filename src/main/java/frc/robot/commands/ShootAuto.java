package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import java.util.function.DoubleSupplier;

public class ShootAuto extends ProxyCommand {

    private final Command endCommand;

    public ShootAuto(DoubleSupplier angle, double speed, Commander commander) {
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
