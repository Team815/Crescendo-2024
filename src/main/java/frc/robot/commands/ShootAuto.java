package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.DoubleSupplier;

public class ShootAuto extends ProxyCommand {

    private final Command endCommand;

    public ShootAuto(DoubleSupplier angle, double speed, Commander commander) {
        super(commander.startShootingAuto(() -> calculateShooterAngle(angle), speed));
        endCommand = commander.stopShooting();
    }

    public ShootAuto(double angle, double speed, Commander commander) {
        super(commander.startShootingAuto(() -> Math.toRadians(angle), speed));
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

    private static double calculateShooterAngle(DoubleSupplier aprilTagAngle) {
        var shooterAngle = MathUtil.clamp(16.623 - aprilTagAngle.getAsDouble() * 0.686, 5, 30);
        return Math.toRadians(shooterAngle);
    }
}
