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

    public ShootAuto(DoubleSupplier angle, double speed, double delay, Commander commander) {
        super(commander.startShootingAuto(() -> calculateShooterAngle(angle), speed, delay));
        endCommand = commander.stopShooting();
    }

    public ShootAuto(double angle, double speed, Commander commander) {
        super(commander.startShootingAuto(() -> angle, speed));
        System.out.println(angle);
        endCommand = commander.stopShooting();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("Ending");
        endCommand.schedule();
    }

    private static double calculateShooterAngle(DoubleSupplier aprilTagAngle) {
        return MathUtil.clamp(10 - aprilTagAngle.getAsDouble() * 0.7, 5, 30);
    }
}
