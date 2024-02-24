package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;

public class Commander {
    private final Pickup pickup;
    private final Arm arm;
    private final Shooter shooter;

    public Commander(Pickup pickup, Arm arm, Shooter shooter) {
        this.pickup = pickup;
        this.arm = arm;
        this.shooter = shooter;
    }

    public Command shootManual(double angle, double speed) {
        return Commands.startEnd(
            () -> {
                arm.setPosition(Math.toRadians(angle));
                shooter.run(speed);
            },
            () -> {
                shooter.stop();
                dropArm().schedule();
            },
            arm, shooter);
    }

    public Command startShootingAuto(double angle, double speed) {
        return Commands.runOnce(() -> {
                arm.setPosition(Math.toRadians(angle));
                shooter.run(speed);
            },
                arm,
                shooter)
            .andThen(Commands.waitSeconds(4d)
                .raceWith(Commands.waitUntil(() -> arm.getController().atGoal() && shooter.getController().atSetpoint())))
            .andThen(Commands.run(() -> pickup.run(0.3d), pickup));
    }

    public Command stopShooting() {
        return Commands.runOnce(() -> {
            shooter.stop();
            pickup.stop();
            dropArm().schedule();
        }, shooter, pickup);
    }

    public Command dropArm() {
        return Commands.runOnce(() -> arm.setPosition(Math.toRadians(0d)), arm)
            .andThen(Commands.waitSeconds(2d)
                .raceWith(Commands.waitUntil(() -> arm.getController().atGoal())))
            .andThen(arm::stop, arm);
    }
}
