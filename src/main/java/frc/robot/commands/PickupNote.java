package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pickup;

public class PickupNote extends Command {
    private final Pickup pickup;

    public PickupNote(Pickup pickup) {
        this.pickup = pickup;
        addRequirements(pickup);
    }

    @Override
    public void initialize() {
        pickup.run(Pickup.PICKUP_SPEED);
    }

    @Override
    public boolean isFinished() {
        return pickup.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        pickup.stop();
    }
}
