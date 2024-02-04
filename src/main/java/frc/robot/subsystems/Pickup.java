package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase {
    private final MotorController motor;

    public Pickup(MotorController motor) {
        this.motor = motor;
    }

    public void run(double speed) {
        motor.set(speed);
    }
}
