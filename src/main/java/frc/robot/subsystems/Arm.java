package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PositionalController;

public class Arm extends SubsystemBase {
    PositionalController motor;

    public Arm(PositionalController motor) {
        this.motor = motor;
    }

    @Override
    public void periodic() {
        //System.out.println(motor.getPositionalPosition());
    }

    public void run(double speed) {
        motor.set(speed);
    }
}
