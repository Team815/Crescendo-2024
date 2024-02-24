package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pickup extends SubsystemBase {
    private final CANSparkBase motor;

    public Pickup(CANSparkBase motor) {
        motor.restoreFactoryDefaults();
        this.motor = motor;
    }

    public void run(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }
}
