package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final CANSparkBase motor;

    public Climber(CANSparkBase motor1, CANSparkBase motor2) {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor1.setInverted(true);
        motor2.follow(motor1, true);
        motor = motor1;
    }

    public void run(double speed) {
        motor.set(speed);
    }
}
