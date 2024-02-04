package frc.robot;

import com.revrobotics.CANSparkMax;

public class CanSparkMax815 extends CANSparkMax implements PositionalController {
    public CanSparkMax815(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public double getPositionalPosition() {
        return getEncoder().getPosition();
    }
}
