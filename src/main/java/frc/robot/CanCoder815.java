package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

public class CanCoder815 extends CANcoder implements Positional {
    public CanCoder815(int deviceId) {
        super(deviceId);
    }

    @Override
    public double getPositionalPosition() {
        return getAbsolutePosition().getValue();
    }
}
