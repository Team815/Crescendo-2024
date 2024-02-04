package frc.robot;

public class DummyMotorController implements PositionalController {
    @Override
    public void set(double v) {
    }

    @Override
    public double get() {
        return 0;
    }

    @Override
    public void setInverted(boolean b) {
    }

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public void disable() {
    }

    @Override
    public void stopMotor() {
    }

    @Override
    public double getPositionalPosition() {
        return 0;
    }
}
