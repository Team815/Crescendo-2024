package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class XboxController extends CommandXboxController implements InputDevice {
    public XboxController() {
        super(0);
    }

    @Override
    public double getSidewaysVelocity() {
        return -getLeftX();
    }

    @Override
    public double getForwardVelocity() {
        return -getLeftY();
    }

    @Override
    public double getAngularVelocity() {
        return -getRightX();
    }
}
