package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxController extends CommandXboxController implements InputDevice {
    private static final double DEADBAND = 0.12d;
    public XboxController() {
        super(0);
    }

    @Override
    public double getSidewaysVelocity() {
        return MathUtil.applyDeadband(-getLeftX(), DEADBAND);
    }

    @Override
    public double getForwardVelocity() {
        return MathUtil.applyDeadband(-getLeftY(), DEADBAND);
    }

    @Override
    public double getAngularVelocity() {
        return MathUtil.applyDeadband(-getRightX(), DEADBAND);
    }

    @Override
    public Trigger resetHeading() {
        return start();
    }

    @Override
    public Trigger centerOnNote() {
        return a();
    }
}
