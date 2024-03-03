package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface InputDevice {
    double getSidewaysVelocity();

    double getForwardVelocity();

    double getAngularVelocity();

    Trigger resetHeading();
    Trigger resetHeadingForward();
    Trigger resetHeadingRight();
    Trigger resetHeadingBack();
    Trigger resetHeadingLeft();
    Trigger centerOnAprilTag();
    Trigger pickup();
    Trigger drop();
    Trigger test();
    Trigger shoot();
}
