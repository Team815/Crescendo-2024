package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface InputDevice {
    double getSidewaysVelocity();

    double getForwardVelocity();

    double getAngularVelocity();

    Trigger resetHeading();
    Trigger centerOnNote();
    Trigger centerOnAprilTag();
    Trigger pickup();
    Trigger test();
    Trigger shoot();
}
