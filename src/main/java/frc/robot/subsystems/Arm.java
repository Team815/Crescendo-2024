package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
    private final CANSparkBase motor;
    private final ArmFeedforward feedforward;
    private final double offset = 0.237;

    public Arm(CANSparkBase motor, CANSparkBase follower) {
        super(new ProfiledPIDController(
            1d,
            0d,
            0d,
            new TrapezoidProfile.Constraints(1d, 4d)));
        getController().setTolerance(0.1d);
        motor.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        follower.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setInverted(true);
        var encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(0.10101584d);
        encoder.setPosition(0d);
        follower.follow(motor, true);
        this.motor = motor;
        feedforward = new ArmFeedforward(0d, 0.1d, 0d);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void setPosition(double position) {
        setGoal(position + offset);
        enable();
    }

    public void stop() {
        disable();
        motor.stopMotor();
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var setpoint = getController().getSetpoint();
        motor.set(v + feedforward.calculate(setpoint.position - offset, 0));
    }

    @Override
    protected double getMeasurement() {
        return motor.getEncoder().getPosition();
    }

    public double getPosition() {
        return motor.getEncoder().getPosition() - offset;
    }
}
