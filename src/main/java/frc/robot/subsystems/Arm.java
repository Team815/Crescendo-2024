package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class Arm extends ProfiledPIDSubsystem {
    private final CANSparkBase motor;
    private final ArmFeedforward feedforward;
    private final CANcoder angleSensor;

    public Arm(CANSparkBase motor, CANSparkBase follower, CANcoder angleSensor) {
        super(new ProfiledPIDController(
            1d,
            0d,
            0d,
            new TrapezoidProfile.Constraints(4d, 8d)));
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
        this.angleSensor = angleSensor;
        feedforward = new ArmFeedforward(0d, 0.124d, 0d);
    }

    public void setPosition(double position) {
        setGoal(Math.toRadians(position));
        enable();
    }

    public void stop() {
        System.out.println("Stopping");
        disable();
        motor.stopMotor();
    }

    @Override
    protected void useOutput(double v, TrapezoidProfile.State state) {
        var setpoint = getController().getSetpoint();
        motor.set(v + feedforward.calculate(setpoint.position, 0d));
    }

    @Override
    protected double getMeasurement() {
        return Math.toRadians(getPosition());
    }

    public double getPosition() {
        final var encoderOffset = 0.421d;
        final var armOffset = -12d;

        return 360d * (encoderOffset - angleSensor.getAbsolutePosition().getValue()) + armOffset;
    }

    public void set(double power) {
        motor.set(power);
    }
}
