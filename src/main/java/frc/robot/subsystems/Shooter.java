package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
    private final CANSparkBase motor;
    private final SimpleMotorFeedforward feedforward;

    public Shooter(CANSparkBase motor, CANSparkBase follower) {
        super(new PIDController(0.0d, 0.000004d, 0d));
        getController().setTolerance(20);
        this.motor = motor;
        follower.follow(this.motor);
        feedforward = new SimpleMotorFeedforward(0d, 0.000185d);
    }

    @Override
    protected void useOutput(double v, double v1) {
        motor.set(v + feedforward.calculate(getSetpoint()));
    }

    @Override
    protected double getMeasurement() {
        return getVelocity();
    }

    public void run(double speed) {
        setSetpoint(speed);
        this.getController().reset();
        enable();
    }

    public void stop() {
        disable();
        motor.stopMotor();
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }
}
