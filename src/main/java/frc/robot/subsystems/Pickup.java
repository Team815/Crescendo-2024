package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import java.util.Arrays;

public class Pickup extends PIDSubsystem {
    public static final double PICKUP_SPEED = 2000d;
    public static final double SHOOT_SPEED = 3600d;
    private final CANSparkBase motor;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.000192);
    private final DigitalInput[] noteSensors = new DigitalInput[] {
        new DigitalInput(0),
        new DigitalInput(1)
    };

    @Override
    protected void useOutput(double v, double v1) {
        motor.set(feedforward.calculate(getSetpoint()));
    }

    @Override
    protected double getMeasurement() {
        return motor.getEncoder().getVelocity();
    }

    public Pickup(CANSparkBase motor) {
        super(new PIDController(0d, 0.0001d, 0d));
        motor.restoreFactoryDefaults();
        this.motor = motor;
    }

    public void run(double speed) {
        setSetpoint(speed);
        getController().reset();
        enable();
    }

    public void stop() {
        disable();
        motor.stopMotor();
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public double getPower() {
        return motor.get();
    }

    public boolean hasNote() {
        return Arrays.stream(noteSensors).anyMatch(DigitalInput::get);
    }
}
