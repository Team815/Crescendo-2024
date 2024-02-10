package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
    private final CANSparkBase motor;


    public Shooter(CANSparkBase motor, CANSparkBase follower) {
        super(new PIDController(1d, 0d, 0d));
        this.motor = motor;
        follower.follow(this.motor, true);
        enable();
    }

    @Override
    public void periodic() {
        System.out.println(motor.getEncoder().getVelocity());
    }

    @Override
    protected void useOutput(double v, double v1) {
        System.out.println(v);
        motor.set(MathUtil.clamp(v, -1d, 1d));
    }

    @Override
    protected double getMeasurement() {
        return motor.getEncoder().getVelocity();
    }

    public void run(double speed) {
        setSetpoint(speed);
    }
}
