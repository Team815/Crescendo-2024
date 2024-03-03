package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private final CANSparkBase[] motors;
    private final PIDController[] pids;
    private boolean enabled = false;
    private final SimpleMotorFeedforward[] feedForwards = new SimpleMotorFeedforward[] {
        new SimpleMotorFeedforward(0d, 0.000185d),
        new SimpleMotorFeedforward(0d, 0.000205d)
    };
    private final DoubleSupplier motorSpeedDifference;

    public Shooter(CANSparkBase motor1, CANSparkBase motor2) {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor1.setInverted(true);
        motor2.setInverted(true);
        var pid1 = new PIDController(0.0d, 0.00001d, 0d);
        var pid2 = new PIDController(0.0d, 0.00001d, 0d);
        pid1.setTolerance(50d);
        pid2.setTolerance(50d);
        pids = new PIDController[] {
            pid1,
            pid2
        };
        motorSpeedDifference = () -> motor1.getEncoder().getVelocity() - motor2.getEncoder().getVelocity();
        motors = new CANSparkBase[] {
            motor1,
            motor2
        };
    }

    @Override
    public void periodic() {
        System.out.println(motorSpeedDifference.getAsDouble());
        if (enabled) {
            for (int i = 0; i <= 1; i++) {
                motors[i].set(
                    pids[i].calculate(motors[i].getEncoder().getVelocity())
                        + feedForwards[i].calculate(pids[i].getSetpoint()));
            }
        }
    }

    public void run(double speed) {
        for (var pid : pids) {
            pid.reset();
            pid.setSetpoint(speed);
        }
        enabled = true;
    }

    public void stop() {
        enabled = false;
        for (var motor : motors) {
            motor.stopMotor();
        }
    }

    public double getVelocity(int i) {
        return motors[i].getEncoder().getVelocity();
    }

    public double getPower(int i) {
        return motors[i].get();
    }

    public boolean atSetpoint() {
        return Arrays.stream(pids).allMatch(PIDController::atSetpoint);
    }
}
