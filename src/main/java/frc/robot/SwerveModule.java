package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public static final double DEFAULT_MAX_LINEAR_SPEED = 1d;
    public static final double DEFAULT_MAX_ANGULAR_SPEED = 0.2d;
    public static final double DEFAULT_MAX_LINEAR_ACCELERATION = 0.1d;
    public static final double DEFAULT_MAX_ANGULAR_ACCELERATION = 0.1d;
    public static final double DEFAULT_P = 0.015d;

    private double maxLinearSpeed = DEFAULT_MAX_LINEAR_SPEED;
    private double maxAngularSpeed = DEFAULT_MAX_ANGULAR_SPEED;
    private final CANSparkMax linearMotor;
    private final CANSparkMax angularMotor;
    private final CANcoder angleSensor;
    private final Translation2d translation;
    private final PIDController pid = new PIDController(DEFAULT_P, 0d, 0d);

    public SwerveModule(
            CANSparkMax linearMotor,
            CANSparkMax angularMotor,
            CANcoder angleSensor,
            Translation2d translation) {
        this.linearMotor = linearMotor;
        this.angularMotor = angularMotor;
        this.angleSensor = angleSensor;
        this.translation = translation;
        //angleSensor.configFactoryDefault();
        //angleSensor.configMagnetOffset(angularOffset);
        pid.enableContinuousInput(0d, 360d);
    }

    public static SwerveModule fromIds(
            int linearMotorId,
            int angularMotorId,
            int angleSensorId,
            double x,
            double y) {
        var linearMotor = new CANSparkMax(linearMotorId, MotorType.kBrushless);
        var angularMotor = new CANSparkMax(angularMotorId, MotorType.kBrushless);
        linearMotor.restoreFactoryDefaults();
        angularMotor.restoreFactoryDefaults();
        setMaxAcceleration(linearMotor, DEFAULT_MAX_LINEAR_ACCELERATION);
        setMaxAcceleration(angularMotor, DEFAULT_MAX_ANGULAR_ACCELERATION);
        var angleSensor = new CANcoder(angleSensorId);
        return new SwerveModule(linearMotor, angularMotor, angleSensor, new Translation2d(x, y));
    }

    public void drive(SwerveModuleState state) {
        state = SwerveModuleState.optimize(
                state,
                Rotation2d.fromDegrees(angleSensor.getAbsolutePosition().getValue()));
        var spinSpeed = MathUtil.clamp(state.speedMetersPerSecond, -maxLinearSpeed, maxLinearSpeed);
        linearMotor.set(spinSpeed);

        var rotation = state.angle.getDegrees();
        pid.setSetpoint(rotation);
        var response = -pid.calculate(angleSensor.getAbsolutePosition().getValue());
        angularMotor.set(MathUtil.clamp(response, -maxAngularSpeed, maxAngularSpeed));
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                linearMotor.getEncoder().getPosition(),
                Rotation2d.fromDegrees(angleSensor.getAbsolutePosition().getValue()));
    }

    public void setMaxPropelAcceleration(double maxAcceleration) {
        setMaxAcceleration(linearMotor, maxAcceleration);
    }

    public void setMaxRotateAcceleration(double maxAcceleration) {
        setMaxAcceleration(angularMotor, maxAcceleration);
    }

    private static void setMaxAcceleration(CANSparkMax motor, double maxAcceleration) {
        motor.setOpenLoopRampRate(Math.abs(maxAcceleration));
    }
}
