package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class SwerveModule {
    CANSparkMax propelMotor;
    CANSparkMax rotateMotor;

    public SwerveModule(int propelMotorId, int rotateMotorId) {
        propelMotor = new CANSparkMax(propelMotorId, MotorType.kBrushless);
        rotateMotor = new CANSparkMax(rotateMotorId, MotorType.kBrushless);
    }

    public void drive(double propelSpeed, double rotateSpeed) {
        propelMotor.set(propelSpeed);
        rotateMotor.set(rotateSpeed);
    }
}
