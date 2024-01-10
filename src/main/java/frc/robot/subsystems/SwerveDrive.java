package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GyroAngles;
import frc.robot.SwerveModule;

import java.util.Arrays;

public class SwerveDrive extends SubsystemBase {
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public SwerveDrive(Pigeon2 gyro, SwerveModule... module) {
        this.gyro = gyro;
        this.modules = module;
        kinematics = new SwerveDriveKinematics(Arrays
                .stream(modules)
                .map(SwerveModule::getTranslation)
                .toArray(Translation2d[]::new));
        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyro.getYaw().getValue()),
                Arrays
                        .stream(modules)
                        .map(SwerveModule::getPosition)
                        .toArray(SwerveModulePosition[]::new));
    }

    public void drive(double forwardVelocity, double sidewaysVelocity, double angularVelocity) {
        speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelocity);
        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(gyro.getYaw().getValue()));
        var states = kinematics.toSwerveModuleStates(robotSpeeds);
        for (var i = 0; i < modules.length; i++) {
            modules[i].drive(states[i]);
        }
    }

    public GyroAngles getAngles() {
        return new GyroAngles(gyro.getPitch().getValue(), gyro.getRoll().getValue(), gyro.getYaw().getValue());
    }
}
