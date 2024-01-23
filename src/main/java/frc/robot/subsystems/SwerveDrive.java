package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AngleCorrectionModulator;
import frc.robot.SwerveModule;

import java.util.Arrays;

public class SwerveDrive extends SubsystemBase {
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final AngleCorrectionModulator angleCorrector;

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
        angleCorrector = new AngleCorrectionModulator(this::getYaw);
        resetHeading();
    }

    @Override
    public void periodic() {
        odometry.update(
                gyro.getRotation2d(),
                Arrays
                        .stream(modules)
                        .map(SwerveModule::getPosition)
                        .toArray(SwerveModulePosition[]::new));
    }

    public void drive(ChassisSpeeds speeds) {
        speeds = angleCorrector.modulate(speeds);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(gyro.getYaw().getValue()));
        this.speeds = speeds;
        var states = kinematics.toSwerveModuleStates(speeds);
        for (var i = 0; i < 4; i++) {
            modules[i].drive(states[i]);
        }
    }

    public void resetHeading() {
        gyro.reset();
        angleCorrector.reset();
    }

    public double getYaw() {
        return gyro.getYaw().getValue();
    }

    public ChassisSpeeds getSpeeds() {
        return speeds;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
}
