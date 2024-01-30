package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.speedsmodulator.SpeedsModulator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class SwerveDrive extends SubsystemBase {
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private final double maxSpeed;
    private final Pigeon2 gyro;
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final List<SpeedsModulator> modulators;

    public SwerveDrive(
        double maxSpeed,
        Pigeon2 gyro,
        SwerveModule[] modules,
        SpeedsModulator... modulators) {
        this.maxSpeed = maxSpeed;
        this.gyro = gyro;
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Arrays
            .stream(this.modules)
            .map(SwerveModule::getTranslation)
            .toArray(Translation2d[]::new));
        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw().getValue()),
            getSwerveModulePositions());
        this.modulators = Arrays
            .stream(modulators)
            .collect(Collectors.toCollection(ArrayList::new));
        resetHeading();
        configureAutoBuilder();
    }

    @Override
    public void periodic() {
        odometry.update(
            gyro.getRotation2d(),
            getSwerveModulePositions());
    }

    public void drive(ChassisSpeeds speeds) {
        for (var modulator : modulators) {
            speeds = modulator.modulate(speeds);
        }
        driveRobotRelativePercent(ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            Rotation2d.fromDegrees(gyro.getYaw().getValue())));
    }

    public void resetHeading() {
        gyro.reset();
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

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose);
    }

    public void addModulator(SpeedsModulator modulator) {
        modulators.add(modulator);
    }

    private void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelativeVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5d, 0d, 0d), // Translation PID constants
                new PIDConstants(5d, 0d, 0d), // Rotation PID constants
                4.4d, // Max module speed, in m/s
                0.43d, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
            },
            this // Reference to this subsystem to set requirements
        );
    }

    private void driveRobotRelativeVelocity(ChassisSpeeds speeds) {
        driveRobotRelativePercent(speeds.div(maxSpeed));
    }

    private void driveRobotRelativePercent(ChassisSpeeds speeds) {
        this.speeds = speeds.times(maxSpeed);
        var states = kinematics.toSwerveModuleStates(speeds);
        for (var i = 0; i < 4; i++) {
            modules[i].drive(states[i]);
        }
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return Arrays
            .stream(modules)
            .map(SwerveModule::getPosition)
            .toArray(SwerveModulePosition[]::new);
    }
}
