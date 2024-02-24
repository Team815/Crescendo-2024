// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.speedsmodulator.AccelerationLimiter;
import frc.robot.subsystems.speedsmodulator.AngleCorrector;

import java.util.function.DoubleSupplier;

public class RobotContainer {
    private final InputDevice controller = new XboxController();
    private final AprilTagLimelight aprilTagCamera = new AprilTagLimelight("limelight-tags");
    private double startingAngle;
    private Command resetHeadingCommand;
    private final AngleCorrector angleCorrector;
    private final Commander commander;
    private final SwerveDrive drive;
    private final Pickup pickup;
    private final Shooter shooter;
    private final Arm arm;

    public RobotContainer() {
        final int frontLeftSpinId = 1;
        final int frontLeftRotateId = 2;
        final int frontRightSpinId = 7;
        final int frontRightRotateId = 8;
        final int backLeftSpinId = 5;
        final int backLeftRotateId = 6;
        final int backRightSpinId = 4;
        final int backRightRotateId = 3;
        final int pickupId = 9;
        final int shooterId1 = 10;
        final int shooterId2 = 11;
        final int armId1 = 12;
        final int armId2 = 13;
        final int frontLeftRotateSensorId = 1;
        final int frontRightRotateSensorId = 2;
        final int backLeftRotateSensorId = 3;
        final int backRightRotateSensorId = 4;
        final double frontLeftAngularOffset = 0.271d;
        final double frontRightAngularOffset = 0.57d;
        final double backLeftAngularOffset = 0.47d;
        final double backRightAngularOffset = 0.18d;
        final double maxDriveSpeed = 4.4d;

        // The max frame perimeter length is 120 in. For a square chassis,
        // each side would be 30 in. For safety, our chassis sides are 29 in.
        // Half of this is 14.5 in., or 0.368 m.

        final var halfLength = 0.368d;
        final var halfWidth = 0.368d;

        var moduleFrontLeft = SwerveModule.fromIds(
            frontLeftSpinId,
            frontLeftRotateId,
            frontLeftRotateSensorId,
            frontLeftAngularOffset,
            halfLength,
            halfWidth
        );

        var moduleFrontRight = SwerveModule.fromIds(
            frontRightSpinId,
            frontRightRotateId,
            frontRightRotateSensorId,
            frontRightAngularOffset,
            halfLength,
            -halfWidth);

        var moduleBackLeft = SwerveModule.fromIds(
            backLeftSpinId,
            backLeftRotateId,
            backLeftRotateSensorId,
            backLeftAngularOffset,
            -halfLength,
            halfWidth);

        var moduleBackRight = SwerveModule.fromIds(
            backRightSpinId,
            backRightRotateId,
            backRightRotateSensorId,
            backRightAngularOffset,
            -halfLength,
            -halfWidth);

        drive = new SwerveDrive(
            maxDriveSpeed,
            new Pigeon2(0),
            new SwerveModule[]{
                moduleFrontLeft,
                moduleFrontRight,
                moduleBackLeft,
                moduleBackRight});

        angleCorrector = new AngleCorrector(() -> drive.getAngle().getDegrees());
        drive.addModulator(angleCorrector);
        var accelerationLimiter = new AccelerationLimiter(0.05d, 0.05d);
        Dashboard.AddDoubleEntry("Speeds Modulator", "Max Linear Acceleration", accelerationLimiter::setMaxLinearAcceleration);
        drive.addModulator(accelerationLimiter);

        // Initialize pickup

        pickup = new Pickup(new CANSparkMax(pickupId, CANSparkLowLevel.MotorType.kBrushless));

        // Initialize shooter

        shooter = new Shooter(
            new CANSparkMax(shooterId1, CANSparkLowLevel.MotorType.kBrushless),
            new CANSparkMax(shooterId2, CANSparkLowLevel.MotorType.kBrushless));

        // Initialize arm

        arm = new Arm(
            new CANSparkMax(armId1, CANSparkLowLevel.MotorType.kBrushless),
            new CANSparkMax(armId2, CANSparkLowLevel.MotorType.kBrushless));

        commander = new Commander(pickup, arm, shooter);

        //var autoChooser = AutoBuilder.buildAutoChooser();

        configureDashboard();

        configureAutoCommands();

        configureBindings();
    }

    private void configureDashboard() {
        Dashboard.PublishDouble("Velocity", "Forward", () -> drive.getSpeeds().vxMetersPerSecond);
        Dashboard.PublishDouble("Velocity", "Sideways", () -> drive.getSpeeds().vyMetersPerSecond);
        Dashboard.PublishDouble("Velocity", "Angular", () -> drive.getSpeeds().omegaRadiansPerSecond);
        Dashboard.PublishDouble("Pose", "Forward", () -> drive.getPose().getX());
        Dashboard.PublishDouble("Pose", "Sideways", () -> drive.getPose().getY());
        Dashboard.PublishDouble("Pose", "Angle", () -> drive.getPose().getRotation().getDegrees());
        Dashboard.PublishDouble("Arm", "Angle", arm::getPosition);
        Dashboard.PublishDouble("Shooter", "Speed", shooter::getVelocity);
    }

    private void configureAutoCommands() {

        NamedCommands.registerCommand("Pickup", Commands.startEnd(
            () -> pickup.run(0.4d),
            () -> pickup.run(0d),
            pickup));

        NamedCommands.registerCommand(
            "PrimeFeeder",
            Commands.waitSeconds(0.1d)
                .alongWith(Commands.runOnce(() -> pickup.run(-0.2d)))
                .andThen(Commands.runOnce(() -> pickup.run(0d))));

        NamedCommands.registerCommand(
            "ShootClose",
            new PrimeShooter(10d, 3000d, shooter, arm).withTimeout(1.5d)
                .andThen(Commands.run(() -> pickup.run(0.4d), pickup)).withTimeout(3d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));

        NamedCommands.registerCommand(
            "ShootMiddle",
            new PrimeShooter(22d, 3000d, shooter, arm).withTimeout(1.5d)
                .andThen(Commands.run(() -> pickup.run(0.4d), pickup)).withTimeout(3d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));

        NamedCommands.registerCommand(
            "ShootOuter",
            new PrimeShooter(24d, 3000d, shooter, arm).withTimeout(1.5d)
                .andThen(Commands.run(() -> pickup.run(0.4d), pickup)).withTimeout(3d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));
    }

    private void configureBindings() {
        drive.setDefaultCommand(Commands.run(
            () -> drive.drive(new ChassisSpeeds(
                controller.getForwardVelocity(),
                controller.getSidewaysVelocity(),
                controller.getAngularVelocity())),
            drive));

        resetHeadingCommand = Commands.runOnce(() -> {
            drive.setAngle(0d);
            angleCorrector.reset(0d);
        }, drive);

        controller.resetHeading().onTrue(resetHeadingCommand);

        controller.centerOnAprilTag().whileTrue(new CenterOnTarget(
            drive,
            controller::getForwardVelocity,
            controller::getSidewaysVelocity,
            aprilTagCamera::getX));

        controller.pickup().whileTrue(Commands.startEnd(
            () -> pickup.run(0.4d),
            () -> Commands.waitSeconds(0.1d)
                .alongWith(Commands.runOnce(() -> pickup.run(-0.2d)))
                .andThen(() -> pickup.run(0d))
                .schedule(),
            pickup));

        controller.drop().whileTrue(Commands.startEnd(
            () -> pickup.run(-0.3d),
            () -> pickup.run(0d),
            pickup));

        controller.shoot().whileTrue(new ShootAuto(10d, 3000d, commander));

        controller.test().whileTrue(new ShootAuto(22d, 3000d, commander));
    }

    public Command getAutonomousCommand() {
        var autoName = "Shoot Outer Note";
        startingAngle = PathPlannerAuto
            .getStaringPoseFromAutoFile(autoName)
            .getRotation()
            .getDegrees();
        return new PathPlannerAuto(autoName);
    }

    public Command getTeleopInitCommand() {
        return Commands.runOnce(() -> {
            var angle = drive.getAngle().getDegrees() - startingAngle;
            drive.setAngle(angle);
            angleCorrector.reset(angle);
        });
    }
}
