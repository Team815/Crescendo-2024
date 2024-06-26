// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
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
import frc.robot.subsystems.*;
import frc.robot.subsystems.speedsmodulator.AccelerationLimiter;
import frc.robot.subsystems.speedsmodulator.AngleCorrector;
import frc.robot.subsystems.speedsmodulator.SpeedScaler;

public class RobotContainer {
    private final InputDevice controller = new XboxController();
    private final Limelight noteCamera = new Limelight("limelight-note");
    private final AprilTagLimelight aprilTagCamera = new AprilTagLimelight("limelight-tags");
    private double startingAngle;
    private Command resetHeadingCommand;
    private final AngleCorrector angleCorrector;
    private final SpeedScaler speedScaler;
    private final Commander commander;
    private String autoName;
    private final SwerveDrive drive;
    private final Pickup pickup;
    private final Shooter shooter;
    private final Arm arm;
    private final Climber climber;

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
        final int climberId1 = 14;
        final int climberId2 = 15;
        final int frontLeftRotateSensorId = 1;
        final int frontRightRotateSensorId = 2;
        final int backLeftRotateSensorId = 3;
        final int backRightRotateSensorId = 4;
        final int armSensorId = 5;
        final double frontLeftAngularOffset = 0.87d;
        final double frontRightAngularOffset = 0.57d;
        final double backLeftAngularOffset = 0.48d;
        final double backRightAngularOffset = 0.17d;
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
        speedScaler = new SpeedScaler();
        drive.addModulator(speedScaler);
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
            new CANSparkMax(armId2, CANSparkLowLevel.MotorType.kBrushless),
            new CANcoder(armSensorId));

        climber = new Climber(
            new CANSparkMax(climberId1, CANSparkLowLevel.MotorType.kBrushless),
            new CANSparkMax(climberId2, CANSparkLowLevel.MotorType.kBrushless)
        );

        commander = new Commander(pickup, arm, shooter);

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
        Dashboard.PublishDouble("Arm", "In Position", () -> arm.getController().atGoal() ? 1 : 0);
        Dashboard.PublishDouble("Shooter", "Speed1", () -> shooter.getVelocity(0));
        Dashboard.PublishDouble("Shooter", "Power1", () -> shooter.getPower(0));
        Dashboard.PublishDouble("Shooter", "Speed2", () -> shooter.getVelocity(1));
        Dashboard.PublishDouble("Shooter", "Power2", () -> shooter.getPower(1));
        Dashboard.PublishDouble("Shooter", "At Speed", () -> shooter.atSetpoint() ? 1 : 0);
        Dashboard.PublishDouble("Pickup", "Speed", pickup::getVelocity);
        Dashboard.PublishDouble("Pickup", "Power", pickup::getPower);
        Dashboard.PublishDouble("Pickup", "Note", () -> pickup.hasNote() ? 1 : 0);
        Dashboard.PublishDouble("Camera", "Y", aprilTagCamera::getY);
        Dashboard.createAutoLayout(this);
    }

    private void configureAutoCommands() {

        NamedCommands.registerCommand("Pickup", new PickupNote(pickup));

        NamedCommands.registerCommand(
            "ShootCloseMiddle",
            new PrimeShooter(11d, 3250d, shooter, arm).withTimeout(1.5d)
                .andThen(Commands.run(() -> pickup.run(Pickup.PICKUP_SPEED), pickup)).withTimeout(2d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));

        NamedCommands.registerCommand(
            "ShootCloseOutside",
            new PrimeShooter(8d, 3250d, shooter, arm).withTimeout(1.5d)
                .andThen(Commands.run(() -> pickup.run(Pickup.PICKUP_SPEED), pickup)).withTimeout(2d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));

        NamedCommands.registerCommand(
            "ShootMiddle",
            new PrimeShooter(23d, 3250d, shooter, arm).withTimeout(1.5d)
                .andThen(Commands.run(() -> pickup.run(Pickup.PICKUP_SPEED), pickup)).withTimeout(2d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));

        NamedCommands.registerCommand(
            "ShootOuter",
            new PrimeShooter(21.5d, 3250d, shooter, arm).withTimeout(1d)
                .andThen(Commands.run(() -> pickup.run(Pickup.PICKUP_SPEED), pickup)).withTimeout(2d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));

        NamedCommands.registerCommand(
            "MiddleAndShootOuter",
            new PrimeShooter(23d, 3250d, shooter, arm).withTimeout(1d)
                .andThen(Commands.run(() -> pickup.run(Pickup.PICKUP_SPEED), pickup)).withTimeout(2d)
                .andThen(new StopShooting(arm, shooter, pickup).withTimeout(1d)));

        NamedCommands.registerCommand(
            "PickupAndShootSlow",
            Commands.run(
                () -> {
                    pickup.run(Pickup.PICKUP_SPEED);
                    shooter.run(2750d);
                },
                pickup,
                shooter)
        );

        NamedCommands.registerCommand(
            "PickupAndShoot",
            Commands.run(
                () -> {
                    pickup.run(Pickup.PICKUP_SPEED);
                    shooter.run(3000d);
                },
                pickup,
                shooter)
        );
    }

    private void configureBindings() {
        drive.setDefaultCommand(Commands.run(
            () -> drive.drive(new ChassisSpeeds(
                controller.getForwardVelocity(),
                controller.getSidewaysVelocity(),
                controller.getAngularVelocity())),
            drive));

        resetHeadingCommand = Commands.runOnce(() -> {
            drive.setAngle(180d);
            angleCorrector.reset(180d);
        }, drive);

        controller.resetHeading().onTrue(resetHeadingCommand);

        controller.resetHeadingForward().onTrue(Commands.runOnce(() -> resetHeading(0d)));
        controller.resetHeadingRight().onTrue(Commands.runOnce(() -> resetHeading(270d)));
        controller.resetHeadingBack().onTrue(Commands.runOnce(() -> resetHeading(180d)));
        controller.resetHeadingLeft().onTrue(Commands.runOnce(() -> resetHeading(90d)));

        controller.centerOnNote().whileTrue(new CenterOnTarget(
            drive,
            controller::getForwardVelocity,
            controller::getSidewaysVelocity,
            noteCamera::getX));

        controller.centerOnAprilTag().whileTrue(new CenterOnTarget(
            drive,
            controller::getForwardVelocity,
            controller::getSidewaysVelocity,
            aprilTagCamera::getX));

        controller.pickup().whileTrue(new PickupNote(pickup));

        controller.drop().whileTrue(Commands.startEnd(
            () -> {
                pickup.run(-Pickup.PICKUP_SPEED);
                shooter.run(-300d);
            },
            () -> {
                pickup.stop();
                shooter.stop();
            },
            pickup));

        controller.shoot().whileTrue(new ShootAuto(aprilTagCamera::getY, 3000d, commander));

        controller.test().whileTrue(new ShootAuto(21.4d, 3000d, commander));

        controller.scoreAmp().whileTrue(new ShootAuto(90d, 600d, commander));

        controller.climb().whileTrue(Commands.startEnd(
            () -> climber.run(0.5d),
            () -> climber.run(0d)
        ));
    }

    public void setAuto(String autoName) {
        this.autoName = autoName;
    }

    public Command getAutonomousCommand() {
        startingAngle = PathPlannerAuto
            .getStaringPoseFromAutoFile(autoName)
            .getRotation()
            .getDegrees();
        return new PathPlannerAuto(autoName).andThen(Commands.waitSeconds(15d));
    }

    public Command getTeleopInitCommand() {
        return Commands.runOnce(() -> {
            var angle = drive.getAngle().getDegrees() - startingAngle;
            drive.setAngle(angle);
            angleCorrector.reset(angle);
        });
    }

    private void resetHeading(double angle) {
        drive.setAngle(angle);
        angleCorrector.reset(angle);
    }
}
