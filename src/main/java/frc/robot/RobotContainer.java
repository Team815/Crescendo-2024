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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.CenterOnTarget;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.speedsmodulator.AccelerationLimiter;
import frc.robot.subsystems.speedsmodulator.AngleCorrector;

public class RobotContainer {
    private final InputDevice controller = new XboxController();
    private final Limelight noteCamera = new Limelight("limelight-note");
    private final AprilTagLimelight aprilTagCamera = new AprilTagLimelight("limelight-tags");
    private double startingAngle;
    private Command resetHeadingCommand;
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
        final int frontLeftRotateSensorId = 1;
        final int frontRightRotateSensorId = 2;
        final int backLeftRotateSensorId = 3;
        final int backRightRotateSensorId = 4;
        final double frontLeftAngularOffset = 0.271d;
        final double frontRightAngularOffset = 0.49d;
        final double backLeftAngularOffset = 0.47d;
        final double backRightAngularOffset = 0.72d;
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

        var angleCorrector = new AngleCorrector(() -> drive.getAngle().getDegrees());
        //drive.addModulator(angleCorrector);
        var accelerationLimiter = new AccelerationLimiter(0.05d, 0.05d);
        Dashboard.AddDoubleEntry("Speeds Modulator", "Max Linear Acceleration", accelerationLimiter::setMaxLinearAcceleration);
        drive.addModulator(accelerationLimiter);

        pickup = new Pickup(new CANSparkMax(pickupId, CANSparkLowLevel.MotorType.kBrushless));

        var shooter1 = new CANSparkMax(shooterId1, CANSparkLowLevel.MotorType.kBrushless);
        shooter1.setInverted(true);
        shooter = new Shooter(
            shooter1,
            new CANSparkMax(shooterId2, CANSparkLowLevel.MotorType.kBrushless)
        );

        //arm = new Arm(new CanSparkMax815(armId, CANSparkLowLevel.MotorType.kBrushless));
        arm = new Arm(new DummyMotorController());

        Dashboard.PublishDouble("Velocity", "Forward", () -> drive.getSpeeds().vxMetersPerSecond);
        Dashboard.PublishDouble("Velocity", "Sideways", () -> drive.getSpeeds().vyMetersPerSecond);
        Dashboard.PublishDouble("Velocity", "Angular", () -> drive.getSpeeds().omegaRadiansPerSecond);
        Dashboard.PublishDouble("Pose", "Forward", () -> drive.getPose().getX());
        Dashboard.PublishDouble("Pose", "Sideways", () -> drive.getPose().getY());
        Dashboard.PublishDouble("Pose", "Angle", () -> drive.getPose().getRotation().getDegrees());

        NamedCommands.registerCommand("Pickup", Commands.startEnd(
            () -> pickup.run(0.4d),
            () -> pickup.run(0d),
            pickup));
        NamedCommands.registerCommand("Shoot", new PrintCommand("Shooting"));

        configureBindings(angleCorrector);
    }

    private void configureBindings(AngleCorrector angleCorrector) {
        drive.setDefaultCommand(Commands.run(
            () -> drive.drive(new ChassisSpeeds(
                controller.getForwardVelocity(),
                controller.getSidewaysVelocity(),
                controller.getAngularVelocity())),
            drive));

        resetHeadingCommand = Commands.runOnce(() -> {
            drive.setAngle(0d);
            angleCorrector.reset();
        }, drive);

        controller.resetHeading().onTrue(resetHeadingCommand);

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

        controller.pickup().whileTrue(Commands.startEnd(
            () -> pickup.run(0.3d),
            () -> pickup.run(0d),
            pickup));

        controller.shoot().whileTrue(Commands.startEnd(
            () -> shooter.run(2000d),
            () -> shooter.run(0d),
            shooter));

//        controller.test().whileTrue(Commands.startEnd(
//            () -> arm.run(0.1d),
//            () -> arm.run(0d)));

        controller.test().whileTrue(Commands.run(() -> drive.drive(new ChassisSpeeds(0.01d, 0d, 0d))));
    }

    public Command getAutonomousCommand() {
        var autoName = "Pickup And Shoot 3 Notes";
        startingAngle = PathPlannerAuto
            .getStaringPoseFromAutoFile(autoName)
            .getRotation()
            .getDegrees();
        return new PathPlannerAuto(autoName);
    }

    public Command getTeleopInitCommand() {
        return Commands.runOnce(() -> {
            drive.setAngle(drive.getAngle().getDegrees() - startingAngle);
        });
    }
}
