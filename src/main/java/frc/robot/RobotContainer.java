// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CenterOnTarget;
import frc.robot.commands.DriveTo;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.speedsmodulator.AccelerationLimiter;
import frc.robot.subsystems.speedsmodulator.AngleCorrector;

public class RobotContainer {
    private final InputDevice controller = new XboxController();
    private final Limelight noteCamera = new Limelight("limelight-note");
    private final AprilTagLimelight aprilTagCamera = new AprilTagLimelight("limelight-tags");
    private final SwerveDrive drive;

    public RobotContainer() {
        final int frontLeftSpinId = 1;
        final int frontLeftRotateId = 2;
        final int frontRightSpinId = 7;
        final int frontRightRotateId = 8;
        final int backLeftSpinId = 5;
        final int backLeftRotateId = 6;
        final int backRightSpinId = 4;
        final int backRightRotateId = 3;
        final int frontLeftRotateSensorId = 1;
        final int frontRightRotateSensorId = 2;
        final int backLeftRotateSensorId = 3;
        final int backRightRotateSensorId = 4;
        final double frontLeftAngularOffset = 0.271d;
        final double frontRightAngularOffset = 0.515d;
        final double backLeftAngularOffset = 0.47d;
        final double backRightAngularOffset = 0.68d;

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
            new Pigeon2(0),
            new SwerveModule[]{
                moduleFrontLeft,
                moduleFrontRight,
                moduleBackLeft,
                moduleBackRight});

        var angleCorrector = new AngleCorrector(drive::getYaw);
        drive.addModulator(angleCorrector);
        var accelerationLimiter = new AccelerationLimiter(0.01d, 0.01d);
        Dashboard.AddDoubleEntry("Speeds Modulator", "Max Linear Acceleration", accelerationLimiter::setMaxLinearAcceleration);
        drive.addModulator(accelerationLimiter);

        Dashboard.PublishDouble("Velocity", "Forward", () -> drive.getSpeeds().vxMetersPerSecond);
        Dashboard.PublishDouble("Velocity", "Sideways", () -> drive.getSpeeds().vyMetersPerSecond);
        Dashboard.PublishDouble("Velocity", "Angular", () -> drive.getSpeeds().omegaRadiansPerSecond);
        Dashboard.PublishDouble("Pose", "Forward", () -> drive.getPose().getX());
        Dashboard.PublishDouble("Pose", "Sideways", () -> drive.getPose().getY());
        Dashboard.PublishDouble("Pose", "Angle", () -> drive.getPose().getRotation().getDegrees());

        configureBindings(angleCorrector);
    }

    private void configureBindings(AngleCorrector angleCorrector) {
        drive.setDefaultCommand(Commands.run(
            () -> drive.drive(new ChassisSpeeds(
                controller.getForwardVelocity(),
                controller.getSidewaysVelocity(),
                controller.getAngularVelocity())),
            drive));

        controller.resetHeading().onTrue(Commands.runOnce(() -> {
            drive.resetHeading();
            angleCorrector.reset();
        }, drive));

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

        controller.test().whileTrue(Commands.run(() -> drive.drive(new ChassisSpeeds(1, 0, 0))));
        CommandScheduler.getInstance().schedule(Commands.run(() -> {}));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Loop");
    }
}
