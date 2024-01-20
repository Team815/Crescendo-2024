// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CenterOnTarget;
import frc.robot.commands.SimpleDrive;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    private final InputDevice controller = new XboxController();
    private final Limelight noteCamera = new Limelight("limelight-note");
    private final Limelight aprilTagCamera = new Limelight("limelight-tags");
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
        final double backRightAngularOffset = 0.58d;

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
                moduleFrontLeft,
                moduleFrontRight,
                moduleBackLeft,
                moduleBackRight);

        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(new SimpleDrive(
                drive,
                controller::getForwardVelocity,
                controller::getSidewaysVelocity,
                controller::getAngularVelocity));

        controller.resetHeading().onTrue(Commands.runOnce(drive::resetHeading, drive));

        controller.centerOnNote().whileTrue(new CenterOnTarget(
                drive,
                controller::getForwardVelocity,
                controller::getSidewaysVelocity,
                noteCamera::getX));

        controller.test().whileTrue(Commands.run(() -> System.out.println(drive.getPose())));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
