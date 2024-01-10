// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SimpleDrive;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    private InputDevice controller = new XboxController();
    private final SwerveDrive drive;

    public RobotContainer() {
        final int frontLeftSpinId = 6;
        final int frontLeftRotateId = 5;
        final int frontRightSpinId = 3;
        final int frontRightRotateId = 4;
        final int backLeftSpinId = 8;
        final int backLeftRotateId = 2;
        final int backRightSpinId = 7;
        final int backRightRotateId = 1;
        final int frontLeftRotateSensorId = 9;
        final int frontRightRotateSensorId = 10;
        final int backLeftRotateSensorId = 11;
        final int backRightRotateSensorId = 12;
        final double frontLeftAngularOffset = -173d;
        final double frontRightAngularOffset = -35d;
        final double backLeftAngularOffset = 95d;
        final double backRightAngularOffset = 170d;

        // The max frame perimeter length is 120 in. For a square chassis,
        // each side would be 30 in. For safety, our chassis sides are 29 in.
        // Half of this is 14.5 in., or 0.368 m.

        final var halfLength = 0.368d;
        final var halfWidth = 0.368d;

        var moduleFrontLeft = SwerveModule.fromIds(
                frontLeftSpinId,
                frontLeftRotateId,
                frontLeftRotateSensorId,
                halfLength,
                halfWidth
        );

        var moduleFrontRight = SwerveModule.fromIds(
                frontRightSpinId,
                frontRightRotateId,
                frontRightRotateSensorId,
                halfLength,
                -halfWidth);

        var moduleBackLeft = SwerveModule.fromIds(
                backLeftSpinId,
                backLeftRotateId,
                backLeftRotateSensorId,
                -halfLength,
                halfWidth);

        var moduleBackRight = SwerveModule.fromIds(
                backRightSpinId,
                backRightRotateId,
                backRightRotateSensorId,
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
                () -> controller.getForwardVelocity(),
                () -> controller.getSidewaysVelocity(),
                () -> controller.getAngularVelocity()));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
