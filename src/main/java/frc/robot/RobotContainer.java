// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SimpleDrive;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

public class RobotContainer {
    private InputDevice controller = new XboxController();
    private SwerveDrive drive;

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

        SwerveModule module = new SwerveModule(backRightSpinId, backRightRotateId);
        drive = new SwerveDrive(module);

        configureBindings();
        Command command = new SimpleDrive(
            drive,
            () -> controller.getForwardVelocity(),
            () -> controller.getSidewaysVelocity(),
            () -> controller.getAngularVelocity());
        drive.setDefaultCommand(command);
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.run(() -> {
        });
    }
}
