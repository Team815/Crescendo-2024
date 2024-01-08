// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PrintGreeting;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.Printer;

public class RobotContainer {
    private InputDevice controller = new XboxController();
    private Printer printer = new Printer();

    public RobotContainer() {
        configureBindings();
        Command command = new PrintGreeting(printer);
        printer.setDefaultCommand(command);
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.run(() -> {
        });
    }
}
