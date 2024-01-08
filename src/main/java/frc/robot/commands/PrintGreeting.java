package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Printer;

public class PrintGreeting extends Command {
    Printer printer;

    public PrintGreeting(Printer printer) {
        this.printer = printer;
        addRequirements(printer);
    }

    @Override
    public void execute() {
        printer.print("Hello world!");
    }
}
