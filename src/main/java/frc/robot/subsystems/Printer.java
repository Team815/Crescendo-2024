package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Printer extends SubsystemBase {
    public void print(String message) {
        System.out.println(message);
    }
}
