package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.SwerveDrive;

import java.util.EnumSet;
import java.util.Map;
import java.util.function.Supplier;

public final class Dashboard {
    private Dashboard() {
        throw new AssertionError("utility class");
    }

    public static void createVelocityLayout(String tabName, int column, int row, Supplier<ChassisSpeeds> speeds) {
        var layout = Shuffleboard.getTab(tabName)
                .getLayout("Velocity", BuiltInLayouts.kGrid)
                .withSize(2, 2)
                .withPosition(column, row)
                .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 3));

        layout
                .addString("Forward Velocity", () -> String.format("%.2f", speeds.get().vxMetersPerSecond))
                .withPosition(0, 0);
        layout
                .addString("Sideways Velocity", () -> String.format("%.2f", speeds.get().vyMetersPerSecond))
                .withPosition(0, 1);
        layout
                .addString("Angular Velocity", () -> String.format("%.2f", speeds.get().omegaRadiansPerSecond))
                .withPosition(0, 2);
    }
}
