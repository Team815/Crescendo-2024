package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTableEntry tx;

    public Limelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        tx = table.getEntry("tx");

    }
    public double getX() {
        return tx.getDouble(0d);
    }
}
