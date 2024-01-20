package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTableEntry tx;
    private final NetworkTableEntry tid;

    public Limelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        tx = table.getEntry("tx");
        tid = table.getEntry("tid");
    }

    public double getX() {
        return tx.getDouble(0d);
    }

    public long getId() {
        return tid.getInteger(0L);
    }
}
