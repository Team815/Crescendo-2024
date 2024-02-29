package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;

    public Limelight(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
    }

    public double getX() {
        return tx.getDouble(0d);
    }

    public double getY() {
        return ty.getDouble(0d);
    }
}
