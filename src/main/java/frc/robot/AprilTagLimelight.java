package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLimelight extends Limelight {
    private final NetworkTableEntry tid;

    public AprilTagLimelight(String name) {
        super(name);
        var table = NetworkTableInstance.getDefault().getTable(name);
        tid = table.getEntry("tid");
    }

    public long getId() {
        return tid.getInteger(0L);
    }
}
