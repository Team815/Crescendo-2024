package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

public final class Dashboard {
    private Dashboard() {
        throw new AssertionError("utility class");
    }

    private static final Map<DoubleSupplier, DoublePublisher> doublePublishers = new HashMap<>();

    public static void addDouble(String topicName, DoubleSupplier supplier) {
        var topic = NetworkTableInstance.getDefault().getDoubleTopic(topicName);
        var publisher = topic.publish();
        doublePublishers.put(supplier, publisher);
    }

    public static void update() {
        doublePublishers.forEach((supplier, publisher) -> publisher.set(supplier.getAsDouble()));
    }
}
