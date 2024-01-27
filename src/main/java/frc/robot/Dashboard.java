package frc.robot;

import edu.wpi.first.networktables.*;

import java.util.*;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public final class Dashboard {
    private Dashboard() {
        throw new AssertionError("utility class");
    }

    private static final Map<DoubleSupplier, DoublePublisher> doublePublishers = new HashMap<>();
    private static final List<DoubleEntry> doubleSubscribers = new ArrayList<>();

    public static void PublishDouble(String tableName, String topicName, DoubleSupplier supplier) {
        var publisher = NetworkTableInstance
            .getDefault()
            .getTable(tableName)
            .getDoubleTopic(topicName)
            .publish();
        doublePublishers.put(supplier, publisher);
    }

    public static void AddDoubleEntry(String tableName, String topicName, DoubleConsumer consumer) {
        var inst = NetworkTableInstance.getDefault();

        var subscriber = inst
            .getTable(tableName)
            .getDoubleTopic(topicName)
            .getEntry(0d);

        doubleSubscribers.add(subscriber);
        subscriber.set(0d);

        inst.addListener(
            subscriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> consumer.accept(e.valueData.value.getDouble()));
    }

    public static void update() {
        doublePublishers.forEach((supplier, publisher) -> publisher.set(supplier.getAsDouble()));
    }
}
