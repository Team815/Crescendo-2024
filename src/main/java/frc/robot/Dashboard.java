package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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

    public static void createAutoLayout(RobotContainer container) {
        var options = new String[] {
            "1. Inner",
            "2. Middle",
            "3. Outer",
            "4. Collect Center"
        };
        container.setAuto(options[0]);
        var autoChooser = new SendableChooser<String>();
        autoChooser.setDefaultOption(options[0], options[0]);
        for (var option : Arrays.stream(options).skip(1).toArray(String[]::new)) {
            autoChooser.addOption(option, option);
        }
        Shuffleboard
            .getTab("Configuration")
            .add("Autonomous", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
        NetworkTableInstance
            .getDefault()
            .getTable("Shuffleboard")
            .getSubTable("Configuration")
            .getSubTable("Autonomous")
            .addListener(
                "selected",
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                (table, key, event) -> {
                    container.setAuto(event.valueData.value.getString());
                });
    }
}
