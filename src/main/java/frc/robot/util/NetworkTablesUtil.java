package frc.robot.util;

import edu.wpi.first.networktables.*;
import frc.robot.Constants.NetworkTablesConstants;

import java.util.HashMap;
import java.util.Map;

public final class NetworkTablesUtil {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    public static final NetworkTable MAIN_ROBOT_TABLE = INSTANCE.getTable(NetworkTablesConstants.MAIN_TABLE_NAME);
    private static final Map<String, GenericPublisher> publishers = new HashMap<>();
    private static final Map<String, GenericSubscriber> subscribers = new HashMap<>();

    /**
     * Gets the NetworkTablesConstants Instance being used by the program
     *
     * @return {@link NetworkTableInstance} used
     */
    public static NetworkTableInstance getNTInstance() {
        return INSTANCE;
    }

    /**
     * Returns the table reference from NetworkTablesConstants
     *
     * @param tableName The name of the table
     * @return {@link NetworkTable} corresponding
     */
    public static NetworkTable getTable(String tableName) {
        return INSTANCE.getTable(tableName);
    }
    
    public static double[] getAprilTagEntryA() {
        return INSTANCE.getTable("limelight-a").getEntry("botpose").getDoubleArray(new double[]{0.0});
    }

    public static double[] getAprilTagEntryB() {
        return INSTANCE.getTable("limelight-b").getEntry("botpose").getDoubleArray(new double[]{0.0});
    }
    
    public static double[] getAprilTagEntryC() {
        return INSTANCE.getTable("limelight-c").getEntry("botpose").getDoubleArray(new double[]{0.0});
    }
    
    /**
     * Returns the entry reference from NetworkTablesConstants
     *
     * @param tableName Name of the table
     * @param entryName Name of the entry
     * @return {@link NetworkTableEntry} corresponding
     */
    public static NetworkTableEntry getEntry(String tableName, String entryName) {
        return getTable(tableName).getEntry(entryName);
    }

    public static GenericPublisher getPublisher(String tableName, String entryName, NetworkTableType type) {
        String path = "/" + tableName + "/" + entryName;
        var temp = publishers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newPublisher = entry.getTopic().genericPublish(type.getValueStr(), PubSubOption.keepDuplicates(true));
        publishers.put(path, newPublisher);
        return newPublisher;
    }

    public static GenericSubscriber getSubscriber(String tableName, String entryName) {
        String path = "/" + tableName + "/" + entryName;
        var temp = subscribers.get(path);
        if (temp != null) {
            return temp;
        }
        var entry = getEntry(tableName, entryName);
        var newSubscriber = entry.getTopic().genericSubscribe(entry.getType().getValueStr(), PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(10));
        subscribers.put(path, newSubscriber);
        return newSubscriber;
    }

    public static void printConnections() {
        for (ConnectionInfo connection : INSTANCE.getConnections()) {
            System.out.println("Connection: Using version " + connection.protocol_version + ", ID: " + connection.remote_id + ", IP: " + connection.remote_ip + ", last update: " + connection.last_update);
        }
        if (INSTANCE.getConnections().length == 0) {
            System.out.println("NO CONNECTIONS FOUND");
        }
        System.out.println("END CONNECTIONS LIST \n\n\n\n");
    }

    /**
     * Use in conjunction w/ latencyTest() in network_tables.py to test latency.
     * (Also good example code)
     */
    public static void latencyTesterPeriodicRun() {
        var trajectorySub = getSubscriber("test", "test");
        final var EMPTY = new double[]{};

        TimestampedDoubleArray tsDA = new TimestampedDoubleArray(NetworkTablesJNI.now(), trajectorySub.getLastChange(), trajectorySub.getDoubleArray(EMPTY));
        var timeDiff = (tsDA.timestamp - tsDA.serverTime) / 1000;
        if (timeDiff > 1000) {
            System.out.println(timeDiff - 1000);
        }
    }
}