package swervelib.imu;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Custom driver to interface with the QuestNav IMU via a serial port.
 * NOTE: The data parsing logic (processData) is highly conceptual 
 * and depends entirely on the specific QuestNav message format.
 */
public class QuestNavIMUDriver implements AutoCloseable {

    private final SerialPort serialPort;
    // Thread-safe storage for the latest IMU data
    private final AtomicReference<IMUData> latestData = new AtomicReference<>(new IMUData());
    private final Thread readerThread;
    private volatile boolean keepRunning = true;

    // --- Constructor ---
    public QuestNavIMUDriver(String portName, int baudRate) {
        // Map common port names to WPILib SerialPort.Port enum
        Port port = switch (portName.toUpperCase()) {
            case "USB" -> Port.kUSB;
            case "USB1" -> Port.kUSB1;
            case "USB2" -> Port.kUSB2;
            case "ONBOARD" -> Port.kOnboard;
            default -> Port.kUSB; // Default to a standard USB port
        };

        this.serialPort = new SerialPort(baudRate, port);
        
        // Clear any old data in the buffer
        this.serialPort.flush();
        this.serialPort.reset();

        // Start a separate thread to continuously read data
        readerThread = new Thread(this::readDataLoop);
        readerThread.setName("QuestNavIMUReader");
        readerThread.setDaemon(true);
        readerThread.start();
    }

    // --- Communication Loop ---
    private void readDataLoop() {
        while (keepRunning) {
            try {
                // Check if any data is available (wait up to 50ms)
                if (serialPort.getBytesReceived() > 0) {
                    // Read all available bytes
                    String received = serialPort.readString();
                    
                    // Call the custom method to parse the QuestNav message
                    processData(received);
                } else {
                    // Sleep briefly to avoid busy-waiting
                    Thread.sleep(10);
                }
            } catch (Exception e) {
                System.err.println("QuestNav IMU Read Error: " + e.getMessage());
                try {
                    // Sleep longer after an error to prevent a rapid loop
                    Thread.sleep(100); 
                } catch (InterruptedException interruptedException) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    // --- Data Parsing (Conceptual) ---
    /**
     * This method must be customized based on the exact QuestNav message format.
     * It needs to find the start of a message, extract the fields (Roll, Pitch, Yaw, Accel, etc.),
     * and update the latestData object.
     * @param rawData The raw string/byte data read from the serial port.
     */
    private void processData(String rawData) {
        // --- QuestNav MESSAGE PARSING LOGIC GOES HERE ---
        // This is highly specific to the device.
        // It might involve:
        // 1. Finding a synchronization header (e.g., "$QNI," or a specific byte pattern).
        // 2. Splitting a CSV/NMEA-like string, OR
        // 3. Converting raw binary bytes into floats/doubles using ByteBuffer.
        
        // --- Example: Assuming a simple CSV-like string: Roll,Pitch,Yaw,Ax,Ay,Az,Gz ---
        
        String[] parts = rawData.split(",");
        if (parts.length >= 7) { 
            try {
                IMUData newData = new IMUData();
                newData.roll = Double.parseDouble(parts[0]);
                newData.pitch = Double.parseDouble(parts[1]);
                newData.yaw = Double.parseDouble(parts[2]);
                newData.accelX = Double.parseDouble(parts[3]);
                newData.accelY = Double.parseDouble(parts[4]);
                newData.accelZ = Double.parseDouble(parts[5]);
                newData.yawRate = Double.parseDouble(parts[6]); // Z-axis angular velocity

                latestData.set(newData);
            } catch (NumberFormatException e) {
                // Ignore corrupted messages
            }
        }
    }

    // --- Public Access Methods ---

    /**
     * Gets the current 3D rotation (Roll, Pitch, Yaw) in radians.
     */
    public Rotation3d getRotation3d() {
        IMUData data = latestData.get();
        // Convert degrees (common IMU output) to radians for WPILib's Rotation3d
        return new Rotation3d(
            Math.toRadians(data.roll), 
            Math.toRadians(data.pitch), 
            Math.toRadians(data.yaw)
        );
    }
    
    // ... other public getter methods for the SwerveIMU interface ...

    public double getLinearAccelerationX() { return latestData.get().accelX; }
    public double getLinearAccelerationY() { return latestData.get().accelY; }
    public double getLinearAccelerationZ() { return latestData.get().accelZ; }

    /**
     * Gets the angular velocity around the Z-axis (Yaw Rate) in degrees per second.
     */
    public double getAngularVelocityZ() { return latestData.get().yawRate; }

    /**
     * Sends a command to the IMU to reset the current Yaw angle to zero.
     * This command is specific to QuestNav's protocol.
     */
    public void zeroYaw() {
        // Example: Sending a custom command string/byte sequence to the device
        // serialPort.writeString("ZERO_YAW\r\n"); 
        
        // For the software side, apply a persistent offset to the latest data
        IMUData data = latestData.get();
        // The swerve IMU wrapper handles the offset, so a physical zero command is better.
    }
    
    public void clearFaults() {
        // serialPort.writeString("CLEAR_FAULTS\r\n");
    }

    // --- Internal Data Structure ---
    private static class IMUData {
        public double roll = 0.0;
        public double pitch = 0.0;
        public double yaw = 0.0;
        public double accelX = 0.0;
        public double accelY = 0.0;
        public double accelZ = 0.0;
        public double yawRate = 0.0;
    }

    // --- Cleanup ---
    @Override
    public void close() {
        keepRunning = false;
        try {
            if (readerThread.isAlive()) {
                readerThread.join(200); // Wait for the thread to finish
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        if (serialPort != null) {
            serialPort.close();
        }
    }
}