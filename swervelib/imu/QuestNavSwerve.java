package swervelib.imu;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.subsystems.questnav.QuestNavSubsystem;

public class QuestNavSwerve extends SwerveIMU {

    QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();

    @Override
    public void close() {
        // TODO close
    }

    /**
     * Reset IMU to factory default.
     */
    @Override
    public void factoryDefault() {
        // TODO factoryDefault
    }

    /**
     * Clear sticky faults on IMU.
     */
    @Override
    public void clearStickyFaults() {
        // TODO clearStickyFaulta
    }

    /**
     * Set the gyro offset.
     *
     * @param offset gyro offset as a {@link Rotation3d}.
     */
    @Override
    public void setOffset(Rotation3d offset) {
        // TODO setOffset
    }

    /**
     * Set the gyro to invert its default direction.
     *
     * @param invertIMU gyro direction
     */
    @Override
    public void setInverted(boolean invertIMU) {
        // TODO setInverted
    }

    /**
     * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
     *
     * @return {@link Rotation3d} from the IMU.
     */
    @Override
    public Rotation3d getRawRotation3d() {
        // TODO getRawRotation3d properly
        return new Rotation3d(questNavSubsystem.getCurrentRotation());
    }

    /**
     * Fetch the {@link Rotation3d} from the IMU. Robot relative.
     *
     * @return {@link Rotation3d} from the IMU.
     */
    @Override
    public Rotation3d getRotation3d() {
        // TODO getRotation3d
        return new Rotation3d(questNavSubsystem.getCurrentRotation());
    }
//questNavSubsystem.getCurrentRotation()
    /**
     * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
     * empty.
     *
     * @return {@link Translation3d} of the acceleration as an {@link Optional}.
     */
    @Override
    public Optional<Translation3d> getAccel() {
        // TODO getAccel
    }

    /**
     * Fetch the rotation rate from the IMU as {@link MutAngularVelocity}
     *
     * @return {@link MutAngularVelocity} of the rotation rate.
     */
    @Override
    public MutAngularVelocity getYawAngularVelocity() {
        return questNavSubsystem.getYawVelocity();
    }

    /**
     * Get the instantiated IMU object.
     *
     * @return IMU object.
     */
    @Override
    public Object getIMU() {
        // TODO getIMU
    }

}