package org.frcteam6941.drivers;
 
import edu.wpi.first.math.geometry.Rotation2d;
 
public class DummyGyro implements Gyro {
    Rotation2d yaw = new Rotation2d();
    Rotation2d pitch = new Rotation2d();
    Rotation2d roll = new Rotation2d();
    Rotation2d previousYaw = new Rotation2d();
    Rotation2d previousPitch = new Rotation2d();
    Rotation2d previousRoll = new Rotation2d();
 
    double dt;
 
    // Constructor to initialize the time interval for calculations
    public DummyGyro(double looperDt) {
        dt = looperDt;
    }
 
    // Returns the current yaw angle
    @Override
    public Rotation2d getYaw() {
        return yaw;
    }
 
    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setYaw(double angleDeg) {
        this.yaw = Rotation2d.fromDegrees(angleDeg);
    }
 
    // Returns the current roll angle
    @Override
    public Rotation2d getRoll() {
        return roll;
    }
 
    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New roll in degrees
     */
    @Override
    public void setRoll(double angleDeg) {
        this.roll = Rotation2d.fromDegrees(angleDeg);
    }
 
    // Returns the current pitch angle
    @Override
    public Rotation2d getPitch() {
        return pitch;
    }
 
    /**
     * Sets the pitch register to read the specified value.
     *
     * @param angleDeg New pitch in degrees
     */
    @Override
    public void setPitch(double angleDeg) {
        this.pitch = Rotation2d.fromDegrees(angleDeg);
    }
 
    // Returns the raw angular rates for yaw, pitch, and roll
    @Override
    public double[] getRaw() {
        return new double[]{
                (yaw.getDegrees() - previousYaw.getDegrees()) / dt,
                (pitch.getDegrees() - previousPitch.getDegrees()) / dt,
                (roll.getDegrees() - previousRoll.getDegrees()) / dt
        };
    }
}