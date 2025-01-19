package org.frcteam6941.drivers;
 
import edu.wpi.first.math.geometry.Rotation2d;
 
public interface Gyro {
    // Returns the current yaw angle of the gyro as a Rotation2d object
    Rotation2d getYaw();
 
    // Sets the yaw angle of the gyro to the specified angle in degrees
    void setYaw(double angle);
 
    // Returns the current pitch angle of the gyro as a Rotation2d object
    Rotation2d getPitch();
 
    // Sets the pitch angle of the gyro to the specified angle in degrees
    void setPitch(double angle);
 
    // Returns the current roll angle of the gyro as a Rotation2d object
    Rotation2d getRoll();
 
    // Sets the roll angle of the gyro to the specified angle in degrees
    void setRoll(double angle);
 
    // Returns the raw sensor values of the gyro as an array of doubles
    double[] getRaw();
}