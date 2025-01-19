package org.frcteam6941.control;
 
import edu.wpi.first.math.geometry.Translation2d;
 
/**
 * Drive Signal for Holonomic Drivetrains.
 *
 * @param translation   Translation with x and y component, ranging from -1 to 1.
 * @param rotation      Rotational Magnitude, ranging from -1 to 1.
 * @param fieldOriented Whether the signal is relative to the field or not.
 */
public class HolonomicDriveSignal {
    private final Translation2d translation;
    private final double rotation;
    private final boolean fieldOriented;
    private final boolean isOpenLoop;
 
    // Constructor to initialize the HolonomicDriveSignal with translation, rotation, field orientation, and open loop status.
    public HolonomicDriveSignal(Translation2d translation, double rotation, boolean fieldOriented, boolean isOpenLoop) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
        this.isOpenLoop = isOpenLoop;
    }
 
    // Getter method to retrieve the translation component of the drive signal.
    public Translation2d getTranslation() {
        return translation;
    }
 
    // Getter method to retrieve the rotation component of the drive signal.
    public double getRotation() {
        return rotation;
    }
 
    // Getter method to check if the drive signal is field-oriented.
    public boolean isFieldOriented() {
        return fieldOriented;
    }
 
    // Getter method to check if the drive signal is in open loop mode.
    public boolean isOpenLoop() {
        return isOpenLoop;
    }
 
    // Method to validate the drive signal based on given translation and rotation thresholds.
    public boolean isValid(double translationThreshold, double rotationThreshold) {
        return translation.getNorm() > translationThreshold && Math.abs(rotation) > rotationThreshold;
    }
}