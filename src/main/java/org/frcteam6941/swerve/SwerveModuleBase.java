package org.frcteam6941.swerve;
 
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
 
// Interface for a swerve module that defines the basic operations and properties
public interface SwerveModuleBase {
    // Returns the module number which uniquely identifies the swerve module
    int getModuleNumber();
 
    // Updates the control signals for the swerve module based on the current state and desired state
    void updateSignals();
 
    // Sets the desired state for the swerve module, with options for open loop control and motion override
    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion);
 
    // Retrieves the current state of the swerve module, including speed and angle
    SwerveModuleState getState();
 
    // Retrieves the current position of the swerve module
    SwerveModulePosition getPosition();
}