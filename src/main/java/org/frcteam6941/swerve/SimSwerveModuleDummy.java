package org.frcteam6941.swerve;
 
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
 
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
 
import frc.robot.RobotConstants;
 
// A dummy implementation of the SimSwerveModuleBase interface, used for simulation purposes.
public class SimSwerveModuleDummy implements SwerveModuleBase {
    private final int moduleNumber;
    private SwerveModuleState currentState;
    private final SwerveModulePosition currentPosition;
 
    // Constructor to initialize the module with an ID and constants.
    public SimSwerveModuleDummy(int id, LegacySwerveModuleConstants constants) {
        moduleNumber = id;
        currentState = new SwerveModuleState();
        currentPosition = new SwerveModulePosition();
    }
 
    // Returns the module number.
    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }
 
    // Updates the signals for the module. In this dummy implementation, this method does nothing.
    @Override
    public void updateSignals() {
    }
 
    // Sets the desired state for the module, updating the current state and position accordingly.
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        currentState = desiredState;
        currentPosition.distanceMeters += desiredState.speedMetersPerSecond * RobotConstants.LOOPER_DT;
        currentPosition.angle = desiredState.angle;
    }
 
    // Returns the current state of the module.
    @Override
    public SwerveModuleState getState() {
        return currentState;
    }
 
    // Returns the current position of the module.
    @Override
    public SwerveModulePosition getPosition() {
        return currentPosition;
    }
}