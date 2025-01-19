package org.frcteam6941.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Represents the setpoint for a swerve drive system, including chassis speeds and individual module states.
public class SwerveSetpoint {
    public ChassisSpeeds mChassisSpeeds;
    public SwerveModuleState[] mModuleStates;

    // Constructor to initialize the SwerveSetpoint with given chassis speeds and module states.
    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }

    // Overrides the toString method to provide a string representation of the SwerveSetpoint, including chassis speeds and module states.
    @Override
    public String toString() {
        String ret = mChassisSpeeds.toString() + "\n";
        for (int i = 0; i < mModuleStates.length; ++i) {
            ret += "  " + mModuleStates[i].toString() + "\n";
        }
        return ret;
    }
}