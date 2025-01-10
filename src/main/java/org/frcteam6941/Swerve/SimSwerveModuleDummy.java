package org.frcteam6941.Swerve;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotConstants;

public class SimSwerveModuleDummy implements SwerveModuleBase {
    private int moduleNumber;
    private SwerveModuleState currentState;
    private SwerveModulePosition currentPosition;

    public SimSwerveModuleDummy(int id, LegacySwerveModuleConstants constants) {
        moduleNumber = id;
        currentState = new SwerveModuleState();
        currentPosition = new SwerveModulePosition();
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public void updateSignals() {
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        currentState = desiredState;
        //int flipCoefficient = Utils.flip() ? -1 : 1;
        currentPosition.distanceMeters += desiredState.speedMetersPerSecond * RobotConstants.LOOPER_DT;
        currentPosition.angle = desiredState.angle;
    }

    @Override
    public SwerveModuleState getState() {
        return currentState;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return currentPosition;
    }
}
