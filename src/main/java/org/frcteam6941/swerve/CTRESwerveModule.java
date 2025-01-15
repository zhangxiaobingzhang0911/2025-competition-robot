package org.frcteam6941.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotConstants;

public class CTRESwerveModule implements SwerveModuleBase {
    private final int moduleNumber;
    private final CTRESwerveIO module;

    // edu.wpi.first.math.geometry.Rotation2d angleLowSpeed = new edu.wpi.first.math.geometry.Rotation2d(0, 0);
    // boolean lowSpeed = false;
    public CTRESwerveModule(int id, LegacySwerveModuleConstants constants, String canbusName) {
        moduleNumber = id;
        module = new CTRESwerveIO(constants, canbusName);
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public SwerveModuleState getState() {
        return module.getCurrentState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return module.getInternalState();
    }

    @Override
    public void updateSignals() {
        // module.getPosition(true);
        SwerveModulePosition pos = module.getPosition(true);
    }

    // int cnt = 0;
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        module.getSteerMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(RobotConstants.SwerveConstants.steerGainsClass.STEER_KP.get())
                .withKI(RobotConstants.SwerveConstants.steerGainsClass.STEER_KI.get())
                .withKD(RobotConstants.SwerveConstants.steerGainsClass.STEER_KD.get())
                .withKA(RobotConstants.SwerveConstants.steerGainsClass.STEER_KA.get())
                .withKV(RobotConstants.SwerveConstants.steerGainsClass.STEER_KV.get())
                .withKS(RobotConstants.SwerveConstants.steerGainsClass.STEER_KS.get()));
        module.getDriveMotor().getConfigurator().apply(new Slot0Configs()
                .withKP(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KP.get())
                .withKI(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KI.get())
                .withKD(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KD.get())
                .withKA(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KA.get())
                .withKV(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KV.get())
                .withKS(RobotConstants.SwerveConstants.driveGainsClass.DRIVE_KS.get()));
        // if (Math.abs(desiredState.speedMetersPerSecond) <= 0.15 * Constants.SwerveConstants.maxSpeed.magnitude()) {
        // 	if (!lowSpeed) {
        // 		angleLowSpeed = desiredState.angle;
        // 		lowSpeed = true;
        // 	}
        // 	desiredState.angle = angleLowSpeed;
        // }
        // else if (lowSpeed) {
        // 	lowSpeed = false;
        // }

        module.apply(desiredState, isOpenLoop ? DriveRequestType.OpenLoopVoltage : DriveRequestType.Velocity);
        // System.out.println(moduleNumber + " = " + desiredState.speedMetersPerSecond + " = "
        // 		+ module.getDriveMotor().getMotorVoltage() + " " + module.getSteerMotor().getMotorVoltage());//speed output
		/*
		SmartDashboard.putNumber("Speed m/s", desiredState.speedMetersPerSecond);
		SmartDashboard.putString("Drive Motor Voltage", module.getDriveMotor().getMotorVoltage());
		SmartDashboard.putNumber("Steer Motor Voltage", module.getSteerMotor().getMotorVoltage());
		*/
        // cnt++;
        // if (cnt % 50 == 0) {
        // 	System.out.println(desiredState.speedMetersPerSecond);
        // }
    }
}
