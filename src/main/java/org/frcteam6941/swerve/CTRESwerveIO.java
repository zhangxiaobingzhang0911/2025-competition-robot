package org.frcteam6941.swerve;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.RobotConstants;

import lombok.Getter;

public class CTRESwerveIO {
    @Getter
    private final CoreTalonFX driveMotor, steerMotor;
    private final CANcoder cancoder;

    private final StatusSignal<Angle> sigDrivePosition;
    private final StatusSignal<AngularVelocity> sigDriveVelocity;
    private final StatusSignal<Angle> sigSteerPosition;
    private final StatusSignal<AngularVelocity> sigSteerVelocity;

    private final double driveRotationsPerMeter;
    private final double couplingRatioDriveRotorToCANCoder;
    private final double speedAt12VoltsMps;
    private final LegacySwerveModule.ClosedLoopOutputType steerClosedLoopOutput;
    private final LegacySwerveModule.ClosedLoopOutputType driveClosedLoopOutput;

    @Getter
    private final SwerveModulePosition internalState = new SwerveModulePosition();
    private final MotionMagicVoltage angleVoltageSetter = new MotionMagicVoltage(0.0);
    private final MotionMagicTorqueCurrentFOC angleTorqueSetter = new MotionMagicTorqueCurrentFOC(0.0);
    private final MotionMagicExpoVoltage angleVoltageExpoSetter = new MotionMagicExpoVoltage(0.0);
    private final MotionMagicExpoTorqueCurrentFOC angleTorqueExpoSetter = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final VoltageOut voltageOpenLoopSetter = new VoltageOut(0.0);
    private final VelocityVoltage velocityVoltageSetter = new VelocityVoltage(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueSetter = (new VelocityTorqueCurrentFOC(0.0)).withOverrideCoastDurNeutral(true);
    @Getter
    private SwerveModuleState targetState = new SwerveModuleState();


    public CTRESwerveIO(LegacySwerveModuleConstants constants, String canbusName) {
        // motors
        driveMotor = new CoreTalonFX(constants.DriveMotorId, canbusName);
        steerMotor = new CoreTalonFX(constants.SteerMotorId, canbusName);
        cancoder = new CANcoder(constants.CANcoderId, canbusName);
        initMotorConfigs(constants);

        // signals
        sigDrivePosition = driveMotor.getPosition().clone();
        sigDriveVelocity = driveMotor.getVelocity().clone();
        sigSteerPosition = steerMotor.getPosition().clone();
        sigSteerVelocity = steerMotor.getVelocity().clone();

        // constants
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 6.283185307179586 * Units.inchesToMeters(constants.WheelRadius);
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        couplingRatioDriveRotorToCANCoder = constants.CouplingGearRatio;
        speedAt12VoltsMps = constants.SpeedAt12VoltsMps;
        steerClosedLoopOutput = constants.SteerMotorClosedLoopOutput;
        driveClosedLoopOutput = constants.DriveMotorClosedLoopOutput;

        // setters
        angleVoltageSetter.UpdateFreqHz = 0.0;
        angleTorqueSetter.UpdateFreqHz = 0.0;
        angleVoltageExpoSetter.UpdateFreqHz = 0.0;
        angleTorqueExpoSetter.UpdateFreqHz = 0.0;
        velocityTorqueSetter.UpdateFreqHz = 0.0;
        velocityVoltageSetter.UpdateFreqHz = 0.0;
        voltageOpenLoopSetter.UpdateFreqHz = 0.0;
    }

    public SwerveModulePosition getPosition(boolean refreshSignals) {
        if (refreshSignals) {
            sigDrivePosition.refresh();
            sigDriveVelocity.refresh();
            sigSteerPosition.refresh();
            sigSteerVelocity.refresh();
        }

        double drive_rot = BaseStatusSignal.getLatencyCompensatedValueAsDouble(sigDrivePosition, sigDriveVelocity);
        double angle_rot = BaseStatusSignal.getLatencyCompensatedValueAsDouble(sigSteerPosition, sigSteerVelocity);
        drive_rot -= angle_rot * couplingRatioDriveRotorToCANCoder;
        internalState.distanceMeters = drive_rot / driveRotationsPerMeter;
        internalState.angle = Rotation2d.fromRotations(angle_rot);
        return internalState;
    }


    public void apply(SwerveModuleState state, LegacySwerveModule.DriveRequestType driveRequestType) {
        apply(state, driveRequestType, LegacySwerveModule.SteerRequestType.MotionMagic);
    }

    public void apply(SwerveModuleState state, LegacySwerveModule.DriveRequestType driveRequestType, LegacySwerveModule.SteerRequestType steerRequestType) {
        double angleToSetDeg;
        SwerveModuleState optimized;
        optimized = SwerveModuleState.optimize(state, internalState.angle);
        targetState = optimized;
        angleToSetDeg = optimized.angle.getRotations();

        // README: why use setters instead of instantiating a new one every call?
        // well first the original implementation used it.
        // second this method gets called quite a lot; if instantiation is used,
        // it may overwhelm the GC and crash the entire robot!

        // steer
        requestSwitch:
        switch (steerRequestType) {
            case MotionMagic:
                switch (steerClosedLoopOutput) {
                    case Voltage:
                        steerMotor.setControl(angleVoltageSetter.withPosition(angleToSetDeg));
                        break requestSwitch;
                    case TorqueCurrentFOC:
                        steerMotor.setControl(angleTorqueSetter.withPosition(angleToSetDeg));
                    default:
                        break requestSwitch;
                }
            case MotionMagicExpo:
                switch (steerClosedLoopOutput) {
                    case Voltage:
                        steerMotor.setControl(angleVoltageExpoSetter.withPosition(angleToSetDeg));
                        break;
                    case TorqueCurrentFOC:
                        steerMotor.setControl(angleTorqueExpoSetter.withPosition(angleToSetDeg));
                }
        }

        // drive
        double velocityToSet = optimized.speedMetersPerSecond * driveRotationsPerMeter;
        double steerMotorError = angleToSetDeg - sigSteerPosition.getValueAsDouble();
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }

        velocityToSet *= cosineScalar;
        double azimuthTurnRps = sigSteerVelocity.getValueAsDouble();
        double driveRateBackOut = azimuthTurnRps * couplingRatioDriveRotorToCANCoder;
        velocityToSet += driveRateBackOut;
        switch (driveRequestType) {
            case OpenLoopVoltage:
                velocityToSet /= driveRotationsPerMeter;
                driveMotor.setControl(voltageOpenLoopSetter.withOutput(velocityToSet / speedAt12VoltsMps * 12.0));
                break;
            case Velocity:
                switch (driveClosedLoopOutput) {
                    case Voltage:
                        driveMotor.setControl(velocityVoltageSetter.withVelocity(velocityToSet));
                        break;
                    case TorqueCurrentFOC:
                        driveMotor.setControl(velocityTorqueSetter.withVelocity(velocityToSet));
                }
        }

    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                sigDriveVelocity.getValueAsDouble() / driveRotationsPerMeter,
                Rotation2d.fromRotations(sigSteerPosition.getValueAsDouble())
        );
    }

    private void initMotorConfigs(LegacySwerveModuleConstants constants) {
        // copied from official implementation.
        // do not modify unless you really sure what you're doing!
        // drive
        TalonFXConfiguration driveConfigs = constants.DriveMotorInitialConfigs;
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfigs.MotorOutput.Inverted = constants.DriveMotorInverted ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;
        driveConfigs.Slot0 = constants.DriveMotorGains;

        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfigs.CurrentLimits.StatorCurrentLimit = RobotConstants.SwerveConstants.statorCurrent;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = RobotConstants.SwerveConstants.supplyCurrent;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        driveConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = RobotConstants.SwerveConstants.VOLTAGE_CLOSED_LOOP_RAMP_PERIOD;

        StatusCode response = driveMotor.getConfigurator().apply(driveConfigs);
        if (!response.isOK()) {
            System.out.println("TalonFX ID " + driveMotor.getDeviceID() + " failed config with error " + response);
        }

        // steer
        TalonFXConfiguration steerConfigs = constants.SteerMotorInitialConfigs;
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfigs.MotorOutput.Inverted = constants.SteerMotorInverted ?
                InvertedValue.Clockwise_Positive :
                InvertedValue.CounterClockwise_Positive;
        steerConfigs.Slot0 = constants.SteerMotorGains;

        steerConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        switch (constants.FeedbackSource) {
            case RemoteCANcoder:
                steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        }
        steerConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        steerConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        steerConfigs.MotionMagic.MotionMagicAcceleration = steerConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.1;
        steerConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        steerConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;
        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        response = steerMotor.getConfigurator().apply(steerConfigs);
        if (!response.isOK()) {
            System.out.println("TalonFX ID " + driveMotor.getDeviceID() + " failed config with error " + response);
        }

        // cancoder
        CANcoderConfiguration cancoderConfigs = constants.CANcoderInitialConfigs;
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;

        response = cancoder.getConfigurator().apply(cancoderConfigs);
        if (!response.isOK()) {
            System.out.println("CANcoder ID " + driveMotor.getDeviceID() + " failed config with error " + response);
        }
    }
}
