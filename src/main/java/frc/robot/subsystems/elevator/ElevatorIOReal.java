package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.ElevatorGainsClass;

import static frc.robot.RobotConstants.CANIVORE_CAN_BUS_NAME;
import static frc.robot.RobotConstants.ElevatorConstants.*;


public class ElevatorIOReal implements ElevatorIO {
    // Hardware
    private final TalonFX motor;

    // Configurators
    private final TalonFXConfigurator config;

    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs;

    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> stator;
    private final StatusSignal<Current> supply;
    private final StatusSignal<Temperature> temp;
    private double setpointMeters = 0;

    public ElevatorIOReal() {
        this.motor = new TalonFX(ELEVATOR_MOTOR_ID, CANIVORE_CAN_BUS_NAME);

        this.config = motor.getConfigurator();

        // Configs
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 80.0;
        currentLimitsConfigs.SupplyCurrentLimit = 30.0;

        motor.setPosition(heightToTalonPos(ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED));

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
        motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
        motionMagicConfigs.MotionMagicJerk = motionJerk.get();

        slot0Configs = new Slot0Configs();
        slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.get();
        slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.get();
        slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.get();
        slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.get();
        slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.get();
        slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.get();

        //Since elevator don't start at zero, not needed
        //resetElevatorPosition();

        config.apply(currentLimitsConfigs);
        config.apply(motorConfigs);
        config.apply(slot0Configs);
        config.apply(motionMagicConfigs);

        motor.clearStickyFaults();

        velocity = motor.getVelocity();
        position = motor.getPosition();
        voltage = motor.getSupplyVoltage();
        stator = motor.getStatorCurrent();
        supply = motor.getSupplyCurrent();
        temp = motor.getDeviceTemp();

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocity,
                position,
                voltage,
                stator,
                supply,
                temp
        );

        inputs.positionMeters = getElevatorHeight();
        inputs.setpointMeters = setpointMeters;
        inputs.velocityMetersPerSec = getElevatorVelocity();
        inputs.appliedVolts = voltage.getValueAsDouble();
        inputs.statorCurrentAmps = stator.getValueAsDouble();
        inputs.supplyCurrentAmps = supply.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();

        if (RobotConstants.TUNING) {
            slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.get();
            slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.get();
            slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.get();
            slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.get();
            slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.get();
            slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.get();
            slot0Configs.kG = ElevatorGainsClass.ELEVATOR_KG.get();

            motionMagicConfigs.MotionMagicAcceleration = motionAcceleration.get();
            motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocity.get();
            motionMagicConfigs.MotionMagicJerk = motionJerk.get();

            config.apply(slot0Configs);
            config.apply(motionMagicConfigs);
        }
    }

    @Override
    public void setElevatorVoltage(double volts) {
        motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElevatorTarget(double meters) {
        setpointMeters = meters;
        motor.setControl(motionRequest.withPosition(heightToTalonPos(meters)));
    }

    @Override
    public void resetElevatorPosition() {
        motor.setPosition(0.0);
    }

    @Override
    public double getElevatorVelocity() {
        return talonPosToHeight(motor.getVelocity().getValueAsDouble());
    }

    @Override
    public double getElevatorHeight() {
        return talonPosToHeight(motor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean isNearZeroExtension() {
        return MathUtil.isNear(heightToTalonPos(0.05), motor.getPosition().getValueAsDouble(), 0.3);
    }

    @Override
    public boolean isNearExtension(double expected) {
        return MathUtil.isNear(expected, talonPosToHeight(motor.getPosition().getValueAsDouble()), 0.02);
    }

    private double heightToTalonPos(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double talonPosToHeight(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }
}