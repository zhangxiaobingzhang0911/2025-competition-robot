package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID;
import static frc.robot.RobotConstants.ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leftElevatorTalon = new TalonFX(LEFT_ELEVATOR_MOTOR_ID, RobotConstants.CAN_BUS_NAME);
    private final TalonFX rightElevatorTalon = new TalonFX(RIGHT_ELEVATOR_MOTOR_ID,
            RobotConstants.CAN_BUS_NAME);
    private final StatusSignal<AngularVelocity> leftElevatorVelocity = leftElevatorTalon.getVelocity();
    private final StatusSignal<Angle> leftElevatorPosition = leftElevatorTalon.getPosition();
    private final StatusSignal<Voltage> leftElevatorAppliedVoltage = leftElevatorTalon.getMotorVoltage();
    private final StatusSignal<Current> leftElevatorSupplyCurrent = leftElevatorTalon.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> rightElevatorVelocity = rightElevatorTalon.getVelocity();
    private final StatusSignal<Angle> rightElevatorPosition = rightElevatorTalon.getPosition();
    private final StatusSignal<Voltage> rightElevatorAppliedVoltage = rightElevatorTalon.getMotorVoltage();
    private final StatusSignal<Current> rightElevatorSupplyCurrent = rightElevatorTalon.getSupplyCurrent();
    private double targetElevatorVelocity = 0;

    public ElevatorIOTalonFX() {
        TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevatorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        elevatorMotorConfig.Feedback.SensorToMechanismRatio = 1;
        elevatorMotorConfig.MotorOutput.Inverted = RobotConstants.ElevatorConstants.leftMotorClockwise ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        MotionMagicConfigs mmConfigs = elevatorMotorConfig.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = RobotConstants.ElevatorConstants.elevatorMotorRPS;
        mmConfigs.MotionMagicAcceleration = RobotConstants.ElevatorConstants.elevatorMotorAccel;
        leftElevatorTalon.getConfigurator().apply(elevatorMotorConfig);
        StatusCode response = leftElevatorTalon.getConfigurator().apply(elevatorMotorConfig);
        if (response.isError())
            System.out.println("Left Elevator TalonFX failed config with error" + response);
        response = leftElevatorTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Left Elevator TalonFX failed sticky fault clearing with error" + response);
        rightElevatorTalon.setControl(new Follower(leftElevatorTalon.getDeviceID(), true));
//        response = rightElevatorTalon.getConfigurator().apply(ElevatorMotorConfig);
//        if (response.isError())
//            System.out.println("Right Elevator TalonFX failed config with error" + response);
//        response = rightElevatorTalon.clearStickyFaults();
//        if (response.isError())
//            System.out.println("Right Elevator TalonFX failed sticky fault clearing with error" + response);
//        rightElevatorTalon.setControl(new Follower(leftElevatorTalon.getDeviceID(),
//                true));
    }

    public void runVolts(double volts) {
        leftElevatorTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftElevatorVelocity,
                leftElevatorPosition,
                leftElevatorAppliedVoltage,
                leftElevatorSupplyCurrent,
                rightElevatorVelocity,
                rightElevatorPosition,
                rightElevatorAppliedVoltage,
                rightElevatorSupplyCurrent);

        inputs.leftElevatorVelocity = RadiansPerSecond
                .of(Units.rotationsToRadians(leftElevatorVelocity.getValueAsDouble()));
        inputs.leftElevatorPosition = Radians.of(Units.rotationsToRadians(leftElevatorPosition.getValueAsDouble()));
        inputs.leftElevatorAppliedVoltage = Volts.of(leftElevatorAppliedVoltage.getValueAsDouble());
        inputs.leftElevatorSupplyCurrent = Amps.of(leftElevatorSupplyCurrent.getValueAsDouble());

        inputs.rightElevatorVelocity = RadiansPerSecond
                .of(Units.rotationsToRadians(rightElevatorVelocity.getValueAsDouble()));
        inputs.rightElevatorPosition = Radians.of(Units.rotationsToRadians(rightElevatorPosition.getValueAsDouble()));
        inputs.rightElevatorAppliedVoltage = Volts.of(rightElevatorAppliedVoltage.getValueAsDouble());
        inputs.rightElevatorSupplyCurrent = Amps.of(rightElevatorSupplyCurrent.getValueAsDouble());

        inputs.targetElevatorVelocity = RadiansPerSecond.of(targetElevatorVelocity);

        leftElevatorTalon.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.ElevatorKP)
                .withKI(inputs.ElevatorKI)
                .withKD(inputs.ElevatorKD)
                .withKA(inputs.ElevatorKA)
                .withKV(inputs.ElevatorKV)
                .withKS(inputs.ElevatorKS));
    }

    @Override
    public void setElevatorDirectVoltage(Measure<VoltageUnit> volts) {
        leftElevatorTalon.setControl(new VoltageOut(volts.magnitude()));
    }

    @Override
    public void setElevatorVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        leftElevatorTalon.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec)
        ));
        targetElevatorVelocity = velocityRadPerSec;
    }

    @Override
    public void setElevatorVelocity(double velocityRPM, double ffVoltage) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        leftElevatorTalon.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec)
        ));
        targetElevatorVelocity = velocityRadPerSec;
    }

    public void setElevatorPosition(double position) {
        MotionMagicVoltage request = new MotionMagicVoltage(leftElevatorTalon.getPosition().getValueAsDouble());
        leftElevatorTalon.setControl(request.withPosition(position));
    }

    @Override
    public double getVelocity() {
        return rightElevatorVelocity.getValueAsDouble() * 60;
    }

}
