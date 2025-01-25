package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import static frc.robot.RobotConstants.ShooterConstants.SHOOTER_MOTOR_ID;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX ShooterTalon = new TalonFX(SHOOTER_MOTOR_ID, RobotConstants.CAN_BUS_NAME);
    private final StatusSignal<AngularVelocity> ShooterVelocity = ShooterTalon.getVelocity();
    private final StatusSignal<Angle> ShooterPosition = ShooterTalon.getPosition();
    private final StatusSignal<Voltage> ShooterAppliedVoltage = ShooterTalon.getMotorVoltage();
    private final StatusSignal<Current> ShooterSupplyCurrent = ShooterTalon.getSupplyCurrent();
    private double targetShooterVelocity = 0;

    public ShooterIOTalonFX() {
        var shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterMotorConfig.Feedback.SensorToMechanismRatio = 1;
        ShooterTalon.getConfigurator().apply(shooterMotorConfig);
        var response = ShooterTalon.getConfigurator().apply(shooterMotorConfig);
        if (response.isError())
            System.out.println("Shooter TalonFX failed config with error" + response);
        response = ShooterTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Shooter TalonFX failed sticky fault clearing with error" + response);
    }

    public void runVolts(double volts) {
        ShooterTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                ShooterVelocity,
                ShooterPosition,
                ShooterAppliedVoltage,
                ShooterSupplyCurrent);

        inputs.ShooterVelocity = RadiansPerSecond
                .of(Units.rotationsToRadians(ShooterVelocity.getValueAsDouble()));
        inputs.ShooterPosition = Radians.of(Units.rotationsToRadians(ShooterPosition.getValueAsDouble()));
        inputs.ShooterAppliedVoltage = Volts.of(ShooterAppliedVoltage.getValueAsDouble());
        inputs.ShooterSupplyCurrent = Amps.of(ShooterSupplyCurrent.getValueAsDouble());

        inputs.targetShooterVelocity = RadiansPerSecond.of(targetShooterVelocity);

        ShooterTalon.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.ShooterKP)
                .withKI(inputs.ShooterKI)
                .withKD(inputs.ShooterKD)
                .withKA(inputs.ShooterKA)
                .withKV(inputs.ShooterKV)
                .withKS(inputs.ShooterKS));
    }


    @Override
    public void setFlyWheelDirectVoltage(Measure<VoltageUnit> volts) {
        ShooterTalon.setControl(new VoltageOut(volts.magnitude()));
    }


    @Override
    public void setFlyWheelVelocity(double velocityRPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        ShooterTalon.setControl(new VelocityVoltage(Units.radiansToRotations(velocityRadPerSec)));
        targetShooterVelocity = velocityRadPerSec;
    }

    @Override
    public double getVelocity() {
        return ShooterVelocity.getValueAsDouble() * 60;
    }
}