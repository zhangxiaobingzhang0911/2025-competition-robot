package frc.robot.subsystems.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class RollerIOReal implements RollerIO {
    private final TalonFX motor;
    private final String CANBUS_NAME = RobotConstants.CAN_BUS_NAME;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    private final StatusSignal<AngularVelocity> velocityRotPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Current> supplyCurrentAmps;
    private final StatusSignal<Temperature> tempCelsius;

    public RollerIOReal(final int motorID, final TalonFXConfiguration config) {
        this.motor = new TalonFX(motorID, CANBUS_NAME);

        motor.getConfigurator().apply(config);

        velocityRotPerSec = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        statorCurrentAmps = motor.getStatorCurrent();
        supplyCurrentAmps = motor.getSupplyCurrent();
        tempCelsius = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocityRotPerSec, appliedVolts, statorCurrentAmps,
                supplyCurrentAmps, tempCelsius);

        motor.optimizeBusUtilization();
    }

    public static TalonFXConfiguration getDefaultConfig() {
        return new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs().withStatorCurrentLimit(20.0).withStatorCurrentLimitEnable(true))
                .withSlot0(new Slot0Configs().withKV(0.113).withKP(0.2))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocityRotPerSec, appliedVolts, statorCurrentAmps, supplyCurrentAmps, tempCelsius);
        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setVelocity(double velocityRPS) {
        motor.setControl(velocityVoltage.withVelocity(velocityRPS));
    }
}
