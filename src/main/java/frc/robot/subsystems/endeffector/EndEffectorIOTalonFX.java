package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import frc.robot.drivers.BeamBreak;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    private final TalonFX endEffectorMotor = new TalonFX(RobotConstants.EndEffectorConstants.ENDEFFECTOR_MOTOR_ID, RobotConstants.CAN_BUS_NAME);
    private final BeamBreak higherEndEffectorBeamBreak = new BeamBreak(RobotConstants.BeamBreakConstants.ENDEFFECTOR_HIGHERBEAMBREAK_ID);
    private final BeamBreak lowerEndEffectorBeamBreak = new BeamBreak(RobotConstants.BeamBreakConstants.ENDEFFECTOR_LOWERBEAMBREAK_ID);

    private final StatusSignal<AngularVelocity> velocityRotPerSec = endEffectorMotor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = endEffectorMotor.getMotorVoltage();
    private final StatusSignal<Current> currentAmps = endEffectorMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> tempCelsius = endEffectorMotor.getDeviceTemp();

    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    public EndEffectorIOTalonFX() {
        var endEffectorMotorConfig = new TalonFXConfiguration();
        endEffectorMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        endEffectorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        endEffectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        endEffectorMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        endEffectorMotorConfig.Feedback.SensorToMechanismRatio = 1;
        endEffectorMotor.getConfigurator().apply(endEffectorMotorConfig);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocityRotPerSec, appliedVolts, currentAmps, tempCelsius);

        inputs.velocityRotationsPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();

        inputs.isHigherEndEffectorBeamBreakOn = higherEndEffectorBeamBreak.get();
        inputs.islowerEndEffectorBeamBreakOn = lowerEndEffectorBeamBreak.get();

        endEffectorMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.endEffectorKP)
                .withKI(inputs.endEffectorKI)
                .withKD(inputs.endEffectorKD)
                .withKA(inputs.endEffectorKA)
                .withKV(inputs.endEffectorKV)
                .withKS(inputs.endEffectorKS));
    }

    @Override
    public void setVoltage(double voltage) {
        endEffectorMotor.setControl(voltageOut.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setVelocity(double velocityRPS) {
        endEffectorMotor.setControl(velocityVoltage.withVelocity(velocityRPS));
    }
}
