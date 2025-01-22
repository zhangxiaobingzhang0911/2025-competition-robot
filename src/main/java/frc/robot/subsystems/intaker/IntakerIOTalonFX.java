package frc.robot.subsystems.intaker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotConstants;
import frc.robot.drivers.BeamBreak;

import static edu.wpi.first.units.Units.*;

public class IntakerIOTalonFX implements IntakerIO {

    private double targetIntakerVelocity = 0;
    private final TalonFX intakerMotor = new TalonFX(
            RobotConstants.IntakerConstants.INTAKER_MOTOR_ID,
            RobotConstants.CAN_BUS_NAME);
    private final BeamBreak intakerBeamBreak =
            new BeamBreak(RobotConstants.BeamBreakConstants.INTAKER_BEAMBREAK_ID);

    public IntakerIOTalonFX() {
        intakerMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.2)
                .withKI(0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));

    }

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        inputs.intakerConnected = BaseStatusSignal.refreshAll(
                intakerMotor.getVelocity(),
                intakerMotor.getMotorVoltage(),
                intakerMotor.getSupplyCurrent()
        ).isOK();
        inputs.beamBreakState = intakerBeamBreak.get();
        inputs.intakerSpeed = RotationsPerSecond.of(intakerMotor.getVelocity().getValueAsDouble());
        inputs.voltage = Volts.of(intakerMotor.getMotorVoltage().getValueAsDouble());
        inputs.intakerSupplyCurrent = Amps.of(intakerMotor.getSupplyCurrent().getValueAsDouble());
    }

    @Override
    public void setVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        intakerMotor.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec)
        ));
        targetIntakerVelocity = velocityRadPerSec;
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        intakerMotor.setVoltage(voltage.magnitude());
    }
}
