package frc.robot.subsystems.Intaker;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.drivers.BeamBreak;

public class Intaker extends SubsystemBase {
    private final TalonFX intakeMotor =
            new TalonFX(
                    RobotConstants.IntakerConstants.INTAKER_MOTOR_ID,
                    RobotConstants.CAN_BUS_NAME);
    private final BeamBreak intakerBeamBreak =
            new BeamBreak(RobotConstants.BeamBreakConstants.INTAKER_BEAMBREAK_ID);
    private boolean lastRecordedState;
    private boolean noteState = false;

    public Intaker() {
        boolean isIntakerBeamBreakOn = intakerBeamBreak.get();
        if (!lastRecordedState && isIntakerBeamBreakOn) {
            noteState = true;
        }
        lastRecordedState = isIntakerBeamBreakOn;

        intakeMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.2)
                .withKI(0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));
    }

    public void setIntakeRPM(double velocityRPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        intakeMotor.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec)
        ));
    }

    public void periodic() {

    }

    public boolean intakerHasNote() {
        return intakerBeamBreak.get();
    }

    public void setIntakeDirectVoltage(Measure<VoltageUnit> volts) {
        intakeMotor.setControl(new VoltageOut(volts.magnitude()));
    }

    public enum State {

    }

}
