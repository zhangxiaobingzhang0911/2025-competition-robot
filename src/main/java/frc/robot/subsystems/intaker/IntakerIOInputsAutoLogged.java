package frc.robot.subsystems.intaker;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakerIOInputsAutoLogged extends IntakerIO.IntakerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IntakeVelocity", intakeVelocity);
    table.put("IntakePosition", intakePosition);
    table.put("IntakeAppliedVoltage", intakeAppliedVoltage);
    table.put("IntakeSupplyCurrent", intakeSupplyCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    intakeVelocity = table.get("IntakeVelocity", intakeVelocity);
    intakePosition = table.get("IntakePosition", intakePosition);
    intakeAppliedVoltage = table.get("IntakeAppliedVoltage", intakeAppliedVoltage);
    intakeSupplyCurrent = table.get("IntakeSupplyCurrent", intakeSupplyCurrent);
  }

  public IntakerIOInputsAutoLogged clone() {
    IntakerIOInputsAutoLogged copy = new IntakerIOInputsAutoLogged();
    copy.intakeVelocity = this.intakeVelocity;
    copy.intakePosition = this.intakePosition;
    copy.intakeAppliedVoltage = this.intakeAppliedVoltage;
    copy.intakeSupplyCurrent = this.intakeSupplyCurrent;
    return copy;
  }
}
