package frc.robot.subsystems.IndicatorSubsystem;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IndicatorIOInputsAutoLogged extends IndicatorIO.IndicatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CurrentPattern", currentPattern);
  }

  @Override
  public void fromLog(LogTable table) {
    currentPattern = table.get("CurrentPattern", currentPattern);
  }

  public IndicatorIOInputsAutoLogged clone() {
    IndicatorIOInputsAutoLogged copy = new IndicatorIOInputsAutoLogged();
    copy.currentPattern = this.currentPattern;
    return copy;
  }
}
