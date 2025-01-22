package frc.robot.subsystems.intaker;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakerIOInputsAutoLogged extends IntakerIO.IntakerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("IntakerConnected", intakerConnected);
    table.put("Voltage", voltage);
    table.put("IntakerSpeed", intakerSpeed);
    table.put("IntakerSupplyCurrent", intakerSupplyCurrent);
    table.put("BeamBreakState", beamBreakState);
  }

  @Override
  public void fromLog(LogTable table) {
    intakerConnected = table.get("IntakerConnected", intakerConnected);
    voltage = table.get("Voltage", voltage);
    intakerSpeed = table.get("IntakerSpeed", intakerSpeed);
    intakerSupplyCurrent = table.get("IntakerSupplyCurrent", intakerSupplyCurrent);
    beamBreakState = table.get("BeamBreakState", beamBreakState);
  }

  public IntakerIOInputsAutoLogged clone() {
    IntakerIOInputsAutoLogged copy = new IntakerIOInputsAutoLogged();
    copy.intakerConnected = this.intakerConnected;
    copy.voltage = this.voltage;
    copy.intakerSpeed = this.intakerSpeed;
    copy.intakerSupplyCurrent = this.intakerSupplyCurrent;
    copy.beamBreakState = this.beamBreakState;
    return copy;
  }
}
