package frc.robot.subsystems.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeambreakIO {
  @AutoLog
  class BeambreakIOInputs {
    public boolean isBeambreakOn = false;
  }

  public void updateInputs(BeambreakIOInputs inputs);
}
