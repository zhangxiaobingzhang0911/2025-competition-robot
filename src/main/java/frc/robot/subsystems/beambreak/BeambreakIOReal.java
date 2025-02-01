package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeambreakIOReal implements BeambreakIO {
  final DigitalInput beambreak;

  public BeambreakIOReal(int id) {
    beambreak = new DigitalInput(id);
  }

  public void updateInputs(BeambreakIOInputs inputs) {
    inputs.get = beambreak.get();
  }
}
