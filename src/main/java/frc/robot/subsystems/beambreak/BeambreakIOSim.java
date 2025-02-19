package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import frc.robot.drivers.BeamBreak;

public class BeambreakIOSim implements BeambreakIO {

    private AnalogInputSim breamBreakSim;
    public BeambreakIOSim(int id) {
        this.breamBreakSim = new AnalogInputSim(id);
    }
    @Override
    public void updateInputs(BeambreakIOInputs inputs) {
        inputs.isBeambreakOn =  breamBreakSim.getVoltage()>3;
        inputs.voltage = breamBreakSim.getVoltage();
    }
}

