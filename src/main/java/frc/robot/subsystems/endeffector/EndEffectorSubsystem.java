package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;

import static frc.robot.RobotConstants.EndEffectorConstants.*;
import static frc.robot.RobotConstants.EndEffectorConstants.EndEffectorGainsClass.*;

public class EndEffectorSubsystem extends RollerSubsystem {

    public static final String NAME = "EndEffector";

    private final EndEffectorIO endEffectorIO;
    private final BeambreakIO firstBBIO, secondBBIO, thirdBBIO;
    private final BeambreakIOInputsAutoLogged firstBBInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged secondBBInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged thirdBBInputs = new BeambreakIOInputsAutoLogged();

    public double kp = ENDEFFECTOR_KP.get();
    public double ki = ENDEFFECTOR_KI.get();
    public double kd = ENDEFFECTOR_KD.get();
    public double ka = ENDEFFECTOR_KA.get();
    public double kv = ENDEFFECTOR_KV.get();
    public double ks = ENDEFFECTOR_KS.get();

    private double intakeRPS = INTAKE_RPS.get();
    private double indexRPS = INDEX_RPS.get();
    private double holdRPS = HOLD_RPS.get();
    private double ShootRPS = SHOOT_RPS.get();
    private double SpitRPS = SPIT_RPS.get();

    public EndEffectorSubsystem(EndEffectorIO endEffectorIO , BeambreakIO firstBBIO, BeambreakIO secondBBIO,
            BeambreakIO thirdBBIO) {
        super(endEffectorIO, NAME);
        this.endEffectorIO = endEffectorIO;
        this.firstBBIO = firstBBIO;
        this.secondBBIO = secondBBIO;
        this.thirdBBIO = thirdBBIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        endEffectorIO.updateConfigs(kp, ki, kd, ka, kv, ks);
        firstBBIO.updateInputs(firstBBInputs);
        secondBBIO.updateInputs(secondBBInputs);
        thirdBBIO.updateInputs(thirdBBInputs);

        Logger.processInputs(NAME + "/First Beambreak", firstBBInputs);
        Logger.processInputs(NAME + "/Second Beambreak", secondBBInputs);
        Logger.processInputs(NAME + "/Third Beambreak", thirdBBInputs);

        
        if (RobotConstants.TUNING) {
            intakeRPS = INTAKE_RPS.get();
            indexRPS = INDEX_RPS.get();
            holdRPS = HOLD_RPS.get();
            ShootRPS = SHOOT_RPS.get();
            SpitRPS = SPIT_RPS.get();

            kp = ENDEFFECTOR_KP.get();
            ki = ENDEFFECTOR_KI.get();
            kd = ENDEFFECTOR_KD.get();
            ka = ENDEFFECTOR_KA.get();
            kv = ENDEFFECTOR_KV.get();
            ks = ENDEFFECTOR_KS.get();
        }
    }

    public Command index() {
        return Commands.sequence(setVelocity(intakeRPS).until(() -> firstBBInputs.get),
                setVelocity(indexRPS).until(() -> !secondBBInputs.get && thirdBBInputs.get), setVelocity(holdRPS));
    }

    public Command hold() {
        return this.run(() -> setVelocity(holdRPS));
    }

    public Command shoot() {
        return this.run(() -> setVelocity(ShootRPS).until(() -> !thirdBBInputs.get));
    }

    public Command spit() {
        return this.run(() -> setVelocity(SpitRPS).until(() -> !thirdBBInputs.get));
    }

    public boolean getFirstBeambreak() {
        return firstBBInputs.get;
    }

    public boolean getSecondBeambreak() {
        return secondBBInputs.get;
    }

    public boolean getThirdBeambreak() {
        return thirdBBInputs.get;
    }
}
