package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerSubsystem;

import static frc.robot.RobotConstants.EndEffectorConstants.*;

public class EndEffectorSubsystem extends RollerSubsystem {

    public static final String NAME = "EndEffector";

    private final BeambreakIO firstBBIO, secondBBIO, thirdBBIO;
    private final BeambreakIOInputsAutoLogged firstBBInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged secondBBInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged thirdBBInputs = new BeambreakIOInputsAutoLogged();

    private double intakeRPS = INTAKE_RPS.get();
    private double indexRPS = INDEX_RPS.get();
    private double holdRPS = HOLD_RPS.get();
    private double ShootRPS = SHOOT_RPS.get();
    private double SpitRPS = SPIT_RPS.get();

    public EndEffectorSubsystem(RollerIO rollerIO, BeambreakIO firstBBIO, BeambreakIO secondBBIO,
            BeambreakIO thirdBBIO) {
        super(rollerIO, NAME);
        this.firstBBIO = firstBBIO;
        this.secondBBIO = secondBBIO;
        this.thirdBBIO = thirdBBIO;
    }

    @Override
    public void periodic() {
        super.periodic();
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
