package frc.robot.commands;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.lang.annotation.Target;

import static edu.wpi.first.units.Units.Volts;


public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private int level; // level = 0 to restore to starting position
    private boolean isFinished;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, int level) {
        this.elevatorSubsystem = elevatorSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.elevatorSubsystem);
        this.level = level;
        this.isFinished = false;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_TARGET);
        elevatorSubsystem.getIo().setElevatorPosition(RobotConstants.ElevatorConstants.Position[level]);
    }

    @Override
    public boolean isFinished() {
        return (elevatorSubsystem.getIo().getElevatorPosition() >= RobotConstants.ElevatorConstants.highestPosition + elevatorSubsystem.getStartingPosition()) || isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.getIo().setElevatorDirectVoltage(Volts.of(0));
    }
}
