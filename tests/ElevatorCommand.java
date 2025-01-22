package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import static edu.wpi.first.units.Units.Volts;


public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final int level; // level = 0 to restore to starting position
    private double relativePosition;
    private double absolutePosition;
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
        this.relativePosition = new double[]{
                0,
                RobotConstants.ElevatorConstants.l1Position,
                RobotConstants.ElevatorConstants.l2Position,
                RobotConstants.ElevatorConstants.l3Position,
                RobotConstants.ElevatorConstants.l4Position
        }[this.level];
        this.absolutePosition = this.elevatorSubsystem.getStartingPosition() + this.relativePosition;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        this.elevatorSubsystem.getIo().setElevatorPosition(this.absolutePosition);
        this.isFinished = true;
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return (this.elevatorSubsystem.getIo().getElevatorPosition() >= RobotConstants.ElevatorConstants.highestPosition + this.elevatorSubsystem.getStartingPosition()) || this.isFinished;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.getIo().setElevatorDirectVoltage(Volts.of(0));
    }
}
