package frc.robot.commands;
 
import static edu.wpi.first.units.Units.Volts;
 
import com.google.flatbuffers.Constants;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
 
// Represents a command to move the elevator to level 1
public class ElevatorL1 extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
 
    // Constructor for the ElevatorL1 command, which takes an ElevatorSubsystem as a parameter
    public ElevatorL1(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }
 
    // Called when the command is initially scheduled
    @Override
    public void initialize() {
    }
 
    // Called repeatedly when the command is scheduled
    @Override
    public void execute() {
        elevatorSubsystem.getIo().setElevatorVelocity(2000);

    }
 
    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.getIo().setElevatorDirectVoltage(Volts.of(0));
    }
 
}