package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.*;

public class ElevatorCommand extends Command{
    private DoubleSupplier position;
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorCommand(DoubleSupplier position,ElevatorSubsystem elevatorSubsystem){
        this.position = position;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){
        elevatorSubsystem.getIo().setElevatorTarget(position.getAsDouble());
    }
}
