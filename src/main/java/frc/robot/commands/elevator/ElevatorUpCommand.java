package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorUpCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorUpCommand(ElevatorSubsystem elevatorSubsystem){
        this.elevatorSubsystem = elevatorSubsystem;

    }

    @Override
    public void execute(){
        elevatorSubsystem.getIo().setElevatorDirectVoltage(1);
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.getIo().setElevatorDirectVoltage(0);
    }
}