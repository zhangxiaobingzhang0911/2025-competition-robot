package frc.robot.commands;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import java.lang.annotation.Target;

import static edu.wpi.first.units.Units.Volts;


public class ElevatorCommand extends Command {
    private int levels;
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem,int levels){
        this.elevatorSubsystem = elevatorSubsystem;
        this.levels = levels;
    }

    @Override
    public void execute() {
        elevatorSubsystem.getIo().setElevatorTarget(RobotConstants.ElevatorConstants.Position[levels]);
    }

    /*@Override
    public boolean isFinished() {
       //undo: finish if the elevator is out of the maximum range
    }*/

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.getIo().brake();
    }
}
