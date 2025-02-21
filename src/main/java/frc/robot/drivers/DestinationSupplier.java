package frc.robot.drivers;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import org.frcteam6941.looper.Updatable;

public class DestinationSupplier implements Updatable {
    private static DestinationSupplier instance;
    public static DestinationSupplier getInstance() {
        if (instance == null) {
            instance = new DestinationSupplier();
        }
        return instance;
    }


    public enum elevatorSetpoint {
        L1, L2, L3, L4, P1, P2
    }

    private int targetTagID = 0;
    private boolean coralRight = false;
    private boolean useCoral = false;

    private elevatorSetpoint currentElevSetpointCoral = elevatorSetpoint.L2;
    private elevatorSetpoint currentElevSetpointPoke = elevatorSetpoint.P1;


    private DestinationSupplier() {
    }


    public void updateElevatorSetpoint(elevatorSetpoint setpoint) {
        switch (setpoint) {
            case L1, L2, L3, L4:
                currentElevSetpointCoral = setpoint;
                break;
            case P1, P2:
                currentElevSetpointPoke = setpoint;
                break;
            default:
                System.out.println("Unknown elevator setpoint: " + setpoint);
        }
    }

    public double getElevatorSetpoint(boolean useCoral) {
        this.useCoral = useCoral;
        if (useCoral) {
            return switch (currentElevSetpointCoral) {
                case L1 -> RobotConstants.ElevatorConstants.L1_EXTENSION_METERS.get();
                case L2 -> RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get();
                case L3 -> RobotConstants.ElevatorConstants.L3_EXTENSION_METERS.get();
                case L4 -> RobotConstants.ElevatorConstants.L4_EXTENSION_METERS.get();
                default -> RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get();
            };
        }else{
            return switch (currentElevSetpointPoke) {
                case P1 -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
                case P2 -> RobotConstants.ElevatorConstants.P2_EXTENSION_METERS.get();
                default -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
            };
        }
    }

    public void updateBranch(boolean coralRight) {
        this.coralRight = coralRight;
    }
    public void updateTagID(int tagID) {
        this.targetTagID = tagID;
    }
    public int getTargetTagID(){
        return targetTagID;
    }

    public boolean getCurrentBranch() {
        return coralRight;
    }


}

