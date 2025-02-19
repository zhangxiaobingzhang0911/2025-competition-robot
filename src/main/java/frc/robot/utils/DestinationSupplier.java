package frc.robot.utils;

import frc.robot.RobotConstants;

public class DestinationSupplier {
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
    public enum chassisSetpoint {
        LEFT, RIGHT
    }

    private boolean useCoral = false;
    private elevatorSetpoint currentElevSetpointCoral = elevatorSetpoint.L2;
    private elevatorSetpoint currentElevSetpointPoke = elevatorSetpoint.P1;

    private chassisSetpoint currentChassisSetpoint;

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

    public void updateChassisSetpoint(chassisSetpoint setpoint) {
        currentChassisSetpoint = setpoint;
    }

    public double getElevatorSetpoint(boolean useCoral) {
        this.useCoral = useCoral;
        if (useCoral) {
            switch (currentElevSetpointCoral) {
                case L1:
                    return RobotConstants.ElevatorConstants.L1_EXTENSION_METERS.get();
                case L2:
                    return RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get();
                case L3:
                    return RobotConstants.ElevatorConstants.L3_EXTENSION_METERS.get();
                case L4:
                    return RobotConstants.ElevatorConstants.L4_EXTENSION_METERS.get();
                default:
                    return RobotConstants.ElevatorConstants.L2_EXTENSION_METERS.get();
            }
        }else{
            switch (currentElevSetpointPoke) {
                case P1:
                    return RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
                case P2:
                    return RobotConstants.ElevatorConstants.P2_EXTENSION_METERS.get();
                default:
                    return RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
            }
        }
    }

}
