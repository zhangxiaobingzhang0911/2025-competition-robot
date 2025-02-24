package frc.robot.drivers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
import org.frcteam6941.looper.Updatable;
import org.littletonrobotics.junction.Logger;

public class DestinationSupplier implements Updatable {
    private static DestinationSupplier instance;
    Swerve swerve;
    private int targetTagID = 0;
    private boolean coralRight = false;
    private boolean useCoral = false;
    private elevatorSetpoint currentElevSetpointCoral = elevatorSetpoint.L2;
    private elevatorSetpoint currentElevSetpointPoke = elevatorSetpoint.P1;

    private DestinationSupplier() {
        swerve = Swerve.getInstance();
    }

    public static DestinationSupplier getInstance() {
        if (instance == null) {
            instance = new DestinationSupplier();
        }
        return instance;
    }

    public void updateElevatorSetpoint(elevatorSetpoint setpoint) {
        switch (setpoint) {
            case L1, L2, L3, L4:
                currentElevSetpointCoral = setpoint;
                Logger.recordOutput("DestinationSupplier/currentElevSetpointCoral", setpoint);
                break;
            case P1, P2:
                currentElevSetpointPoke = setpoint;
                Logger.recordOutput("DestinationSupplier/currentElevSetpointPoke", setpoint);
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
        } else {
            return switch (currentElevSetpointPoke) {
                case P1 -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
                case P2 -> RobotConstants.ElevatorConstants.P2_EXTENSION_METERS.get();
                default -> RobotConstants.ElevatorConstants.P1_EXTENSION_METERS.get();
            };
        }
    }

    /**
     * @param coralRight: It always means the right reef RELATIVE TO TAG
     *                    (i.e when you are facing the tag, rightReef = true means the tag on your right is the target)
     */

    public void updateBranch(boolean coralRight) {
        this.coralRight = coralRight;
        Logger.recordOutput("DestinationSupplier/RightCoral", coralRight);
    }

    public void updateTagID(int tagID) {
        this.targetTagID = tagID;
    }

    public int getTargetTagID() {
        return targetTagID;
    }

    public boolean getCurrentBranch() {
        return coralRight;
    }

    public int getNearestTagID(Pose2d robotPose) {
        double minDistance = Double.MAX_VALUE;
        int minDistanceID = 0;
        // Loop from 6 to 11
        for (int i = 6; i <= 11; i++) {
            if (minDistance > FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation())) {
                minDistanceID = i;
                minDistance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                        toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            }
        }

        // Loop from 17 to 22
        for (int i = 17; i <= 22; i++) {
            if (minDistance > FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation())) {
                minDistanceID = i;
                minDistance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
                        toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            }
        }
        minDistance = Double.MAX_VALUE;
        return minDistanceID;
    }

    @Override
    public void update(double time, double dt) {
    }

    public enum elevatorSetpoint {
        L1, L2, L3, L4, P1, P2
    }


}

