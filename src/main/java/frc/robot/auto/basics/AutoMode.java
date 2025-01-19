package frc.robot.auto.basics;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
 
// Interface defining the basic structure and requirements for an autonomous mode in the robot's code
public interface AutoMode {
    // Method to retrieve the name of the autonomous mode
    String getAutoName();
 
    // Method to determine if a warning should be issued for this autonomous mode
    boolean shouldWarn();
 
    // Default method to provide the starting position for the robot in the autonomous mode
    default Pose2d getStartingPose() {
        return new Pose2d();
    }
 
    // Default method to provide the command to be executed during the autonomous mode
    default Command getAutoCommand() {
        return new InstantCommand();
    }
}