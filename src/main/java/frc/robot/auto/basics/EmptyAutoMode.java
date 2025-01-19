package frc.robot.auto.basics;
 
import edu.wpi.first.wpilibj2.command.Command;
 
// Represents an empty autonomous mode that does not perform any specific actions
public class EmptyAutoMode implements AutoMode{
    // Returns null as this mode does not have a specific name
    @Override
    public String getAutoName() {
        return null;
    }
 
    // Indicates that a warning should be issued when this mode is selected
    @Override
    public boolean shouldWarn() {
        return true;
    }
}