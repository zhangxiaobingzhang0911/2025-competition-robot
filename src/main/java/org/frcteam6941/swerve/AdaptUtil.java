package org.frcteam6941.swerve;
 
import com.team254.lib.geometry.Twist2d;
 
import edu.wpi.first.math.kinematics.ChassisSpeeds;
 
// Utility class for adapting chassis speeds to Twist2d objects
public class AdaptUtil {
    // Converts ChassisSpeeds to Twist2d
    public static Twist2d toTwist2d(ChassisSpeeds speed) {
        return new Twist2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond, speed.omegaRadiansPerSecond);
    }
}