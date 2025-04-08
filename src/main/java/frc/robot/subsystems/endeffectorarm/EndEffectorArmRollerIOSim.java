package frc.robot.subsystems.endeffectorarm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.roller.RollerIOSim;

public class EndEffectorArmRollerIOSim extends RollerIOSim implements EndEffectorArmRollerIO {
    // Default values for simulation
    private static final double ROLLER_RATIO = 1.0;

    public EndEffectorArmRollerIOSim() {
        super(
                1.0, // Moment of inertia (kg*m²)
                ROLLER_RATIO, // Gear ratio
                new SimpleMotorFeedforward(0.0, 0.24), // Feedforward constants (static friction, velocity)
                new ProfiledPIDController(
                        0.5, // kP
                        0.0, // kI
                        0.0, // kD
                        new TrapezoidProfile.Constraints(15, 1) // Max velocity, max acceleration
                )
        );
    }
} 