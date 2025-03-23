package frc.robot.subsystems.endeffectorarm;

import edu.wpi.first.math.MathUtil;
import frc.robot.RobotConstants;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.EndEffectorArmConstants.*;

public class EndEffectorArmSubsystem extends RollerSubsystem {
    public static final String NAME = "EndEffectorArm/Roller";
    
    // Static variables to hold the current values from TunableNumbers
    private static double homeAngle = HOME_ANGLE.get();
    private static double intakeAngle = INTAKE_ANGLE.get();
    private static double outtakeAngle = OUTTAKE_ANGLE.get();
    private static double holdingAngle = HOLDING_ANGLE.get();
    private static double shootingAngle = SHOOTING_ANGLE.get();
    private static double algaeIntakeAngle = ALGAE_INTAKE_ANGLE.get();
    private static double algaeShootingAngle = ALGAE_SHOOTING_ANGLE.get();
    
    private static double intakeVoltage = INTAKE_VOLTAGE.get();
    private static double outtakeVoltage = OUTTAKE_VOLTAGE.get();
    private static double holdVoltage = HOLD_VOLTAGE.get();
    private static double shootVoltage = SHOOT_VOLTAGE.get();
    private static double algaeIntakeVoltage = ALGAE_INTAKE_VOLTAGE.get();
    private static double algaeShootVoltage = ALGAE_SHOOT_VOLTAGE.get();
    
    // IO devices and their inputs
    private final EndEffectorArmPivotIO armPivotIO;
    private final EndEffectorArmRollerIO armRollerIO;
    private final EndEffectorArmPivotIOInputsAutoLogged armPivotIOInputs = new EndEffectorArmPivotIOInputsAutoLogged();
    private final RollerIOInputsAutoLogged armRollerIOInputs = new RollerIOInputsAutoLogged();
    
    // Optional beambreak sensor (can be null)
    private final BeambreakIO beambreakIO;
    private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();
    
    // State tracking
    private WantedState wantedState = WantedState.HOME;
    @Getter
    private SystemState systemState = SystemState.HOMING;

    /**
     * Creates a new EndEffectorArmSubsystem
     * 
     * @param armPivotIO The IO interface for the arm pivot
     * @param armRollerIO The IO interface for the roller
     * @param beambreakIO Optional beambreak sensor (can be null)
     */
    public EndEffectorArmSubsystem(
            EndEffectorArmPivotIO armPivotIO,
            EndEffectorArmRollerIO armRollerIO,
            BeambreakIO beambreakIO) {
        super(armRollerIO, NAME);
        this.armPivotIO = armPivotIO;
        this.armRollerIO = armRollerIO;
        this.beambreakIO = beambreakIO;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        // Update inputs from hardware
        armPivotIO.updateInputs(armPivotIOInputs);
        if (beambreakIO != null) {
            beambreakIO.updateInputs(beambreakInputs);
        }
        
        // Calculate current state transition
        SystemState newState = handleStateTransition();
        
        // Process and log inputs - note: This would normally use auto-generated classes
        // In a real scenario, AdvantageKit will generate EndEffectorArmPivotIOInputsAutoLogged
        // For now we'll log the raw values
        Logger.processInputs(NAME + "/Pivot", armPivotIOInputs);
        
        if (beambreakIO != null) {
            Logger.processInputs(NAME + "/Beambreak", beambreakInputs);
        }
        
        // Log the system state
        Logger.recordOutput("EndEffectorArm/SystemState", systemState.toString());
        
        // Apply the new state if it has changed
        if (newState != systemState) {
            systemState = newState;
        }
        
        // Handle actions based on current system state
        switch (systemState) {
            case INTAKING:
                armRollerIO.setVoltage(intakeVoltage);
                armPivotIO.setPivotAngle(intakeAngle);
                break;
                
            case OUTTAKING:
                armRollerIO.setVoltage(outtakeVoltage);
                armPivotIO.setPivotAngle(outtakeAngle);
                break;
                
            case HOLDING:
                armRollerIO.setVoltage(holdVoltage);
                armPivotIO.setPivotAngle(holdingAngle);
                break;
                
            case SHOOTING:
                armRollerIO.setVoltage(shootVoltage);
                armPivotIO.setPivotAngle(shootingAngle);
                break;
                
            case ALGAE_INTAKING:
                armRollerIO.setVoltage(algaeIntakeVoltage);
                armPivotIO.setPivotAngle(algaeIntakeAngle);
                break;
                
            case ALGAE_SHOOTING:
                armRollerIO.setVoltage(algaeShootVoltage);
                armPivotIO.setPivotAngle(algaeShootingAngle);
                break;
                
            case HOMING:
                armRollerIO.stop();
                armPivotIO.setPivotAngle(homeAngle);
                break;
        }
        
        // Update tunable numbers if tuning is enabled
        if (RobotConstants.TUNING) {
            homeAngle = HOME_ANGLE.get();
            intakeAngle = INTAKE_ANGLE.get();
            outtakeAngle = OUTTAKE_ANGLE.get();
            holdingAngle = HOLDING_ANGLE.get();
            shootingAngle = SHOOTING_ANGLE.get();
            algaeIntakeAngle = ALGAE_INTAKE_ANGLE.get();
            algaeShootingAngle = ALGAE_SHOOTING_ANGLE.get();
            
            intakeVoltage = INTAKE_VOLTAGE.get();
            outtakeVoltage = OUTTAKE_VOLTAGE.get();
            holdVoltage = HOLD_VOLTAGE.get();
            shootVoltage = SHOOT_VOLTAGE.get();
            algaeIntakeVoltage = ALGAE_INTAKE_VOLTAGE.get();
            algaeShootVoltage = ALGAE_SHOOT_VOLTAGE.get();
        }
    }
    
    /**
     * Handles the transition between states based on the current wanted state
     * @return The new system state
     */
    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case INTAKE -> SystemState.INTAKING;
            case OUTTAKE -> SystemState.OUTTAKING;
            case HOLD -> SystemState.HOLDING;
            case SHOOT -> SystemState.SHOOTING;
            case ALGAE_INTAKE -> SystemState.ALGAE_INTAKING;
            case ALGAE_SHOOT -> SystemState.ALGAE_SHOOTING;
            case HOME -> SystemState.HOMING;
        };
    }
    
    /**
     * Sets the wanted state of the end effector arm
     * @param wantedState The new state to set
     */
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }
    
    /**
     * Checks if the arm is near a target angle
     * @param targetAngleDeg The target angle in degrees
     * @return True if the arm is within 1 degree of the target
     */
    public boolean isNearAngle(double targetAngleDeg) {
        return MathUtil.isNear(targetAngleDeg, armPivotIOInputs.currentAngleDeg, 1);
    }
    
    /**
     * The wanted state for the EndEffectorArm subsystem
     */
    public enum WantedState {
        INTAKE,
        OUTTAKE,
        HOLD,
        SHOOT,
        ALGAE_INTAKE,
        ALGAE_SHOOT,
        HOME
    }
    
    /**
     * The current system state of the EndEffectorArm subsystem
     */
    public enum SystemState {
        INTAKING,
        OUTTAKING,
        HOLDING,
        SHOOTING,
        ALGAE_INTAKING,
        ALGAE_SHOOTING,
        HOMING
    }
} 