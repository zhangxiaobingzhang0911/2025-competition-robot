package frc.robot.display;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
    private static SuperstructureVisualizer instance;

    // Conversion helper
    private static double mmToM(double mm) {
        return mm / 1000.0;
    }

    // Elevator constants
    private static final Translation3d ELEVATOR_START = new Translation3d(
            mmToM(-102), mmToM(125), mmToM(90));
    private static final Translation3d ELEVATOR_END = new Translation3d(
            mmToM(132), mmToM(125), mmToM(90));
    private static final Translation3d ELEVATOR_CENTER = ELEVATOR_START.interpolate(ELEVATOR_END, 0.5);

    // Stage specifications
    private static final double STAGE1_RETRACT_LENGTH = mmToM(958 + 8 + 8);
    private static final double STAGE1_TRAVEL = mmToM(790);
    private static final double STAGE2_RETRACTED_OFFSET = mmToM(18);
    private static final double STAGE2_TRAVEL = mmToM(820);
    private static final double STAGE2_LENGTH = mmToM(352);

    // Intake constants
    private static final Translation3d INTAKE_PIVOT_START = new Translation3d(
            mmToM(-280), mmToM(-181), mmToM(250));
    private static final Translation3d INTAKE_PIVOT_END = new Translation3d(
            mmToM(-280), mmToM(181), mmToM(250));
    private static final Translation3d INTAKE_CENTER = INTAKE_PIVOT_START.interpolate(INTAKE_PIVOT_END, 0.5);
    private static final double INTAKE_LENGTH = mmToM(452);

    // EndEffectorArm constants
    private static final Translation3d END_EFFECTOR_PIVOT_START = new Translation3d(
            mmToM(-280), mmToM(181), mmToM(250));
    private static final Translation3d END_EFFECTOR_PIVOT_END = new Translation3d(
            mmToM(-280), mmToM(-181), mmToM(250));
    private static final Translation3d END_EFFECTOR_CENTER = END_EFFECTOR_PIVOT_START.interpolate(END_EFFECTOR_PIVOT_END, 0.5);
    private static final double END_EFFECTOR_LENGTH_CORAL = mmToM(450);
    private static final double END_EFFECTOR_LENGTH_ALGAE = mmToM(300);
    private static final double END_EFFECTOR_MOUNT_ARM_LENGTH = mmToM(200);

    // Visualization components
    private final LoggedMechanism2d elevatorMechanism;
    private final LoggedMechanismLigament2d elevatorStage1;
    private final LoggedMechanismLigament2d endEffectorMountArm;
    private final LoggedMechanismLigament2d endEffectorArmCoral;
    private final LoggedMechanismLigament2d endEffectorArmAlgae;

    private final LoggedMechanism2d intakeMechanism;
    private final LoggedMechanismLigament2d intakeArm;

    // Current state tracking
    private double currentElevatorHeight = 0.0;
    private double currentIntakeAngleDeg = 0.0;
    private double currentEndEffectorAngleDeg = 310;

    public static SuperstructureVisualizer getInstance() {
        if (instance == null) {
            instance = new SuperstructureVisualizer();
        }
        return instance;
    }

    public SuperstructureVisualizer() {
        // Elevator mechanism setup
        elevatorMechanism = new LoggedMechanism2d(
                0,
                0,
                new Color8Bit(Color.kWhite));

        LoggedMechanismRoot2d elevatorRoot = elevatorMechanism.getRoot(
                "ElevatorBase",
                -0.2,
                ELEVATOR_CENTER.getZ());

        elevatorStage1 = new LoggedMechanismLigament2d(
                "stage1",
                STAGE1_RETRACT_LENGTH,
                90,
                10,
                new Color8Bit(Color.kBlue));

        endEffectorMountArm = new LoggedMechanismLigament2d(
                "endEffectorMountArm",
                END_EFFECTOR_MOUNT_ARM_LENGTH,
                90,
                8,
                new Color8Bit(Color.kPurple));

        endEffectorArmCoral = new LoggedMechanismLigament2d(
                "endEffectorArmCoral",
                END_EFFECTOR_LENGTH_CORAL,
                270,
                8,
                new Color8Bit(Color.kGreen));

        endEffectorArmAlgae = new LoggedMechanismLigament2d(
                "endEffectorArmAlgae",
                END_EFFECTOR_LENGTH_ALGAE,
                90,
                8,
                new Color8Bit(Color.kYellow));

        elevatorRoot.append(elevatorStage1);
        elevatorStage1.append(endEffectorMountArm);
        endEffectorMountArm.append(endEffectorArmCoral);
        endEffectorMountArm.append(endEffectorArmAlgae);

        // Intake mechanism setup
        intakeMechanism = new LoggedMechanism2d(
                0,
                0,
                new Color8Bit(Color.kWhite));

        LoggedMechanismRoot2d intakeRoot = intakeMechanism.getRoot(
                "IntakePivot",
                0.25,
                INTAKE_CENTER.getZ());

        intakeArm = new LoggedMechanismLigament2d(
                "intakeArm",
                INTAKE_LENGTH,
                90,
                8,
                new Color8Bit(Color.kRed));

        intakeRoot.append(intakeArm);
    }

    // Full update
    public void update(double elevatorHeight, double intakeAngleRad, double endEffectorAngleRad) {
        this.currentElevatorHeight = elevatorHeight;
        this.currentIntakeAngleDeg = intakeAngleRad;
        this.currentEndEffectorAngleDeg = endEffectorAngleRad;
        updateVisuals();
    }

    // Elevator-only update
    public void updateElevator(double elevatorHeight) {
        this.currentElevatorHeight = elevatorHeight;
        updateVisuals();
    }

    // Intake-only update
    public void updateIntake(double intakeAngleDeg) {
        this.currentIntakeAngleDeg = intakeAngleDeg;
        updateVisuals();
    }

    // EndEffectorArm-only update
    public void updateEndEffector(double endEffectorAngleDeg) {
        this.currentEndEffectorAngleDeg = endEffectorAngleDeg;
        updateVisuals();
    }

    private void updateVisuals() {
        // Update elevator components
        elevatorStage1.setLength(currentElevatorHeight); // Stage 1 extends

        // Update intake components
        intakeArm.setAngle(Rotation2d.fromRadians(Math.toRadians(-currentIntakeAngleDeg+90)));

        // Update end effector components
        endEffectorMountArm.setAngle(Rotation2d.fromRadians(Math.toRadians(currentEndEffectorAngleDeg)));


        // Log 2D mechanisms
        Logger.recordOutput("Superstructure/Elevator/Mechanism2d", elevatorMechanism);
        Logger.recordOutput("Superstructure/Intake/Mechanism2d", intakeMechanism);
    }
}