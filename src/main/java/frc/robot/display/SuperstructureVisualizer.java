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

    // Visualization components
    private final LoggedMechanism2d elevatorMechanism;
    private final LoggedMechanismLigament2d elevatorStage1;


    private final LoggedMechanism2d intakeMechanism;
    private final LoggedMechanismLigament2d intakeArm;

    // Current state tracking
    private double currentElevatorHeight = 0.0;
    private double currentIntakeAngleDeg = 0.0;

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
                0,
                ELEVATOR_CENTER.getZ());

        elevatorStage1 = new LoggedMechanismLigament2d(
                "stage1",
                STAGE1_RETRACT_LENGTH,
                90,
                10,
                new Color8Bit(Color.kBlue));



        elevatorRoot.append(elevatorStage1);


        // Intake mechanism setup
        intakeMechanism = new LoggedMechanism2d(
                0,
                0,
                new Color8Bit(Color.kWhite));

        LoggedMechanismRoot2d intakeRoot = intakeMechanism.getRoot(
                "IntakePivot",
                -0.25,
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
    public void update(double elevatorHeight, double intakeAngleRad) {
        this.currentElevatorHeight = elevatorHeight;
        this.currentIntakeAngleDeg = intakeAngleRad;
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

    private void updateVisuals() {


        // Update elevator components

        elevatorStage1.setLength(currentElevatorHeight); // Stage 1 extends

        // Update intake components
        intakeArm.setAngle(Rotation2d.fromRadians(Math.toRadians(currentIntakeAngleDeg+90)));

        // Log 2D mechanisms
        Logger.recordOutput("Superstructure/Elevator/Mechanism2d", elevatorMechanism);
        Logger.recordOutput("Superstructure/Intake/Mechanism2d", intakeMechanism);

        // Log 3D poses
        logElevator3D();
        logIntake3D();
    }

    private void logElevator3D() {
        Pose3d basePose = new Pose3d(ELEVATOR_CENTER, new Rotation3d());
        Pose3d carriagePose = basePose.transformBy(new Transform3d(0,0,currentElevatorHeight,new Rotation3d()));

        Logger.recordOutput("Superstructure/Elevator/Mechanism3d/Base", basePose);
        Logger.recordOutput("Superstructure/Elevator/Mechanism3d/carriagePose", carriagePose);
    }

    private void logIntake3D() {
        Rotation3d rotation = new Rotation3d(0, currentIntakeAngleDeg, 0);
        Translation3d endPosition = INTAKE_CENTER.plus(
                new Translation3d(
                        -INTAKE_LENGTH * Math.sin(currentIntakeAngleDeg),
                        0,
                        INTAKE_LENGTH * Math.cos(currentIntakeAngleDeg)));

        Logger.recordOutput("Superstructure/Intake/Mechanism3d/ArmEnd",
                new Pose3d(endPosition, rotation));
    }
}