package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import lombok.Getter;

import java.io.IOException;
import java.nio.file.Path;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double wingOpponentX = fieldLength - wingX;
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);
    public static final Translation2d ampCenter = new Translation2d(
            Units.inchesToMeters(72.455), fieldWidth);
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;


    @Getter
    public enum AprilTagLayoutType {
        OFFICIAL("2025-official");
//        OFFICIAL2024("2024-official"),
//        SPEAKERS_ONLY("2024-speakers"),
//        AMPS_ONLY("2024-amps"),
//        WPI("2024-wpi");

        private final AprilTagFieldLayout layout;
        private final String layoutString;

        AprilTagLayoutType(String name) {
            if (RobotConstants.disableHAL) {
                layout = null;
            } else {
                try {
                    layout =
                            new AprilTagFieldLayout(
                                    Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
            if (layout == null) {
                layoutString = "";
            } else {
                try {
                    layoutString = new ObjectMapper().writeValueAsString(layout);
                } catch (JsonProcessingException e) {
                    throw new RuntimeException(
                            "Failed to serialize AprilTag layout JSON " + "for Northstar");
                }
            }
        }
    }
}