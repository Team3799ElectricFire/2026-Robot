package frc.robot;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.LerpTable;

public class VisionConstants {
    public record CameraConfig(String name, double trustScalar, Transform3d transform, double width, double height) {
    }

    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1,0.1,0.1);
    public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(1,1,1);

    public static final CameraConfig[] CONFIGS = {
        new CameraConfig(
            "FrontRightCam",
            1.0,
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-2.43),
                    Units.inchesToMeters(-6.96),
                    Units.inchesToMeters(19.57)
                ),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-26.0),
                    Units.degreesToRadians(-45.0)
                )
            ),
            1280, 
            800
        ),
        new CameraConfig(
            "FrontLeftCam",
            1.0,
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-2.43),
                    Units.inchesToMeters(6.96),
                    Units.inchesToMeters(19.57)
                ),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-26.0),
                    Units.degreesToRadians(45.0)
                )
            ),
            1280, 
            800
        ),
        new CameraConfig(
            "BackRightCam",
            1.0,
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-11.21), 
                    Units.inchesToMeters(-11.03),
                    Units.inchesToMeters(8.41)
                ),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-57.0),
                    Units.degreesToRadians(-155)
                )
            ),
            1280, 
            800
        ),
        new CameraConfig(
            "BackLeftCam",
            1.0,
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-11.21), 
                    Units.inchesToMeters(11.03),
                    Units.inchesToMeters(8.41)
                ),
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-57.0),
                    Units.degreesToRadians(155)
                )
            ),
            1280, 
            800
        )
    };

    public static final class Filtering {
        public static final LerpTable HEIGHT_WIDTH_PROPORTION_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.25, 0.0),
                new LerpTable.LerpTableEntry(0.7, 0.9),
                new LerpTable.LerpTableEntry(1.0, 1.0));

        public static final LerpTable AREA_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 0.0),
                new LerpTable.LerpTableEntry(0.2, 0.35),
                new LerpTable.LerpTableEntry(1.0, 0.45),
                new LerpTable.LerpTableEntry(4.0, 0.70),
                new LerpTable.LerpTableEntry(7.5, 1.0));

        public static final LerpTable PIXEL_OFFSET_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 1.0),
                new LerpTable.LerpTableEntry(0.2, 1.0),
                new LerpTable.LerpTableEntry(0.65, 0.75),
                new LerpTable.LerpTableEntry(1.0, 0.35));

        public static final LerpTable LINEAR_VELOCITY_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 1.0),
                new LerpTable.LerpTableEntry(2.5, 0.8),
                new LerpTable.LerpTableEntry(5.0, 0.1));

        public static final LerpTable ANGULAR_VELOCITY_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 1.0),
                new LerpTable.LerpTableEntry(7.0, 0.65),
                new LerpTable.LerpTableEntry(12.0, 0.0));
    

        public static final HashMap<Integer, Double> TAG_RANKINGS = new HashMap<>() {
            {
                put(1, 0.0); // red trench 
                put(2, 1.0); // red hub 
                put(3, 1.0); // red hub
                put(4, 1.0); // red hub
                put(5, 1.0); // red hub
                put(6, 0.0); // red trench
                put(7, 0.0); // red trench
                put(8, 1.0); // red hub
                put(9, 1.0); // red hub
                put(10, 1.0); // red hub
                put(11, 1.0); // red hub
                put(12, 1.0); // red trench
                put(13, 1.0); // red outpost
                put(14, 1.0); // red outpost
                put(15, 1.0); // red tower wall
                put(16, 1.0); // red tower wall
                put(17, 0.0); // blue trench
                put(18, 1.0); // blue hub
                put(19, 1.0); // blue hub
                put(20, 1.0); // blue hub
                put(21, 1.0); // blue hub
                put(22, 0.0); // blue trench
                put(23, 0.0); // blue trench
                put(24, 1.0); // blue hub
                put(25, 1.0); // blue hub
                put(26, 1.0); // blue hub
                put(27, 1.0); // blue hub
                put(28, 0.0); // blue trench
                put(29, 1.0); // blue outpost
                put(30, 1.0); // blue outpost
                put(31, 1.0); // blue tower wall
                put(32, 1.0); // blue tower wall
            }
        };
    }
    
    public static final AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.2);

}
