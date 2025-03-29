package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses have a blue alliance
 * origin.
 */
public class FieldConstants {
    public static final double offsetLength = 21;
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX = Units.inchesToMeters(299.438); // Measured from the inside of starting line

    public static final class HPS {
        public static final Pose2d left_side = new Pose2d(new Translation2d(1.1446170806884766, 0.9114338755607605), Rotation2d.fromRadians(-2.203650142759433));
        public static final Pose2d right_side = new Pose2d(new Translation2d(1.112796425819397, 7.0924458503723145), Rotation2d.fromRadians(2.2091159647641847));
    
        public static final List<Pose2d> both_hps = List.of(left_side, right_side);
    }

    public static class Reef {
        public static List<Pose2d> lefts = new ArrayList<>();
        public static List<Pose2d> rights = new ArrayList<>();

         // Side of the reef to the inside of the reef zone line
        public static final Translation2d center = new Translation2d(
            Units.inchesToMeters(176.746),
            Units.inchesToMeters(158.501)
        );

        static {
            for (int face = 0; face < 6; face++) {
                Pose2d centerWithAngle = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                double adjustX = Units.inchesToMeters(30.738+offsetLength); // set pose back for aligning
                double adjustY = Units.inchesToMeters(6.469);

                Pose2d left = new Pose2d(
                    new Translation2d(
                        centerWithAngle
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getX(),
                        centerWithAngle
                             .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getY()
                    ),
                    new Rotation2d(centerWithAngle.getRotation().getRadians()).minus(Rotation2d.fromDegrees(180))
                );

                Pose2d right = new Pose2d(
                    new Translation2d(
                        centerWithAngle
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getX(),
                        centerWithAngle
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getY()
                    ),
                    new Rotation2d(centerWithAngle.getRotation().getRadians()).minus(Rotation2d.fromDegrees(180))
                );

                lefts.add(left);
                rights.add(right);
            }
        }
    }

    public static class Lollipops {
        public static Pose2d leftLollipop = new Pose2d(
            new Translation2d(48.25, 84),
            new Rotation2d()
        );
    }
}
