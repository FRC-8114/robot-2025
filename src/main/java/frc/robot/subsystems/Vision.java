package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision {
    private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    private final CommandSwerveDrivetrain chassis;
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

    public final List<NamedPhotonPoseEstimator> poseEstimators;

    public Vision(CommandSwerveDrivetrain chassis) {
        this.chassis = chassis;

        NamedPhotonPoseEstimator front_cam = 
            new NamedPhotonPoseEstimator(
                FIELD_LAYOUT,
                POSE_STRATEGY,
                new PhotonCamera("front_cam"),
                new Transform3d(
                    new Translation3d(0.051, 0.195, 0.394),
                    new Rotation3d(0.0, 0.0, -0.174532925)),
                "front_cam"
            );

        NamedPhotonPoseEstimator back_cam = 
                new NamedPhotonPoseEstimator(
                    FIELD_LAYOUT,
                    POSE_STRATEGY,
                    new PhotonCamera("back_cam"),
                    new Transform3d(
                        new Translation3d(-0.0732, -0.2134, 0.621),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                    "back_cam"
                );

        poseEstimators = List.of(front_cam, back_cam);
        // poseEstimators = List.of(front_cam);
        // poseEstimators = List.of();
    }

    public void updateVision() {
        for (NamedPhotonPoseEstimator poseEstimator : poseEstimators) {
            for (PhotonPipelineResult pipelineResult : poseEstimator.getCamera().getAllUnreadResults()) {
                Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update(pipelineResult);

                if (estimatedPoseOptional.isPresent()) {
                    EstimatedRobotPose estimatedRobotPose = estimatedPoseOptional.get();
                    
                    Pose2d estPose = estimatedRobotPose.estimatedPose.toPose2d();

                    Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose, estimatedRobotPose.targetsUsed);

                    chassis.addVisionMeasurement(estPose, estimatedRobotPose.timestampSeconds, estStdDevs);
                }
            }
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, List<PhotonTrackedTarget> targetsUsed) {
        var estStdDevs = SINGLE_TAG_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targetsUsed) {
            var tagPose = FIELD_LAYOUT.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            
            avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) {
        return estStdDevs;
        }
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
        estStdDevs = MULTI_TAG_STD_DEVS;
        }
        if (avgDist > 4) {
            return estStdDevs.times(10);
        }
        // Increase std devs based on (average) distance
        return estStdDevs.times(1 + (avgDist * avgDist / 30));
    }
}

class NamedPhotonPoseEstimator extends PhotonPoseEstimator {
    private final String name;
    private final PhotonCamera camera;
  
    public NamedPhotonPoseEstimator(
        AprilTagFieldLayout layout,
        PoseStrategy strategy,
        PhotonCamera camera,
        Transform3d robotToCamera,
        String name) {
      super(layout, strategy, robotToCamera);
      this.name = name;
      this.camera = camera;
    }

    public PhotonCamera getCamera() {
        return camera;
    }
  
    public String getName() {
      return name;
    }
}
