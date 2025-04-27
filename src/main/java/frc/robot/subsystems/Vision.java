package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;


public class Vision {

    private final PhotonCamera lowerLeftCamera;
    private final PhotonPoseEstimator lowerLeftPoseEstimator;
    private CommandSwerveDrivetrain swerveDrivetrain;
    Field2d field2DLowerLeft = new Field2d();
    Field2d field2DRobot = new Field2d();

    public Vision(CommandSwerveDrivetrain swerveDrivetrainIn) {
        lowerLeftCamera = new PhotonCamera(LOWER_LEFT_CAMERA_NAME);
        lowerLeftPoseEstimator = new PhotonPoseEstimator(TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAM);
        lowerLeftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        swerveDrivetrain = swerveDrivetrainIn;
        SmartDashboard.putData("photonPose Lower Left", field2DLowerLeft);
        SmartDashboard.putData("robotPose", field2DRobot);
    }

    public void updateVision() {
        // add new cameras to process here
        processPipelineForCamera(USE_VISION, lowerLeftCamera, lowerLeftPoseEstimator, field2DLowerLeft, VISION_MAX_DIST);
        field2DRobot.setRobotPose(swerveDrivetrain.getState().Pose);
    }

    public void processPipelineForCamera(boolean useCamera, PhotonCamera camera, PhotonPoseEstimator estimator,
            Field2d photonField, double maxDistance) {
        List<PhotonPipelineResult> cameraPipeline = camera.getAllUnreadResults();

        for (var pipelineResult : cameraPipeline) {
            Optional<EstimatedRobotPose> estPose = lowerLeftPoseEstimator.update(pipelineResult);

            if (estPose.isPresent()) {
                photonField.setRobotPose(estPose.get().estimatedPose.toPose2d());
                double bestTagDist = pipelineResult.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
                double poseAmbiguitiy = pipelineResult.getBestTarget().getPoseAmbiguity();

                if (useCamera && bestTagDist < maxDistance && poseAmbiguitiy < MAX_TAG_AMBIGUITY) {
                    swerveDrivetrain.addVisionMeasurement(
                        estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds);
                }
            };
        }
    }
}
