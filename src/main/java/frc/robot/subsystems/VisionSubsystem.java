package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonPoseEstimator m_photonPoseEstimator;

    private final PhotonCamera m_aprilTagCamera = new PhotonCamera("USB_ATag_Camera");

    public VisionSubsystem() {
        m_photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToCamera);
    }

    public List<PhotonPipelineResult> getVisionResults() {
        return m_aprilTagCamera.getAllUnreadResults();
    }

    public Optional<Pose2d> estimateRobotPose(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimatedRobotPose = m_photonPoseEstimator.update(result);

        if (estimatedRobotPose.isPresent()) {
            return Optional.of(estimatedRobotPose.get().estimatedPose.toPose2d());
        }

        return Optional.empty();
    }
}
