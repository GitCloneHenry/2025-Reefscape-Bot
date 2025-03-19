package frc.robot.subsystems;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final RobotContainer m_robotContainer;
    private final PhotonPoseEstimator m_photonPoseEstimator;

    public VisionSubsystem(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;

        m_photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToCamera);
    }

    public Optional<Pose2d> estimateRobotPose(List<PhotonPipelineResult> results) {
        if (!results.isEmpty()) {
            try {
                Optional<EstimatedRobotPose> estimatedRobotPose = m_photonPoseEstimator.update(results.get(0));

                if (estimatedRobotPose.isPresent()) {
                    return Optional.of(estimatedRobotPose.get().estimatedPose.toPose2d());
                }
            } catch (NoSuchElementException e) {
                System.err.println("Error: No valid AprilTag pose found.");
            }
        }

        return Optional.empty();
    }
}
