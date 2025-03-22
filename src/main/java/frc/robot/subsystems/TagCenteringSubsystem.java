package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class TagCenteringSubsystem extends SubsystemBase {
    public Command CenterOnTagCommand(Translation2d tagOffset, VisionSubsystem visionSubsystem) {
        List<PhotonPipelineResult> results = visionSubsystem.getLastVisionResults();

        PhotonTrackedTarget bestTarget = null;
        double minDistance = Double.MAX_VALUE;

        for (PhotonPipelineResult result : results) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();

                if (distanceToTag < minDistance) {
                    minDistance = distanceToTag;
                    bestTarget = target;
                }
            }
        }

        if (bestTarget == null) {
            return Commands.print("No targets seen.");
        }

        int desiredTagID = bestTarget.fiducialId;

        Optional<Pose3d> potentialTagPose = FieldConstants.aprilTagFieldLayout.getTagPose(desiredTagID);

        if (potentialTagPose.isEmpty()) {
            return Commands.print("No valid tag position found for tag ID " + desiredTagID);
        }

        Pose2d desiredTagPose = potentialTagPose.get().toPose2d();

        Rotation2d desiredRobotRotation = Rotation2d.fromRadians(desiredTagPose.getRotation().getRadians() + Math.PI);

        Pose2d desiredRobotPose = new Pose2d(
            desiredTagPose.getTranslation().plus(tagOffset.rotateBy(desiredTagPose.getRotation())),
            desiredRobotRotation
        );

        return AutoBuilder.pathfindToPose(desiredRobotPose, VisionConstants.pathConstraints);
    }
}
