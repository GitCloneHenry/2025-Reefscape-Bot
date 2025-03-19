package frc.robot.subsystems;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TagCenteringSubsystem extends SubsystemBase {
  private final DriveSubsystem driveSubsystem;
  private final PhotonCamera aprilTagCamera;

  private Field2d field;
  private FollowPathCommand followPathCommand;

  public TagCenteringSubsystem(RobotContainer robotContainer) {
    this.driveSubsystem = robotContainer.m_robotDrive;
    this.aprilTagCamera = robotContainer.m_aprilTagCamera;

    this.field = new Field2d();
    SmartDashboard.putData("Field", this.field);
  }

  public void runFollowPathCommand(Translation2d translation) {
    driveSubsystem.getDefaultCommand().cancel();
    followPathCommand(translation).schedule();
  }

  public Command followPathCommand(Translation2d translation) {
    List<PhotonPipelineResult> results = this.aprilTagCamera.getAllUnreadResults();
    PhotonTrackedTarget bestTarget = null;
    double minDistance = Double.MAX_VALUE;

    for (PhotonPipelineResult result : results) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        // if (target.getPoseAmbiguity() > 0.2) continue;

        double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();

        if (distanceToTag < minDistance) {
          bestTarget = target;
          minDistance = distanceToTag;
        }
      }
    }

    if (bestTarget == null) {
      return Commands.print("Didn't see an April Tag!");
    }

    Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
    Transform3d robotToTag = VisionConstants.robotToCamera.plus(cameraToTag);
    Translation2d tagOffset = robotToTag.getTranslation().toTranslation2d();
    Translation2d targetPosition =
        driveSubsystem
            .getPose()
            .getTranslation()
            .plus(tagOffset.rotateBy(driveSubsystem.getPose().getRotation()));

    Rotation2d targetRotation =
        Rotation2d.fromRadians(Math.atan2(tagOffset.getY(), tagOffset.getX()))
            .plus(driveSubsystem.getPose().getRotation());

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
              this.driveSubsystem.getPose().getX(),
              this.driveSubsystem.getPose().getY(),
              targetRotation
            ),
            new Pose2d(
                targetPosition
                    .minus(new Translation2d(0.683, targetRotation))
                    .minus(translation.rotateBy(targetRotation)),
                targetRotation));

    PathConstraints pathConstraints =
        new PathConstraints(
            1.500,
            3.00,
            240.0 * Math.PI / 180.0,
            4 * Math.PI);

    IdealStartingState idealStartingState = new IdealStartingState(0.0, this.driveSubsystem.getPose().getRotation());
    GoalEndState goalEndState = new GoalEndState(0.0, targetRotation);

    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, idealStartingState, goalEndState);

    PPHolonomicDriveController holonomicDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(3.7, 1.6, 0.6), 
            new PIDConstants(7.3, 2.8, 0.6)
        );

    RobotConfig robotConfig;
    try {
        robotConfig = RobotConfig.fromGUISettings();

        followPathCommand =
            new FollowPathCommand(
                path,
                this.driveSubsystem::getPose,
                this.driveSubsystem::getRobotRelativeSpeeds,
                this.driveSubsystem.driveRobotRelative,
                holonomicDriveController,
                robotConfig,
                () -> false,
                this.driveSubsystem);

        return followPathCommand;
    } catch (IOException | ParseException e) {
        e.printStackTrace();
    }
    
    return Commands.print("Didn't see an April Tag!");
  }
}
