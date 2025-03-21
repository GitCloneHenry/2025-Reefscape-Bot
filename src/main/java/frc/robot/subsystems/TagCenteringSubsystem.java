// package frc.robot.subsystems;

// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.IdealStartingState;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.RobotContainer;

// import java.io.IOException;
// import java.util.List;
// import java.util.Optional;

// import org.json.simple.parser.ParseException;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// public class TagCenteringSubsystem extends SubsystemBase {
//   private final RobotContainer robotContainer;
//   private final DriveSubsystem driveSubsystem;
//   private final VisionSubsystem visionSubsystem;
//   private final PhotonCamera aprilTagCamera;

//   public TagCenteringSubsystem(RobotContainer robotContainer) {
//     this.robotContainer = robotContainer;
//     this.driveSubsystem = robotContainer.m_robotDrive;
//     this.visionSubsystem = robotContainer.m_visionSubsystem;
//     this.aprilTagCamera = this.visionSubsystem.m_aprilTagCamera;
//   }

//   public Command FollowPathCommand(Translation2d translation) {
//     List<PhotonPipelineResult> results = this.aprilTagCamera.getAllUnreadResults();
//     PhotonTrackedTarget bestTarget = null;
//     double minDistance = Double.MAX_VALUE;

//     Optional<Pose2d> estimatedResult = visionSubsystem.estimateRobotPose(results);

//     if (estimatedResult.isPresent()) {
//       this.driveSubsystem.resetPose(estimatedResult.get()); 
//     }

    // for (PhotonPipelineResult result : results) {
    //   for (PhotonTrackedTarget target : result.getTargets()) {
    //     double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();

    //     if (distanceToTag < minDistance) {
    //       bestTarget = target;
    //       minDistance = distanceToTag;
    //     }
    //   }
    // }

//     if (bestTarget == null) {
//       return Commands.print("Didn't see an April Tag!");
//     }

    
//     Pose3d aprilTagPosition = FieldConstants.aprilTagFieldLayout.getTagPose(bestTarget.fiducialId).get();

//     Translation2d targetPosition = aprilTagPosition.toPose2d().getTranslation();
//     Rotation2d targetRotation = Rotation2d.fromRadians(aprilTagPosition.toPose2d().getRotation().getRadians() + Math.PI);

//     List<Waypoint> waypoints =
//         PathPlannerPath.waypointsFromPoses(
//             new Pose2d(
//               this.driveSubsystem.getPose().getX(),
//               this.driveSubsystem.getPose().getY(),
//               this.driveSubsystem.getPose().getRotation()
//             ),
//             new Pose2d(
//                 targetPosition
//                     .minus(new Translation2d(0.683 + Units.inchesToMeters(6.0), targetRotation))
//                     .minus(translation.rotateBy(targetRotation)),
//                 targetRotation));
          
    

//     IdealStartingState idealStartingState = new IdealStartingState(0.0, this.driveSubsystem.getPose().getRotation());
//     GoalEndState goalEndState = new GoalEndState(0.0, targetRotation);

//     PathPlannerPath path = new PathPlannerPath(waypoints, VisionConstants.pathConstraints, idealStartingState, goalEndState);

//     PPHolonomicDriveController holonomicDriveController =
//         new PPHolonomicDriveController(
//             new PIDConstants(3.7, 1.6, 0.6), 
//             new PIDConstants(14.3, 5.2, 1.2)
//             // new PIDConstants(3.7, 1.6, 0.6)
//         );

//     RobotConfig robotConfig;
//     try {
//         robotConfig = RobotConfig.fromGUISettings();

//         robotContainer.m_field2d.setRobotPose(
//           new Pose2d(
//             targetPosition
//                 .minus(new Translation2d(0.683 + Units.inchesToMeters(6.0), targetRotation))
//                 .minus(translation.rotateBy(targetRotation)),
//             targetRotation)
//         );

//         // return new SequentialCommandGroup(new FollowPathCommand(
//         //   path,
//         //   this.driveSubsystem::getPose,
//         //   this.driveSubsystem::getRobotRelativeSpeeds,
//         //   this.driveSubsystem.driveRobotRelative,
//         //   holonomicDriveController,
//         //   robotConfig,
//         //   () -> false,
//         //   this.driveSubsystem),
//         //   Commands.print("Expected: " + targetRotation.getDegrees()));
//     } catch (IOException | ParseException e) {
//         e.printStackTrace();
//     }
    
//     return Commands.print("Didn't see an April Tag!");
//   }
// }
