package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TagCenteringCommand extends Command {
  // private final RobotContainer robotContainer;
  private final DriveSubsystem driveSubsystem;
  private final PhotonCamera aprilTagCamera;
  private final RunCommand defaultDriveCommand;

  private Field2d field;
  private FollowPathCommand followPathCommand;
  // private FollowPathCommand followPathCommand2;

  private Translation2d translation;

  public TagCenteringCommand(RobotContainer robotContainer, Translation2d translation) {
    // this.robotContainer = robotContainer;
    this.driveSubsystem = robotContainer.m_robotDrive;
    this.aprilTagCamera = robotContainer.m_aprilTagCamera;
    this.defaultDriveCommand = robotContainer.m_defaultDriveCommand;

    this.translation = translation;

    this.field = new Field2d();
    SmartDashboard.putData("Field", this.field);

    this.driveSubsystem.removeDefaultCommand();

    addRequirements(this.driveSubsystem);
  }

  @Override
  public void initialize() {
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
      end(true);
      return;
    }

    Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
    Transform3d robotToTag = VisionConstants.robotToCamera.plus(cameraToTag);
    Translation2d tagOffset = robotToTag.getTranslation().toTranslation2d();
    Translation2d targetPosition =
        driveSubsystem
            .getPose()
            .getTranslation()
            .plus(tagOffset.rotateBy(driveSubsystem.getPose().getRotation()));

    // Rotation2d targetRotation = Rotation2d.fromRadians(
    // Math.atan2(tagOffset.getY(), tagOffset.getX())
    // ).plus(driveSubsystem.getPose().getRotation());
    Rotation2d targetRotation =
        Rotation2d.fromDegrees(cameraToTag.getRotation().getZ())
            .plus(driveSubsystem.getPose().getRotation());

    Translation2d robotPosition = driveSubsystem.getPose().getTranslation();
    // double aprilTagAngle = targetRotation.getRadians();

    // double sinAng = Math.sin(targetRotation.getRadians() + 3 * Math.PI / 2.0);
    // double cosAng = Math.cos(targetRotation.getRadians() + 3 * Math.PI / 2.0);

    // double x = sinAng*cosAng*(robotPosition.getY() -
    // targetPosition.getY())+robotPosition.getX()*cosAng*cosAng+targetPosition.getX()*sinAng*sinAng;
    // double y = sinAng*(x - targetPosition.getX()) / cosAng + targetPosition.getY();

    // double dist = Math.sqrt(Math.pow(x - robotPosition.getX(), 2.0) + Math.pow(y -
    // robotPosition.getY(), 2.0));

    // double intermediateX = targetPosition.getX() - dist * Math.cos(aprilTagAngle);
    // double intermediateY = targetPosition.getY() - dist * Math.sin(aprilTagAngle);

    // double intermediateX = targetPosition.getX() - targetRotation.getCos();
    // double intermediateY = targetPosition.getY() - targetRotation.getSin();

    // Pose2d intermediatePose = new Pose2d(intermediateX, intermediateY, targetRotation);

    SmartDashboard.putNumber("April X", targetPosition.getX());
    SmartDashboard.putNumber("April Y", targetPosition.getY());
    SmartDashboard.putNumber("April T", targetRotation.getRadians());

    SmartDashboard.putNumber("Robot X", robotPosition.getX());
    SmartDashboard.putNumber("Robot Y", robotPosition.getY());
    SmartDashboard.putNumber("Robot T", this.driveSubsystem.getPose().getRotation().getRadians());

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            this.driveSubsystem.getPose(),
            new Pose2d(
                targetPosition
                    .minus(new Translation2d(0.4, targetRotation))
                    .minus(translation.rotateBy(driveSubsystem.getPose().getRotation())),
                targetRotation));

    PathConstraints pathConstraints =
        new PathConstraints(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    GoalEndState goalEndState = new GoalEndState(0.0, targetRotation);

    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, goalEndState);
    // PathPlannerPath path2 = new PathPlannerPath(waypoints2, pathConstraints, null, goalEndState);

    PPHolonomicDriveController holonomicDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.1, 0.0), new PIDConstants(10.0, 0.5, 0.2));

    double robotMass = Units.lbsToKilograms(105);
    double trackWidth = DriveConstants.kTrackWidth;
    double wheelBase = DriveConstants.kWheelBase;

    double MOI = (robotMass * (Math.pow(trackWidth, 2.0) + Math.pow(wheelBase, 2.0))) / 12.0;

    double wheelCOF = 1.3;

    ModuleConfig moduleConfig =
        new ModuleConfig(
            ModuleConstants.kWheelDiameterMeters / 2,
            DriveConstants.kMaxSpeedMetersPerSecond,
            wheelCOF,
            DCMotor.getNEO(1),
            60.0,
            4);

    RobotConfig robotConfig =
        new RobotConfig(
            robotMass,
            MOI,
            moduleConfig,
            DriveConstants.wheelTranslations.get(0),
            DriveConstants.wheelTranslations.get(1),
            DriveConstants.wheelTranslations.get(2),
            DriveConstants.wheelTranslations.get(3));

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

    followPathCommand.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    if (followPathCommand != null) return followPathCommand.isFinished();
    else return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (followPathCommand != null) {
      followPathCommand.cancel();
    }

    driveSubsystem.setDefaultCommand(defaultDriveCommand);
  }
}
