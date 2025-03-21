// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  private final VisionSubsystem m_visionSubsystem = 
      new VisionSubsystem();

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(0);
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          // Rotation2d.fromDegrees(m_gyro.getAngle()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  // Boolean used to enable / disable slow mode
  private boolean slowMode = false;

  // Boolean used to enable / disable camera-centric driving
  // private boolean cameraCentric = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    RobotConfig robotConfig;
    
    try {
      robotConfig = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        driveRobotRelative,
        new PPHolonomicDriveController(
          new PIDConstants(2.25, 0.85, 0.31), 
          new PIDConstants(3.7, 1.6, 0.6)
        ),
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public void updateOdometryBasic() {
    // Update the odometry in the periodic block
    m_odometry.update(
    m_gyro.getRotation2d(),
    // Rotation2d.fromDegrees(m_gyro.getAngle()),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    });
  }

  @Override
  public void periodic() {
    // updateOdometryBasic();

    List<PhotonPipelineResult> visionResults = m_visionSubsystem.getVisionResults();

    if (visionResults.size() < 1) {
      updateOdometryBasic();
      return;
    }

    PhotonPipelineResult visionResult = visionResults.get(0);

    double minDistance = Double.MAX_VALUE;

    for (PhotonTrackedTarget target : visionResult.getTargets()) {
      double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();

      if (distanceToTag < minDistance) {
        minDistance = distanceToTag;
      }
    }

    // // if (minDistance > 2.5) {
    // //   updateOdometryBasic();
    // //   return; 
    // // }

    Optional<Pose2d> potentialPose = m_visionSubsystem.estimateRobotPose(visionResult);

    if (potentialPose.isEmpty()) {
      updateOdometryBasic();
      return;
    }

    resetPose(potentialPose.get());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d newPose) {
    m_odometry.resetPose(newPose);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        // Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered =
        xSpeed
            * DriveConstants.kMaxSpeedMetersPerSecond
            * (slowMode ? DriveConstants.kSlowModeSpeedPercentage : 1);
    double ySpeedDelivered =
        ySpeed
            * DriveConstants.kMaxSpeedMetersPerSecond
            * (slowMode ? DriveConstants.kSlowModeSpeedPercentage : 1);
    double rotDelivered =
        rot
            * DriveConstants.kMaxAngularSpeed
            * (slowMode ? DriveConstants.kSlowModeSpeedPercentage : 1);

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d()/*Rotation2d.fromDegrees(m_gyro.getAngle())*/)
                : /*cameraCentric
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeedDelivered,
                        ySpeedDelivered,
                        rotDelivered,
                        Rotation2d.fromDegrees(VisionConstants.climbCameraAngle))
                    :*/ new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Enables the robot's slow mode */
  public Command enableSlowModeCommand() {
    return Commands.runOnce(
        () -> {
          slowMode = true;
        });
  }

  /** Disables the robot's slow mode */
  public Command disableSlowModeCommand() {
    return Commands.runOnce(
        () -> {
          slowMode = false;
        });
  }

  /** Enables the robot's slow mode */
  public void enableSlowMode() {
    slowMode = true;
  }

  /** Disables the robot's slow mode */
  public void disableSlowMode() {
    slowMode = false;
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return Commands.runOnce(
        () -> {
          m_gyro.reset();
        });
  }

  /** Intelligent gyro zeroing based on alliance and field positioning */
  public Command smartZeroHeading() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      return Commands.runOnce(
        () -> m_gyro.setYaw(getPose().getRotation().getDegrees() + (alliance.get() == DriverStation.Alliance.Blue ? 0.0 : 180.0)),
        this
      );
    }

    return Commands.print("Couldn't get alliance!");
  }

  /** Drives the robot relative to the head of the robot */
  public BiConsumer<ChassisSpeeds, DriveFeedforwards> driveRobotRelative =
      (chassisSpeeds, feedForwards) -> {
        SwerveModuleState[] swerveModuleStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
      };

  /** Gets the relative speeds of the individual swerve modules */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    List<SwerveModuleState> moduleStates = this.getModuleStates();
    ChassisSpeeds robotRelativSpeeds =
        DriveConstants.kDriveKinematics.toChassisSpeeds(
            moduleStates.get(0), moduleStates.get(1), moduleStates.get(2), moduleStates.get(3));
    return robotRelativSpeeds;
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Gets the states of the individual swerve modules */
  public List<SwerveModuleState> getModuleStates() {
    return List.of(
        this.m_frontLeft.getState(),
        this.m_frontRight.getState(),
        this.m_rearLeft.getState(),
        this.m_rearRight.getState());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }
}
