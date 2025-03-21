package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class AutoSubsystem extends SubsystemBase {
    public AutoSubsystem() {}

    public Pose2d getOffsetTagPose(Translation2d tagRelativeTranslation, int tagID) {
        Pose2d aprilTagPosition = FieldConstants.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();

        return new Pose2d(
            aprilTagPosition.getTranslation()
            .plus(tagRelativeTranslation).rotateBy(aprilTagPosition.getRotation()),
            aprilTagPosition.getRotation().plus(Rotation2d.fromRadians(Math.PI))
        );
    }

    public Pose2d getOffsetTagPose(Translation2d tagRelativeTranslation, Rotation2d rotationalOffset, int tagID) {
        Pose2d aprilTagPosition = FieldConstants.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();

        return new Pose2d(
            aprilTagPosition.getTranslation()
            .plus(tagRelativeTranslation).rotateBy(aprilTagPosition.getRotation()),
            aprilTagPosition.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)).rotateBy(rotationalOffset)
        );
    }
}
