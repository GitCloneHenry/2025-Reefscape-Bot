import edu.wpi.first.wpilibj2.command.SubsystemBase;

class VisionSubsystem extends SubsystemBase {
    private PhotonCamera leftCamera;
    private PhotonCamera rightCamera;

    private PhotonPoseEstimator leftPoseEstimator;
    private PhotonPoseEstimator rightPoseEstimator;

    public double leftPoseAmbiguity;
    public double rightPoseAmbiguity;
    
    private AprilTagFieldLayout aprilTagFieldLayout;

    public VisionSubsystem() {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(
            AprilTagField.kDefaultField
        )
    }

    @Override 
    public void periodic() {

    }
} 