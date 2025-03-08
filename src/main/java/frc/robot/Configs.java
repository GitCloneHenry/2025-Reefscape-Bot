package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class Neo550 {
        public static final SparkMaxConfig neoConfig = new SparkMaxConfig();

        static {
            double floorIntakeDriveFeedForward = 1 / ModuleConstants.kFloorIntakeDriveFreeSpeedRps;

            neoConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(10);
            neoConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0, 0.0, 0.0)
                .velocityFF(0.00036)
                .outputRange(-1, 1);
            neoConfig
                .closedLoop
                .maxMotion
                .maxVelocity(700)
                .maxAcceleration(1800)
                .allowedClosedLoopError(0.01);
        }
    }
}
