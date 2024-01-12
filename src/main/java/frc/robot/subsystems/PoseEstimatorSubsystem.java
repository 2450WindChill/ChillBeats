package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;

public class PoseEstimatorSubsystem extends SubsystemBase{
    public final SwerveDrivePoseEstimator poseEstimate;

    public final DrivetrainSubsystem m_drivetrainSubsystem;

    public double limelightPoseTimestamp;

    public PoseEstimatorSubsystem(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelight) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        poseEstimate = new SwerveDrivePoseEstimator(
            Constants.swerveKinematics,
            m_drivetrainSubsystem.getGyroYaw(),
            m_drivetrainSubsystem.getPositions(),
            new Pose2d());
    }

    public void periodic() {
        Pose2d currentPoseEstimate = poseEstimate.update(m_drivetrainSubsystem.getGyroYaw(), m_drivetrainSubsystem.getPositions());

        limelightPoseTimestamp = -((LimelightHelpers.getLatency_Capture("limelight")
                                 + LimelightHelpers.getLatency_Pipeline("limelight"))
                                 / 1000.0);

        if (DriverStation.getAlliance() == null || DriverStation.getAlliance() == null) {

        } else {

        }


        poseEstimate.addVisionMeasurement(null, limelightPoseTimestamp);
    }
}
