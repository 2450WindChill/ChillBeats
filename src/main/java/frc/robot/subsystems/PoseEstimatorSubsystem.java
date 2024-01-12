package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase{

    public final SwerveDrivePoseEstimator poseEstimate;
    public final DrivetrainSubsystem m_drivetrainSubsystem;

    public PoseEstimatorSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        poseEstimate = new SwerveDrivePoseEstimator(
            Constants.swerveKinematics,
            m_drivetrainSubsystem.getGyroYaw(),
            m_drivetrainSubsystem.getPositions(),
            new Pose2d());
    }

    public void periodic() {
        Pose2d currentPoseEstimate = poseEstimate.update(
                                        m_drivetrainSubsystem.getGyroYaw(),
                                        m_drivetrainSubsystem.getPositions());
        
        if (LimelightHelpers.getTV("limelight")) {
            poseEstimate.addVisionMeasurement(getLimelightPose(), getLimelightTimeStamp());
        }

        SmartDashboard.putNumberArray(
                "Pose Estimate",
                new double[] {
                    currentPoseEstimate.getX(),
                    currentPoseEstimate.getY(),
                    currentPoseEstimate.getRotation().getDegrees()
        });
    }

    public static Pose2d getLimelightPose() {
        return LimelightHelpers.getBotPose2d("limelight");
    }

    public static double getLimelightTimeStamp() {
         return -((LimelightHelpers.getLatency_Capture("limelight")
                    + LimelightHelpers.getLatency_Pipeline("limelight"))
                        / 1000.0);
    }

    public Pose2d getBotPose() {
        return poseEstimate.getEstimatedPosition();
    }

    public double getBotX() {
        return getBotPose().getX();
    }

    public double getBotY() {
        return getBotPose().getY();
    }

    public double getBotRotation() {
        return getBotPose().getRotation().getDegrees();
    }
}
