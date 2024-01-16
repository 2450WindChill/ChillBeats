package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase{

    public final SwerveDrivePoseEstimator poseEstimate;
    public final DrivetrainSubsystem m_drivetrainSubsystem;

    public PoseEstimatorSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        // Sets pose of Limelight relative to robot center (currently in inches and degrees)
        LimelightHelpers.setCameraPose_RobotSpace("limelight", 9.464, 7.462, 17.24, 0, 13, 0);

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
            poseEstimate.addVisionMeasurement(getLimelightPose(), getTimeStamp());
        }

        SmartDashboard.putNumberArray(
                "Pose Estimate",
                new double[] {
                    currentPoseEstimate.getX(),
                    currentPoseEstimate.getY(),
                    currentPoseEstimate.getRotation().getDegrees()
        });
    }

    public Pose2d getLimelightPose() {
            if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
                return LimelightHelpers.getBotPose2d_wpiRed("limelight");
            } else {
                return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            }
    }

    // Gets gametime and then subtracts the latency of the pose
    public double getTimeStamp() {
         return Timer.getFPGATimestamp()
                -((LimelightHelpers.getLatency_Capture("limelight")
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
