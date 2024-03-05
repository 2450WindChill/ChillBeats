package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase {

    public static SwerveDrivePoseEstimator poseEstimate;
    public static DrivetrainSubsystem m_drivetrainSubsystem;

    public PoseEstimatorSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        poseEstimate = new SwerveDrivePoseEstimator(
                Constants.swerveKinematics,
                m_drivetrainSubsystem.getGyroYaw(),
                m_drivetrainSubsystem.getPositions(),
                new Pose2d());
        SmartDashboard.putData("Reset pose", Commands.runOnce(() -> zeroPose()));
    }

    public void periodic() {
        poseEstimate.update(
                m_drivetrainSubsystem.getGyroYaw(),
                m_drivetrainSubsystem.getPositions());

        // if (LimelightHelpers.getTV("limelight")) {
        //     poseEstimate.addVisionMeasurement(getLimelightPose(), getTimeStamp());
        // }

        SmartDashboard.putNumber("Pose X", getBotX());
        SmartDashboard.putNumber("Pose Y", getBotY());
        SmartDashboard.putNumber("Pose Rotation", getBotRotation());
    }

    // Gets bot pose using limelight always relative to blue alliance wall
    public Pose2d getLimelightPose() {
        return LimelightHelpers.getBotPose2d("limelight");
    }

    // Gets gametime and then subtracts the latency of the pose
    public double getTimeStamp() {
        return Timer.getFPGATimestamp()
                - ((LimelightHelpers.getLatency_Capture("limelight")
                        + LimelightHelpers.getLatency_Pipeline("limelight"))
                        / 1000.0);
    }

    // Gets current estimated bot pose
    public static Pose2d getBotPose() {
        return poseEstimate.getEstimatedPosition();
    }

    public Pose2d getThisPose() {
        return poseEstimate.getEstimatedPosition();
    }

    public void zeroPose() {
        poseEstimate = new SwerveDrivePoseEstimator(
                Constants.swerveKinematics,
                m_drivetrainSubsystem.getGyroYaw(),
                m_drivetrainSubsystem.getModulePositions(),
                new Pose2d(0, 0, new Rotation2d(0)));
    }

    public static void resetPose(Pose2d newPose) {
        poseEstimate.resetPosition(m_drivetrainSubsystem.gyro.getRotation2d(), m_drivetrainSubsystem.getPositions(),
                newPose);
    }

    // Gets estimated bot x
    public double getBotX() {
        return getBotPose().getX();
    }

    // Gets estimated bot y
    public double getBotY() {
        return getBotPose().getY();
    }

    // Gets estimated bot rotation
    public double getBotRotation() {
        return getBotPose().getRotation().getDegrees();
    }

    // Gets the ID of the primary apriltag in the limelights view
    public double getAprilTagID() {
        return LimelightHelpers.getFiducialID("limelight");
    }

    // Get the 3d bot pose of the primary apriltag in the limelights view relative
    // to the robots pose
    public Pose3d getAprilTagPoseToBot3d() {
        return LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    }

    // Get the 2d bot pose of the primary apriltag in the limelights view relative
    // to the robots pose
    public Pose2d getAprilTagPoseToBot2d() {
        return new Pose2d(getAprilTagPoseToBot3d().getX(), getAprilTagPoseToBot3d().getY(),
                getAprilTagPoseToBot3d().getRotation().toRotation2d());
    }

    // Gets distance to primary april tag
    // Returns in meters
    public double getDistanceToAprilTag2d() {
        return Math.sqrt(
            Math.pow(getAprilTagPoseToBot2d().getX(), 2)
             + Math.pow(getAprilTagPoseToBot2d().getY(), 2)
        );
    }
}
