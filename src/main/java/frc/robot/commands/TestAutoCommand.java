package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class TestAutoCommand extends Command {
    DrivetrainSubsystem drivetrainSubsystem;
    PoseEstimatorSubsystem poseEstimatorSubsystem;
    Pose2d target;
    Pose2d currentPose;
    double distance;
    double targetDistance;
    double angle;
    Pose2d initialPose;

    public TestAutoCommand(DrivetrainSubsystem drivetrainSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem, double targetDistance, double angle) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.targetDistance = targetDistance;
        this.angle = angle;
    }

    public void initialize() {
        initialPose = poseEstimatorSubsystem.getBotPose();
        System.err.println("Auto Initialize");
    }

    public void execute() {
        currentPose = poseEstimatorSubsystem.getBotPose();
        drivetrainSubsystem.testAuto(.4, angle);
        distance = Math.sqrt((Math.pow(currentPose.getY() - initialPose.getY(), 2))
                             + (Math.pow(currentPose.getX() - initialPose.getX(), 2))
                            );
        SmartDashboard.putNumber("Auto Distance", distance);
        System.err.println("Auto Execute");
    }

    public void end(boolean isFinished) {

    }

    public boolean isFinished() {
        if (distance < targetDistance) {
            System.err.println("Auto Not Finished");
            return false;
        } else {
            System.err.println("Auto Finished");
            return true;
        }
    }
}
