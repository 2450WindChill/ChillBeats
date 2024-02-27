package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.WindChillSwerveModule;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class TrajectoryCommand extends Command {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final PoseEstimatorSubsystem m_poseEstimator;
    private final Timer timer;
    private final Trajectory trajectory;
    private final Pose2d m_targetPose;

    public TrajectoryCommand(DrivetrainSubsystem drivetrainSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d targetPose) {
        
        m_driveSubsystem = drivetrainSubsystem;
        m_poseEstimator = poseEstimatorSubsystem;
        timer = new Timer();
        m_targetPose = targetPose;
        trajectory = generateTrajectory(m_poseEstimator.getThisPose(), m_targetPose);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentTime = timer.get();
        State desiredState = trajectory.sample(currentTime);

        ChassisSpeeds targetChassisSpeeds =
            m_driveSubsystem.holonomicDriveController.calculate(m_poseEstimator.getThisPose(), desiredState, m_targetPose.getRotation());

        SwerveModuleState[] targetModuleStates = Constants.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds);

        for (WindChillSwerveModule mod : m_driveSubsystem.swerveModules) {
            mod.setDesiredState(targetModuleStates[mod.moduleNumber]);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public Trajectory generateTrajectory(Pose2d startPoint, Pose2d endPoint) {

        var interiorWaypoints = new ArrayList<Translation2d>();

        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
        config.setReversed(false);

        return TrajectoryGenerator.generateTrajectory(
                startPoint,
                interiorWaypoints,
                endPoint,
                config);
    }
}
