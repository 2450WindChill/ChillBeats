package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightySubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class TrajectoryCommand extends Command {
    private final DrivetrainSubsystem m_driveSubsystem;
    private final PoseEstimatorSubsystem m_Estimator;
    private final ChassisSpeeds adjustedSpeeds;

    public TrajectoryCommand(DrivetrainSubsystem m_drivetrainSubsystem,
            PoseEstimatorSubsystem m_EstimatorSubsystem,
            Translation2d translation2d) {
        m_driveSubsystem = m_drivetrainSubsystem;
        m_Estimator = m_EstimatorSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveSubsystem);
        Pose2d startPoint = m_Estimator.getThisPose();
        Pose2d endPoint = new Pose2d(translation2d, m_drivetrainSubsystem.getGyroYaw());

        Trajectory trajectory = generateTrajectory(startPoint, endPoint);
        // Sample the trajectory at 3.4 seconds from the beginning.
        Trajectory.State goal = trajectory.sample(3.4);

        // Get the adjusted speeds. Here, we want the robot to be facing
        // 70 degrees (in the field-relative coordinate system).
         adjustedSpeeds = m_drivetrainSubsystem.holonomicDriveController.calculate(
                m_Estimator.getThisPose(), goal,  m_drivetrainSubsystem.getGyroYaw());

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SwerveModuleState[] moduleStates = Constants.swerveKinematics.toSwerveModuleStates(adjustedSpeeds);
        m_driveSubsystem.setStates(moduleStates);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
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
