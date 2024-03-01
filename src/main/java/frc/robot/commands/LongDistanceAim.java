package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class LongDistanceAim extends Command {
    private AimSubsystem m_aimSubsystem;
    private PoseEstimatorSubsystem m_poseEstimatorSubsystem;
    private double distanceToTarget;
    private int roundedDistanceToTarget;
    private boolean roundedUp;
    private int idOfClosestMeasuedDistance;
    private double estiamtedAngle;
    private boolean isGoingUp;
    
    public LongDistanceAim(AimSubsystem aimSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        m_aimSubsystem = aimSubsystem;
        m_poseEstimatorSubsystem = poseEstimatorSubsystem;

        addRequirements(aimSubsystem);
    }

    public void initialize() {
        distanceToTarget = m_poseEstimatorSubsystem.getDistanceToAprilTag2d();
        roundedDistanceToTarget = (int) (distanceToTarget + 0.5);
        roundedUp = (distanceToTarget < roundedDistanceToTarget);
        
        for (int i = 0; i < Constants.distancesFromSpeaker.length; i++) {
            if (roundedDistanceToTarget == Constants.distancesFromSpeaker[i]) {
                idOfClosestMeasuedDistance = i;
            }
        }

        if (roundedUp) {
            estiamtedAngle = ((Constants.angleToShootInSpeaker[idOfClosestMeasuedDistance] 
                                + Constants.angleToShootInSpeaker[idOfClosestMeasuedDistance - 1])
                                    /2);
        } else {
            estiamtedAngle = ((Constants.angleToShootInSpeaker[idOfClosestMeasuedDistance] 
                                + Constants.angleToShootInSpeaker[idOfClosestMeasuedDistance + 1])
                                    /2);
        }

        double currentPosition = m_aimSubsystem.wristMotor.getEncoder().getPosition();

        if (currentPosition < estiamtedAngle) {
            isGoingUp = true;
        } else {
            isGoingUp = false;
        }
    }

    public void execute() {
        m_aimSubsystem.wristController.setReference(estiamtedAngle, ControlType.kPosition);
    }

    public void end(boolean isFinished) {

    }

    public boolean isFinished() {
        double currentPosition = m_aimSubsystem.wristMotor.getEncoder().getPosition();
        if (isGoingUp) {
            if (currentPosition >= estiamtedAngle - 0.3) {
                return true;
            } else {
                return false;
            }
        } else {
            if (currentPosition <= estiamtedAngle + 0.3) {
                return true;
            } else {
                return false;
            }
        }
    }
}
