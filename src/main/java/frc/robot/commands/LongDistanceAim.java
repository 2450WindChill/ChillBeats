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
    private double percentError;

    private int closestID;
    private double estiamtedAngle;

    private boolean isGoingUp;

    private double[] angles = Constants.anglesToShootInSpeaker;
    private int[] distances = Constants.distancesFromSpeaker;
    
    public LongDistanceAim(AimSubsystem aimSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        m_aimSubsystem = aimSubsystem;
        m_poseEstimatorSubsystem = poseEstimatorSubsystem;

        addRequirements(aimSubsystem);
    }

    public void initialize() {
        // Gets distance (in meters) to an april tag and then rounds it to nearest integer
        distanceToTarget = m_poseEstimatorSubsystem.getDistanceToAprilTag2d();
        roundedDistanceToTarget = (int) (distanceToTarget + 0.5);
        roundedUp = (distanceToTarget < roundedDistanceToTarget);

        // Percent error is used to find a more accurate value between two known poitns
        percentError = distanceToTarget - (int) distanceToTarget;
        
        // Finds the known angle that matches with the closest known distance
        for (int i = 0; i < distances.length; i++) {
            if (roundedDistanceToTarget == distances[i]) {
                closestID = i;
            }
        }

        // Finds average of closest values
        // if (roundedUp) {
        //     estiamtedAngle = ((angles[closestID] 
        //                         + angles[closestID - 1])
        //                             /2);
        // } else {
        //     estiamtedAngle = ((angles[closestID] 
        //                         + angles[closestID + 1])
        //                             /2);
        // }

        // Estimates a value proportionally between closest value
        if (roundedUp) {
            estiamtedAngle = (angles[closestID - 1]
                                + (angles[closestID] - angles[closestID - 1] * percentError)
                            );
        } else {
            estiamtedAngle = (angles[closestID]
                                + (angles[closestID + 1] - angles[closestID] * percentError)
                            );
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
