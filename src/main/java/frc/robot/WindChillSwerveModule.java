package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.libs.OnboardModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WindChillSwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  public WindChillSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, "canivore");

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;

  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState);
  }

  private void setSpeed(SwerveModuleState desiredState) {
    double percentOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed;
    driveMotor.set(percentOutput);
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    double desiredAngle = angle.getDegrees();

    while (desiredAngle < 0) {
      desiredAngle += 360;
    }

    while (desiredAngle > 360) {
      desiredAngle -= 360;
    }

    SmartDashboard.putNumber("Desired Angle " + moduleNumber, desiredAngle);

    angleController.setReference(desiredAngle, CANSparkMax.ControlType.kPosition);
    lastAngle = angle;    
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoderInDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  // Using getCancoder instead
  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public double getCanCoderInDegrees() {
    return getRawCanCoder() * 360.0;
  }

  public double getRawCanCoder() {
    return angleEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getDriveEncoder() {
    return driveEncoder.getPosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  private void configAngleMotor() {
    // angleMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.angleInvert);
    // angleMotor.setIdleMode(Constants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.angleConversionFactor);
    angleController.setP(Constants.angleKP);
    angleController.setI(Constants.angleKI);
    angleController.setD(Constants.angleKD);
    angleController.setFF(Constants.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    // driveMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.driveContinuousCurrentLimit);
    if (this.moduleNumber == 0) {
      driveMotor.setInverted(true);
    }
    else {
       driveMotor.setInverted(Constants.driveInvert);
    }

    // if (this.moduleNumber == 1 || this.moduleNumber == 3) {
    //   driveMotor.setInverted(true);
    // }
    // else {
    //   driveMotor.setInverted(Constants.driveInvert);
    // }

    // driveMotor.setIdleMode(Constants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.driveConversionPositionFactor);
    driveController.setP(Constants.angleKP);
    driveController.setI(Constants.angleKI);
    driveController.setD(Constants.angleKD);
    driveController.setFF(Constants.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        (getDriveEncoder() / Constants.rotationsPerOneFoot) * Constants.feetToMeters,
        Rotation2d.fromDegrees(getCanCoderInDegrees()));
  }

  public void setPosition(double position) {
    integratedAngleEncoder.setPosition(position);
  }
}
