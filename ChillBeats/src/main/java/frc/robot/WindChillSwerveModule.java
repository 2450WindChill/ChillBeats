/**
 * Need:
 * Define swerve module constants
 * Pass in constants on construct
 * Define Class variables
 * SetDesiredStates(public) with setAngle(private) and setSpeed(private)
 * 
 * 
 * Backlog:
 * Configure components
 */

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
// import frc.lib.util.CANCoderUtil;
// import frc.lib.util.CANSparkMaxUtil;
// import frc.lib.util.CANCoderUtil.CCUsage;
// import frc.lib.util.CANSparkMaxUtil.Usage;

public class WindChillSwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private double ENCODER_RESEED_SECONDS = 5.0;
  private Timer reseedTimer = new Timer();

  private final double motorEncoderPositionCoefficient;
  private final double motorEncoderVelocityCoefficient;

  private static final double ENCODER_RESEED_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

  public WindChillSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    reseedTimer.start();

    motorEncoderPositionCoefficient = 2.0 * Math.PI / Constants.TICKS_PER_ROTATION
        * ModuleConfiguration.MK4I_L1.getSteerReduction();
    motorEncoderVelocityCoefficient = motorEncoderPositionCoefficient * 10.0;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

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

    if (reseedTimer.advanceIfElapsed(ENCODER_RESEED_SECONDS) &&
      angleEncoder.getVelocity() * motorEncoderVelocityCoefficient < ENCODER_RESEED_MAX_ANGULAR_VELOCITY) {
      resetToAbsolute();
      System.out.println(moduleNumber + " reseting to Absolute");
    }
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not

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
    // Rotation2d angle = desiredState.angle;

    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    double newAngle = angle.getDegrees();

    while (newAngle < 0) {
      newAngle += 360;
    }

    while (newAngle > 360) {
      newAngle -= 360;
    }

    //angleController.setReference(newAngle, ControlType.kPosition);

    if (moduleNumber == 0) {
      // System.err.println("Angle: " + newAngle);
    }
    lastAngle = angle;    
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
    // System.err.println("Reseting Mod " + moduleNumber + " to " +
    // absolutePosition);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public double getDriveEncoder() {
    return driveEncoder.getPosition();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
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
    driveMotor.restoreFactoryDefaults();
    // CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.driveInvert);
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

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        (getDriveEncoder() / Constants.rotationsPerOneFoot) * 0.3048,
        getAngle());
  }

  public void setPosition(double position) {
    integratedAngleEncoder.setPosition(position);
  }
}
