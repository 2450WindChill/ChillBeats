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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.libs.ModuleConfiguration;
// import frc.lib.util.CANCoderUtil;
// import frc.lib.util.CANSparkMaxUtil;
// import frc.lib.util.CANCoderUtil.CCUsage;
// import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.libs.OnboardModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WindChillSwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

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
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    // configAngleEncoder();

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

    // if (reseedTimer.advanceIfElapsed(ENCODER_RESEED_SECONDS) &&
    //   angleEncoder.getVelocity().getValue() * motorEncoderVelocityCoefficient < ENCODER_RESEED_MAX_ANGULAR_VELOCITY) {
    //   // resetToAbsolute();
    //   System.out.println(moduleNumber + " reseting to Absolute");
    // }
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

  // private void setAngle(SwerveModuleState state){
  //   double steerSetpoint = state.angle.getRadians();
  //   if(steerSetpoint < 0){
  //     steerSetpoint = 2*Math.PI + steerSetpoint % (2* Math.PI);
  //   }else{
  //     steerSetpoint = steerSetpoint % (2* Math.PI);
  //   }
  //   //steerSetpoint = (Math.abs(steerSetpoint-getSteerPosition()) < 0.15) ? 0 : steerSetpoint;
  //   angleController.setReference(steerSetpoint, CANSparkMax.ControlType.kPosition);
  //   //System.out.println(name + "pid out " + steerMotor.get());
  // }

  // Not using because we're just using the absolute encoder
  private void resetToAbsolute() {
    double absolutePosition = getCanCoderInDegrees(); /* - angleOffset.getDegrees() */
    integratedAngleEncoder.setPosition(absolutePosition);
    // System.err.println("Reseting Mod " + moduleNumber + " to " +
    // absolutePosition);
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
    // if (this.moduleNumber == 0) {
    //   driveMotor.setInverted(true);
    // }
    // else {
       //driveMotor.setInverted(Constants.driveInvert);
    //}

    if (this.moduleNumber == 1 || this.moduleNumber == 3) {
      driveMotor.setInverted(true);
    }
    else {
      driveMotor.setInverted(Constants.driveInvert);
    }

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
    System.err.println("Configing CanCoder");
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    angleEncoder.getConfigurator().apply(cancoderConfig);
    // CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
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
