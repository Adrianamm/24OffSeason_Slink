// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.CANSparkMaxCurrent;
import frc.robot.util.SwerveModuleConfig;
import frc.robot.Constants.Module;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
  
  // motor controller types: uses CANSparkMaxCurrent from util
  public CANSparkMaxCurrent driveMotor;
  public CANSparkMaxCurrent angleMotor;

  // number for module (1 through 4)
  public int moduleNumber;

  public SparkPIDController driveController;
  public SparkPIDController angleController;

  // a helper class that calcuates feedforward
  private SimpleMotorFeedforward driveFeedforward;

  // Sets up the encoder 
  // Relative: It tells you 0 when the robot is booted up
  // relative to their stop/start points (inside the encoder)
  public RelativeEncoder driveEncoder;
  public RelativeEncoder angleEncoder;

  public double angleReference;
  public double driveReference;

  public SparkAbsoluteEncoder absoluteEncoder;

  // tells the robot how much it has turned and which way?
  private Rotation2d KModuleAbsoluteOffset_rad; 

  /*
   * Sets up the module
   */
  public SwerveModule(int moduleNumber, SwerveModuleConfig config){
    // sets up module number (1 through 4)
    this.moduleNumber = moduleNumber;

    // Continuing the drive and angle motor set up
    // takes the id from SwerveModuleConfig. Sets motor type to Brushless
    driveMotor = new CANSparkMaxCurrent(config.driveMotorID, MotorType.kBrushless);
    angleMotor = new CANSparkMaxCurrent(config.angleMotorID, MotorType.kBrushless);

    // sets up encoder 
    driveEncoder = driveMotor.getEncoder();
    angleEncoder = angleMotor.getEncoder();

    // "object for interfacing with the integrated PID controller"
    driveController = driveMotor.getPIDController();
    angleController = angleMotor.getPIDController();

    /* Creates an additional FF controller for extra drive motor control */
    // Takes S, V and A from constants 
    driveFeedforward = new SimpleMotorFeedforward(Module.kDriveS, Module.kDriveV, Module.kDriveA);
  
    // variable is being set to the angle of the motor's absolute encoder
    absoluteEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  
    this.KModuleAbsoluteOffset_rad = config.absoluteEncoderOffset_Rad;

    // configures drive and angle motors using pareamters from constasnts??
    configureDriveMotor();
    configureAngleMotor();
  }

  // I put the getters first but now i'm regretting it eh

  /**
   * Returns the position of the Angle Motor, measured with an integrated encoders
   * 
   * @return Angle Motor Position
   */
  public Rotation2d getAnglePosition(){
    return Rotation2d.fromDegrees(angleEncoder.getPosition());
  }

  /**
   * Returns the velocity of the Drive Motor, measured with an integrated encoder
   * 
   * @return Drive Motor Velocity
   */
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }
  
  /**
   * Gets the position of the Drive Motor, measured with an integrated encoder
   * 
   * @return Drive Motor Position
   */
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  /**
   * 
   * @return Swerve Module Position (includes: Position & Angle)
   */
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(-getDrivePosition(), getAbsolutePosition());
  }

  /**
   * 
   * @return Swerve Module State (includes: Velocity & Angle)
   */
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), getAnglePosition());
  }

  /**
   * Returns the assigned module number
   */
  public int getModuleNumber(){
    return moduleNumber;
  }

  /**
   * Gets the position of the module using the absolute encoder
   * 
   * @return Position of the modlue between 0 and 360, as Rotation R2
   */
  public Rotation2d getAbsolutePosition(){
    // gets position from SparkMax absolute encoder * 360 to get degrees
    double positionDeg = absoluteEncoder.getPosition() * 360.0d;

    // Substracts magnetic offset to get the wheel position
    positionDeg -= KModuleAbsoluteOffset_rad.getDegrees();

    // inverts if needed
    // funky if statement
    positionDeg *= (Module.KAbsoluteEncoderInverted ? - 1: 1);

    return Rotation2d.fromDegrees(positionDeg);
  }

  // for debugging reasons
  //public Rotation2d getAbsolutePositionNoOffset

  /**
   * Sets both Angle and Drive to the desired states
   * 
   * @param state: Desired module state
   * @param isOpenLoop: Controls if the dirve motor uses a PID loop or not
   */
  public void setModuleState(SwerveModuleState state, boolean isOpenLoop){
    state = SwerveModuleState.optimize(state, getAbsolutePosition());

    setAngleState(state); // no pid here?
    setDriveState(state, isOpenLoop);
  }

  /**
   * Sets the drive motor to a desired state
   * If isOpenLoop is true, it will be set as percent, if false then it
   * will use a velcocity PIDF loop
   * 
   * @param state: Desired module state
   * @param isOpenLoop: Is drive uses PID or not
   */
  public void setDriveState(SwerveModuleState state, boolean isOpenLoop){
    if (isOpenLoop){ // uses precent
      // convert the speed to a percent, 
      double motorPercent = state.speedMetersPerSecond / SwerveConstants.kMaxSpeedTele_MPS;
      driveMotor.set(motorPercent);
    } else { // uses PID Loop
      // sets the desired behavior
      driveController.setReference(
        state.speedMetersPerSecond, //sets the seed in "config of the module" to mps
        ControlType.kVelocity, // tells controller to work by velocity
        0, //default
        driveFeedforward.calculate(state.speedMetersPerSecond) //figured out the feedforward voltage 
      );

      driveReference = state.speedMetersPerSecond;
    }
  }

  /**
   * Sets the Angle Motor to a desired state, 
   * I removed anti-jitter code because it was commmented out 
   * 
   * @param state: Desired module state
   */
  public void setAngleState(SwerveModuleState state){
    Rotation2d angle = state.angle; // gets target angle from state.angle 
    if (angle != null){ // if we have an angle
      // two things: angle to degrees, tells the controller to work by going to position
      angleController.setReference(angle.getDegrees(),ControlType.kPosition); 
      angleReference = angle.getDegrees(); // updates varible to keep current target angle in degrees
    }
  }


  /**
   * Configures Drive Motors using parameters from Constants
   */
  public void configureDriveMotor(){
    driveMotor.restoreFactoryDefaults();

    driveMotor.setInverted(Module.driveMotorInverted);
    driveMotor.setIdleMode(Module.kDriveIdleMode);

    // sets encoder rations to actual module gear ratios
    driveEncoder.setPositionConversionFactor(Module.kDrivePositionConversionFactor);
    driveEncoder.setVelocityConversionFactor(Module.kDriveVelocityConverstionFactor);

    // configures PID loop
    driveController.setP(Module.kDriveP);
    driveController.setI(Module.kDriveI);
    driveController.setD(Module.kDriveD);

    
    driveMotor.setSpikeCurrentLimit(
      Module.DriveCurrentLimit.KlimitToAmps,
      Module.DriveCurrentLimit.kMaxSpikeTime,
      Module.DriveCurrentLimit.kMaxSpikeAmps,
      Module.DriveCurrentLimit.KSmartLimit
    );

  driveEncoder.setPosition(0.0);
  }

  /**
   * Configures Angle Motors using parameters from Constants
   */
  public void configureAngleMotor(){
    angleMotor.restoreFactoryDefaults();

    angleMotor.setInverted(Module.angleMotorInverted);
    angleMotor.setIdleMode(Module.kAngleIdleMode);

    // sets encoder ratio to the real Module gear ratio 
    angleEncoder.setPositionConversionFactor(Module.kAnglePositionConversionFactor);
    angleEncoder.setVelocityConversionFactor(Module.kAngleVelocityConverstionFactor);

    /* Configures PID loop */
    angleController.setP(Module.kAngleP);
    angleController.setI(Module.kAngleI);
    angleController.setD(Module.kAngleD);

    /* Defines wheel angles as -pi to pi */
    angleController.setPositionPIDWrappingMaxInput(180.0d);
    angleController.setPositionPIDWrappingMinInput(-180.0d);
    angleController.setPositionPIDWrappingEnabled(true);

    // angleMotor.setSmartCurrentLimit(Module.kAngleCurrentLimit);
    angleMotor.setSpikeCurrentLimit(
        Module.AngleCurrentLimit.kLimitToAmps,
        Module.AngleCurrentLimit.kMaxSpikeTime,
        Module.AngleCurrentLimit.kMaxSpikeAmps,
        Module.AngleCurrentLimit.kSmartLimit
    );

    // angleMotor.burnFlash();

    setIntegratedAngleToAbsolute();
  }

     /**
     * Resets the Angle Motor to the position of the absolute position
     */
    public void setIntegratedAngleToAbsolute() {
      angleEncoder.setPosition(getAbsolutePosition().getDegrees());
  }

  public void runPeriodicLimiting() {
    driveMotor.periodicLimit();
    angleMotor.periodicLimit();
  }
}
