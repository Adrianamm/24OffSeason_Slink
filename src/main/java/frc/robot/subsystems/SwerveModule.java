// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CANSparkMaxCurrent;
import frc.robot.util.SwerveModuleConfig;
import frc.robot.Constants.Module;

public class SwerveModule extends SubsystemBase {
  
  // motor controller types
  // uses CANSparkMaxCurrent from util
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
   * Configures Drive Motors using parameters from Constants
   */
  public void configureDriveMotor(){

  }

  /**
   * Configures Angle Motors using parameters from Constants
   */
  public void configureAngleMotor(){

  }


  public SwerveModule() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
