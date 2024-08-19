// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Modules;

public class Swerve extends SubsystemBase {
  public SwerveModule[] modules; // array of moudles
  private AHRS gyro; // gyro
  public ChassisSpeeds chassisSpeeds; // repsents speed of robot chassis

  public Swerve() {
    // initializes moudles from constants
    modules = 
      new SwerveModule[]{
        new SwerveModule(0, Modules.FrontLeft.FL0),
        new SwerveModule(1, Modules.FrontRight.FR1),
        new SwerveModule(2, Modules.BackLeft.BL2),
        new SwerveModule(3, Modules.BackRight.BR3)
      };
    
    // setting up gyro
   gyro = new AHRS(SPI.Port.kMXP);
   // zero the gyro
   zeroGyro();

   chassisSpeeds = new ChassisSpeeds();

  }


  @Override
  public void periodic() { // This method will be called once per scheduler run
    for (SwerveModule m : modules){
      m.runPeriodicLimiting();
    }
  }

  /**
   * Runs all IK and sets module states
   * 
   * @param translate Desired translation speeds in m/s
   * @param rotate Desired rotation rate in Rotation2d
   * @param fieldRelative Driving Mode
   * @param isOpenLoop Drive controller mode
   */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.kMaxSpeedTele);

        for (SwerveModule m : modules) {
            m.setModuleState(swerveModuleStates[m.moduleNumber], Constants.SwerveConstants.kOpenLoop);
        }
    }
  
    /**
     * Zeros the navX
     */
    public void zeroGyro(){
      gyro.zeroYaw();
    }

    /**
     * Returns the gyro's yaw
     * 
     * @return Yaw of gyro, includes zeroing
     */
    public Rotation2d getYaw(){
      return Rotation2d.fromDegrees(-1 * gyro.getYaw());
    }
    // path planner stuff was here

    /** 
     * Gets swere modules positions for all modules
     * 
     * @return Array of module positions in order of module IDs
     */
    public SwerveModulePosition[] getModulePositions(){
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for (SwerveModule mod : modules){
        positions[mod.moduleNumber] = mod.getPosition();
      }
      return positions;
    }

    public Pose2d getPose(){
      return RobotContainer.getLocalizedPose.get();
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
      return Constants.SwerveConstants.kinematics.toChassisSpeeds(getStates());
    }
  
    /**
     * Get swerve STATES for all modules
     * 
     * @return Array of Modules states in order of module IDs
     */
    public SwerveModuleState[] getStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (SwerveModule mod: modules) {
        states[mod.moduleNumber] = mod.getState();
      }
      return states;
    }

    public void resetAllModulesToAsbol(){
      for (SwerveModule m : modules){
        m.setIntegratedAngleToAbsolute();
      }
    }

    public void resetAllModules(){
      for (SwerveModule m : modules){
        m.configureAngleMotor();
        m.configureDriveMotor();
      }
    }
  }
