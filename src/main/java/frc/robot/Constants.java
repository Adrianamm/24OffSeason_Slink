// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModuleConfig;

public final class Constants {
    /*
     * This class contains
     *  PID Stuff:
     *      A boolean to store if we are using PID
     *      PID values for translating the robot
     *      PID values for rotating the robot
     *  Max values:
     *      maxspeed for when robot drives in telepo
     *      "max angular rate in the fast mode" aka leftover stolen code
     * 
     *      speedlimit (fast and slow)
     *      accelerationlimit (fast and slow)
     *      angluarvelcoitylimit (fast and slow)
     *  Deadbands
     *      joystick deadbanding
     *  Physical wheel constants
     *      Trackwidth
     *      WheelBase
     *  SwerveDriveKinematics
     *      Helper class that helps convert chassis velcoty (x, y and angle)
     *      into indiviudal moudle states (speed and angle)
     *      It be fancy
     */
    public static class SwerveConstants {
        // True = using precentage
        // False = using PID
        public static final boolean kOpenLoop = true;

        // PID for translating (Slink numbers)
        public static final double kTranslateP = 1;
        public static final double KtranslateI = 0;
        public static final double kTranslateD = 0.1;

        // PID for turning (Slink numbers)
        public static final double kRotateP = 2;
        public static final double kRotateI = 0; // 0.0 for some reason
        public static final double kRotateD = 0;

        // Limiting Numbers (Slink numbers)
        public static final double kMaxSpeedTele_MPS = 3.0; //Meters pre Second

        public static final double speedlimit = 3.0;
        public static final double slowSpeedLimit = 0.5;

        public static final double accelerationlimit = 1.5;
        public static final double slowAccelerationlimit = 0.5;

        public static final double angluarvelcoitylimit = 180.0;
        public static final double slowAngluarvelcoitylimit = 45.0;

        // Deadband
        public static final double kStickDeadband = 0.01;

        // Physical wheel constants
        public static final double kTrackWidth_m = Units.inchesToMeters(20.5);
        public static final double kWheelBase_m = Units.inchesToMeters(20.5);

        // SwerveDriveKinematics
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // 4 trans2D because we have 4 modules 
            new Translation2d(kWheelBase_m / 2.0, kTrackWidth_m / 2.0),
            new Translation2d(kWheelBase_m / 2.0, -kTrackWidth_m / 2.0),
            new Translation2d(-kWheelBase_m / 2.0, kTrackWidth_m / 2.0),
            new Translation2d(-kWheelBase_m / 2.0, -kTrackWidth_m / 2.0)
        );
    }
    /*
     * Each class is one swerve moudle
     * Each Moudle contains
     *  Angle Motor
     *  Drive Motor
     */
    public static class Module{

        // Current limits avoid burn outs while driving
        public static class DriveCurrentLimit {
            
            // I do not understand but I will go back to this
            public static final double KlimitToAmps = 30.0f;
            public static final double kMaxSpikeTime = 20.0f;
            public static final double kMaxSpikeAmps = 35.0f;
            public static final int KSmartLimit = 35;
        }

        // Current limits avoid burn outs while turning
        public static class AngleCurrentLimits{

            // I do not understand but I will go back to this
            public static final double KlimitToAmps = 20.0f;
            public static final double kMaxSpikeTime = 25.0f;
            public static final double kMaxSpikeAmps = 20.0f;
            public static final int KSmartLimit = 20;
        }

        /*
         * Nesscary information about the Swerve Modules
         *  Gear Ratios
         *      Drive gear ratio
         *      Angle gear ratio
         *  Wheel sizes
         *      Diameter
         *      Circumfrence
         *  kDrivePositionConversionFactor
         *      Converts encoder position (rotations) to distance
         *      This is used to convert the motor's encoder position to something nice such as the distance the robot has traveled. 
         *      how much distance corrsponds to one rotation of the motor 
         *  KDriveVelocityConverstionFactor
         *      Converts encoder velocity (rpm) to speed (distance pre second)
         */
        
        public static final double kDriveGearRatio = 1.0f / 8.14f;
        public static final double kAngleGearRatio = 1.0f / 12.8f;

        public static final double kWheelDiameter_m = Units.inchesToMeters(4);
        public static final double kWheelCircumfrence = kWheelDiameter_m * Math.PI;

        // combines gear ratio and wheel circum to convert the encoder reading into the actual distance that the wheel has traveled
        public static final double kDrivePositionConversionFactor = kDriveGearRatio * kWheelCircumfrence;

        // divides kDrivePositionConversionFactor by 60 (seconds in a minute) to get robot speed in distance per second
        public static final double KDriveVelocityConverstionFactor = kDrivePositionConversionFactor / 60.0f;

        // coverts the encoder position for the angle motor
        public static final double kAnglePositionConversionFactor = kAngleGearRatio * 360;
        
        //
        public static final double kAngleVelocityConverstionFactor = kAnglePositionConversionFactor / 60.0f;

        // PID values for Drive (Slink numbers)
        public final double kDriveP = 0.2;
        public final double kDriveI = 0;
        public final double kDriveD = 3;

        // PID values for Angle (Slink numbers)
        public final double kAngleP = 0.05;
        public final double kAngleI = 0;
        public final double kAngleD = 0.002;

        // Feedforward values
        public static final double kDriveS = 0.375;
        public static final double kDriveV = 2.5;
        public static final double kDriveA = 0;

        // needed for when the SPARKMax wires were flipped
        public static final boolean angleMotorInverted = false;
        public static final boolean driveMotorInverted = false;
        public static final boolean KAbsoluteEncoderInverted = false;
    }

    /*
     * Each class is one swerve moudle
     * Each Moudle contains
     *  Angle Motor
     *  Drive Motor
     */

     public static class Modules {
        
        //Front left modules :)
        public static class FrontLeft {
            // Setting motor controller IDs
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;

            // When the wheel is pointing at zero, the number is the difference between the encoder reading and the actual 0 position
            public static final Rotation2d absoluteEncoderOffset_Rad = new Rotation2d(Math.toRadians(347.1482491493225));

            // creates Front Left 0 (FL0) using the SwerveModuleConfig file 
            public static final SwerveModuleConfig FL0 = new SwerveModuleConfig(driveMotorID,angleMotorID, absoluteEncoderOffset_Rad);
        }

        public static class FrontRight{
            // Setting motor controller IDs
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;

            public static final Rotation2d absoluteEncoderOffset_Rad = new Rotation2d(Math.toRadians(307));

            public static final SwerveModuleConfig FR1 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset_Rad);
        }

        public static class BackLeft {

            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;

            public static final Rotation2d absoluteEncoderOffset_Rad = new Rotation2d(Math.toRadians(57.84718573093414));

            public static final SwerveModuleConfig BL2 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset_Rad);
        }

        public static class BackRight {

            public static final int driveMotorID = 32;
            public static final int angleMotorID = 31;

            public static final Rotation2d absoluteEncoderOffset_Rad = new Rotation2d(Math.toRadians(256.65536928176877));

            public static final SwerveModuleConfig BR3 = new SwerveModuleConfig(driveMotorID, angleMotorID, absoluteEncoderOffset_Rad);
        }

     }

     public static final class SwerveMoveConstants{
        //
        public static final float posPosTolerance = 0.05f;
        
        //
        public static final float posVelTolerance = 0.1f;

        //
        public static final float aPosTolerance = 5f;
        
        //
        public static final float aVelTolerance = 2f;

        //
        public static final float aVelocityTolerance = 1;
     }


}