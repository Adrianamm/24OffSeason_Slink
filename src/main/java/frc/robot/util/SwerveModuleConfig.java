package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfig {
    
    public int driveMotorID;
    public int angleMotorID;

    public Rotation2d absoluteEncoderOffset_Rad;

    public SwerveModuleConfig(int driveMotorID, int angleMotorID, Rotation2d absoluteEncoderOffset_Rad){
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;

        this.absoluteEncoderOffset_Rad = absoluteEncoderOffset_Rad;
    }
}
