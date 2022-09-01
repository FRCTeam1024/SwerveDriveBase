// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive extends SubsystemBase {

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); //need to get size of chassis
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule a = new SwerveModule(angleMotorChannel, driveMotorChannel, turningEncoderChannel, driveEncoderChannel, encoderReversed, driveReversed);
  private final SwerveModule b = new SwerveModule(angleMotorChannel, driveMotorChannel, turningEncoderChannel, driveEncoderChannel, encoderReversed, driveReversed);
  private final SwerveModule c = new SwerveModule(angleMotorChannel, driveMotorChannel, turningEncoderChannel, driveEncoderChannel, encoderReversed, driveReversed);
  private final SwerveModule d = new SwerveModule(angleMotorChannel, driveMotorChannel, turningEncoderChannel, driveEncoderChannel, encoderReversed, driveReversed);
  
  private final WPI_PigeonIMU pigeon;
  
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, pigeon.getRotation2d());

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    pigeon = new WPI_PigeonIMU(DriveConstants.gyroID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] moduleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxSpeed);
    a.setDesiredState(moduleStates[0]);
    b.setDesiredState(moduleStates[1]);
    c.setDesiredState(moduleStates[2]);
    d.setDesiredState(moduleStates[3]);
  }

}
