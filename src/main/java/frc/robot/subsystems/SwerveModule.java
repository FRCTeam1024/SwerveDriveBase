// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  private static final double kWheelRadius = 0.0508; //need to double check the actual wheel radius
  private static final int kEncoderResolution = 4096;

  private final WPI_TalonFX m_angleMotor;
  private final WPI_TalonFX m_driveMotor;

  private final CANCoder m_turnEncoder;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);  //need to characterize drivetrain, using same setup as a tank drive

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(                      //these also need characterized, but differently, and I don't know exactly how
          1, //kP
          0, //kI
          0, //kD
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
    DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
          
  SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
    DriveConstants.ksTurning, DriveConstants.kvTurning, DriveConstants.kaTurning);

  /** Creates a new SwerveModule. */
  public SwerveModule(int angleMotorChannel, int driveMotorChannel, int turningEncoderChannel, boolean encoderReversed, boolean driveReversed) {
    m_angleMotor = new WPI_TalonFX(angleMotorChannel);
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turnEncoder = new CANCoder(turningEncoderChannel);

    m_driveMotor.setInverted(driveReversed);
    m_angleMotor.setInverted(encoderReversed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
