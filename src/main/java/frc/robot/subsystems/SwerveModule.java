// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  private final CANCoder m_driveEncoder;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);  //need to characterize drivetrain, using same setup as a tank drive

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(                      //these also need characterized, but differently, and I don't know exactly how
          1, //kP
          0, //kI
          0, //kD
          new TrapezoidProfile.Constraints(
              DriveConstants.kModuleMaxAngularVelocity, DriveConstants.kModuleMaxAngularAcceleration));

  SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
    DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
          
  SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
    DriveConstants.ksTurning, DriveConstants.kvTurning, DriveConstants.kaTurning);

  /** Creates a new SwerveModule. */
  public SwerveModule(int angleMotorChannel, int driveMotorChannel, int turningEncoderChannel, int driveEncoderChannel, boolean encoderReversed, boolean driveReversed) {
    m_angleMotor = new WPI_TalonFX(angleMotorChannel);
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turnEncoder = new CANCoder(turningEncoderChannel);
    m_driveEncoder = new CANCoder(driveEncoderChannel);
    m_driveMotor.setInverted(driveReversed);
    m_angleMotor.setInverted(encoderReversed);
  }

  public void setDesiredState(SwerveModuleState moduleState){
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
        SwerveModuleState.optimize(moduleState, new Rotation2d(m_turnEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turnEncoder.getPosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_angleMotor.setVoltage(turnOutput + turnFeedforward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
