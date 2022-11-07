// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
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


//DP:  I am wondering now if we need to have our own swerve module class.  There is SwerveDriveSpecialties 
//library that provides a module class.  Either way this class would not be 
//a subsystem on its own so wouldnt extend SubsystemBase.  It is just a helper class to the swervedrive 
//subsystem.  
public class SwerveModule {
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
              DriveConstants.kModuleMaxAngularVelocity, DriveConstants.kModuleMaxAngularAcceleration));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
    DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
          
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(
    DriveConstants.ksTurning, DriveConstants.kvTurning, DriveConstants.kaTurning);

  private SwerveModuleState state;

  /** Creates a new SwerveModule. */
  public SwerveModule(int angleMotorChannel, int driveMotorChannel, int turnEncoderChannel, 
    double turnOffset, boolean turnReversed, boolean driveReversed) {

    m_angleMotor = new WPI_TalonFX(angleMotorChannel);
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turnEncoder = new CANCoder(turnEncoderChannel);

    //Always start from known settings
    m_angleMotor.configFactoryDefault();
    m_driveMotor.configFactoryDefault();
    m_turnEncoder.configFactoryDefault();

    //Set motor directions based on parameters
    m_driveMotor.setInverted(driveReversed);
    m_angleMotor.setInverted(turnReversed);

    //Set how we want to the encoder to read
    m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turnEncoder.configMagnetOffset(turnOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  public void setDesiredState(SwerveModuleState moduleState){
    // Optimize the reference state to avoid spinning further than 90 degrees
    state = SwerveModuleState.optimize(moduleState, new Rotation2d(m_turnEncoder.getAbsolutePosition()*Math.PI/180));

    // Calculate the drive output from the drive PID controller.
    // 
    // DP: I think we can configure the selected sensor in the constructor with the desired resolution so we done have 
    // to perform that math here.  Also to note, SwerveModuleState expects velocity in m/s, so wheel diameter woul
    // need to be include in this conversion somehow.
    //
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity()*(10*2*Math.PI/2048), state.speedMetersPerSecond);

    //DP: should we be passing the curent setpoint of the drivePID as the desired speed rather than the state speed?
    // I guess this may not matter since it is not a profiled PID controller those values should be the same?
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turnEncoder.getAbsolutePosition()*Math.PI/180, state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_angleMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public double getDriveVelocity(){
    return m_driveMotor.getSelectedSensorVelocity()*(10*2*Math.PI/2048);
  }

  public double getAngleRadians(){
    return m_turnEncoder.getAbsolutePosition()*Math.PI/180;
  } 

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity()*(10*2*Math.PI/2048),
                new Rotation2d(m_turnEncoder.getAbsolutePosition()*Math.PI/180));
  }

  public SwerveModuleState getTargetState() {
    return state;
  }

  public double getTargetAngleRadians(){
    return state.angle.getRadians();
  }
}
