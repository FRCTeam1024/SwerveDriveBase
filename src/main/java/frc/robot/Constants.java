// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double PI = 3.14159;
    public static final boolean CompBot = Robot.isCompBot();

    public static final int PCMID = 3;  // CAN ID for PCM

    // IDs for physical input devices
    // Make sure order matches that of DriverStation
    public static final class Inputs {
    public static final int driverControllerID = 0; // ID for Xbox/Logitech controller
    public static final int operatorControllerID = 1;
    }

    //Swerve Drive Drivetrain Related Constants
    public static final class DriveConstants {
        public static final double ksVolts = 0.73394;  //all need to be characterized by the SysId tool
        public static final double kvVoltSecondsPerMeter = 2.4068;
        public static final double kaVoltSecondsSquaredPerMeter = 0.28749;

        public static final double ksTurning = 0.77; //also need to be characterized, not certain about how we do so
        public static final double kvTurning = 0.75; 
        public static final double kaTurning = 0; 
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2; //Math.PI * 2

        //no numbers below this are accurate, everything must be configure/found
        public static final double kModuleMaxAngularVelocity = Math.PI / 2; // Math.PI
        public static final double kModuleMaxAngularAcceleration = Math. PI / 2; //Math.PI * 2

        public static final int gyroID = 01;

        public static final int kMaxSpeed = 1; //3

        public static final int angleMotorA = 20;
        public static final int angleMotorB = 30;
        public static final int angleMotorC = 40;
        public static final int angleMotorD = 50;

        public static final int driveMotorA = 21;
        public static final int driveMotorB = 31;
        public static final int driveMotorC = 41;
        public static final int driveMotorD = 51;

        public static final int turnEncoderA = 22;
        public static final int turnEncoderB = 32;
        public static final int turnEncoderC = 42;
        public static final int turnEncoderD = 52;
    }

    public static final class LimelightConstants {
        public static final double driverPipe = 2.0;
        public static final double targetPipe = 1.0;
        public static final double offPipe = 0.0;
        
        public static final double kP = 0.015;
        public static final double kI = 0;
        public static final double kD = 0;

        //Reduced these for safety until we are comfortable withe autoaim command
        public static final double minOutput = -0.75;
        public static final double maxOutput = 0.75;
        // 1 degree angle of error which is considered tolerable for the PID
        public static final double threshold = 0.1;
    }
}
