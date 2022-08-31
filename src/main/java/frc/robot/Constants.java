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
    public static final class DriveConstants {
        public static final double ksVolts = 0.73394;  //all need to be characterized by the SysId tool
        public static final double kvVoltSecondsPerMeter = 2.4068;
        public static final double kaVoltSecondsSquaredPerMeter = 0.28749;

        public static final double ksTurning = 0.77; //also need to be characterized, not certain about how we do so
        public static final double kvTurning = 0.75; 
        public static final double kaTurning = 0; 
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
    }
}
