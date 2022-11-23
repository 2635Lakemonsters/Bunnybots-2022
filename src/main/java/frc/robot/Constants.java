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
    public static final int FRONT_RIGHT_DRIVE_CHANNEL = 4;
    public static final int BACK_RIGHT_DRIVE_CHANNEL = 6;
    public static final int FRONT_LEFT_DRIVE_CHANNEL = 7;
    public static final int BACK_LEFT_DRIVE_CHANNEL = 5;
    public static final int INTAKE_MOTOR_CHANNEL = 2;

    public static final int RIGHT_JOYSTICK_CHANNEL = 0;
    public static final int LEFT_JOYSTICK_CHANNEL = 1;
    public static final int R_SPIN_INTAKE_FORWARD_BUTTON = 2;

    // Constants for intake subsystem
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final boolean kSensorPhase = true;
    public static final boolean kMotorInvert = false;
    
    //speed for driveTrainCommand autonomous 
    public static final double AUTO_SPEED_1 = -0.3;
    public static final double AUTO_SPEED_2 = -0.2;
}
