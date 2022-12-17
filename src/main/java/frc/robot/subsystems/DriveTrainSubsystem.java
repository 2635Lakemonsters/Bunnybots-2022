// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import frc.robot.Robot;

public class DriveTrainSubsystem extends SubsystemBase {

  private WPI_TalonSRX frontRightMotor;
  private WPI_TalonSRX backRightMotor;
  private WPI_TalonSRX frontLeftMotor;
  private WPI_TalonSRX backLeftMotor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public DifferentialDrive differentialDrive;

  public MotorControllerGroup leftMotors;
  public MotorControllerGroup rightMotors;

  // default auto speeds, will be changed in DriveTrainCommand

  // REVIEW THIS SECTION - MEG
  private double leftSpeedAuto = 0;
  private double rightSpeedAuto = 0;

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    frontRightMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_DRIVE_CHANNEL);
    backRightMotor = new WPI_TalonSRX(Constants.BACK_RIGHT_DRIVE_CHANNEL);
    frontLeftMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_DRIVE_CHANNEL);
    backLeftMotor = new WPI_TalonSRX(Constants.BACK_LEFT_DRIVE_CHANNEL);

    leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
    rightMotors = new  MotorControllerGroup(frontRightMotor, backRightMotor);

    // differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isInAuto) {
      // System.out.println("DT distance AUTO = " + m_colorSensor.getProximity());
      this.leftMotors.set(-this.leftSpeedAuto);
      this.rightMotors.set(this.rightSpeedAuto);
    } else {
      // System.out.println("DT distance TELE = " + m_colorSensor.getProximity());
      this.leftMotors.set(-RobotContainer.leftJoystick.getY());
      this.rightMotors.set(RobotContainer.rightJoystick.getY());    
    }
  }

  public void setAutoSpeeds(double leftspeed, double rightspeed) {
    this.leftSpeedAuto = leftspeed;
    this.rightSpeedAuto = rightspeed; 
    // System.out.println("setting auto speed L: " + leftSpeedAuto + " R: " + rightSpeedAuto);
  }
  /*
  public void driveAuto(double leftSpeed, double rightSpeed) {
    System.out.println("leftSpeed:" + leftSpeed);
    System.out.println("rightSpeed:" + rightSpeed);
    differentialDrive.tankDrive(-leftSpeed, rightSpeed);
    // System.out.println("drive auto");
  }
  */
}
