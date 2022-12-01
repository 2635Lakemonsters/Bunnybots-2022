// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import frc.robot.Robot;

public class DriveTrainSubsystem extends SubsystemBase {

  private TalonSRX frontRightMotor;
  private TalonSRX backRightMotor;
  private TalonSRX frontLeftMotor;
  private TalonSRX backLeftMotor;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    frontRightMotor = new TalonSRX(Constants.FRONT_RIGHT_DRIVE_CHANNEL);
    backRightMotor = new TalonSRX(Constants.BACK_RIGHT_DRIVE_CHANNEL);
    frontLeftMotor = new TalonSRX(Constants.FRONT_LEFT_DRIVE_CHANNEL);
    backLeftMotor = new TalonSRX(Constants.BACK_LEFT_DRIVE_CHANNEL);

    backRightMotor.follow(frontRightMotor);
    backLeftMotor.follow(frontLeftMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(!Robot.isInAuto){
      frontRightMotor.set(ControlMode.PercentOutput, RobotContainer.rightJoystick.getY());
      frontLeftMotor.set(ControlMode.PercentOutput, -RobotContainer.leftJoystick.getY());
    } else if(Robot.isInAuto){
      //uses rightsidespeed parameter and leftsidespeed parameter to set the speed
      //set front right to right sppeed param
      //set front left top left speeed param


      //if turn test isn't being used or called
      frontRightMotor.set(ControlMode.PercentOutput, rightSpeed);
      frontLeftMotor.set(ControlMode.PercentOutput, leftSpeed);
      // or turn test is being called 

      
    }
  }

  // public double getSpeed() {
  //   return speed;
  // }

  public void setSpeed(double newSpeed) {
    //for going straight
    rightSpeed = newSpeed;
    leftSpeed = -newSpeed;
  }

  public void setTurningSpeed(double newTurningSpeed){
    //for turning left or right based on if param is neg/pos
    rightSpeed = newTurningSpeed;
    leftSpeed = newTurningSpeed;
  }
}
