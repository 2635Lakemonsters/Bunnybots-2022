// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class NavxDriveCommand extends CommandBase {
  /** Creates a new NavxCommand. */
  private DriveTrainSubsystem m_driveSubsystem;
  private double autoSpeed;
  private double targetYaw;

  public NavxDriveCommand(DriveTrainSubsystem driveSubsystem, double initAutoSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    autoSpeed = initAutoSpeed;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Curretn Heading before command = " + RobotContainer.ahrs.getYaw());
    //caution TO-DO WARNING this effects the whole navx
    //future note: keep track of targets inside command
    RobotContainer.ahrs.zeroYaw();
    targetYaw = RobotContainer.ahrs.getYaw();
    Robot.isInAuto = true; //WARNING NEEDS TO BE FIXED!!!
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentYaw = RobotContainer.ahrs.getYaw();
    double yawError = currentYaw - targetYaw;
    double diffPow = (1.0 / 180.0) * yawError;
    if (diffPow > Constants.MAX_DIFF_POW) {
      diffPow = Constants.MAX_DIFF_POW;
    } else if (diffPow < (-1.0 * Constants.MAX_DIFF_POW)) {
      diffPow = (-1.0 * Constants.MAX_DIFF_POW);
    }
    double magnitude = Constants.AUTO_SPEED_2;
    double leftStickValue = magnitude + diffPow;
    double rightStickValue = magnitude - diffPow;
    m_driveSubsystem.setAutoSpeeds(leftStickValue, rightStickValue);
    System.out.println("RSV: " + rightStickValue);
    System.out.println("LSV: " + leftStickValue);
    System.out.println("MAG: " + magnitude);
    System.out.println("DIFFPOW: " + diffPow);
    System.out.println("YAW ERROR: " + yawError);
    System.out.println("TARG YAW: " + targetYaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}