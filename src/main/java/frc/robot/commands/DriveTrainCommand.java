// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainCommand extends CommandBase {
  /** Creates a new DriveTrainCommand. */
  private DriveTrainSubsystem m_driveSubsystem;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private double leftAutoSpeed;
  private double rightAutoSpeed;
  
  public DriveTrainCommand(DriveTrainSubsystem driveSubsystem, double l_initAutoSpeed, double r_initAutoSpeed) {
    this.m_driveSubsystem = driveSubsystem;
    this.leftAutoSpeed = l_initAutoSpeed;
    this.rightAutoSpeed = r_initAutoSpeed; 

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Drivetrain command initialize()");
    // m_driveSubsystem.setSpeed(Constants.AUTO_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_driveSubsystem.driveAuto(leftAutoSpeed, rightAutoSpeed);
    // System.out.println("distance = " + m_colorSensor.getProximity());
    // if (m_colorSensor.getProximity() >= 110) {
    //   this.end(true);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.driveAuto(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //tested color sensor with wood plank w/ led ON @ lab
    //WARNING: shoudl be calibrated @ field
    //105      @ 5"
    //120      @ 4"
    //135      @ 3"
    //190      @ 2"
    //450      @ 1"
    //750-1100 @ 0.5"
    if (m_colorSensor.getProximity() >= 130) {
      return true;
    }
    return false;
  }
}
