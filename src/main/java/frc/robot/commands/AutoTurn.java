// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoTurn extends CommandBase {
  /** Creates a new DriveTrainCommand. */
  private DriveTrainSubsystem m_driveSubsystem;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private double autoTurningSpeed;
  private boolean isGoingRight;
  private double initYaw;
  private double currentYaw;
  
  public AutoTurn(DriveTrainSubsystem driveSubsystem, double initTurningSpeed, boolean isGoingRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    autoTurningSpeed = initTurningSpeed;
    initYaw = RobotContainer.ahrs.getYaw();
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RightAngleTurnTest command initialize()");
    // m_driveSubsystem.setSpeed(Constants.AUTO_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isGoingRight){
      m_driveSubsystem.setAutoSpeeds(autoTurningSpeed, -autoTurningSpeed);
    } else {
      m_driveSubsystem.setAutoSpeeds(-autoTurningSpeed, autoTurningSpeed);
    }
    currentYaw = RobotContainer.ahrs.getYaw();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setAutoSpeeds(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(currentYaw - initYaw) >= 90){
      return true;
    }
    return false;
  }
}
