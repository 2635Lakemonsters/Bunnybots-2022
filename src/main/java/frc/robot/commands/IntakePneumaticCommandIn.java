// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePneumaticSubsystem;

public class IntakePneumaticCommandIn extends CommandBase {
  IntakePneumaticSubsystem m_intakePneumaticSubsystem;
  public boolean isExtended;

  /** Creates a new IntakePneumaticCommand. */
  public IntakePneumaticCommandIn(IntakePneumaticSubsystem intakePneumaticSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakePneumaticSubsystem = intakePneumaticSubsystem;
    addRequirements(m_intakePneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isExtended = m_intakePneumaticSubsystem.isOpen();
    if(isExtended){ //if extended then retract
      m_intakePneumaticSubsystem.intakeIn();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
