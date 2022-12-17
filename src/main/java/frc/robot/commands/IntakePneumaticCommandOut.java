// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePneumaticSubsystem;

public class IntakePneumaticCommandOut extends CommandBase {
  IntakePneumaticSubsystem m_intakePneumaticSubsystem;
  public boolean isExtended;

  /** Creates a new IntakePneumaticCommand. */
  public IntakePneumaticCommandOut(IntakePneumaticSubsystem intakePneumaticSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakePneumaticSubsystem = intakePneumaticSubsystem;
    addRequirements(m_intakePneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("intakePneumaticCommandOut INITIALIZE");
    isExtended = m_intakePneumaticSubsystem.isOpen();
    System.out.println(isExtended);
    if(!isExtended){ // if retracted then extend
      m_intakePneumaticSubsystem.intakeOut();
      System.out.println(":((((((((((((");
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
