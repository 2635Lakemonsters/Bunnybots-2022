// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakePneumaticSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveForwardExtendIntake extends SequentialCommandGroup {
  /** Creates a new AutoDriveForwardExtendIntake. */
  public AutoDriveForwardExtendIntake(DriveTrainSubsystem ds, IntakePneumaticSubsystem ips) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveTrainCommand(ds, 0.2, 0.2).withTimeout(2),
      new IntakePneumaticCommandOut(ips)
    );
  }
}
