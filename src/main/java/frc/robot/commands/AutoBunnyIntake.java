// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBunnyIntake extends SequentialCommandGroup {
  /** Creates a new AutoBunnyIntake. */
  public AutoBunnyIntake(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup( //drive straight and spin intake for bunny
        new DriveTrainCommand(driveTrainSubsystem, Constants.AUTO_SPEED_1, Constants.AUTO_SPEED_1),
        new IntakeCommand(intakeSubsystem, Constants.SPIN_FREE)
      ).withTimeout(3), //need to test timeout
      new IntakeCommand(intakeSubsystem, !Constants.SPIN_FREE)
    );
  }
}
