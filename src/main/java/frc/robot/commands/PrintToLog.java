// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utilities.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PrintToLog extends CommandBase {
  /** Creates a new PrintToLog. */
  double m_result;
  public PrintToLog() {
    double[] elements = {1001,1002,1003,1004,2005,2006,2007,2008,3009,3010,3011,3012};
    int rows = 3;
    int cols = 4;
    int offset = 0;
    int rowStride = 4;
    int colStride = 1;
    MatrixBuffer matrixBuffer = new MatrixBuffer(elements, rows, cols, offset, rowStride, colStride);
    Matrix matrix = new Matrix(matrixBuffer);
    double[] xs = {2,4,6};
    double[] ys = {10,11,12,13};
    double x = 2.1;
    double y = 10.1;
    double result = MathUtil.bilinearInterpolate(xs, ys, matrix, x, y);
   
    m_result = result;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Hello World!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    System.out.println("Result: " + m_result);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
