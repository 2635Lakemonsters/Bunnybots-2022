// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePneumaticSubsystem extends SubsystemBase {
  /** Creates a new CompressorSubsystem. */
  DoubleSolenoid doubleSolenoid;

  public IntakePneumaticSubsystem() {
    doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL, Constants.REVERSE_CHANNEL);

    doubleSolenoid.set(kOff);
  }
  public boolean isOpen()
  {
    if (doubleSolenoid.get() == kReverse) {
      return true;
    } else
    {
      return false;
    }
    
  }
  public void intakeIn() {
		doubleSolenoid.set(kForward);

	}
	public void intakeOut() {
		doubleSolenoid.set(kReverse);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}