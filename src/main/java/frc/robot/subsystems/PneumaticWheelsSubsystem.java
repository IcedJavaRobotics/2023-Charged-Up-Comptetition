// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CTREPCMJNI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticWheelsSubsystem extends SubsystemBase {
  /** Creates a new PnematicWheelsSubsystem. */
  DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  public PneumaticWheelsSubsystem() {
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleSolenoid() {
    doubleSolenoid.toggle();
  }

  public void forwardSolenoid() {
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void reverseSolenoid() {
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
