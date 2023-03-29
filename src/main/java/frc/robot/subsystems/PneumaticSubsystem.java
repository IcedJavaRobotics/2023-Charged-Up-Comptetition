// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.CTREPCMJNI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
  /** Creates a new PnematicWheelsSubsystem. */
  DoubleSolenoid doubleDriveSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid doubleAutoSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  public PneumaticSubsystem() {
    doubleDriveSolenoid.set(DoubleSolenoid.Value.kReverse);
    doubleAutoSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void forwardDriveSolenoid() {
    doubleDriveSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void reverseDriveSolenoid() {
    doubleDriveSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void forwardAutoSolenoid() {
    doubleAutoSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void reverseAutoSolenoid() {
    doubleAutoSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
