// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticWheelsSubsystem extends SubsystemBase {
  /** Creates a new PnematicWheelsSubsystem. */
  DoubleSolenoid leftDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  DoubleSolenoid rightDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

  Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);

  public PneumaticWheelsSubsystem() {
    leftDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleSolenoid() {
    leftDoubleSolenoid.toggle();
    rightDoubleSolenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
