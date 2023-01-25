// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendoSubsystem extends SubsystemBase {
  /** Creates a new ExtendoSubsystem. */

  private TalonSRX extendoMotor = new TalonSRX(Constants.EXTENDO_MOTOR);
  DigitalInput extendoLimit = new DigitalInput(Constants.EXTENDO_LIMIT_SWITCH);

  public ExtendoSubsystem() {

    //Change when testing
    extendoMotor.setInverted(null);

    extendoMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void ExtendoLower() {

  }

  public void ExtendoMiddle() {

  }

  public void ExtendoUpper() {

  }

  public void ExtendoDefault() {

    while()
  
  }

  

  private void ExtendoExtend() {

    extendoMotor.set(ControlMode.PercentOutput, Constants.EXTENDO_SPEED);

  }

  private void ExtendoRetract() {
    
    extendoMotor.set(ControlMode.PercentOutput, -Constants.EXTENDO_SPEED);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
