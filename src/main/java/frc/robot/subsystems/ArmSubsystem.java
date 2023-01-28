// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private VictorSPX armMotor = new VictorSPX(Constants.ARM_VICTOR);
  DigitalInput armLimitSwtich = new DigitalInput(Constants.ARM_LIMIT_SWITCH);

  public ArmSubsystem() {

    //Change when testing
    armMotor.setInverted(null);

    armMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void RaiseArm() {

    

  }

  public void LowerArm() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
