// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */

  private VictorSPX rightClawMotor = new VictorSPX(Constants.RIGHT_CLAW);
  private VictorSPX leftClawMotor = new VictorSPX(Constants.LEFT_CLAW);
  DigitalInput rightLimit;
  DigitalInput leftLimit;
 
  public ClawSubsystem(DigitalInput rightSwitch, DigitalInput leftSwitch) {

    rightLimit = rightSwitch;
    leftLimit = leftSwitch;
    // Need to set this during testing
    rightClawMotor.setInverted(true);
    leftClawMotor.setInverted(true);

  }

  public void clawClose(double clawSpeed) {

    // Arms fold in
    leftClawMotor.set(ControlMode.PercentOutput, -clawSpeed);
    rightClawMotor.set(ControlMode.PercentOutput, -clawSpeed);

  }

  public Boolean clawOpen(double clawSpeed) {
    
    // Arms fold out until limit switch is hit
    if (leftLimit.get()) {

      leftClawMotor.set(ControlMode.PercentOutput, clawSpeed);

    } else {

      leftClawMotor.set(ControlMode.PercentOutput, 0);

    }
    if (rightLimit.get()) {

      rightClawMotor.set(ControlMode.PercentOutput, clawSpeed);

    } else {

      rightClawMotor.set(ControlMode.PercentOutput, 0);

    }

    // if (leftLimit.get() && rightLimit.get()) {
    // return false; // returns false when done.
    // } else {
    return true; // returns true when not done. duh.
    // }

  }

  public void clawStop() {

    // Stops claw
    leftClawMotor.set(ControlMode.PercentOutput, 0);
    rightClawMotor.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Right limit", rightLimit.get());
    SmartDashboard.putBoolean("Left limit", leftLimit.get());
  }
}
