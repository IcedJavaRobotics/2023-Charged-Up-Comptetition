// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendoSubsystem extends SubsystemBase {
  /** Creates a new ExtendoSubsystem. */

  private TalonSRX extendoMotor = new TalonSRX(Constants.EXTENDO_MOTOR);
  DigitalInput extendoLimitSwitch = new DigitalInput(Constants.EXTENDO_LIMIT_SWITCH);

  public ExtendoSubsystem() {

    // Change when testing
    extendoMotor.setInverted(InvertType.None);

    extendoMotor.setNeutralMode(NeutralMode.Brake);

  }

  public Boolean extendoLower() {

    if (extendoMotor.getSelectedSensorPosition() <= Constants.LOW_BOUND_LOW_POS) {

      extendoExtend();
      return true;
    } else if (extendoMotor.getSelectedSensorPosition() >= Constants.HIGH_BOUND_LOW_POS) {

      extendoRetract();
      return true;
    } else {

      extendoStop();
      return false;
    }

  }

  public Boolean extendoMiddle() {

    if (extendoMotor.getSelectedSensorPosition() <= Constants.LOW_BOUND_MID_POS) {

      extendoExtend();
      return true;
    } else if (extendoMotor.getSelectedSensorPosition() >= Constants.HIGH_BOUND_MID_POS) {

      extendoRetract();
      return true;
    } else {

      extendoStop();
      return false;
    }
    
  }

  public boolean extendoUpper() {             //does this

    if (extendoMotor.getSelectedSensorPosition() <= Constants.LOW_BOUND_HIGH_POS) {

      extendoExtend();
      return true;

    } else if (extendoMotor.getSelectedSensorPosition() >= Constants.HIGH_BOUND_HIGH_POS) {

      extendoRetract();
      return true;

    } else {

      extendoStop();

    }
    return false;
  }

  public Boolean extendoDefault() {                //does this

    if (extendoMotor.getSelectedSensorPosition() <= Constants.LOW_BOUND_DEFAULT_POS) {

      extendoExtend();
      return true;

    } else if (extendoMotor.getSelectedSensorPosition() >= Constants.HIGH_BOUND_DEFAULT_POS) {

      extendoRetract();
      return true;

    } else {

      extendoStop();
      return false;
    }

  }

  public void extendoExtend() {

    extendoMotor.set(ControlMode.PercentOutput, Constants.EXTENDO_SPEED);

  }

  public void extendoRetract() {

    extendoMotor.set(ControlMode.PercentOutput, -Constants.EXTENDO_SPEED);

  }

  public void extendoStop() {

    extendoMotor.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}