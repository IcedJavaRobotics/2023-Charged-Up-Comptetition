// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private TalonSRX armMotor = new TalonSRX(Constants.ARM_TALON);
  DigitalInput armLimitSwtich = new DigitalInput(Constants.ARM_LIMIT_SWITCH);

  double kP = 0;
  double kI = 0;
  double kD = 0;
  int bottomValue = 0;
  int middleValue = 0;
  int highValue = 0;
  double upperlimit = 0;

  public final PIDController armController = new PIDController(kP, kI, kD);

  public ArmSubsystem() {

    //Change when testing
    armMotor.setInverted(InvertType.None);

    armMotor.setNeutralMode(NeutralMode.Brake);

  }

  public boolean bottomArm() {

    moveMotor( armController.calculate( armMotor.getSelectedSensorPosition(), bottomValue ), armMotor);
    return false;
  }

  public boolean middleArm() {

    moveMotor( armController.calculate( armMotor.getSelectedSensorPosition(), middleValue ), armMotor);
    return false;
  }

  public boolean highArm() {
    moveMotor( armController.calculate( armMotor.getSelectedSensorPosition(), highValue ), armMotor);
    return false;
  }

  public void moveMotor( double speed, TalonSRX talon) {

    talon.set(ControlMode.PercentOutput, speed);

  }

  public void raiseArm() {

    // if (encoder < upperlimit) {
      armMotor.set(ControlMode.PercentOutput, Constants.ARM_SPEED);
    // } else {
    //   stopArm();
    // }

  }

  public void lowerArm() {

    if (armLimitSwtich.get() == false) {
      armMotor.set(ControlMode.PercentOutput, -Constants.ARM_SPEED);
    } else {
      stopArm();
      //set encoder to 0
    }

  }

  public void stopArm() {

    armMotor.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}