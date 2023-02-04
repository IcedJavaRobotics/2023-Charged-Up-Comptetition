// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private VictorSPX armMotor = new VictorSPX(Constants.ARM_VICTOR);
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
    armMotor.setInverted(null);

    armMotor.setNeutralMode(NeutralMode.Brake);

  }

  public void bottomArm() {

    // moveMotor( armController.calculate( encoder value, bottomValue ), armMotor);

  }

  public void middleArm() {

    // moveMotor( armController.calculate( encoder value, middleValue ), armMotor);

  }

  public boolean highArm() {
    return true;
    // moveMotor( armController.calculate( encoder value, highValue ), armMotor);
    return false;
  }

  public void moveMotor( double speed, VictorSPX victor) {

    victor.set(ControlMode.PercentOutput, speed);

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