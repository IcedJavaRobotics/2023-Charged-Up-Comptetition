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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendoSubsystem extends SubsystemBase {
  /** Creates a new ExtendoSubsystem. */

  private double kP = 0; // PID values
  private double kI = 0;
  private double kD = 0;

  double upperLimit = 10000;

  private TalonSRX extendoMotor = new TalonSRX(Constants.EXTENDO_MOTOR); // motor
  DigitalInput extendoLimitSwitch = new DigitalInput(Constants.EXTENDO_LIMIT_SWITCH); // limit switch
  PIDController extendoController = new PIDController(kP, kI, kD); // PID controller

  public ExtendoSubsystem() {

    extendoController.setTolerance(5, 10);
    extendoController.setIntegratorRange(-1, 1);

  }

  public void extendoExtend() {

    System.out.println("extending");
    // if (extendoMotor.getSelectedSensorPosition() <= upperLimit) {
    extendoMotor.set(ControlMode.PercentOutput, Constants.EXTENDO_SPEED);
    // } else {
    // extendoStop();
    // }

  }

  public void extendoJoystick(double I) {

    if (I >= 0.5) {
      extendoExtend();
    } else if (I <= -0.5) {
      extendoRetract();
      // SmartDashboard.putNumber("extendo encoder", extendoMotor.getSelectedSensorPosition());
    } else {
      extendoStop();
    }

  }

  public void extendoStop() {

    extendoMotor.set(ControlMode.PercentOutput, 0);

  }

  public void extendoRetract() {
    // System.out.println("Rectracting");
    // if (extendoLimitSwitch.get() == false) {
    extendoMotor.set(ControlMode.PercentOutput, -Constants.EXTENDO_SPEED);
    // }else {
    // extendoStop();
    // extendoMotor.setSelectedSensorPosition(0);
    // }
  }

  public Boolean extendoLower() { // extend to the lower goal method
/*
    extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.LOW_SETPOINT));

    // I don't know if this will go up AND down or just one of the two

    // if (encoder.getDistance() > Constants.LOW_BOUND_SETPOINT) { //if the current
    // distance is more than the setpoint

    // extendoMotor.set(ControlMode.PercentOutput,
    // extendoController.calculate(encoder.getDistance(),
    // Constants.LOW_BOUND_SETPOINT));

    // }

    if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
      return false;
    } else {
      return true;
    }

    // if (extendoMotor.getSelectedSensorPosition() <= Constants.LOW_BOUND_LOW_POS)
    // {

    // extendoExtend();
    // return true;
    // } else if (extendoMotor.getSelectedSensorPosition() >=
    // Constants.HIGH_BOUND_LOW_POS) {

    // extendoRetract();
    // return true;
    // } else {

    // extendoStop();
    // return false;
    // }
*/
return false;
  }

  public Boolean extendoMiddle() {

    // extendoMotor.set(ControlMode.PercentOutput,
    //     extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.MID_SETPOINT));

    // if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
    //   return false;
    // } else {
      return true;
    // }

  }

  public boolean extendoUpper() {

    // extendoMotor.set(ControlMode.PercentOutput,
    //     extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.HIGH_SETPOINT));

    // if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
    //   return false;
    // } else {
      return true;
    // }

  }

  public boolean extendoDefault() {

    // extendoMotor.set(ControlMode.PercentOutput,
    //     extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.DEFAULT_SETPOINT));

    // if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
    //   return false;
    // } else {
      return true;
    // }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}