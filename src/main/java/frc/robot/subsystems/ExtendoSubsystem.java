// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  double upperLimit = 25000;

  private TalonSRX extendoMotor = new TalonSRX(Constants.EXTENDO_MOTOR); // motor
  DigitalInput extendoLimitSwitch = new DigitalInput(Constants.EXTENDO_LIMIT_SWITCH); // limit switch
  PIDController extendoController = new PIDController(kP, kI, kD); // PID controller

  public ExtendoSubsystem() {

    extendoMotor.setNeutralMode(NeutralMode.Brake);
    extendoController.setTolerance(5, 10);
    extendoController.setIntegratorRange(-1, 1);

  }

  public void extendoJoystick(double I) {

    SmartDashboard.putBoolean("extendo limit switch", extendoLimitSwitch.get());
    if (I >= 0.5) {
      extendoExtend();
    } else if (I <= -0.5) {
      extendoRetract();
    } else {
      extendoStop();
    }

  }

  public void extendoStop() {

    extendoMotor.set(ControlMode.PercentOutput, 0);

  }

  public void extendoExtend() {

    System.out.println("extending");
    if (extendoMotor.getSelectedSensorPosition() <= upperLimit) {
      extendoMotor.set(ControlMode.PercentOutput, Constants.EXTENDO_SPEED);
    } else {
      extendoStop();
    }

  }

  public void extendoRetract() {
    if (extendoLimitSwitch.get() == false) {
      extendoMotor.set(ControlMode.PercentOutput, -Constants.EXTENDO_SPEED);
    } else {
      extendoStop();
      extendoMotor.setSelectedSensorPosition(0);
    }
  }

  public boolean extendoReturn() {
    if (extendoLimitSwitch.get() == false) {
      extendoMotor.set(ControlMode.PercentOutput, -0.7);
      return false;
    } else {
      extendoStop();
      extendoMotor.setSelectedSensorPosition(0);
      return true;
    }
  }

  /********** Set scoring extendo positions **********/

  // Has methods for upper and middle cones and cubes

  public boolean extendoUpperCube() {

    extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_UPPER_CUBE_SETPOINT));

    if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
      return false;
    } else {
      return true;
    }

  }

  public Boolean extendoMiddleCube() {

    extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_MIDDLE_CUBE_SETPOINT));

    if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
      return false;
    } else {
      return true;
    }

  }

  public boolean extendoUpperCone() {

    extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_UPPER_CONE_SETPOINT));

    if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
      return false;
    } else {
      return true;
    }

  }

  public Boolean extendoMiddleCone() {

    extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_MIDDLE_CONE_SETPOINT));

    if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
      return false;
    } else {
      return true;
    }

  }

  public boolean extendoDefault() {

    extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.DEFAULT_SETPOINT));

    if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
      return false;
    } else {
      return true;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("extendo encoder", extendoMotor.getSelectedSensorPosition());
  }
}