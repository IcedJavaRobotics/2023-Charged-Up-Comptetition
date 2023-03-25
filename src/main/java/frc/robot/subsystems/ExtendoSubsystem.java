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

  private double kP = 0.0002; // PID values
  private double kI = 0.00001;
  private double kD = 0.00005;

  double upperLimit = 22750;

  private TalonSRX extendoMotor = new TalonSRX(Constants.EXTENDO_MOTOR); // motor
  DigitalInput extendoLimitSwitch = new DigitalInput(Constants.EXTENDO_LIMIT_SWITCH); // limit switch
  PIDController extendoController = new PIDController(kP, kI, kD); // PID controller
  DigitalInput rightLimit;
  DigitalInput leftLimit;

  public ExtendoSubsystem(DigitalInput rightSwitch, DigitalInput leftSwitch) {

    rightLimit = rightSwitch;
    leftLimit = leftSwitch;
    extendoMotor.setNeutralMode(NeutralMode.Brake);
    extendoController.setTolerance(400, 1000); // 1000 is default
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
    if (rightLimit.get() && leftLimit.get()){
      if (extendoMotor.getSelectedSensorPosition() <= upperLimit) {
        extendoMotor.set(ControlMode.PercentOutput, Constants.EXTENDO_SPEED);
      } else {
        extendoStop();
      }
    } else {
      extendoStop();
    }
    

  }

  public void extendoRetract() {
    if (extendoLimitSwitch.get()) {
      extendoMotor.set(ControlMode.PercentOutput, -Constants.EXTENDO_RETRACT_SPEED);
    } else {
      extendoStop();
      extendoMotor.setSelectedSensorPosition(0);
    }
  }

  public boolean extendoReturn() {
    if (extendoLimitSwitch.get() == true) {
      extendoMotor.set(ControlMode.PercentOutput, -0.7);
      return false;
    } else {
      extendoStop();
      extendoMotor.setSelectedSensorPosition(0);  //qqqqqqqqqqqqqqqqqqqqqqqqqq22TODO add this line to other extendoStop();
      return true;
    }
  }

  /********** Set scoring extendo positions **********/

  // Has methods for upper and middle cones and cubes

  public void extendoTucked() {
      extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_TUCKED));
  }

  public void extendoPickup() {
      extendoMotor.set(ControlMode.PercentOutput,
          extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_PICKUP));
  }

  public void extendoMidGrid() {
      extendoMotor.set(ControlMode.PercentOutput,
          extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_MID_GRID));
  }

  public void extendoHighGrid() {
      extendoMotor.set(ControlMode.PercentOutput,
          extendoController.calculate(extendoMotor.getSelectedSensorPosition(), Constants.EXTENDO_HIGH_GRID));
  }
  

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