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

  double upperLimit = 24500;

  private TalonSRX extendoMotor = new TalonSRX(Constants.EXTENDO_MOTOR); // motor
  DigitalInput extendoLimitSwitch = new DigitalInput(Constants.EXTENDO_LIMIT_SWITCH); // limit switch
  PIDController extendoController = new PIDController(kP, kI, kD); // PID controller
  DigitalInput rightLimit;
  DigitalInput leftLimit;

  public ExtendoSubsystem(DigitalInput rightSwitch, DigitalInput leftSwitch) {

    rightLimit = rightSwitch;
    leftLimit = leftSwitch;
    extendoMotor.setNeutralMode(NeutralMode.Brake);
    extendoController.setTolerance(5, 10);
    extendoController.setIntegratorRange(-1, 1);

  }

  public void extendoJoystick(double I, double speed) {

    SmartDashboard.putBoolean("extendo limit switch", extendoLimitSwitch.get());
    if (I >= 0.5) {
      extendoExtend(speed);
    } else if (I <= -0.5) {
      extendoRetract(speed);
    } else {
      extendoStop();
    }

  }

  public void extendoStop() {

    extendoMotor.set(ControlMode.PercentOutput, 0);

  }

  public void extendoExtend(double speed) {

    System.out.println("extending");
    if (rightLimit.get() && leftLimit.get()) {
      if (extendoMotor.getSelectedSensorPosition() <= upperLimit) {
        extendoMotor.set(ControlMode.PercentOutput, speed);
      } else {
        extendoStop();
      }
    } else {
      extendoStop();
    }

  }

  public void extendoRetract(double speed) {
    if (extendoLimitSwitch.get()) {
      extendoMotor.set(ControlMode.PercentOutput, -speed);
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
      extendoMotor.setSelectedSensorPosition(0); // TODO add this line to other extendoStop();
      return true;
    }
  }

  /********** Set scoring extendo positions **********/

  // Has methods for upper and middle cones and cubes
  public boolean extendoSet(int setPoint) {

    extendoMotor.set(ControlMode.PercentOutput,
        extendoController.calculate(extendoMotor.getSelectedSensorPosition(), setPoint));

    if (extendoController.atSetpoint()) { // if it's at the setpoint return false, if it isn't return true
      return false;
    } else {
      return true;
    }

  }

  public boolean extendoUpperCube() {
    return extendoSet(Constants.EXTENDO_UPPER_CUBE_SETPOINT);
  }
  public boolean extendoMiddleCube() {
    return extendoSet(Constants.EXTENDO_MIDDLE_CUBE_SETPOINT);
  }
  public boolean extendoUpperCone() {
    return extendoSet(Constants.EXTENDO_UPPER_CONE_SETPOINT);
  }
  public boolean extendoMiddleCone) {
    return extendoSet(Constants.EXTENDO_MIDDLE_CONE_SETPOINT);
  }
  public boolean extendoMiddleCone() {
   return extendoSet(Constants.EXTENDO_MIDDLE_CONE_SETPOINT);
  }
  public boolean extendoDefault() {
    return extendoSet(Constants.DEFAULT_SETPOINT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("extendo encoder", extendoMotor.getSelectedSensorPosition());
  }
}
