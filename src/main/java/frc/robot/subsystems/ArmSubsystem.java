// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private CANSparkMax armMotor = new CANSparkMax(Constants.ARM_SPARK, MotorType.kBrushless);
  DigitalInput armLimitSwtich = new DigitalInput(Constants.ARM_LIMIT_SWITCH);

  double kP = 0;
  double kI = 0;
  double kD = 0;
  double upperLimit = 275;

  public final PIDController armController = new PIDController(kP, kI, kD);

  public ArmSubsystem() {

    armMotor.setInverted(true);
    armController.setTolerance(5, 10);
    armController.setIntegratorRange(-1, 1);

  }

  public void armJoystick(double I) {

    SmartDashboard.putNumber("Neo Value", armMotor.getEncoder().getPosition());
    if (I >= 0.5) {
      lowerArm();
    } else if (I <= -0.5) {
      raiseArm();
    } else {
      stopArm();
    }

  }

  public void raiseArm() {

    if (Math.abs(armMotor.getEncoder().getPosition()) < upperLimit) {
      armMotor.set(Constants.ARM_SPEED);
    } else {
      stopArm();
    }

  }

  public void lowerArm() {

    armMotor.set(-Constants.ARM_SPEED);

  }

  public void stopArm() {

    armMotor.set(0);

  }

  public void moveMotor(double speed, CANSparkMax sparkMax) {

    sparkMax.set(speed);

  }

  public void zeroEncoder() {

    armMotor.getEncoder().setPosition(0);

  }

  /********** Set arm scoring positions **********/

  public boolean armUpperCube() {

    moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_UPPER_CUBE_SETPOINT),
        armMotor);

    if (armController.atSetpoint()) {
      return false;
    } else {
      return true;
    }
  }

  public boolean armMiddleCube() {

    moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_MIDDLE_CUBE_SETPOINT),
        armMotor);

    if (armController.atSetpoint()) {
      return false;
    } else {
      return true;
    }

  }

  public boolean armUpperCone() {

    moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_UPPER_CONE_SETPOINT),
        armMotor);

    if (armController.atSetpoint()) {
      return false;
    } else {
      return true;
    }
  }

  public boolean armMiddleCone() {

    moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_MIDDLE_CONE_SETPOINT),
        armMotor);

    if (armController.atSetpoint()) {
      return false;
    } else {
      return true;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Neo Value", armMotor.getEncoder().getPosition());

  }
}
