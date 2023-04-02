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

  double kP = 0.01;
  double kI = 0.001;
  double kD = 0.005; // Deafault 0.002
  double upperLimit = 500; // 63:1 gearbox use 670, 45:1 485

  public final PIDController armController = new PIDController(kP, kI, kD);

  public ArmSubsystem() {

    armMotor.setInverted(true);
    armController.setTolerance(25, 5);
    armController.setIntegratorRange(-1, 1);

  }

  public void armJoystick(double Y) {

    SmartDashboard.putNumber("Neo Value", armMotor.getEncoder().getPosition());
    if (Math.abs(armMotor.getEncoder().getPosition()) < upperLimit && Y > 0) {
      armMotor.set(Y);
    } else if (armLimitSwtich.get() == false && Y < 0) {
      armMotor.set(Y);
    } else if (armLimitSwtich.get() == true) {
      stopArm();
      armMotor.getEncoder().setPosition(0);
    } else {
      stopArm();
    }

  }
  /**
   * raises arm
   * @param none literally nothing
   * @returns literally nothing
   */
  public void raiseArm() {

    if (Math.abs(armMotor.getEncoder().getPosition()) < upperLimit) {
      armMotor.set(Constants.ARM_SPEED);
    } else {
      stopArm();
    }

  }

  /**
   * If limit switch is pressed it stops the arms and zeros encoder
   * If limit swtich is not pressed it moves the arm down
   */
  public void lowerArm() {

    if(armLimitSwtich.get() == true) {
      stopArm();
      armMotor.getEncoder().setPosition(0);
    } else {
      armMotor.set(-Constants.ARM_SPEED);
    }

  }

  public void stopArm() {

    armMotor.stopMotor();

  }

  public void moveMotor(double speed, CANSparkMax sparkMax) {

    sparkMax.set(speed);

  }

  public void zeroEncoder() {

    armMotor.getEncoder().setPosition(0);

  }

  /********** Set arm scoring positions **********/

  public void armTucked() {
    if(armLimitSwtich.get() == true) {
      stopArm();
      zeroEncoder();
    } else {
      moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_TUCKED), armMotor);
    }
  }

  public void armPickup() {
    if(armLimitSwtich.get() == true) {
      stopArm();
      zeroEncoder();
    } else {
      moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_PICKUP), armMotor);
    }
  }

  public void armMidGrid() {
    if(armLimitSwtich.get() == true) {
      stopArm();
      zeroEncoder();
    } else {
      moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_MID_GRID), armMotor);
    }
  }

  public void armHighGrid() {
    if(armLimitSwtich.get() == true) {
      stopArm();
      zeroEncoder();
    } else {
      moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_HIGH_GRID), armMotor);
    }
  }
  

  // public boolean armUpperCube() {

  //   moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_UPPER_CUBE_SETPOINT),
  //       armMotor);

  //   if (armController.atSetpoint()) {
  //     return false;
  //   } else {
  //     return true;
  //   }
  // }

  // public boolean armMiddleCube() {

  //   moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_MIDDLE_CUBE_SETPOINT),
  //       armMotor);

  //   if (armController.atSetpoint()) {
  //     return false;
  //   } else {
  //     return true;
  //   }

  // }

  // public boolean armUpperCone() {

  //   moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_UPPER_CONE_SETPOINT),
  //       armMotor);

  //   if (armController.atSetpoint()) {
  //     return false;
  //   } else {
  //     return true;
  //   }
  // }

  // public boolean armMiddleCone() {

  //   moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), Constants.ARM_MIDDLE_CONE_SETPOINT),
  //       armMotor);

  //   if (armController.atSetpoint()) {
  //     return false;
  //   } else {
  //     return true;
  //   }

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Neo Value", armMotor.getEncoder().getPosition());

  }
}
