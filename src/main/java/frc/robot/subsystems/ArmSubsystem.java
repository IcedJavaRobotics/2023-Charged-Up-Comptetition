// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private CANSparkMax armMotor = new CANSparkMax(Constants.ARM_SPARK, MotorType.kBrushless);
  DigitalInput armLimitSwtich = new DigitalInput(Constants.ARM_LIMIT_SWITCH);

  double kP = 0;
  double kI = 0;
  double kD = 0;
  int bottomValue = 0;
  int middleValue = 0;
  int highValue = 0;
  double upperLimit = 0;

  public final PIDController armController = new PIDController(kP, kI, kD);

  public ArmSubsystem() {

    // Change when testing
    armMotor.setInverted(false);

  }

  public void armJoystick(double I) {

    if (I >= 0.5) {
      raiseArm();
      System.out.println(armMotor.getEncoder().getPosition());
    } else if (I <= -0.5) {
      lowerArm();
      System.out.println(armMotor.getEncoder().getPosition());
    } else {
      stopArm();
    }

  }

  public void bottomArm() {

    moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), bottomValue), armMotor);

  }

  public void middleArm() {

    moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), middleValue), armMotor);

  }

  public void highArm() {

    moveMotor(armController.calculate(armMotor.getEncoder().getPosition(), highValue), armMotor);

  }

  public void moveMotor(double speed, CANSparkMax sparkMax) {

    sparkMax.set(speed);

  }

  public void raiseArm() {

    if (armMotor.getEncoder().getPosition() < upperLimit) {
      armMotor.set(Constants.ARM_SPEED);
    } else {
      stopArm();
    }
  }

  public void lowerArm() {

    if (armLimitSwtich.get() == false) {
      armMotor.set(-Constants.ARM_SPEED);
    } else {
      stopArm();
      armMotor.getEncoder().setPosition(0);
    }

  }

  public void stopArm() {

    armMotor.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
