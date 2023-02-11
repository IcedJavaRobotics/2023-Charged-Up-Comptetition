// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  final LimelightSubsystem limelight = new LimelightSubsystem();
  final TalonFX frontLeftTalon = new TalonFX(Constants.FRONT_LEFT_TALON);
  final TalonFX backLeftTalon = new TalonFX(Constants.BACK_LEFT_TALON);
  final TalonFX frontRightTalon = new TalonFX(Constants.FRONT_RIGHT_TALON);
  final TalonFX backRightTalon = new TalonFX(Constants.BACK_RIGHT_TALON);
  double driveTime;
  double speedMod;
  double rampUpTime = 1.5;
  double kP = 0.1;
  double kI = 0.01; // PID values
  double kD = 0.01;
  double leftSideSetpoint = 15;
  double midSetpoint = 0;
  double rightSideSetpoint = -15;
  double shortTaxi = 97; // taxi
  double longTaxi = 105; // its inches

  public final PIDController scoreController = new PIDController(kP, kI, kD); // PID controller being declared

  public DriveTrainSubsystem() {

    frontLeftTalon.setInverted(true);
    backLeftTalon.setInverted(true);
    frontRightTalon.setInverted(false);
    backRightTalon.setInverted(false);

    scoreController.setTolerance(5, 10);
    scoreController.setIntegratorRange(-0.5, 0.5);

  }

  public void moveMotor(double speed, TalonFX talon) {

    talon.set(ControlMode.PercentOutput, speed);

  }

  public double ensureRange(double val) {

    return Math.min(Math.max(val, -1), 1);

  }

  public void taxiOutShort() {
    double distance = limelight.getDistance();

    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), frontLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), backLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), frontRightTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), backRightTalon);
  }

  public void taxiOutLong() {
    double distance = limelight.getDistance();

    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), frontLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), backLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), frontRightTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), backRightTalon);
  }

  public boolean moveLeft() {

    double horiOffset = limelight.getTx();

    if (limelight.getTv() == 1) {

      if (horiOffset > leftSideSetpoint) {

        moveMotor(ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), frontLeftTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), backLeftTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), frontRightTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), backRightTalon);

        return true;

      } else if (horiOffset < leftSideSetpoint) {

        moveMotor(ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), frontLeftTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), backLeftTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), frontRightTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), backRightTalon);

        return true;

      }

    } else {
      System.out.println("Target not found");
    }

    return false;

  }
  public boolean moveRight() {

    double horiOffset = limelight.getTx();

    if (limelight.getTv() == 1) {

      if (horiOffset > -15) {

        moveMotor(ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), frontLeftTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), backLeftTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), frontRightTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), backRightTalon);

        return true;

      } else if (horiOffset < -15) {

        moveMotor(ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), frontLeftTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), backLeftTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), frontRightTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), backRightTalon);

        return true;

      }

    } else {
      System.out.println("Target not found");
    }

    return false;

  }
  public boolean moveCenter() {

    double horiOffset = limelight.getTx();

    if (limelight.getTv()== 1) {

      if (horiOffset > 0) {

        moveMotor(ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), frontLeftTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, midSetpoint)), backLeftTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), frontRightTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, midSetpoint)), backRightTalon);

        return true;

      } else if (horiOffset < 0) {

        moveMotor(ensureRange(scoreController.calculate(horiOffset, midSetpoint)), frontLeftTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), backLeftTalon);
        moveMotor(ensureRange(scoreController.calculate(horiOffset, midSetpoint)), frontRightTalon);
        moveMotor(ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), backRightTalon);

        return true;

      }

    } else {
      System.out.println("Target not found");
    }

    return false;

  }
  public void mecanumDrive(double X, double Y, double R, double Z, boolean zoom) {

    if (zoom == true) { // When speed button is pressed it shortens ramp up time and puts it at max
                        // speed
      Z = 1;
      rampUpTime = 1;
    } else { // Normal ramp up time, speed dependant on the slider (Z)
      Z = (-Z + 1) / 2;
      rampUpTime = 1.5;
    }

    if (Math.abs(X) + Math.abs(Y) + Math.abs(R) == 0) {

      driveTime = Timer.getMatchTime();

    }

    if (Timer.getMatchTime() - driveTime <= rampUpTime) {

      speedMod = -1 * ((0.5 * (driveTime - Timer.getMatchTime()) / rampUpTime) + 0.5);

    } else {

      speedMod = 1;

    }

    moveMotor(Z * speedMod * ensureRange(Y + X + R), frontLeftTalon);
    moveMotor(Z * speedMod * ensureRange(Y - X + R), backLeftTalon);
    moveMotor(Z * speedMod * ensureRange(Y - X - R), frontRightTalon);
    moveMotor(Z * speedMod * ensureRange(Y + X - R), backRightTalon);

  }

  // Autonomous Code

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
