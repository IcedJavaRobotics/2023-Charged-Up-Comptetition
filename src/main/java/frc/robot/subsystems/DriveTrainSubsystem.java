// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  final CANSparkMax dropWheelsSpark = new CANSparkMax(Constants.DROP_WHEEL_SPARK, MotorType.kBrushless);
  double driveTime;
  double speedMod;
  double rampUpTime = 1.5;
  double kP = 0.1;
  double kI = 0.01; // PID values
  double kD = 0.01;
  double leftSideSetpoint = 10;
  double midSetpoint = 0;
  double rightSideSetpoint = -10;
  double shortTaxi = 97; // taxi
  double longTaxi = 105; // its inches
  double testSpeed = 0.2;

  public final PIDController scoreController = new PIDController(kP, kI, kD); // PID controller being declared
  public boolean wheelsRaised = true;

  public DriveTrainSubsystem() {

    frontLeftTalon.setInverted(true);
    backLeftTalon.setInverted(true);
    frontRightTalon.setInverted(false);
    backRightTalon.setInverted(false);

    scoreController.setTolerance(3, 5);
    scoreController.setIntegratorRange(-1, 1);

  }

  /**
   * Makes the motor move forward
   * 
   * @param speed speed of the motor
   * @return returns nothing
   * 
   * 
   */
  public void moveMotor(double speed, TalonFX talon) {

    talon.set(ControlMode.PercentOutput, speed);

  }

  public double ensureRange(double val) {

    return Math.min(Math.max(val, -1), 1);

  }

  /**
   * Moves the robot based on the controller
   * 
   * @param X    how far the joystick moved on the x axis(left and right)
   * @param Y    how far the joystick moved on the y axis(up and down)
   * @param R    how much the joystick has twisted
   * @param Z    slider for speed
   * @param zoom whether the speed button is pressed or not
   */

   public void mecanumDrive(double X, double Y, double R, double Z, boolean zoom) {

    if (wheelsRaised) { // checks if pneumatic wheels are dropped (changed in PneumaticWheelsCommand)

      Z = (-Z + 1) / 2;

      moveMotor(Z * speedMod * ensureRange(Y + X + R), frontLeftTalon);
      moveMotor(Z * speedMod * ensureRange(Y - X + R), backLeftTalon);
      moveMotor(Z * speedMod * ensureRange(Y - X - R), frontRightTalon);
      moveMotor(Z * speedMod * ensureRange(Y + X - R), backRightTalon);
    
    } else if(wheelsRaised == false) {

      // Tank drive for when wheels are deployed (only forward)
      moveMotor(ensureRange(-Y), backLeftTalon);
      moveMotor(ensureRange(-Y), frontLeftTalon);
      moveMotor(ensureRange(-Y), frontRightTalon);
      moveMotor(ensureRange(-Y), backRightTalon);
      dropWheelsSpark.set(ensureRange(-Y));

    }

    }

  /************ Methods for centering with the AprilTags ************/

  /**
   * makes robot strafe until lined up with the left poles @return returns
   * false when done
   */
  public boolean moveLeft() {

    double horiOffset = limelight.getTx();

    if (limelight.tagDetected()) {

      if (horiOffset > leftSideSetpoint) {

        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), frontLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), backLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), frontRightTalon);
        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), backRightTalon);

        return true;

      } else if (horiOffset < leftSideSetpoint) {

        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), frontLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), backLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, leftSideSetpoint)), frontRightTalon);
        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, leftSideSetpoint)), backRightTalon);

        return true;

      }

    } else {

      System.out.println("Target not found");

    }

    return false;

  }

  /**
   * makes the robot strafe until it is in line with the right poles from the
   * april tags @return returns
   * false when done
   */
  public boolean moveRight() {

    double horiOffset = limelight.getTx();

    if (limelight.tagDetected()) {

      if (horiOffset > rightSideSetpoint) {

        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), frontLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), backLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), frontRightTalon);
        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), backRightTalon);

        return true;

      } else if (horiOffset < rightSideSetpoint) {

        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), frontLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), backLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, rightSideSetpoint)), frontRightTalon);
        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, rightSideSetpoint)), backRightTalon);

        return true;

      }

    } else {

      System.out.println("Target not found");

    }

    return false;

  }

  /**
   * makes robot move until it lines up accurately with the aprilTag @return
   * returns false when done
   */
  public boolean moveCenter() {

    double horiOffset = limelight.getTx();

    if (limelight.tagDetected()) {

      if (horiOffset > midSetpoint) {

        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), frontLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, midSetpoint)), backLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, midSetpoint)), frontRightTalon);
        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), backRightTalon);

        return true;

      } else if (horiOffset < midSetpoint) {

        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), frontLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, midSetpoint)), backLeftTalon);
        moveMotor(.25 * ensureRange(scoreController.calculate(horiOffset, midSetpoint)), frontRightTalon);
        moveMotor(.25 * ensureRange(-scoreController.calculate(horiOffset, midSetpoint)), backRightTalon);

        return true;

      }

    } else {

      System.out.println("Target not found");

    }

    return false;

  }

  /********** Autonomous Code **********/

  /** makes the robot move backwards out of the community */
  public void taxiOutShort() {

    double distance = limelight.getDistance();

    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), frontLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), backLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), frontRightTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, shortTaxi)), backRightTalon);

  }

  /** makes robot move forwards out of the community */
  public void taxiOutLong() {

    double distance = limelight.getDistance();

    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), frontLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), backLeftTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), frontRightTalon);
    moveMotor(ensureRange(-scoreController.calculate(distance, longTaxi)), backRightTalon);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
