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

  double kP = 0.1;
  double kI = 0.01; // PID values
  double kD = 0.01;
  double shortTaxi = 97; // taxi
  double longTaxi = 105; // its inches
  public boolean wheelsRaised = true;

  public final PIDController scoreController = new PIDController(kP, kI, kD); // PID controller being declared

  public DriveTrainSubsystem() {

    frontLeftTalon.setInverted(true);
    backLeftTalon.setInverted(true);
    frontRightTalon.setInverted(false);
    backRightTalon.setInverted(false);

    scoreController.setTolerance(3, 5);
    scoreController.setIntegratorRange(-1, 1);

  }

  // Autonomous Section
  public void zeroEncoder() {

    frontLeftTalon.setSelectedSensorPosition(0);
    
  }

  public boolean autoTaxi() {
    if ( Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2) * Constants.AUTO_DISTANCE) ) {
      
      autoMoveMotor();

    } else {

      stopMotor();

    }
  }

  public void autoMoveMotor() {

    frontLeftTalon.set(ControlMode.PercentOutput, Constants.AUTO_SPEED);
    backLeftTalon.set(ControlMode.PercentOutput, Constants.AUTO_SPEED);
    frontRightTalon.set(ControlMode.PercentOutput, Constants.AUTO_SPEED);
    backRightTalon.set(ControlMode.PercentOutput, Constants.AUTO_SPEED);

  }

  public void  stopMotor() {
    
    frontLeftTalon.set(ControlMode.PercentOutput, 0);
    backLeftTalon.set(ControlMode.PercentOutput, 0);
    frontRightTalon.set(ControlMode.PercentOutput, 0);
    backRightTalon.set(ControlMode.PercentOutput, 0);

  }

  
  // Teleop Section

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

  public void mecanumDrive(double X, double Y, double R, double Z) {

    if (wheelsRaised) { // checks if pneumatic wheels are dropped (changed in PneumaticWheelsCommand)

      Z = (-Z + 1) / 2;

      moveMotor(Z * ensureRange(Y + X + R), frontLeftTalon);
      moveMotor(Z * ensureRange(Y - X + R), backLeftTalon);
      moveMotor(Z * ensureRange(Y - X - R), frontRightTalon);
      moveMotor(Z * ensureRange(Y + X - R), backRightTalon);

    } else if (wheelsRaised == false) {

      // Tank drive for when wheels are deployed (only forward)
      moveMotor(ensureRange(Y), backLeftTalon);
      moveMotor(ensureRange(Y), frontLeftTalon);
      moveMotor(ensureRange(Y), frontRightTalon);
      moveMotor(ensureRange(Y), backRightTalon);
      dropWheelsSpark.set(ensureRange(Y));

    }

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
