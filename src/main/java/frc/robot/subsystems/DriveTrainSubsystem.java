// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  final ADIS16470_IMU gyro = new ADIS16470_IMU();

  double kP = 0.1;
  double kI = 0.01; // PID values
  double kD = 0.01;
  double shortTaxi = 97; // taxi
  double longTaxi = 105; // its inches
  public boolean wheelsRaised = true;
  boolean stepOne = true;
  boolean stepTwo = true;
  boolean areWePannicing = false;   // set to true when the back right wheel is dead

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

  public void checkGyro(){
    System.out.println(gyro.getYComplementaryAngle()); 
  }

  public void zeroGyro() {
    gyro.calibrate();
  }

  public void zeroEncoder() {

    frontLeftTalon.setSelectedSensorPosition(0);

  }

  public void autoCharging() {

    // if (gyro.getYComplementaryAngle() < Constants.CHARGING_MAX_ANGLE) {
    //   autoMove(Constants.AUTO_TAXI_SPEED);

      if (gyro.getYComplementaryAngle() >= Constants.CHARGING_MAX_ANGLE) {
        autoMove(Constants.AUTO_CHARGING_SPEED);
      } else if (gyro.getYComplementaryAngle() <= Constants.CHARGING_MIN_ANGLE) {
        autoMove(-Constants.AUTO_CHARGING_SPEED);
      } else {
        stopMotors();
      }
    
  }

  /**
   * method for autonomous movement out of the community
   */
  public Boolean autoTaxi() {
    if (stepTwo == true &&  Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2)
    * Constants.AUTO_TAXI_DISTANCE)) {

      autoMove(Constants.AUTO_TAXI_SPEED);
      return false;

    } else {

      stepTwo = false;
      return true;

    }
  }


  /**
   * method for autonomous movement to score low hub
   */
  public Boolean autoScoring() {
    if (stepOne && Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2)
        * Constants.AUTO_SCORING_DISTANCE)) {

      autoMove(Constants.AUTO_SCORING_SPEED);
      return false;

    } else {

      stopMotors();
      frontLeftTalon.setSelectedSensorPosition(0);
      stepOne = false;
      return true;

    }

  }



  /**
   * movement method for autonomous
   * @param speed speed that robot go
   */
  public void autoMove(double speed) {

    frontLeftTalon.set(ControlMode.PercentOutput, speed);
    backLeftTalon.set(ControlMode.PercentOutput, speed);
    frontRightTalon.set(ControlMode.PercentOutput, speed);
    backRightTalon.set(ControlMode.PercentOutput, speed);
    dropWheelsSpark.set(speed);

  }

  /**
   * Movement for auto without the drop wheels
   * @param speed speed that robot go
   * @param distance How far the motors will turn in inches (maybe??)
   */
  public void autoMoveNoTank(double speed, int distance) {

    if(Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2)
        * distance)) {

      frontLeftTalon.set(ControlMode.PercentOutput, speed);
      backLeftTalon.set(ControlMode.PercentOutput, speed);
      frontRightTalon.set(ControlMode.PercentOutput, speed);
      backRightTalon.set(ControlMode.PercentOutput, speed);
    }

  }

  /**
   * Movement for auto with the drop wheels
   * @param speed speed that robot go
   * @param distance How far the motors will turn in inches (maybe??)
   */
  public void autoMove(double speed, int distance) {

    if(Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2)
        * distance)) {

      frontLeftTalon.set(ControlMode.PercentOutput, speed);
      backLeftTalon.set(ControlMode.PercentOutput, speed);
      frontRightTalon.set(ControlMode.PercentOutput, speed);
      backRightTalon.set(ControlMode.PercentOutput, speed);
      dropWheelsSpark.set(speed);
    }

  }

  public void stopMotors() {

    frontLeftTalon.set(ControlMode.PercentOutput, 0);
    backLeftTalon.set(ControlMode.PercentOutput, 0);
    frontRightTalon.set(ControlMode.PercentOutput, 0);
    backRightTalon.set(ControlMode.PercentOutput, 0);
    dropWheelsSpark.set(0);

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

    /*





     * THIS IS IMPORTANT READ THIS
     * 
     * NOTHING SHOULD CHANGE WITH THE DRIVING BUT IF IT DOESN'T WORK FOR SOME REASON 
     * JUST TAKE THE CODE OUT OF THE ELSE STATEMENT IT'S THE SAME AS BEFORE
     


     
     
     */

    SmartDashboard.putNumber("gyro YCompAngle", gyro.getYComplementaryAngle());
    if (areWePannicing == true) {

      if (wheelsRaised) { // checks if pneumatic wheels are dropped (changed in PneumaticCommand)

        Z = (-Z + 1) / 2;
  
        moveMotor(Z * ensureRange(Y + R), frontLeftTalon);
        moveMotor(Z * ensureRange(Y + R), backLeftTalon);
        moveMotor(Z * ensureRange(Y - R) * 1.15, frontRightTalon);
  
      } else if (wheelsRaised == false) {
  
        SmartDashboard.putNumber("gyro YCompAngle", gyro.getYComplementaryAngle());
        // Tank drive for when wheels are deployed (only forward)
        moveMotor(ensureRange(Y), backLeftTalon);
        moveMotor(ensureRange(Y), frontLeftTalon);
        moveMotor(ensureRange(Y), frontRightTalon);
        dropWheelsSpark.set(ensureRange(Y));
  
      }

    } else {

      if (wheelsRaised) { // checks if pneumatic wheels are dropped (changed in PneumaticCommand)

        Z = (-Z + 1) / 2;
  
        moveMotor(Z * ensureRange(Y + X + R), frontLeftTalon);
        moveMotor(Z * ensureRange(Y - X + R), backLeftTalon);
        moveMotor(Z * ensureRange(Y - X - R), frontRightTalon);
        moveMotor(Z * ensureRange(Y + X - R), backRightTalon);
  
      } else if (wheelsRaised == false) {
  
        SmartDashboard.putNumber("gyro YCompAngle", gyro.getYComplementaryAngle());
        // Tank drive for when wheels are deployed (only forward)
        moveMotor(ensureRange(Y), backLeftTalon);
        moveMotor(ensureRange(Y), frontLeftTalon);
        moveMotor(ensureRange(Y), frontRightTalon);
        moveMotor(ensureRange(Y), backRightTalon);
        dropWheelsSpark.set(ensureRange(Y));
  
      }

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