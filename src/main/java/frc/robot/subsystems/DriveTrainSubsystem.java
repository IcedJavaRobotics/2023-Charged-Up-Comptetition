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
  boolean taxiDone = true;

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
   * Method for auto taxi with params
   * @param speed This is the speed at which the robot will move
   * @param distance How far the robot will move in inches (maybe??) and false if it hasn't
   */
  public Boolean autoTaxi(double speed, int distance) {
    if (stepTwo == true &&  Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2)
    * distance)) {

      autoMove(speed);
      return false;

    } else {

      stepTwo = false;
      zeroEncoder();
      return true;

    }
  }

  public Boolean autoTaxi2(double speed, int distance) {
    if (taxiDone == true &&  Math.abs(frontLeftTalon.getSelectedSensorPosition()) <= ((Constants.ROTATIONAL_CONSTANT / 2)
    * distance)) {

      autoMove(speed);
      return false;

    } else {

      taxiDone = false;
      zeroEncoder();
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

    setAllTalons(speed);
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

      setAllTalons(speed);
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

      setAllTalons(speed);
      dropWheelsSpark.set(speed);
    }else {
      stopMotors();
    }

  }

  public void stopMotors() {
    setAllTalons(0);
    dropWheelsSpark.set(0);
  }

  public void setAllTalons(double speed){
    moveMotor(speed,frontRightTalon,frontLeftTalon,backRightTalon,backLeftTalon);
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
  public void moveMotor(double speed, TalonFX...talon) {
    //uses varags to allow you to place as many talons as you want to prevent doing something over and over again
    for(int x = 0; x < talon.length; x++){
      talon[x].set(ControlMode.PercentOutput, speed);
    }
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

    SmartDashboard.putNumber("gyro YCompAngle", gyro.getYComplementaryAngle());

    if (wheelsRaised) { // checks if pneumatic wheels are dropped (changed in PneumaticCommand)

      Z = (-Z + 1) / 2;

      moveMotor(Z * ensureRange(Y + X + R), frontLeftTalon);
      moveMotor(Z * ensureRange(Y - X + R), backLeftTalon);
      moveMotor(Z * ensureRange(Y - X - R), frontRightTalon);
      moveMotor(Z * ensureRange(Y + X - R), backRightTalon);

    } else if (wheelsRaised == false) {

      SmartDashboard.putNumber("gyro YCompAngle", gyro.getYComplementaryAngle());
      // Tank drive for when wheels are deployed (only forward)
      setAllTalons(ensureRange(Y));
      dropWheelsSpark.set(ensureRange(Y));

    }

  }

  public void tankDrive(double x, double y){
    moveLeftTrain(0.25 * (y+x));
    moveRightTrain(0.25 * (y-x));
  }
  public void moveLeftTrain(double speed){
    moveMotor(speed, frontLeftTalon, backLeftTalon);
  }
  public void moveRightTrain(double speed){
    moveMotor(speed,frontRightTalon,backRightTalon);
  }

  /********** Autonomous Code **********/

  /** makes the robot move backwards out of the community */
  public void taxiOutShort() {
    double distance = limelight.getDistance();
    setAllTalons(ensureRange(-scoreController.calculate(distance, shortTaxi)));
  }

  /** makes robot move forwards out of the community */
  public void taxiOutLong() {
    double distance = limelight.getDistance();
    setAllTalons(ensureRange(-scoreController.calculate(distance, longTaxi)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}