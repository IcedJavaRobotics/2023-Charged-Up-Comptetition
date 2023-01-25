// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */ 

  final TalonFX frontLeftTalon = new TalonFX(Constants.FRONT_LEFT_TALON);
  final TalonFX backLeftTalon = new TalonFX(Constants.BACK_LEFT_TALON);
  final TalonFX frontRightTalon = new TalonFX(Constants.FRONT_RIGHT_TALON);
  final TalonFX backRightTalon = new TalonFX(Constants.BACK_RIGHT_TALON);

  double driveTime;
  double speedMod;
  double rampUpTime = 1.5;
  double kP = 0.3;
  double kI = 0.05;                                                               //PID values
  double kD = 0.1;

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);                              //the gyro being declared
  public final PIDController balanceController = new PIDController(kP, kI, kD);   //PID controller being declared

  public DriveTrainSubsystem() {

    frontLeftTalon.setInverted(true);
    backLeftTalon.setInverted(true);
    frontRightTalon.setInverted(false);
    backRightTalon.setInverted(false);

  }

  public void moveMotor( double speed, TalonFX talon ) {

    talon.set(ControlMode.PercentOutput, speed);

  }

  public double ensureRange( double val ) {

    return Math.min(Math.max(val, -1), 1);

  }

  public void mecanumDrive( double X, double Y, double R, double Z, boolean zoom, boolean balance ) {

    if (balance == true) {    //the autobalance button is held down

      moveMotor( balanceController.calculate(ahrs.getAngle(), 0), frontLeftTalon);  //move method, PIDController, get speed when setpoint is 0, motor
      moveMotor( balanceController.calculate(ahrs.getAngle(), 0), backLeftTalon);
      moveMotor( balanceController.calculate(ahrs.getAngle(), 0), frontRightTalon);
      moveMotor( balanceController.calculate(ahrs.getAngle(), 0), backRightTalon);


    } else {                  //the autobalance button is not held down
      
      if (zoom == true) {     //When speed button is pressed it shortens ramp up time and puts it at max speed
        Z = 1;
        rampUpTime = 1;
      } else {                //Normal ramp up time, speed dependant on the slider (Z)
        Z = ( -Z + 1 ) /2;
        rampUpTime = 1.5;
      }
  
      if ( Math.abs(X) + Math.abs(Y) + Math.abs(R) == 0 ) {
  
        driveTime = Timer.getMatchTime();
      
      }
  
      if ( Timer.getMatchTime() - driveTime <= rampUpTime ) {
  
        speedMod = -1 * ((0.5 * (driveTime - Timer.getMatchTime()) / rampUpTime) + 0.5) ;
  
      } else {
  
        speedMod = 1;
  
      }
  
      moveMotor( Z * speedMod * ensureRange(Y + X + R), frontLeftTalon);
      moveMotor( Z * speedMod * ensureRange(Y - X + R), backLeftTalon);
      moveMotor( Z * speedMod * ensureRange(Y - X - R), frontRightTalon);
      moveMotor( Z * speedMod * ensureRange(Y + X - R), backRightTalon);

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
