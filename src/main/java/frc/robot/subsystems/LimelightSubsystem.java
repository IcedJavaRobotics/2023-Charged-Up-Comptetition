// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

  // private Spark blinkin = new Spark(0); //Creates a blinkin as if it were a
  // spark.
  // Creates a new LimelightSubsystem.
  public LimelightSubsystem() {

  }

  public double getTid() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
  }

  public double getTx() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getTy() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double getTa() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  public double getTv() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }

  public double getDistance() {

    double targetOffsetAngle_Vertical = getTy();
    double goalHeightInches = Constants.APRILTAG_HEIGHT;
    double angleToGoalRadians = (Constants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical) * (3.14159 / 180);

    if (getTid() == 4 || getTid() == 5) {
      goalHeightInches = Constants.APRILTAG_DOUBLE_SUBSTATION_HEIGHT;
    } // tid 4 and 5 are the double substations

    // calculate distance
    if (getTy() >= 0) {
      return (goalHeightInches - Constants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
    } else {
      return (Constants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
    }

  }

  // colors

  // public void turnRed(){
  // blinkin.set(0.61);
  // }
  // public void turnDarkBlue(){
  // blinkin.set(0.85);
  // }
  // public void turnBlue(){
  // blinkin.set(0.87);
  // }
  // public void turnLightBlue(){
  // blinkin.set(0.83);
  // }
  // public void turnLightLightBlue(){
  // blinkin.set(0.81);
  // }
  // public void turnGreen(){
  // blinkin.set(0.73);
  // }
  // public void flashRed(){
  // blinkin.set(-0.11);
  // }
  // public void turnDarkGreen(){
  // blinkin.set(0.75);
  // }

  // Methods of what to do, robot is very polite
  public void rotateRight() {
    // limelightSubsystem.turnDarkBlue();
    System.out.println("A little right please");
  }

  public void rotateLeft() {
    // limelightSubsystem.turnLightLightBlue();
    System.out.println("A little left please");
  }

  public void driveForward() {
    // when you have apriltag centered but far
    // limelightSubsystem.turnGreen();
    System.out.println("Go forward please");
  }

  public void stopAndSeek() {
    // when you are close but not perfectly centered
    // limelightSubsystem.flashRed();
    if (getTx() >= 1) {
      System.out.println("Seeking TARGET...Turn LEFT please.");
    }
    if (getTx() <= -1) {
      System.out.println("Seeking TARGET...Turn RIGHT please.");
    }
  }

  public void stopAndDestroy() {
    // when you are close and perfectly centered
    // limelightSubsystem.turnDarkGreen();
    System.out.println("i am in range of the apriltag " + getTid() + "! Great work me!");
  }

  public void searchingForTargets() {
    // no apriltags seen
    // limelightSubsystem.turnRed();
    System.out.println("Scanning for Targets....");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}