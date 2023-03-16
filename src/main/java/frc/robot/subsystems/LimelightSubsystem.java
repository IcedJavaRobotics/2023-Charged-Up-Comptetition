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

  /**
   * whether or not the limelight sees a apriltag
   * 
   * @return true if it sees an apriltag, false if the obvious happens
   */
  public Boolean tagDetected() {

    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
      return true;
    }
    return false;
  }

  /**
   * command for calculating limelight distance
   * 
   * @return horizontal floor distance from robot to apriltag
   */
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}