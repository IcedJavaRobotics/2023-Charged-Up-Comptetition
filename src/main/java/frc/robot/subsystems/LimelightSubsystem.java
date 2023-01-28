// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private Spark blinkin = new Spark(0); //Creates a blinkin as if it were a spark.
   //Creates a new LimelightSubsystem. 
  public LimelightSubsystem() {

  }

  //colors
  
  public void turnRed(){
  blinkin.set(0.61);
  }
  public void turnDarkBlue(){
  blinkin.set(0.85);
  }
  public void turnBlue(){
    blinkin.set(0.87);
  }
  public void turnLightBlue(){
    blinkin.set(0.83);
  }
  public void turnLightLightBlue(){
    blinkin.set(0.81);
  }
  public void turnGreen(){
    blinkin.set(0.73);
  }
  public void flashRed(){
    blinkin.set(-0.11);
  }
  public void turnDarkGreen(){
    blinkin.set(0.75);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
