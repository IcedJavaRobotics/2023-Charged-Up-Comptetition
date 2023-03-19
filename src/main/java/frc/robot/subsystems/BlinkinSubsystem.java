// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new BlinkinSubsystem. */
  private Spark blinkin = new Spark(0);

  public BlinkinSubsystem() {
  }

  public void colorCone() {
    blinkin.set(0.65);
  }

  public void colorCube() {
    blinkin.set(0.91);
  }

  public void autoBlinkin() {
    DriverStation.Alliance color;
    color = DriverStation.getAlliance();
    if (color == DriverStation.Alliance.Blue) {
      blinkin.set(0.83);
    }
    if (color == DriverStation.Alliance.Red) {
      blinkin.set(0.61);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
