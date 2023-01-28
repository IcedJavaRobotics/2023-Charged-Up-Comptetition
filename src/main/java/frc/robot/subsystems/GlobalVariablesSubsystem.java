// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariablesSubsystem extends SubsystemBase {
  /** Creates a new GlobalVariablesSubsystem. */

  public static int slotGoal = 0;

  public GlobalVariablesSubsystem() {}

  public void slotGoal1() {

    slotGoal = 1;

  }

  public void slotGoal2() {

    slotGoal = 2;
    
  }

  public void slotGoal3() {

    slotGoal = 3;
    
  }

  public void slotGoal4() {

    slotGoal = 4;
    
  }

  public void slotGoal5() {

    slotGoal = 5;
    
  }

  public void slotGoal6() {

    slotGoal = 6;
    
  }

  public void slotGoal7() {

    slotGoal = 7;
    
  }

  public void slotGoal8() {

    slotGoal = 8;
    
  }

  public void slotGoal9() {

    slotGoal = 9;
    
  }

  public void slotGoalReset() {

    slotGoal = 0;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
