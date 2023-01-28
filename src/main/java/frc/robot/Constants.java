// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Extendo
  public static final int EXTENDO_MOTOR = 5;
  public static final double EXTENDO_SPEED = 0.45;
    //Encoder values for set arm positions
  public static final int LOW_BOUND_DEFAULT_POS = 10;
  public static final int HIGH_BOUND_DEFAULT_POS = 100;
  public static final int LOW_BOUND_LOW_POS = 100;
  public static final int HIGH_BOUND_LOW_POS = 1000;
  public static final int LOW_BOUND_MID_POS = 1000;
  public static final int HIGH_BOUND_MID_POS = 2000;
  public static final int LOW_BOUND_HIGH_POS = 2000;
  public static final int HIGH_BOUND_HIGH_POS = 3000;

  //Arm
  public static final int ARM_VICTOR = 6;


  //Limit Switches
  public static final int EXTENDO_LIMIT_SWITCH = 2;
  public static final int ARM_LIMIT_SWITCH = 3;


  //Buttons



  //Controllers

  public static final int CONTROLLER = 2;



  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
