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

  //Drive train
  public static final int FRONT_LEFT_TALON = 1;
  public static final int FRONT_RIGHT_TALON = 2;
  public static final int BACK_LEFT_TALON = 3;
  public static final int BACK_RIGHT_TALON = 4;

  //Controllers
  public static final int JOYSTICK = 0;
  public static final int DRIVER_STATION = 1;
  public static final int CONTROLLER = 2;
      // ^ these indicate the spot used on the driverstation ^
  public static final double DEADZONE = 0.2;

  //Claw
  public static final int LEFT_CLAW = 7;
  public static final int RIGHT_CLAW = 8;
  public static final double CLAW_SPEED = .5;

  //Digital imputs
  public static final int RIGHT_CLAW_LIMIT = 0;
  public static final int LEFT_CLAW_LIMIT = 1;
  public static final int EXTENDO_LIMIT_SWITCH = 2;
  public static final int ARM_LIMIT_SWITCH = 3;
  public static final int ARM_ENCODER = 4;

  // Extendo
  public static final int EXTENDO_MOTOR = 5;
  public static final double EXTENDO_SPEED = 0.45;
    // Encoder values for set arm positions
  public static final int LOW_BOUND_DEFAULT_POS = 10;
  public static final int HIGH_BOUND_DEFAULT_POS = 100;
  public static final int LOW_BOUND_LOW_POS = 100;
  public static final int HIGH_BOUND_LOW_POS = 1000;
  public static final int LOW_BOUND_MID_POS = 1000;
  public static final int HIGH_BOUND_MID_POS = 2000;
  public static final int LOW_BOUND_HIGH_POS = 2000;
  public static final int HIGH_BOUND_HIGH_POS = 3000;

  // Arm
  public static final int ARM_TALON = 6;
  public static final double ARM_SPEED = 0.4;

    //Limelight
    public static final double LIMELIGHT_ANGLE = 3.0;    //upward angle of limelight camera [degrees]
    public static final double LIMELIGHT_HEIGHT = 18.5;  //distance lens from floor [inches]
    public static final double APRILTAG_HEIGHT = 14.25;  //distance from apriltag to floor(bottom of tag) [inches]
    public static final double APRILTAG_DOUBLE_SUBSTATION_HEIGHT = 23.375; //ditto but on the double substations [inches]
  
  

}
