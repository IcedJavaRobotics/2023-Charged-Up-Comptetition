// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;


public class LimelightCommand extends CommandBase {

private final LimelightSubsystem limelightSubsystem;
  private boolean targetFound = false;


  /* Creates a new Limelight. */
   public LimelightCommand(LimelightSubsystem subsystem) {
    limelightSubsystem=subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelightSubsystem);
  }

 
  @Override
  public void initialize() {
  //called when robot is first started up

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//variables for examples
  // final double STEER_K = 0.03;                    // how hard to turn toward the target
  // final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
  // final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
  // final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast


//Whether the limelight has any valid targets (0 or 1)
double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

//Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
//Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

//Target Area (0% of image to 100% of image)
double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

//ID of primary AprilTag
double tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

// Camera transform in target space of primary apriltag or solvepnp target.
//  NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
double camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDouble(0);



if (tv == 0)
{
  //limelight has no valid target
  targetFound = false;
}else{
  targetFound = true;
}


//place code of what robot does here.


//Runs only if AprilTag is detected
if(targetFound){
  //turns blue if detected, changes color based on tag area
   if(ta>=8){
    limelightSubsystem.turnDarkBlue();
   }else if(ta>=1){
    limelightSubsystem.turnBlue();
   }else if(ta>=0.25){
    limelightSubsystem.turnLightBlue();
   }else{
    limelightSubsystem.turnLightLightBlue();
  }







}else{
  //turns red if it doesnt see apriltag
  limelightSubsystem.turnRed();


}



//print cause testing thingymajig
System.out.println("hello world");
System.out.println(tx);
System.out.println(ty);
System.out.println(ta);
System.out.println(tv);
System.out.println(tid);
System.out.println(targetFound);
System.out.println(camtran);

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", tx);
SmartDashboard.putNumber("LimelightY", ty);
SmartDashboard.putNumber("LimelightArea", ta);
SmartDashboard.putNumber("LimelightTV", tv);
SmartDashboard.putNumber("AprilTagID", tid);
SmartDashboard.putBoolean("TargetSpotted", targetFound);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
