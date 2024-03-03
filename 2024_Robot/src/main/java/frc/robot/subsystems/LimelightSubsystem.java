// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  //Read the following from the “limelight” table
/* 
tv = Whether the limelight has any valid targets (0 or 1)
tx = Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
ty = Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
ta = Target Area (0% of image to 100% of image)
Write the following to the “limelight” table
ledMode Sets limelight’s LED state
0 = use the LED Mode set in the current pipeline
1 = force off
2 = force blink
3 = force on
12
Limelight Documentation, Release 1.0
camMode Sets limelight’s operation mode
0 = Vision processor
1 = Driver Camera (Increases exposure, disables vision processing) */

package frc.robot.subsystems;

import java.sql.Driver;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}
