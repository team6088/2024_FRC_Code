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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable table;
	private boolean visionActive = false;
	private double kp = 0.05;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    setVisionMode("off");
    setStream(2);
  }


	public boolean isTarget() {
		return getValue("tv").getDouble(0.0) == 1;
	}
	public double getRawTx() {
		return getValue("tx").getDouble(0.0);
	}


 /**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		double output = getValue("tx").getDouble(0.0) * kp;
		if (Math.abs(output) >= 0.3) {
			output = 0.3 * Math.signum(output);
		}
		if (!visionActive) {
			output = 0.0;
		}
		return -output;
	}

	public Boolean atTxTarget() {
		return this.getTx() < 1;
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(0.0);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(0.0);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(0.0);
	}
  public void setLedMode(int ledMode) {
		getValue("ledMode").setNumber(ledMode);
  	}
  
	public void setCameraMode(int cameraMode) {
		getValue("camMode").setNumber(cameraMode);
 	}

	public void setStream(int streamMode) {
		getValue("stream").setNumber(streamMode);
 	}
	
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

  	private NetworkTableEntry getValue(String key) {
		return table.getEntry(key);
	}

  public void setVisionMode(String visionMode) {
		setLedMode(0);
		setCameraMode(0);
		if (visionMode == "cube") { //april

			setPipeline(0);
			visionActive = true;
		} else if (visionMode == "object") {

			setPipeline(1);
			visionActive = true;
		} else if (visionMode == "cone") { //reflective

			setPipeline(2);
			visionActive = true;
		} else {
			visionActive = false;
			setLedMode(1);
			setCameraMode(1);
		}
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
			SmartDashboard.putNumber("limelight get tx", getTx());

  }

}
