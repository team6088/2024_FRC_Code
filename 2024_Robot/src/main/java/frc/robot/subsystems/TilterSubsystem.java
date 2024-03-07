// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteConstants;



 
public class TilterSubsystem extends SubsystemBase {

private final CANSparkMax tiltMotor = new CANSparkMax(NoteConstants.tiltMotorID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless



);
 
   
  /** Creates a new TilterSubsystem. */
  public TilterSubsystem() {
    tiltMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }


  public void manualTilt(double speed){
    tiltMotor.set(speed*.5);
  }

  public void stopTilt(){
    tiltMotor.set(0);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("tilter motor",tiltMotor.get());
    // This method will be called once per scheduler run
  }
}
