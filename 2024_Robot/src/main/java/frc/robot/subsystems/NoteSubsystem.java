// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteConstants;

public class NoteSubsystem extends SubsystemBase {

  private final TalonFX leftLiftMotor = new TalonFX(NoteConstants.leftLiftMotorID);
  private final TalonFX rightLiftMotor = new TalonFX(NoteConstants.rightLiftMotorID);
  private final Spark leftShooterMotor = new Spark(NoteConstants.leftShooterID);
  private final Spark rightShooterMotor = new Spark(NoteConstants.rightShooterID);  
  private final Spark kickerMotor = new Spark(NoteConstants.kickerMotorID);
  private final Spark tiltMotor = new Spark(NoteConstants.tiltMotorID);
  private final DigitalInput bottomSwitch = new DigitalInput(NoteConstants.bottomSwitch);


  /** Creates a new NoteSubsystem. */
  public NoteSubsystem() {
    rightLiftMotor.setControl(new Follower(leftLiftMotor.getDeviceID(),false));
    
  }

  public void manualRaiseLift(){
    leftLiftMotor.set(0.2);
  }

  public void manualLowerLift(){
    if (bottomSwitch.get()){
    leftLiftMotor.set(-.2);
    }
    else{
      leftLiftMotor.set(0);
    }
  }

  public void manualTiltUp(){
    tiltMotor.set(0.1);
  }

  public void manualTiltDown(){
    tiltMotor.set(-.1);
  }

  public void manualShoot(){
    leftShooterMotor.set(1);
    rightShooterMotor.set(-1);
  }

  public void manualIntake(){
    leftShooterMotor.set(-1);
    rightShooterMotor.set(1);
  }

  public void triggerIntake(double speed){
    leftShooterMotor.set(speed);
    rightShooterMotor.set(-speed);
  }

  public void stopLift(){
    leftLiftMotor.set(0);
  }

  public void stopKick(){
    kickerMotor.set(0);
  }

  public void stopShooter(){
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
  }

    public void manualKick(){
    kickerMotor.set(1);
    }
  
  //public double getLiftPosition(){
 //    return rightLiftMotor.getPosition();///2048/4*.5;
   // }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}