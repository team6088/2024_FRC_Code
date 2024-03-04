// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteConstants;

public class NoteSubsystem extends SubsystemBase {

  private final TalonFX leftLiftMotor = new TalonFX(NoteConstants.leftLiftMotorID);
  private final TalonFX rightLiftMotor = new TalonFX(NoteConstants.rightLiftMotorID);
  private final Spark ShooterMotor = new Spark(NoteConstants.ShooterID);
  //private final Spark rightShooterMotor = new Spark(NoteConstants.ShooterID);  
  private final Spark kickerMotor = new Spark(NoteConstants.kickerMotorID);
  private final DigitalInput bottomSwitch = new DigitalInput(NoteConstants.bottomSwitch);



  /** Creates a new NoteSubsystem. */
  public NoteSubsystem() {
    rightLiftMotor.setControl(new Follower(leftLiftMotor.getDeviceID(),false));
    //leftLiftMotor.setPosition(0);
    //rightLiftMotor.setPosition(0);

  }



  public boolean lowerLimit (){
    return bottomSwitch.get();
  }

  public void resetLiftHeight(){
    if (this.lowerLimit()){
    leftLiftMotor.setPosition(0);
    rightLiftMotor.setPosition(0);
    }
  }

  public void manualRaiseLift(){
    leftLiftMotor.set(0.2);
  }

  public void manualLowerLift(){
    if (this.lowerLimit()){
    leftLiftMotor.set(-.2);

    }
    else {
      leftLiftMotor.set(0);
      leftLiftMotor.setPosition(0);
      rightLiftMotor.setPosition(0);
    }
  }

  public void holdLiftPosition(){
    leftLiftMotor.setControl(new PositionDutyCycle(0));
  }



  public void manualShoot(){
    ShooterMotor.set(1);
    //rightShooterMotor.set(-1);
  }

  public void manualIntake(){
    ShooterMotor.set(-1);
    //rightShooterMotor.set(1);
  }

  public void triggerIntake(double speed){
    ShooterMotor.set(speed);
    //rightShooterMotor.set(-speed);
  }


  public void stopLift(){
    leftLiftMotor.set(0);
  }

  public void stopKick(){
    kickerMotor.set(0);
  }

  public void stopShooter(){
    ShooterMotor.set(0);
    //rightShooterMotor.set(0);
  }

    public void manualKick(){
    kickerMotor.set(1);
    }
    
     public double leftLiftPostion(){
      return leftLiftMotor.getPosition().getValueAsDouble()/4*0.5;
    }

    public double rightLiftPosition(){
  return rightLiftMotor.getPosition().getValueAsDouble()/4*0.5;
} 

  
  //public double getLiftPosition(){
 //    return rightLiftMotor.getPosition();///2048/4*.5;
   // }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Left Encoder", leftLiftPostion());
  SmartDashboard.putNumber("Right Encoder", rightLiftPosition());
  SmartDashboard.putNumber("Left Lift Output", leftLiftMotor.get());  
  SmartDashboard.putNumber("Right Lift Output", rightLiftMotor.get());    
  SmartDashboard.putBoolean("Lower Switch Value", bottomSwitch.get());  
  SmartDashboard.putNumber("lelft-right Lift",leftLiftPostion() - rightLiftPosition());
  }
}
