// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteConstants;

public class NoteSubsystem extends SubsystemBase {

  private final TalonFX leftLiftMotor = new TalonFX(NoteConstants.leftLiftMotorID);
  private final TalonFX rightLiftMotor = new TalonFX(NoteConstants.rightLiftMotorID);
  //private final CANSparkMax LeftShooterMotor = new CANSparkMax(NoteConstants.LeftShooterID,MotorType.kBrushless);
  //private final CANSparkMax RighttShooterMotor = new CANSparkMax(NoteConstants.RightShooterID,MotorType.kBrushless);
  //private final CANSparkMax TilterMotor = new CANSparkMax(NoteConstants.TilterMotorID);
  
  
  //private final Spark ShooterMotor = new Spark(NoteConstants.ShooterID);
  private final CANSparkMax rightShooterMotor = new CANSparkMax(NoteConstants.rightShooterID, MotorType.kBrushless);  
  private final CANSparkMax leftShooterMotor = new CANSparkMax(NoteConstants.leftShooterID, MotorType.kBrushless); 
  private final Spark kickerMotor = new Spark(NoteConstants.kickerMotorID);
  private final DigitalInput bottomLeftSwitch = new DigitalInput(NoteConstants.bottomLeftSwitch);
  private final DigitalInput bottomRightSwitch = new DigitalInput(NoteConstants.bottomRightSwitch);
  private final DigitalInput upperSwitch = new DigitalInput(NoteConstants.upperSwitch);



  /** Creates a new NoteSubsystem. */
  public NoteSubsystem() {
    rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    leftLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    //rightLiftMotor.setControl(new Follower(leftLiftMotor.getDeviceID(),false));
    //leftLiftMotor.setPosition(0);
    //rightLiftMotor.setPosition(0);

  }



  public boolean lowerLeftLimit (){
    return bottomLeftSwitch.get();
  }

  public boolean lowerRightLimit (){
    return bottomRightSwitch.get();
  }

  public boolean liftBottomedOut (){
   if (!this.lowerLeftLimit() && !this.lowerRightLimit()) //if both switches are false (contacted) return true
    return true; 
    else 
    return false;
  }

  public boolean liftRaised (){
    return upperSwitch.get();
  }


  public void resetLeftLiftHeight(){
    if (this.lowerLeftLimit()){
    leftLiftMotor.setPosition(0);
    }
  }
  
  public void resetRightLiftHeight(){
    if (this.lowerRightLimit()){
    rightLiftMotor.setPosition(0);
    }
  }

  public void manualRaiseLift(double speed){
    if (this.liftRaised()){
    leftLiftMotor.set(speed);
    rightLiftMotor.set(speed);
  }
  else {
    leftLiftMotor.set(0);
    rightLiftMotor.set(0);
  }
}

  public void raiseLift(){
    leftLiftMotor.set(.6);
    rightLiftMotor.set(.6);
  }


  public void manualRaiseRight(){
  rightLiftMotor.set(.1);
}

  public void manualRaiseLeft(){
  leftLiftMotor.set(.1);

}

public void manualLowerLeftLift(){
  if (this.lowerLeftLimit()){
  leftLiftMotor.set(-.1);

  }
  else {
    leftLiftMotor.set(0);
    leftLiftMotor.setPosition(0);
  }
}

  public void manualLowerRightLift(){
  if (this.lowerRightLimit()){
  rightLiftMotor.set(-.1);

  }
  else 
    rightLiftMotor.set(0);
    rightLiftMotor.setPosition(0);
  
}


  public void manualLowerLift(double liftSpeed){
  if (this.lowerRightLimit() && this.lowerLeftLimit()){
  rightLiftMotor.set(-liftSpeed);
  leftLiftMotor.set(-liftSpeed);

    }
    else if (this.lowerRightLimit() && !this.lowerLeftLimit()){
      rightLiftMotor.set(-liftSpeed);
      leftLiftMotor.set(0);
      leftLiftMotor.setPosition(0);
    }

    else if (!this.lowerRightLimit() && this.lowerLeftLimit()){
    rightLiftMotor.set(0);
    leftLiftMotor.set(-liftSpeed);
    rightLiftMotor.setPosition(0);
  }

    else {
    rightLiftMotor.set(0);
    leftLiftMotor.set(0);
    rightLiftMotor.setPosition(0);
    leftLiftMotor.setPosition(0);
  }

}


  public void holdLiftPosition(){
    leftLiftMotor.setControl(new PositionDutyCycle(0));
  }


  public void manualShoot(){
    leftShooterMotor.set(1);
    rightShooterMotor.set(-1);
    //rightShooterMotor.set(-1);
  }

  public void ampShoot(){
    leftShooterMotor.set(0.25);
    rightShooterMotor.set(-.25);
    kickerMotor.set(0.15);
  }

  public void manualIntake(){
    leftShooterMotor.set(-1);
    rightShooterMotor.set(1);
    //rightShooterMotor.set(1);
  }

  public void triggerIntake(double speed){
    leftShooterMotor.set(speed);
    rightShooterMotor.set(-speed);
    //rightShooterMotor.set(-speed);
  }


  public void stopLift(){
    leftLiftMotor.set(0);
    rightLiftMotor.set(0);
  }

  public void stopKick(){
    kickerMotor.set(0);
  }

  public void stopShooter(){
    leftShooterMotor.set(0);
    rightShooterMotor.set(0);
    //rightShooterMotor.set(0);
  }

    public void manualKick(){
    kickerMotor.set(0.5);
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
  SmartDashboard.putBoolean("Lower Left Switch Value", bottomLeftSwitch.get());  
  SmartDashboard.putBoolean("Lower Right Switch Value", bottomRightSwitch.get());  
  SmartDashboard.putNumber("lelft-right Lift",leftLiftPostion() - rightLiftPosition());
  }
}
