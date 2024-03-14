// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteConstants;



 
public class TilterSubsystem extends SubsystemBase {

private final CANSparkMax tiltMotor = new CANSparkMax(NoteConstants.tiltMotorID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
private final SparkPIDController m_pidController;
private AbsoluteEncoder m_absoluteEncoder; 

 
   
  /** Creates a new TilterSubsystem. */
  public TilterSubsystem() {
    tiltMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    m_absoluteEncoder = tiltMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_pidController = tiltMotor.getPIDController();
    m_pidController.setFeedbackDevice(m_absoluteEncoder);
    m_pidController.setP(1);
    m_pidController.setI(1e-4);
    m_pidController.setD(0);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(0.4, -.4);
    m_pidController.setPositionPIDWrappingMaxInput(.25);
   
  }

  public boolean shootPositionReady(){
    if (m_absoluteEncoder.getPosition() > .155 & m_absoluteEncoder.getPosition() < 0.17)
      return true;
      else
      return false;
  }

  public void basicTilt(double speed){
    if (m_absoluteEncoder.getPosition() < .155 || m_absoluteEncoder.getPosition()>0.6)
    tiltMotor.set(speed);
    else if (m_absoluteEncoder.getPosition() > 0.17)
    tiltMotor.set(-speed);
  }


  public void manualTilt(double speed){
    tiltMotor.set(speed*.5);
  }

  public void stopTilt(){
    tiltMotor.set(0);
  }

  public void pidTilt(){
    m_pidController.setReference(.16,CANSparkMax.ControlType.kDutyCycle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tilter motor",tiltMotor.get());
    SmartDashboard.putNumber("tilt position",m_absoluteEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
