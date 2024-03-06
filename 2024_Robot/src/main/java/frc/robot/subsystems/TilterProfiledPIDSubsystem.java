// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.NoteConstants;

public class TilterProfiledPIDSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new TilterProfiledPIDSubsystem. */


  private final CANSparkMax tiltMotor = new CANSparkMax(NoteConstants.tiltMotorID, MotorType.kBrushless);
  private final DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(NoteConstants.tiltEncoderID);
  private final ArmFeedforward tiltFeedforward = new ArmFeedforward(1,1,.5,.1);



  public TilterProfiledPIDSubsystem() {

    super(new ProfiledPIDController(1,0,0,new TrapezoidProfile.Constraints(
                3,
                10),.02));
    tiltEncoder.setDistancePerRotation(NoteConstants.tiltEncoderDistancePerRevolution);  
        // The PIDController used by the subsystem
    setGoal(Math.PI/4);
    tiltMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = tiltFeedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    tiltMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return tiltEncoder.getDistance();
  }

  public void resetTiltEncoder(){
    tiltEncoder.reset();
  }

  public void manualTiltUp(){
    tiltMotor.set(0.25);
  }

  public void manualTiltDown(){
    tiltMotor.set(-.25);
  }



  public void manualTilt(double speed){
    tiltMotor.set(-speed*0.5);
  }

  public void stopTilter(){
    tiltMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putNumber("Tilt Angle", tiltEncoder.getDistance());
  }

}
