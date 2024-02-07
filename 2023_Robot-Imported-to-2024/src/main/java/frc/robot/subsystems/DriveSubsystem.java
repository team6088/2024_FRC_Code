// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final static WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstants.leftMasterPort);
  private final static WPI_TalonSRX leftSlave = new WPI_TalonSRX(DriveConstants.leftSlavePort);
  private final static WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveConstants.rightMasterPort);
  private final static WPI_TalonSRX rightSlave = new WPI_TalonSRX(DriveConstants.rightSlavePort);
  private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private double turningValue;
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);

      //Need to test
      boolean currentLimitState = true;
    leftMaster.configContinuousCurrentLimit(DriveConstants.current40AmpContinuousCurrentLimit,DriveConstants.timeoutMS);
    leftMaster.configPeakCurrentLimit(DriveConstants.current40AmpPeakCurrentLimit,DriveConstants.timeoutMS);
    leftMaster.configPeakCurrentDuration(DriveConstants.current40AmpPeakCurrentDuration,DriveConstants.timeoutMS);
    leftMaster.enableCurrentLimit(currentLimitState);
    rightMaster.configContinuousCurrentLimit(DriveConstants.current40AmpContinuousCurrentLimit,DriveConstants.timeoutMS);
    rightMaster.configPeakCurrentLimit(DriveConstants.current40AmpPeakCurrentLimit,DriveConstants.timeoutMS);
    rightMaster.configPeakCurrentDuration(DriveConstants.current40AmpPeakCurrentDuration,DriveConstants.timeoutMS);
    rightMaster.enableCurrentLimit(currentLimitState); 

  }

  public static void init(){
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
  }

  public static void setBrakeMode(){
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
  }

  public static void setCoastMode(){
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftMaster.setNeutralMode(NeutralMode.Coast);
  }


  //Zeroes the heading of the robot encoder
  public void zeroHeading(){
    gyro.reset();
  }

      /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate(){
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void manualDrive(double move, double turn) {
    if(Math.abs(move)<.05) {
      move = 0;
    }
    if (Math.abs(turn)<.05){
      turn = 0;
    }
  drive.arcadeDrive(move, turn*.82, true);
  }

  public void autonDrive(double move, double turn){
    drive.arcadeDrive(move, turn);
  }

  public void driveStraight(double move, double angleSetpoint){
    turningValue = (angleSetpoint - gyro.getAngle()) * .01;
     //Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, move);
    drive.arcadeDrive(move, turningValue);
    }  

    public void stop(){
      drive.arcadeDrive(0, 0);
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gyro Angle",Math.IEEEremainder(gyro.getAngle(), 360));
/*     SmartDashboard.putNumber("turningValue",turningValue);
    SmartDashboard.putNumber("Left Master", leftMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Slave", leftSlave.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Master", rightMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Slave", rightSlave.getMotorOutputPercent());    */

  }
}
