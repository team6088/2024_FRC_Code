// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public static Spark armRaise = new Spark(Constants.IntakeConstants.armRaisePort);
  public static Spark armExtend = new Spark(Constants.IntakeConstants.armExtendPort);
  public static DigitalInput retractLimitSwitch = new DigitalInput(Constants.IntakeConstants.retractLimitSwitch);
  public static DigitalInput upperLimitSwitch = new DigitalInput(Constants.IntakeConstants.upperLiftLimitSwitch);
  public static Compressor compressor = new Compressor(0,PneumaticsModuleType.CTREPCM);
  public static DoubleSolenoid clampSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 2);
  public static WPI_TalonFX talon = new WPI_TalonFX(4);
  public static double talonLowerLimit = 2;
  public static double talonUpperLimit = 15;
  public static double initPosition = 0;
  public static double talonPosition;
  public static boolean overExtend;
  public static boolean overRetract;


  public static Spark light = new Spark(2);
  //public static AnalogInput stringPot = new AnalogInput(0);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    talon.setInverted(true);
    talon.setNeutralMode(NeutralMode.Brake);


  }

  
  public static void init() {
    clampSolenoid.set(Value.kReverse);

    talon.setSelectedSensorPosition(initPosition);
    talon.setInverted(true);
    //talon.setInverted(false);
  }

  public static void raiseArm(){
    if (upperLimitSwitch.get()){
      armRaise.set(Constants.IntakeConstants.armRaiseSpeed);
    } else {
      armRaise.set(0);
    }}


  public static void slowRaiseArm(){
    armRaise.set(Constants.IntakeConstants.slowArmRaiseSpeed);
  }

  public static void lowerArm(){
    armRaise.set(Constants.IntakeConstants.armLowerSpeed);
  }

  public static void stopArmRaise (){
    armRaise.set(0);
  }

  public void manualArmControl(double speed){
    if (speed > 0.15) {
      armRaise.set(.15);
      //light.set(speed);
    }
    else {
      armRaise.set(speed*.8);
      //light.set(speed);
    }
  }

  public static void talonExtendArm(){
    talonPosition = talon.getSelectedSensorPosition()/2048/4*.5;
    if (talonPosition > talonUpperLimit) {
      talon.set(0);
      overExtend = true;
    }
    else {
      talon.set(0.3);
      overExtend = false;
    }
  }

  public static void talonRetractArm(){
    talonPosition = talon.getSelectedSensorPosition()/2048/4*.5;
    if (talonPosition < talonLowerLimit){
      talon.set(0);
      overRetract = true;
    } else { 
     talon.set(-.3);
    overRetract = false;
    }
  }
  

public static void extendArm(){
  armExtend.set(Constants.IntakeConstants.armExtendSpeed);
}
  
public static void retractArm(){
  if (retractLimitSwitch.get()){
  armExtend.set(Constants.IntakeConstants.armRetractSpeed);
  }
  else {
    armExtend.set(0);
  }
}

public static void stopTalon(){
  talon.set(0);
}

public static void stopArmExtend(){
  armExtend.set(0);
}

public static void release(){
  clampSolenoid.set(Value.kForward);
}

public static void clamp(){
  clampSolenoid.set(Value.kReverse);
}

/*   public static void controlRaiseArm(){
    if (stringPot.getVoltage()>3.40){ //starts around 4.7 down, 3.5 is near optimal for shooting
    armRaise.set(-.57); //90% output below nearly fully up
    } else {//was 3.35
      armRaise.set(-.06); //Very low output if gone too far
    }
} */




/*   public static void holdAndShoot (){
    if (stringPot.getVoltage()>3.54){ //starts around 4.7 down, 3.5 is near optimal for shooting
      leftBucketLiftMotor.set(-Constants.IntakeConstants.bucketLiftSpeed);
      rightBucketLiftMotor.set(Constants.IntakeConstants.bucketLiftSpeed);
      } else if (stringPot.getVoltage()<3.32){//was 3.35
        leftBucketLiftMotor.set(.12);
        rightBucketLiftMotor.set(-.12); 
      }
      else {
      leftBucketLiftMotor.set(-Constants.IntakeConstants.liftMotorMinimumLimit);
      rightBucketLiftMotor.set(Constants.IntakeConstants.liftMotorMinimumLimit);
    }
      intakeMotor.set(Constants.IntakeConstants.releaseSpeed);
  } */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    talonPosition = talon.getSelectedSensorPosition()/2048/4*.5;
    //SmartDashboard.putBoolean("Retract Limit", retractLimitSwitch.get());
/*     SmartDashboard.putNumber("String Pot Raw Voltage", stringPot.getVoltage());
    SmartDashboard.putNumber("String Pot Average Voltage", stringPot.getAverageVoltage());
    SmartDashboard.putNumber("right lift motor", rightBucketLiftMotor.get());
    SmartDashboard.putNumber("left lift motor", leftBucketLiftMotor.get()); */
    SmartDashboard.putNumber("Talon Position", talonPosition);
    SmartDashboard.putNumber("Talon Retract Limit", talonLowerLimit);
    SmartDashboard.putNumber("Talon Extend Limit", talonUpperLimit);
    SmartDashboard.putNumber("Talon Output", talon.get());
    SmartDashboard.putBoolean("Talon Direction", talon.getInverted());
    SmartDashboard.putBoolean("Over Extend", overExtend);
    SmartDashboard.putBoolean("Over Retract", overRetract);
  }

}

