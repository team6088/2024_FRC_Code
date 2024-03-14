// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnCommand extends Command {

  DriveSubsystem swerve;
  private double setpoint;
  private double currentAngle;
  private double output, pidOutput;
  private double tolerance = 2;
  private double velocityTolerance = 6;



  PIDController pid = new PIDController(.2, 0, 0);
  /** Creates a new TurnCommand. */
  public TurnCommand(DriveSubsystem swerve, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.setpoint = setpoint;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.enableContinuousInput(-180,180);
    pid.setTolerance(tolerance, velocityTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerve.getPose().getRotation().getDegrees();

    pidOutput = pid.calculate(currentAngle, setpoint);
    pidOutput = MathUtil.clamp(pidOutput, -.4, .4);

    output *= DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, output);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);

    SmartDashboard.putBoolean("turn command at setpoint", pid.atSetpoint());
    SmartDashboard.putNumber("turn command pid output", output);
    SmartDashboard.putNumber("turn command error", pid.getPositionError());
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
