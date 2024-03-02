// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightCommand extends Command {
  /** Creates a new DriveStraightCommand. */


  DriveSubsystem swerve;
  double setpoint;
  double targetX;
  double forwardKp = 0.5;
  double angleKp = 0.01;
  double maxSpeed = 0.2;
  double additionalMaxSpeed;
  double initialHeading;
  double xOutput;
  double zOutput;
  double currentX;
  boolean isBackwards;


  public DriveStraightCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
