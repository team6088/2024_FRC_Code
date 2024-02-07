// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveGyroCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_move;
  private final double m_angle;
  
  /** Creates a new TimedDriveCommand. */
  public TimedDriveGyroCommand(DriveSubsystem subsystem, double move, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_drive = subsystem;
  m_move = move;
  m_angle = angle;
  addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.driveStraight(m_move,m_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
