// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCommand extends Command {
  private final DriveSubsystem m_drive;
  private final double m_move;
  private final double m_turn;

  /** Creates a new TimedDriveCommand. */
  public TimedDriveCommand(DriveSubsystem subsystem, double move, double turn) {
    m_drive = subsystem;
    m_move = move;
    m_turn = turn;
    addRequirements(m_drive);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.autonDrive(m_move,m_turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.autonDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
