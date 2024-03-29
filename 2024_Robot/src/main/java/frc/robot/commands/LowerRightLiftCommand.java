// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteSubsystem;

public class LowerRightLiftCommand extends Command {


  NoteSubsystem m_note;



  /** Creates a new LowerLiftCommand. */
  public LowerRightLiftCommand(NoteSubsystem note) {
    m_note = note;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(note);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        m_note.manualLowerRightLift();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_note.stopLift();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_note.lowerRightLimit();
    
  }
}
