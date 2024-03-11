// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteSubsystem;

public class RaiseLiftCommand extends Command {


  NoteSubsystem m_note;
  Double m_speed;



  /** Creates a new LowerLiftCommand. */
  public RaiseLiftCommand(NoteSubsystem note, double speed) {
    m_note = note;
    m_speed = speed;
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
        m_note.manualRaiseLift(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_note.stopLift();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_note.upperEncoderLimit(); 
    
  }
}
