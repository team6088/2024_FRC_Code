// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class ReleasePieceCommand extends Command {

  private final IntakeSubsystem m_intake;
  private final double m_speed;

  /** Creates a new ReleasePieceCommand. */
  public ReleasePieceCommand(IntakeSubsystem subsystem, double speed) {
    m_intake = subsystem;
    m_speed = speed;
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(  ) {
    m_intake.release();
    m_intake.manualArmControl(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopArmRaise();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
