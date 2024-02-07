// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RaiseArmCommand;
import frc.robot.commands.ReleasePieceCommand;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.commands.TimedDriveGyroCommand;
import frc.robot.commands.TurnToAnglePID;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  //private final String check1 = "Is the robot on?";
  //private final String check2 = "Set Autonomous";
  //private final String check3 = "Check Controllers";


  XboxController xboxController = new XboxController(0);
  Joystick logitechController = new Joystick(1);
  Trigger buttonA = new JoystickButton(xboxController, 1),
  buttonB = new JoystickButton(xboxController,2),
  buttonX = new JoystickButton(xboxController,3),
  buttonY = new JoystickButton(xboxController,4),
  buttonLeftBumper = new JoystickButton(xboxController,5),
  buttonRightBumper = new JoystickButton(xboxController,6),
  buttonBack = new JoystickButton(xboxController,7),
  buttonRightStick = new JoystickButton(xboxController,10),
  buttonLeftStick = new JoystickButton(xboxController,9),
  buttonStart = new JoystickButton(xboxController,8);
  public POVButton buttonDpadN = new POVButton(xboxController, 0, 0),
  buttonDpadE = new POVButton(xboxController, 90, 0),
  buttonDpadS = new POVButton(xboxController, 180, 0),
  buttonDpadW = new POVButton(xboxController, 270, 0),
  buttonDpadNE = new POVButton(xboxController, 45, 0),
  buttonDpadSE = new POVButton(xboxController, 135, 0),
  buttonDpadSW = new POVButton(xboxController, 225, 0),
  buttonDpadNW = new POVButton(xboxController, 315, 0);

    //VERIFY LOGITECH BUTTONS!
  Trigger logitechFingerTrigger = new JoystickButton(logitechController, 1),
  logitechThumbButton = new JoystickButton(logitechController, 2),
  logitechButton11 = new JoystickButton(logitechController,11),
  logitechButton12 = new JoystickButton(logitechController,12),
  logitechButton7 = new JoystickButton(logitechController,7),
  logitechButton8 = new JoystickButton(logitechController,8),
  logitechButton9 = new JoystickButton(logitechController,9),
  logitechButton10 = new JoystickButton(logitechController,10),
  
  logitechButton3 = new JoystickButton(logitechController,3),
  logitechButton4 = new JoystickButton(logitechController,4),
  logitechButton5 = new JoystickButton(logitechController,5),
  logitechButton6 = new JoystickButton(logitechController,6),
  logitechButtonDpadS = new POVButton(logitechController, 180, 0),
  logitechButtonDpadE = new POVButton(logitechController, 90, 0),
  logitechButtonDpadN = new POVButton(logitechController, 0, 0),
  logitechButtonDpadW = new POVButton(logitechController, 270, 0);


  SendableChooser<Command> chooser = new SendableChooser<>();
  private final Command timedMoveForStraight = new TimedDriveCommand(driveSubsystem, -.5, 0).withTimeout(2.5); //Drives forwards
  private final Command CableTrayReverse = new TimedDriveCommand(driveSubsystem, .75, 0).withTimeout(2.55); //Drives backwards 
  //2.8 seconds at .75 speed works over the wire strip
  private final Command NoCableTrayReverse = new TimedDriveCommand(driveSubsystem, .75, 0).withTimeout(1.8); //Drives backwards 
  //2.8 seconds at .75 speed works over the wire strip
  private final Command doNothing = new TimedDriveCommand(driveSubsystem, 0, 0).withTimeout(0);
  private final Command CubeAndCable = new SequentialCommandGroup(
    new RaiseArmCommand(intakeSubsystem, -.5).withTimeout(2),
    new ReleasePieceCommand(intakeSubsystem, -.3).withTimeout(1),
    new TimedDriveCommand(driveSubsystem, .75, 0).withTimeout(2.55));
    private final Command CubeAndNoCable = new SequentialCommandGroup(
      new RaiseArmCommand(intakeSubsystem, -.5).withTimeout(2),
      new ReleasePieceCommand(intakeSubsystem, -.3).withTimeout(1),
      new TimedDriveCommand(driveSubsystem, .75, 0).withTimeout(1.8));
      
      

  // Chooser for choosing drive style command
  SendableChooser<Command> driveChooser = new SendableChooser<>();
  private final Command classicDrive = new RunCommand(() -> driveSubsystem.manualDrive(xboxController.getLeftY(), 
        xboxController.getLeftX()), driveSubsystem);
  private final Command leftStickTurn = new RunCommand(() -> 
      driveSubsystem.manualDrive(xboxController.getLeftTriggerAxis()-xboxController.getRightTriggerAxis(), 
      xboxController.getRawAxis(0)), driveSubsystem);
  private final Command rightStickTurn = new RunCommand(() -> 
      driveSubsystem.manualDrive(xboxController.getLeftTriggerAxis()-xboxController.getRightTriggerAxis(),
      xboxController.getRawAxis(4)), driveSubsystem);
/* 

  private final Command MoveTurnMoveGyro = new SequentialCommandGroup(
    new TimedDriveGyroCommand(driveSubsystem, .5, 0).withTimeout(2),
    new TurnToAnglePID(90, driveSubsystem).withTimeout(2),
    new TimedDriveGyroCommand(driveSubsystem, -.5, 90).withTimeout(2));

  private final Command MoveTurnMove = new SequentialCommandGroup(
    new TimedDriveCommand(driveSubsystem, .5, 0).withTimeout(3),
    new TimedDriveCommand(driveSubsystem, 0, .4).withTimeout(3),
    new TimedDriveGyroCommand(driveSubsystem, .5, 0).withTimeout(3)); */


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



    //run arm on logitech
     intakeSubsystem.setDefaultCommand(
      new RunCommand(() ->
      intakeSubsystem.manualArmControl(logitechController.getRawAxis(1)), intakeSubsystem)
    );



    // Add commands to the autonomous command chooser
    chooser.setDefaultOption("Long Reverse", CableTrayReverse);
    chooser.addOption("Short Reverse", NoCableTrayReverse);
    chooser.addOption("Cube Long Side", CubeAndCable);
    chooser.addOption("Cube Short Side", CubeAndNoCable);
    //chooser.addOption("Drop Straight and Reverse Power Station", dropStraightMiddle);
    //chooser.addOption("MoveTurnMoveGyro", MoveTurnMoveGyro);
    //chooser.addOption("Move Turn Move", MoveTurnMove);
    chooser.addOption("Do Nothing", doNothing);

    driveChooser.setDefaultOption("Classic Drive Style", classicDrive);
    driveChooser.addOption("Left Stick Turn", leftStickTurn);
    driveChooser.addOption("Right Stick Turn", rightStickTurn);



    // Put the chooser on the dashboard
    SmartDashboard.putData(chooser);
    SmartDashboard.putData(driveChooser);
    //SmartDashboard.putString("checklist", check1 + "\r" + check2 + "\r" + check3);



    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Turn to Gyro Anle with Dpad
/*     buttonDpadN.whenPressed(new TurnToAnglePID(0, driveSubsystem).withTimeout(3));
    buttonDpadE.whenPressed(new TurnToAnglePID(-90, driveSubsystem).withTimeout(3));
    buttonDpadS.whenPressed(new TurnToAnglePID(180, driveSubsystem).withTimeout(3));
    buttonDpadW.whenPressed(new TurnToAnglePID(90 , driveSubsystem).withTimeout(3)); */
    
    

    //Raise and lower the arm
    buttonB.whileTrue(
      new InstantCommand(IntakeSubsystem::lowerArm, intakeSubsystem))
        .whileFalse(
          new InstantCommand(IntakeSubsystem::stopArmRaise, intakeSubsystem)
        );
     
    buttonA.whileTrue(
        new InstantCommand(IntakeSubsystem::raiseArm, intakeSubsystem))
        .whileFalse(
          new InstantCommand(IntakeSubsystem::stopArmRaise, intakeSubsystem)
        );

    buttonRightBumper.whileTrue(
      new InstantCommand(IntakeSubsystem::slowRaiseArm, intakeSubsystem))
      .whileFalse(
        new InstantCommand(IntakeSubsystem::stopArmRaise, intakeSubsystem)
      );


    // Extend and Retract the arm
    buttonX.whileTrue(
      new InstantCommand(IntakeSubsystem::retractArm, intakeSubsystem))
      .whileFalse(
        new InstantCommand(IntakeSubsystem::stopArmExtend, intakeSubsystem)
      );
    
    buttonA.whileTrue(
      new InstantCommand(IntakeSubsystem::extendArm, intakeSubsystem))
      .whileFalse(
        new InstantCommand(IntakeSubsystem::stopArmExtend, intakeSubsystem)
      );

      /* logitechButton7.whileTrue(
        new InstantCommand(IntakeSubsystem::retractArm, intakeSubsystem))
        .whileFalse(
          new InstantCommand(IntakeSubsystem::stopArmExtend, intakeSubsystem)
        );
      
      logitechButton8.whileTrue(
        new InstantCommand(IntakeSubsystem::extendArm, intakeSubsystem))
        .whileFalse(
          new InstantCommand(IntakeSubsystem::stopArmExtend, intakeSubsystem)
        );
 */
        
       logitechButton7.whileTrue(
        new InstantCommand(IntakeSubsystem::talonRetractArm, intakeSubsystem))
        .whileFalse(
          new InstantCommand(IntakeSubsystem::stopTalon, intakeSubsystem)
        );
      
      logitechButton8.whileTrue(
        new InstantCommand(IntakeSubsystem::talonExtendArm, intakeSubsystem))
        .whileFalse(
          new InstantCommand(IntakeSubsystem::stopTalon, intakeSubsystem)
        ); 


    // Open and close the clamp
      logitechFingerTrigger.onTrue(
        new InstantCommand(IntakeSubsystem::release, intakeSubsystem)
      );

      logitechThumbButton.onTrue(
        new InstantCommand(IntakeSubsystem::clamp, intakeSubsystem)
      );
          

    // Set drive motor brake mode
/*       logitechButton7.onTrue(
        new InstantCommand(DriveSubsystem::setCoastMode, driveSubsystem))
       ;
      
      logitechButton8.whileTrue(
        new InstantCommand(DriveSubsystem::setBrakeMode, driveSubsystem)
        ); */


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }

  public Command getDriveCommand() {
    return driveChooser.getSelected();
  }





}
