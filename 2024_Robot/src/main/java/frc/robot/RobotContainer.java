// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LowerLiftCommand;
import frc.robot.commands.Autos.AutoOne;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.NoteSubsystem;
import frc.robot.subsystems.TilterProfiledPIDSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final NoteSubsystem noteSubsystem = new NoteSubsystem();
  public final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final TilterProfiledPIDSubsystem tilterSubsystem = new TilterProfiledPIDSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Trigger buttonA = new JoystickButton(m_driverController, 1),
    buttonB = new JoystickButton(m_driverController,2),
    buttonX = new JoystickButton(m_driverController,3),
    buttonY = new JoystickButton(m_driverController,4),
    buttonLeftBumper = new JoystickButton(m_driverController,5),
    buttonRightBumper = new JoystickButton(m_driverController,6),
    buttonBack = new JoystickButton(m_driverController,7),
    buttonRightStick = new JoystickButton(m_driverController,10),
    buttonLeftStick = new JoystickButton(m_driverController,9),
    buttonStart = new JoystickButton(m_driverController,8);
  public POVButton buttonDpadN = new POVButton(m_driverController, 0, 0),
    buttonDpadE = new POVButton(m_driverController, 90, 0),
    buttonDpadS = new POVButton(m_driverController, 180, 0),
    buttonDpadW = new POVButton(m_driverController, 270, 0),
    buttonDpadNE = new POVButton(m_driverController, 45, 0),
    buttonDpadSE = new POVButton(m_driverController, 135, 0),
    buttonDpadSW = new POVButton(m_driverController, 225, 0),
    buttonDpadNW = new POVButton(m_driverController, 315, 0);


    SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

  noteSubsystem.setDefaultCommand(
    new RunCommand(
      () -> noteSubsystem.triggerIntake(
        m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis()),noteSubsystem));

        m_chooser.setDefaultOption("Default (does nothing)", new InstantCommand());
        m_chooser.addOption("Test", getAutonomousCommand());
        m_chooser.addOption("Test Auto", new AutoOne(m_robotDrive,noteSubsystem,m_LimelightSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {


    Command zeroGyro = new InstantCommand(m_robotDrive::zeroHeading);
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData("reset Gyro",zeroGyro);
    SmartDashboard.putData("test Auto",getAutonomousCommand());


    //Raise lift, Lower Lift, Tilt Pizza Box, Kick Out, Shoot Out, Intake
    buttonRightBumper.whileTrue(
      new InstantCommand(noteSubsystem::manualRaiseLift))
      .whileFalse(
        new InstantCommand(noteSubsystem::stopLift));

    buttonLeftBumper.whileTrue(new LowerLiftCommand(noteSubsystem)).whileFalse(new InstantCommand(noteSubsystem::stopLift));
    //buttonLeftBumper.whileTrue(new InstantCommand(noteSubsystem::manualLowerLift)).whileFalse(new InstantCommand(noteSubsystem::stopLift));

    buttonA.whileTrue(
      new InstantCommand(noteSubsystem::manualKick))
      .whileFalse(
        new InstantCommand(noteSubsystem::stopKick));

    buttonX.whileTrue(
      new InstantCommand(tilterSubsystem::manualTiltDown))
      .whileFalse(
        new InstantCommand(tilterSubsystem::stopTilter));
    
    buttonY.whileTrue(
      new InstantCommand(tilterSubsystem::manualTiltUp))
      .whileFalse(
        new InstantCommand(tilterSubsystem::stopTilter));

    buttonB.whileTrue(
      new InstantCommand(noteSubsystem::holdLiftPosition))
      .whileFalse(
        new InstantCommand(noteSubsystem::stopLift));
    

    buttonStart.whileTrue(new RunCommand(
      () -> m_robotDrive.setX(),
      m_robotDrive));

    buttonDpadN.onTrue(Commands.runOnce(() -> {tilterSubsystem.setGoal(2);
      tilterSubsystem.enable();},
      tilterSubsystem));
  

    buttonDpadS.onTrue(Commands.runOnce(() -> {tilterSubsystem.setGoal(.5);
      tilterSubsystem.enable();},
      tilterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

 
}
