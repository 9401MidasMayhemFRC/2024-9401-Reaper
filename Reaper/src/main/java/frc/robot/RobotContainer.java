// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auto;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AdjustShotByPose;
import frc.robot.commands.AimShooterAuto;
import frc.robot.commands.AmpHandoff;
import frc.robot.commands.AutoShootByPose;
import frc.robot.commands.AutoShootNoDrive;
import frc.robot.commands.DriveByController;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.SmartShootByPose;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroShooter;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.LEDs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final RackPinion m_rackPinion = new RackPinion();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  private final Shooter m_shooter = new Shooter();  
  private final LEDs m_led = new LEDs(1,300, this::getAuto);
  private final Feeder m_feeder = new Feeder(m_led);
  private final Elevator m_elevator = new Elevator();
  private final Amp m_amp = new Amp();
  private final PoseEstimator m_poseEstimator = new PoseEstimator(m_drive);
  private final IntakeNote m_intakeCommand = new IntakeNote(m_intake, m_rackPinion, m_feeder, m_elevator, m_led);
  private final DriveByController m_driveCommand = new DriveByController(m_drive, m_driverController);

  private final AmpHandoff m_ampHandoff = new AmpHandoff(m_amp, m_intake, m_elevator, m_led, m_rackPinion);
  private final AimShooterAuto m_aimShooterAuto = new AimShooterAuto(m_shooter, m_rackPinion, m_feeder, m_drive);
  private final AutoShootByPose m_autoShootByPose = new AutoShootByPose(m_shooter, m_drive, m_rackPinion,
      m_poseEstimator::getPose);
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private DemoMode m_demoMode = new DemoMode();
  public static boolean m_auto = false;

  private void configureAutoEvents() {

    NamedCommands.registerCommand("ConditionalCommand", new ConditionalCommand(new WaitCommand(15.0), new WaitCommand(0.0), m_feeder::getProx));

    NamedCommands.registerCommand("TelePose", new InstantCommand(() -> m_poseEstimator.setAuto(false)));
    NamedCommands.registerCommand("AutoPose", new InstantCommand(() -> m_poseEstimator.setAuto(true)));
    NamedCommands.registerCommand("Aim&ShootNote", m_aimShooterAuto);
    NamedCommands.registerCommand("ShootByPose", m_autoShootByPose);
    NamedCommands.registerCommand("RunShooter", new AutoShootNoDrive(m_shooter, m_rackPinion, m_poseEstimator::getPose));
    NamedCommands.registerCommand("StopShooter", new InstantCommand(()-> m_shooter.setShooterVelo(0.0)));
    NamedCommands.registerCommand("AdjustByPose",
        new AdjustShotByPose(m_shooter, m_rackPinion, m_drive, m_poseEstimator::getPose));
    NamedCommands.registerCommand("ZeroClimber", new ZeroClimber(m_climber));
    NamedCommands.registerCommand("Kickback", new InstantCommand(() -> m_feeder.setFeedVelo(-0.4))
        .andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> m_feeder.setFeedVelo(0.0))));
    NamedCommands.registerCommand("AutoFeed", m_intake.runCmd(0.8)
        .alongWith(new InstantCommand(() -> m_feeder.setFeedVelo(50.0))));
    NamedCommands.registerCommand("PerpIntake",
        m_intake.runCmd(0.8)
            .alongWith(new InstantCommand(() -> m_rackPinion.setPose(31.0)))
            .alongWith(new InstantCommand(() -> m_feeder.setFeedVelo(50.0)))
            .alongWith(new InstantCommand(() -> m_elevator.setPosition(ElevatorConstants.kRestPose))));
    NamedCommands.registerCommand("AmpHandoff", new AmpHandoff(m_amp, m_intake, m_elevator, m_led, m_rackPinion));
    NamedCommands.registerCommand("StopIntake",
        m_intake.stopCmd().alongWith(new InstantCommand(() -> m_rackPinion.setPose(0.5)))
            .alongWith(new InstantCommand(() -> m_feeder.setFeedVelo(0.0))));
    NamedCommands.registerCommand("StopDrive", new InstantCommand(() -> m_drive.stop()));
    NamedCommands.registerCommand("Pipeline0", switchLimelightPipeline("", 0));
    NamedCommands.registerCommand("Pipeline1", switchLimelightPipeline("", 1));
    NamedCommands.registerCommand("Pipeline2", switchLimelightPipeline("", 2));
    
    NamedCommands.registerCommand("ZeroShooter", new ZeroShooter(m_rackPinion));

    NamedCommands.registerCommand("TrackNote0", new InstantCommand(()->{
      m_poseEstimator.trackNote(0);
    m_elevator.setPosition(ElevatorConstants.kIntakePose);}));
    NamedCommands.registerCommand("TrackNote1", new InstantCommand(()-> {
      m_poseEstimator.trackNote(1);
    m_elevator.setPosition(ElevatorConstants.kIntakePose);}));
    NamedCommands.registerCommand("TrackNote2", new InstantCommand(()-> {
      m_poseEstimator.trackNote(2);
    m_elevator.setPosition(ElevatorConstants.kIntakePose);}));
    NamedCommands.registerCommand("TrackNote3", new InstantCommand(()-> {
      m_poseEstimator.trackNote(3);
    m_elevator.setPosition(ElevatorConstants.kIntakePose);}));
    NamedCommands.registerCommand("TrackNote4", new InstantCommand(()-> {
      m_poseEstimator.trackNote(4);
    m_elevator.setPosition(ElevatorConstants.kIntakePose);}));
    NamedCommands.registerCommand("StopNoteTracking", new InstantCommand(()-> {m_poseEstimator.stopNoteTracking();
    m_elevator.setPosition(ElevatorConstants.kRestPose);}));

  }

  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(m_poseEstimator::getPose,
        m_poseEstimator::resetOdometry,
        m_drive::getChassisSpeed,
        m_drive::drive,
        Auto.autoConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, m_drive);
  }

  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drive.setDefaultCommand(m_driveCommand);
    // Configure the trigger bindings
    configureAutoBuilder();
    configureBindings();
    configureAutoEvents();
    configureAutoChooser();
  }

  private void configureAutoChooser() {
    m_chooser = AutoBuilder.buildAutoChooser("Do Nothing");

    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.pov(180).onTrue(new InstantCommand(
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(15.20, 5.56), new Rotation2d()));
          } else {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(1.34, 5.57), new Rotation2d(Math.PI)));
          }
          m_poseEstimator.stopNoteTracking();
        }));
    m_driverController.pov(0).onTrue(new InstantCommand(
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(15.20, 5.56), new Rotation2d(Math.PI)));
          } else {
            m_poseEstimator.resetOdometry(new Pose2d(new Translation2d(1.34, 5.57), new Rotation2d(0)));
          }
          m_poseEstimator.stopNoteTracking();
        }));

    m_driverController.pov(270).onTrue(new ConditionalCommand(m_shooter.decrementVelo(), new WaitCommand(0.1), m_demoMode::isEnabled));

    m_driverController.pov(90).onTrue(new ConditionalCommand(m_shooter.incrementVelo(), new WaitCommand(0.1), m_demoMode::isEnabled));
    Command bButtonOnTrue = new InstantCommand(() -> {
          m_rackPinion.setPose(25.0);
          m_intake.run(-0.5);
          m_feeder.setFeedVelo(-0.4);});


    Command bButtonOnFalse = new InstantCommand(() -> {
          m_rackPinion.setPose(5.0);
          m_intake.stop();
          m_feeder.stopFeed();});

    m_driverController.b().onTrue(new ConditionalCommand(m_rackPinion.incrementHood(), bButtonOnTrue, m_demoMode::isEnabled))
                          .onFalse(new ConditionalCommand(new WaitCommand(0.1), bButtonOnFalse, m_demoMode::isEnabled));

    Command xButtonOnTrue = new InstantCommand(()-> m_shooter.setShooterVelo(12)).alongWith(new InstantCommand(()-> m_feeder.setFeedVelo(0.4)));

    Command xButtonOnFalse = new InstantCommand(()-> m_shooter.stopShooter()).alongWith(new InstantCommand(()-> m_feeder.stopFeed())).alongWith(new InstantCommand(()-> m_led.setTeamColors()));

    m_driverController.x().onTrue(new ConditionalCommand(m_rackPinion.decrementHood(), xButtonOnTrue, m_demoMode::isEnabled))
                          .onFalse(new ConditionalCommand(new WaitCommand(0.1), xButtonOnFalse, m_demoMode::isEnabled));
  
      
    

    Command LeftTrigger = new SmartShootByPose(m_shooter, m_drive, m_rackPinion, m_driverController, m_poseEstimator::getPose)
            .alongWith(new InstantCommand(() -> {m_poseEstimator.setAuto(false);}))
            .alongWith(switchLimelightPipeline("", 0));

    Command DemoLeftTrigger = new InstantCommand(()-> m_rackPinion.runRack()).alongWith(new InstantCommand(()-> m_shooter.RunShooter()));

    m_driverController.leftTrigger(0.25)
        .whileTrue(new ConditionalCommand(DemoLeftTrigger, LeftTrigger, m_demoMode::isEnabled))
        .onFalse(new ConditionalCommand(new InstantCommand(()-> m_rackPinion.stop()).alongWith(new InstantCommand(()-> m_shooter.stopShooter())),
                                        new InstantCommand(()-> switchLimelightPipeline("", 1)),
                                        m_demoMode::isEnabled));

    m_driverController.leftBumper()
        .whileTrue(m_intakeCommand.alongWith(new InstantCommand(() -> m_amp.setSpeed(-0.4))))
        .onFalse(new InstantCommand(() -> m_feeder.setFeedVelo(-0.4))
            .andThen(new WaitCommand(0.1)).andThen(new InstantCommand(() -> m_feeder.setFeedVelo(0.0)))
            .alongWith(new InstantCommand(() -> m_amp.setSpeed(0.0))));

    m_driverController.rightBumper()
        .whileTrue(new InstantCommand(() -> m_feeder.setFeedVelo(-.1)).alongWith(m_ampHandoff))
        .onFalse(new InstantCommand(() -> m_feeder.stopFeed()));

    m_driverController.rightTrigger(0.25)
        .onTrue(new InstantCommand(() -> {
          m_feeder.setFeedVelo(.40);
          m_amp.setSpeed(0.6);
          m_intake.run(0.8);
        }))
        .onFalse(new InstantCommand(() -> {
          m_feeder.stopFeed();
          m_amp.stop();
          m_intake.stop();

          if (m_demoMode.isEnabled()){
            m_led.setTeamColors();
          } else {
            m_led.setTeamColors();
          }
          


        }));

    m_driverController.back().onTrue(new InstantCommand(() -> m_climber.extend()));
    m_driverController.start().onTrue(new InstantCommand(() -> m_climber.retract()));

    m_driverController.a().onTrue(new InstantCommand(() -> m_elevator.setPosition(78.0)))
        .onFalse(new InstantCommand(() -> m_elevator.setPosition(ElevatorConstants.kRestPose)));

    Command yButton = new ZeroShooter(m_rackPinion).alongWith(new ZeroClimber(m_climber)).alongWith(new InstantCommand(()-> m_poseEstimator.stopNoteTracking()));

    Command yDemo = new InstantCommand(()-> m_rackPinion.setPose(5.0)).alongWith( new InstantCommand(()-> m_shooter.setVelo(0.0)));

    m_driverController.y().onTrue(new ConditionalCommand(yDemo, yButton, m_demoMode::isEnabled));


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

  }

  

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected().alongWith(new InstantCommand(()-> m_auto = true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getTestCommand() {
    return new ZeroShooter(m_rackPinion).alongWith(new ZeroClimber(m_climber));
  }

  public Command getTeleopCommand() {
    return new ZeroClimber(m_climber).alongWith(new InstantCommand(() -> m_poseEstimator.setAuto(false)).alongWith(new InstantCommand(()-> m_poseEstimator.stopNoteTracking())).alongWith(new InstantCommand(()-> m_auto = false)));
  }

  public Command switchLimelightPipeline(String name, int pipeline) {
    return new InstantCommand(() -> LimelightHelpers.setPipelineIndex(name, pipeline));
  }

  public boolean getAuto(){
    return m_auto;
  }

  public Command getAutoPeriodicCMD(){
    return new InstantCommand(()-> m_led.setTeamColors());
  }
}
