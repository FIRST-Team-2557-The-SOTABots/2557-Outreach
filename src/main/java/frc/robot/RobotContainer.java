// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FieldPoints;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterDelivery;
import frc.robot.subsystems.intakeSpin;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final intakeSpin m_intakeSpin = new intakeSpin();
  private final Shooter m_shooter = new Shooter(m_robotDrive);
  private final Delivery m_delivery = new Delivery();
  private final ShooterDelivery m_shooterDelivery = new ShooterDelivery();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerNamedCommands();
    // Configure the button bindings
    configureButtonBindings();

    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    // For convenience a programmer could change this when going to competition.
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith(""))
        : stream
    );

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();


    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void registerNamedCommands(){
    
    //shoot
    NamedCommands.registerCommand("Shoot", new RunCommand(
      () -> {m_shooter.shootAtTarget(FieldPoints.getHubPosition() , m_robotDrive);
         m_robotDrive.turnToFieldPoint(FieldPoints.getHubPosition(), m_driverController);}, m_shooter, m_robotDrive));

    //intake out
    NamedCommands.registerCommand("IntakeOut", new RunCommand(
      () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake));

      //intake fuel
    NamedCommands.registerCommand("IntakeBall", new RunCommand(
      () -> m_intakeSpin.setIntakeVoltage(12), m_intake));

    //intake in
    NamedCommands.registerCommand("IntakeIn", new RunCommand(
      ()-> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake));

              NamedCommands.registerCommand("Delivery slow", new RunCommand(
        ()-> m_delivery.setDeliveryVoltage(-10), m_delivery).alongWith(new RunCommand(
          ()-> m_shooterDelivery.setDeliveryVoltage(7), m_shooterDelivery)));

      //delivery
      NamedCommands.registerCommand("Delivery", new RunCommand(
        ()-> m_delivery.setDeliveryVoltage(-10), m_delivery).alongWith(new RunCommand(
          ()-> m_shooterDelivery.setDeliveryVoltage(Constants.ShooterConstants.kShooterDeliveryVoltage), m_shooterDelivery)));

      //stop shooter, shooter delivery, and delivery
      NamedCommands.registerCommand("StoppyMcStopFace", new RunCommand(
        ()-> m_shooter.setFlyWheelVoltage(0), m_shooter).alongWith(new RunCommand(
          ()-> m_shooterDelivery.setDeliveryVoltage(0), m_shooterDelivery)).alongWith(new RunCommand(
            ()-> m_delivery.setDeliveryVoltage(0), m_delivery)).alongWith(
              new RunCommand(
              () -> m_intakeSpin.setIntakeVoltage(0), m_intake)
           ));
      
    

    
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

    // m_driverController.a().whileTrue(new RunCommand(
    //   () -> m_robotDrive.turnToFieldPoint(FieldPoints.getHubPosition().getX(), FieldPoints.getHubPosition().getY(), m_driverController),
    //    m_robotDrive));

    // m_driverController.a().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake));

    // m_driverController.y().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake));

    // m_driverController.leftTrigger().onTrue(new RunCommand(
    //   () -> m_intake.setIntakeVoltage(12), m_intake)).onFalse(new RunCommand(
    //     () -> m_intake.setIntakeVoltage(0), m_intake));
        
    // m_driverController.b().onTrue(new RunCommand(
    //   () -> m_intake.setIntakeVoltage(-12), m_intake)
    //   .alongWith(new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(-12), m_shooterDelivery)
    //   ).alongWith(new RunCommand(
    //   () -> m_delivery.setDeliveryVoltage(-12), m_delivery)
    //   ).alongWith(
    //     new RunCommand(
    //   () -> m_shooter.setFlyWheelVoltage(-12), m_shooter)

    //   )).onFalse(new RunCommand(
    //   () -> m_intake.setIntakeVoltage(0), m_intake).alongWith(
    //     new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(0), m_shooterDelivery)
    //   ).alongWith(new RunCommand(
    //   () -> m_delivery.setDeliveryVoltage(0), m_delivery))
    //   .alongWith(
    //     new RunCommand(
    //   () -> m_shooter.setFlyWheelRPM(0), m_shooter)
    //   )
    //   );
    
    // m_driverController.leftBumper().onTrue(new RunCommand(
    //   () -> m_shooter.setFlyWheelRPM(2850), m_shooter).alongWith(new RunCommand(
    //     () ->m_shooterHood.setHoodAngle(50))
    //   )).onFalse(new RunCommand(
    //     () -> m_shooter.setFlyWheelRPM(0), m_shooter).alongWith(new RunCommand(
    //     () ->m_shooterHood.setHoodAngle(31.25))
    //   ));


      //   m_driverController.povUp().onTrue(new RunCommand(
      // () -> m_shooter.setFlyWheelRPM(2500), m_shooter).alongWith(new RunCommand(
      //   () ->m_shooterHood.setHoodAngle(25))
      // )).onFalse(new RunCommand(
      //   () -> m_shooter.setFlyWheelRPM(0), m_shooter).alongWith(new RunCommand(
      //   () ->m_shooterHood.setHoodAngle(31.25))
      // ));

    // m_driverController.rightBumper().onTrue(new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(12), m_shooter)).onFalse(new RunCommand(
    //     () -> m_shooterDelivery.setDeliveryVoltage(0), m_shooter));
    
    // m_driverController.rightTrigger().onTrue(new RunCommand(
    //   () -> m_delivery.stutter(-10), m_delivery)).onFalse(new RunCommand(
    //     () -> m_delivery.stutter(0), m_delivery));

    // m_driverController.x().whileTrue(new RunCommand(
    //   () -> m_robotDrive.turnToFieldPoint(
    //     FieldPoints.getHubPosition().getX(),
    //      FieldPoints.getHubPosition().getY(), 
    //      m_driverController), m_robotDrive));

    //shoot at hub
//     m_driverController.leftBumper().onTrue(new RunCommand(
//       () -> m_shooter.shootAtTarget(FieldPoints.getHubPosition()), m_shooter)
//       .alongWith(new RunCommand(
//         () -> m_robotDrive.turnToFieldPoint(FieldPoints.getHubPosition(), m_driverController), m_robotDrive)))
//         .onFalse(new RunCommand(
//       () -> m_shooter.setFlyWheelRPM(0), m_shooter)
//       .alongWith(new RunCommand(
//             () -> m_robotDrive.drive(
//                 -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
//                 -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
//                 -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
//                 true),
//             m_robotDrive)));

    m_driverController.rightBumper()
      .onTrue(
        new RunCommand(
          () -> m_delivery.setDeliveryVoltage(Constants.DeliveryConstants.kDeliveryVoltage), 
          m_delivery
        )
        .alongWith(
          new RunCommand(
            () -> m_shooterDelivery.setDeliveryVoltage(
              Constants.ShooterConstants.kShooterDeliveryVoltage
            ), 
            m_shooterDelivery
          )
        )
      )
      .onFalse(
        new RunCommand(
          () -> m_delivery.setDeliveryVoltage(0), 
          m_delivery
        )
          .alongWith(
            new RunCommand(
              () -> m_shooterDelivery.setDeliveryVoltage(0), 
              m_shooterDelivery
            )
          )
      );

//     //shoot at depot shuttle
//     // m_driverController.rightBumper().whileTrue(new RunCommand(
//     //   () -> m_shooter.shootAtTarget(FieldPoints.getDepotShuttle()), m_shooter)
//     //   .alongWith(new RunCommand(
//     //     () -> m_robotDrive.turnToFieldPoint(FieldPoints.getDepotShuttle(), m_driverController), m_robotDrive))
//     //     .until(
//     //       () -> m_shooter.isAtSetpoint() && m_robotDrive.isAtTurnTarget())
//     //     .andThen(new RunCommand(
//     //       () -> m_delivery.stutter(Constants.DeliveryConstants.kDeliveryVoltage), m_delivery))
//     //       .alongWith(new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(Constants.ShooterConstants.kShooterDeliveryVoltage), m_shooterDelivery)));

//     // Reset heading
    m_driverController.start().onTrue(Commands.runOnce(
      () -> m_robotDrive.zeroHeading()
      , m_robotDrive));

    // // Put intake out
    //  m_driverController.rightStick().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake));

    //       m_driverController.leftStick().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake));




 m_driverController.a().whileTrue(
  new ConditionalCommand(
    new RunCommand(() -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake),
    new RunCommand(() -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake),
    m_intake::isStowed  // returns true if stowed → deploy, false if deployed → stow
  )
);

    //intake
    m_driverController.leftTrigger().onTrue(new RunCommand(
      () -> m_intakeSpin.setIntakeVoltage(12), m_intakeSpin))
    .onFalse(new RunCommand(
        () -> m_intakeSpin.setIntakeVoltage(0), m_intakeSpin));
    
    //clear blockage
    m_driverController.b().onTrue(new RunCommand(
      () -> m_intakeSpin.setIntakeVoltage(-12), m_intake)
      .alongWith(new RunCommand(() -> m_delivery.setDeliveryVoltage(-12)))
      .alongWith(new RunCommand(() -> m_shooter.setFlyWheelVoltage(-12)))
      .alongWith(new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(-12))))
    .onFalse(new RunCommand(
      () -> m_intakeSpin.setIntakeVoltage(0), m_intake)
      .alongWith(new RunCommand(() -> m_delivery.setDeliveryVoltage(0)))
      .alongWith(new RunCommand(() -> m_shooter.setFlyWheelVoltage(0)))
      .alongWith(new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(0))));
      // .onFalse(new RunCommand(() -> m_intake.setIntakeVoltage(0), m_intake))
      // .onFalse(new RunCommand(() -> m_delivery.setDeliveryVoltage(0), m_delivery))
      // .onFalse(new RunCommand(() -> m_shooter.setFlyWheelVoltage(0)));

    //put climber up
    // m_driverController.x().onTrue(new RunCommand(
    //   () -> m_climber.setVoltage(4), m_climber)).onFalse(new RunCommand(
    //     () -> m_climber.setVoltage(0), m_climber));

    //zero intake
    m_driverController.y().onTrue(new InstantCommand(
      () -> m_intake.setUp(), m_intake
      )
    );
    
    // spinny mc spinface
    m_driverController.rightTrigger()
      .onTrue(
        new RunCommand(
          () -> m_shooter.setFlyWheelVoltage(12), 
          m_shooter
        )
      )
      .onFalse(
        new RunCommand(
          () -> m_shooter.setFlyWheelVoltage(0), 
          m_shooter
        )
      );
    


    
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
