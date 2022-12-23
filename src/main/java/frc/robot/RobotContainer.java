/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// TO DO (12/13/2022):
// Increase motor power while strafing
// Gyro zeroing button (B)
// Command for pivoting to 0/90/180/270 using D-pad

package frc.robot;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.MecanumDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /// SHUFFEBOARD TAB ///
  // This object is where we will put the data we want to see on the shuffleboard
  // when the robot is running
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Competition Robot");
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  
  /// SUBSYSTEMS ///
  private final MecanumDrivetrain m_mMecanumDrivetrain = new MecanumDrivetrain(m_tab);
  private static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  /// CONTROLLERS & BUTTONS ///
  private final XboxController m_xbox = new XboxController(0);

  /// COMMANDS ///
  private final AutoDriveTimed m_autoDriveTimedForward = new AutoDriveTimed( m_mMecanumDrivetrain, 0.5, 0.5 , 6.5, 0.0, 0.0);
  //private f qal AutoDriveTimed m_autoDriveTimedReverse = new AutoDriveTimed( m_drivetrain, -0.5, -0.5, 1.0);
  
  //private final DriveTank m_driveTank = new DriveTank(m_drivetrain, () -> -m_xbox.getLeftY(),
  //    () -> -m_xbox.getRightY());

 // private final DriveCheesy m_driveCheesyTriggers = new DriveCheesy(m_drivetrain,
   //   () -> m_xbox.getLeftTriggerAxis() - m_xbox.getRightTriggerAxis(), () -> m_xbox.getLeftX());
      
  private final DriveMecanum m_fieldDrive = new DriveMecanum(m_mMecanumDrivetrain, () -> m_xbox.getLeftY(), ()-> m_xbox.getLeftX(),
      ()-> -m_xbox.getRightX(), ()-> ahrs.getAngle(), m_xbox);


  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the initial default commands
    configureInitialDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
    // Configure the Shuffleboard Command Buttons
    configureShuffleboardData();
    
  }

  /**
   * Use this command to define {@link Shuffleboard} buttons using a
   * {@link ShuffleboardTab} and its add() function. You can put already defined
   * Commands,
   */
  private void configureShuffleboardData() {
    Shuffleboard.selectTab(m_tab.getTitle());
    
    m_chooser.setDefaultOption("First Course", m_autoDriveTimedForward);
    //m_chooser.addOption("Drive Reverse", m_autoDriveTimedReverse);
    m_chooser.addOption("Nothing", null);

    ShuffleboardLayout drivingStyleLayout = m_tab.getLayout("driving styles", BuiltInLayouts.kList)
    .withPosition(0, 0).withSize(2, 6)
    .withProperties(Map.of("label position", "BOTTOM"));
    
    //drivingStyleLayout.add("Tank drive",
    //new InstantCommand(() -> m_drivetrain.setDefaultCommand(m_driveTank), m_drivetrain));

    //drivingStyleLayout.add("Cheesy Drive with Triggers",
    //new InstantCommand(() -> m_drivetrain.setDefaultCommand(m_driveCheesyTriggers), m_drivetrain));

    drivingStyleLayout.add("Mecanum Drive",
    new InstantCommand(() -> m_mMecanumDrivetrain.setDefaultCommand(m_fieldDrive), m_mMecanumDrivetrain));

    drivingStyleLayout.add("Gyro Reset",
    new InstantCommand(()-> ahrs.zeroYaw()));


    ShuffleboardLayout mecanumSensor = m_tab.getLayout("Mecanum Sensors", BuiltInLayouts.kGrid)
    .withPosition(6, 0).withSize(3, 3)
    .withProperties(Map.of("lable psition", "BOTTOM"));
    mecanumSensor.addNumber("Gyro", ()-> ahrs.getAngle())
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);


    ShuffleboardLayout controllerLayout = m_tab.getLayout("xbox", BuiltInLayouts.kGrid)
    .withPosition(3, 0).withSize(2, 6)
    .withProperties(Map.of("label position", "BOTTOM"));
    controllerLayout.addNumber("left y", () -> -m_xbox.getLeftY())
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left x", () -> m_xbox.getLeftX())
    .withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left trigger", () -> m_xbox.getLeftTriggerAxis())
    .withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right y", () -> -m_xbox.getRightY())
    .withPosition(2, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right x", () -> m_xbox.getRightX())
    .withPosition(2, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right trigger", () -> m_xbox.getRightTriggerAxis())
    .withPosition(2, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);

    m_tab.add("Auto Chooser", m_chooser)
    .withPosition(0, 6).withSize(5, 2)
    .withWidget(BuiltInWidgets.kSplitButtonChooser);   
  }
  
  /**
   * Use this method to define the default commands of subsystems. 
   * Default commands are ran whenever no other commands are using a specific subsystem.
   */
  private void configureInitialDefaultCommands() {
    m_mMecanumDrivetrain.setDefaultCommand(m_fieldDrive);
  }
  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }
  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public void displayValues() {
  SmartDashboard.putData(m_mMecanumDrivetrain);
  }
}