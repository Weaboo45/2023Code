package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoDriveTimed extends CommandBase {


  private MecanumDrivetrain m_drivetrain;
  private Timer m_timer;
  private double m_speed, m_endTime, m_speed2, m_zRotation;
  private Rotation2d m_gyroAngle;

  /**
   * Creates a new AutoDriveTimed. double m_shotSpeed,
   */
  public AutoDriveTimed(MecanumDrivetrain drivetrain, double speed, double speed2, double zRotation,
   Rotation2d gyroAngle, double endTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_speed2 = speed2;
    m_zRotation = zRotation;
    m_gyroAngle = gyroAngle;
    m_endTime = endTime;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_timer.get() >= 5 && m_timer.get() <= 6) {
      m_drivetrain.driveCartesian(m_speed, m_speed2, m_zRotation, m_gyroAngle);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveCartesian(0, 0, 0, null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_endTime;
  }
}