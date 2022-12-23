/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDrivetrain;
import com.kauailabs.navx.frc.AHRS;


public class DriveMecanum extends CommandBase {
  /*
   * Creates a new DriveMecanum.
   */

  private MecanumDrivetrain m_drivetrain;
  private Supplier<Double> m_r, m_x, m_y, m_z;
  private XboxController m_xbox;
  private AHRS ahrs;

  public DriveMecanum(MecanumDrivetrain drivetrain, Supplier<Double> forward, Supplier<Double> strafe, Supplier<Double> zRotation,
   Supplier<Double> rAngle, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.m_drivetrain = drivetrain;
    this.m_x = forward;
    this.m_y = strafe;
    this.m_z = zRotation;
    this.m_r = rAngle;
    this.m_xbox = xbox;
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = m_x.get();
    double ySpeed = -m_y.get();
    double zRotation = m_z.get();
    double gyroAngle = m_r.get();

      
    if (m_xbox.getBButton() == true) {
     ahrs.zeroYaw();
    }
      

    m_drivetrain.driveCartesian(xSpeed, ySpeed, zRotation, gyroAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveCartesian(0.0, 0.0, 0.0, 0.0);	
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
