/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTank extends CommandBase {
  /**
   * Creates a new DriveTank.
   */

  private Drivetrain drivetrain;
  private Supplier<Double> m_left, m_right;

  public DriveTank(Drivetrain drive, Supplier<Double> left, Supplier<Double> right) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    drivetrain = drive;
    m_left = left;
    m_right = right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_left.get();
    double right = m_right.get();
    drivetrain.tankDrive(left, right);  // null issue
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
