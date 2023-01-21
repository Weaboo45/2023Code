/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDrivetrain;
//import com.kauailabs.navx.frc.AHRS;


public class DriveMecanum extends CommandBase {
  /*
   * Creates a new DriveMecanum.
   */

  private MecanumDrivetrain drivetrain;
  private double xSpeed, ySpeed, zRotation;
  private Rotation2d gyro;
  
  public DriveMecanum(MecanumDrivetrain drivetrain, Supplier<Double> forward, Supplier<Double> strafe, Supplier<Double> zRotation,
   Supplier<Rotation2d> ahrs) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.xSpeed = forward.get();
    this.ySpeed = strafe.get();
    this.zRotation = zRotation.get();
    this.gyro = ahrs.get();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveCartesian(xSpeed, ySpeed, zRotation, gyro);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveCartesian(0.0, 0.0, 0.0, gyro);	
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
