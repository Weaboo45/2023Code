/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private WPI_VictorSPX m_leftMasterMotor;
  private WPI_VictorSPX m_leftSlaveMotor;
  private WPI_TalonSRX m_rightMasterMotor;
  private WPI_TalonSRX m_rightSlaveMotor;
  

  private DifferentialDrive m_drive;
  private ShuffleboardTab m_tab;

  public Drivetrain(ShuffleboardTab tab) {
     //set up drive motors
    m_leftMasterMotor = new WPI_TalonSRX(FRONT_LEFT_VICTOR_ID);
    m_leftSlaveMotor = new WPI_VictorSPX(REAR_LEFT_TALON_ID);
    
    m_leftSlaveMotor.follow(m_leftMasterMotor);

    m_rightMasterMotor = new WPI_TalonSRX(FRONT_RIGHT_TALON_ID);
    m_rightSlaveMotor = new WPI_VictorSPX(REAR_RIGHT_TALON_ID);

    m_rightMasterMotor.setInverted(true);
    m_rightSlaveMotor.setInverted(true);

    m_rightSlaveMotor.follow(m_rightMasterMotor);

    m_drive = new DifferentialDrive(m_leftMasterMotor, m_rightMasterMotor);

    m_tab = tab;

    configureShuffleboardData();
  }

  private void configureShuffleboardData() {
    ShuffleboardLayout layout = m_tab.getLayout("Drivetrain Data", BuiltInLayouts.kGrid).withPosition(8, 0);
    layout.add(this);
    layout.add("Drive Base", m_drive);

    layout.addNumber("Left Encoder Pos", () -> getLeftEncoderPosition());
    layout.addNumber("Left Encoder Vel", () -> getLeftEncoderVelocity());
    layout.addNumber("Right Encoder Pos", () -> getRightEncoderPosition());
    layout.addNumber("Right Encoder Vel", () -> getRightEncoderVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_drive.feed();
  }

  public void resetEncoderPositions() {
    m_leftMasterMotor.setSelectedSensorPosition(0);
    m_rightMasterMotor.setSelectedSensorPosition(0);
  }

  public double getLeftEncoderPosition() {
    return m_leftMasterMotor.getSelectedSensorPosition();
  }

  public double getRightEncoderPosition() {
    return m_rightMasterMotor.getSelectedSensorPosition();
  }

  public double getLeftEncoderVelocity() {
    return m_leftMasterMotor.getSelectedSensorVelocity();
  }

  public double getRightEncoderVelocity() {
    return m_rightMasterMotor.getSelectedSensorVelocity();
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right, IS_TANKDRIVE_SQUARED);
  }

  public void arcadeDrive(double speed, double rotation) {

    speed = (Math.abs(speed) < SPEED_DEADBAND)?
            0 : (speed < 0)?
                (speed + SPEED_DEADBAND) / (1 - SPEED_DEADBAND):
                (speed - SPEED_DEADBAND) / (1 - SPEED_DEADBAND);
    
    rotation = (Math.abs(rotation) < ROTATION_DEADBAND)?
               0 : (rotation < 0)?
                   (rotation + ROTATION_DEADBAND) / (1 - ROTATION_DEADBAND):
                   (rotation - ROTATION_DEADBAND) / (1 -ROTATION_DEADBAND);

    m_drive.arcadeDrive(speed, rotation, IS_ARCADEDRIVE_SQUARED);
  }

  public void cheesyDrive(double speed, double rotation) {

    speed = (Math.abs(speed) < SPEED_DEADBAND)?
            0 : (speed < 0)?
            (speed + SPEED_DEADBAND) / (1 - SPEED_DEADBAND):
            (speed - SPEED_DEADBAND) / (1 - SPEED_DEADBAND);

    rotation = (Math.abs(rotation) < ROTATION_DEADBAND)?
               0 : (rotation < 0)?
               (rotation + ROTATION_DEADBAND) / (1 - ROTATION_DEADBAND):
               (rotation - ROTATION_DEADBAND) / (1 - ROTATION_DEADBAND);

    m_drive.curvatureDrive(speed, rotation, DOES_CHEESYDRIVE_PIVOT);
  }
}