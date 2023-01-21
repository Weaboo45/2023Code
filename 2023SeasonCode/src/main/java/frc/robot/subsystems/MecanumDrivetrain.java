package frc.robot.subsystems;

import static frc.robot.Constants.MecanumDrivetrianConstants.*;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MecanumDrivetrain extends SubsystemBase {

  //mecanum
  private WPI_TalonSRX m_frontLeftMotor;
  private WPI_VictorSPX m_rearLeftMotor;
  private WPI_TalonSRX m_frontRightMotor;
  private WPI_VictorSPX m_rearRightMotor;

  //tank
  private WPI_TalonSRX m_leftMasterMotor;
  private WPI_VictorSPX m_leftSlaveMotor;
  private WPI_TalonSRX m_rightMasterMotor;
  private WPI_VictorSPX m_rightSlaveMotor;

  private MecanumDrive mDrive;
  private DifferentialDrive tDrive;
  private ShuffleboardTab m_tab;
  private AHRS m_ahrs;

  public MecanumDrivetrain(ShuffleboardTab tab) {
      //Mecanum Drive motors
    m_frontLeftMotor = new WPI_TalonSRX(leftFrontTalonID);
    m_rearLeftMotor = new WPI_VictorSPX(leftRearVictorID);

    m_frontRightMotor = new WPI_TalonSRX(rightFrontTalonID);
    m_rearRightMotor = new WPI_VictorSPX(rightRearVictorID);

    //m_rearLeftMotor.setInverted(true);

    mDrive = new MecanumDrive(m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor);

    //set up drive motors for tank 

    m_leftMasterMotor = new WPI_TalonSRX(leftFrontTalonID);
    m_leftSlaveMotor = new WPI_VictorSPX(leftRearVictorID);
    
    m_leftSlaveMotor.follow(m_leftMasterMotor);

    m_rightMasterMotor = new WPI_TalonSRX(rightFrontTalonID);
    m_rightSlaveMotor = new WPI_VictorSPX(rightRearVictorID);     

    m_rightSlaveMotor.follow(m_rightMasterMotor);

    tDrive = new DifferentialDrive(m_leftMasterMotor, m_rightMasterMotor);

    m_ahrs = new AHRS(SPI.Port.kMXP);

    m_tab = tab;

    configureShuffleboardData();
  }

  private void configureShuffleboardData() {
    ShuffleboardLayout layout = m_tab.getLayout("Drivetrain Data", BuiltInLayouts.kGrid).withPosition(10, 0);
    layout.add(this);
    layout.add("Mecanum Drive Base", mDrive);

    layout.addNumber("Front Left Encoder Pos", () -> getFrontLeftEncoderPosition());
    layout.addNumber("Front Left Encoder Vel", () -> getFrontLeftEncoderVelocity());

    layout.addNumber("Rear Left Encoder Pos", () -> getRearLeftEncoderPosition());
    layout.addNumber("Rear Left Encoder Vel", () -> getRearLeftEncoderVelocity());

    layout.addNumber("Front Right Encoder Pos", () -> getFrontRightEncoderPosition());
    layout.addNumber("Front Right Encoder Vel", () -> getFrontRightEncoderVelocity());   

    layout.addNumber("Rear Right Encoder Pos", () -> getRearRightEncoderPosition());
    layout.addNumber("Rear Right Encoder Vel", () -> getRearRightEncoderVelocity());

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mDrive.feed();
    tDrive.feed();
  }

  public void resetEncoderPositions() {
    m_frontLeftMotor.setSelectedSensorPosition(0);
    m_frontRightMotor.setSelectedSensorPosition(0);

    m_rearLeftMotor.setSelectedSensorPosition(0);
    m_rearRightMotor.setSelectedSensorPosition(0);
  }

  public double getFrontLeftEncoderPosition() {
    return m_frontLeftMotor.getSelectedSensorPosition();
  }

  public double getRearLeftEncoderPosition() {
    return m_rearLeftMotor.getSelectedSensorPosition();
  }
  public double getFrontRightEncoderPosition() {
    return m_frontRightMotor.getSelectedSensorPosition();
  }

  public double getRearRightEncoderPosition() {
    return m_rearRightMotor.getSelectedSensorPosition();
  }

  public double getFrontLeftEncoderVelocity() {
    return m_frontLeftMotor.getSelectedSensorVelocity();
  }

  public double getFrontRightEncoderVelocity() {
    return m_frontRightMotor.getSelectedSensorVelocity();
  }
  public double getRearLeftEncoderVelocity() {
    return m_rearLeftMotor.getSelectedSensorVelocity();
  }

  public double getRearRightEncoderVelocity() {
    return m_rearRightMotor.getSelectedSensorVelocity();
  }

  //Auto Forward
  public void driveCartesian(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle){
    xSpeed = MathUtil.applyDeadband(xSpeed, speedDeadband);
    ySpeed = MathUtil.applyDeadband(ySpeed, speedDeadband);
    zRotation = MathUtil.applyDeadband(zRotation, rotationDeadband);
    gyroAngle = m_ahrs.getRotation2d();

    var speeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotation, gyroAngle);

    m_frontLeftMotor.set(speeds.frontLeft * m_maxOutput);
    m_frontRightMotor.set(speeds.frontRight * m_maxOutput);
    m_rearLeftMotor.set(speeds.rearLeft * m_maxOutput);
    m_rearRightMotor.set(speeds.rearRight * m_maxOutput);
  }

  public void tankDrive(double left, double right) {
    left = MathUtil.applyDeadband(left, speedDeadband);
    right = MathUtil.applyDeadband(right, speedDeadband);

    tDrive.tankDrive(left, right, isTankDriveSquared);
  }
}