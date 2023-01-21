package frc.robot.subsystems;

import static frc.robot.Constants.MecanumDrivetrianConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MecanumDrivetrain extends SubsystemBase {

  private WPI_TalonSRX frontLeftMotor;
  private WPI_VictorSPX rearLeftMotor;
  private WPI_TalonSRX frontRightMotor;
  private WPI_VictorSPX rearRightMotor;

  private MecanumDrive drivetrain;
  //private WheelSpeeds wheelSpeeds;
  //private RobotDriveBase roboDrive;
  private ShuffleboardTab tab;

  //private double frontLeft, frontRight, rearLeft, rearRight;

  public MecanumDrivetrain(ShuffleboardTab tab) {
    // Motors
    frontLeftMotor = new WPI_TalonSRX(leftFrontTalonID);
    rearLeftMotor = new WPI_VictorSPX(leftRearVictorID);

    frontRightMotor = new WPI_TalonSRX(rightFrontTalonID);
    rearRightMotor = new WPI_VictorSPX(rightRearVictorID);

    //frontLeftMotor.setInverted(true);
    //rearLeftMotor.setInverted(true);
    rearRightMotor.setInverted(true);
    frontRightMotor.setInverted(true);

    drivetrain = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    //wheelSpeeds = new WheelSpeeds(frontLeft, frontRight, rearLeft, rearRight);
    
    this.tab = tab;

    configureShuffleboardData();
  }

  // SHUFFLEBOARD
  private void configureShuffleboardData() {
    ShuffleboardLayout layout = tab.getLayout("Drivetrain Data", BuiltInLayouts.kGrid).withPosition(10, 0);
    layout.add(this);
    layout.add("Mecanum Drive Base", drivetrain);

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
    drivetrain.feed();
  }

  public void resetEncoderPositions() {
    frontLeftMotor.setSelectedSensorPosition(0);
    frontRightMotor.setSelectedSensorPosition(0);

    rearLeftMotor.setSelectedSensorPosition(0);
    rearRightMotor.setSelectedSensorPosition(0);
  }

  public double getFrontLeftEncoderPosition() {
    return frontLeftMotor.getSelectedSensorPosition();
  }

  public double getRearLeftEncoderPosition() {
    return rearLeftMotor.getSelectedSensorPosition();
  }
  public double getFrontRightEncoderPosition() {
    return frontRightMotor.getSelectedSensorPosition();
  }

  public double getRearRightEncoderPosition() {
    return rearRightMotor.getSelectedSensorPosition();
  }

  public double getFrontLeftEncoderVelocity() {
    return frontLeftMotor.getSelectedSensorVelocity();
  }

  public double getFrontRightEncoderVelocity() {
    return frontRightMotor.getSelectedSensorVelocity();
  }
  public double getRearLeftEncoderVelocity() {
    return rearLeftMotor.getSelectedSensorVelocity();
  }

  public double getRearRightEncoderVelocity() {
    return rearRightMotor.getSelectedSensorVelocity();
  }

  // Drive Cartesian Mode
  public void driveCartesian(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle){
    xSpeed = MathUtil.applyDeadband(xSpeed, speedDeadband);
    ySpeed = MathUtil.applyDeadband(ySpeed, speedDeadband);
    zRotation = MathUtil.applyDeadband(zRotation, rotationDeadband);

    var speeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotation, gyroAngle);

    frontLeftMotor.set(speeds.frontLeft * maxOutput);
    frontRightMotor.set(speeds.frontRight * maxOutput);
    rearLeftMotor.set(speeds.rearLeft * maxOutput);
    rearRightMotor.set(speeds.rearRight * maxOutput);

    //drivetrain.feed();
  }

  public void driveCartesianNew(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle){
    drivetrain.driveCartesian(xSpeed, ySpeed, zRotation, gyroAngle);
    //drivetrain.feed();
  }

  /* Inverse Kinematic Mode
  public WheelSpeeds driveCartesianIK(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle){
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

    // Compensate for gyro angle.
    var input = new Translation2d(xSpeed, ySpeed).rotateBy(gyroAngle.unaryMinus());

    double[] wheelSpeeds = new double[4];
    wheelSpeeds[MotorType.kFrontLeft.value] = input.getX() + input.getY() + zRotation;
    wheelSpeeds[MotorType.kFrontRight.value] = input.getX() - input.getY() - zRotation;
    wheelSpeeds[MotorType.kRearLeft.value] = input.getX() - input.getY() + zRotation;
    wheelSpeeds[MotorType.kRearRight.value] = input.getX() + input.getY() - zRotation;

    normalize(wheelSpeeds);
    drivetrain.feed();

    return new WheelSpeeds(
        wheelSpeeds[MotorType.kFrontLeft.value],
        wheelSpeeds[MotorType.kFrontRight.value],
        wheelSpeeds[MotorType.kRearLeft.value],
        wheelSpeeds[MotorType.kRearRight.value]);
  }

  protected static void normalize(double[] wheelSpeeds) {
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    for (int i = 1; i < wheelSpeeds.length; i++) {
      double temp = Math.abs(wheelSpeeds[i]);
      if (maxMagnitude < temp) {
        maxMagnitude = temp;
      }
    }
    if (maxMagnitude > 1.0) {
      for (int i = 0; i < wheelSpeeds.length; i++) {
        wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
      }
    }
  }
  */
}