/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import sun.security.jca.ProviderConfig;

import java.lang.Math;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  // Creating motor controllers

  private double kP = 1;
  private double kI = 0;
  private double kD = 0;

  private TalonFX leftMaster = new TalonFX(Constants.leftMaster);
  private TalonFX leftSlave = new TalonFX(Constants.leftSlave);
  private TalonFX rightMaster = new TalonFX(Constants.rightMaster);
  private TalonFX rightSlave = new TalonFX(Constants.rightSlave);

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.wheelDistance));

  private PIDController rightPIDController = new PIDController(kP, kI, kD);
  private PIDController leftPIDController = new PIDController(kP, kI, kD);

  // Drive shifters
  private DoubleSolenoid shifters = new DoubleSolenoid(Constants.pcmOne, Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);

  private ChassisSpeeds currentSpeeds;

  public DriveTrain() {
    double xPosition = 0;
    double yPosition = 0;
    
    // Configure motors
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();

    // Make sure motors are not inverted
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    // Make sure slave motors are receving same power as master motors
    leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
    rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());
  }

  public double getGearRatio() {
    return getShifterState() ? Constants.gearRatioHigh : Constants.gearRatioLow;
  }

  public double getRobotX() {
    return 0; // Fix this based on sensor values
  }

  public double getRobotY() {
    return 0; // Fix this based on sensor values
  }

  public double getRobotAngle() {
    return 0; // Fix this based on sensor values
  }

  public DifferentialDriveWheelSpeeds getSpeeds() { // Returns a differential drive wheel speeds with left and right speeds
    // Left wheel speed in meters per second
    double leftWheelSpeed = Units.inchesToMeters(leftMaster.getSelectedSensorVelocity() * 10 / 4096 * getGearRatio() * Constants.wheelDiameter * Math.PI);

    // Right wheel speed in meters per second
    double rightWheelSpeed = Units.inchesToMeters(rightMaster.getSelectedSensorVelocity() * 10 / 4096 * getGearRatio() * Constants.wheelDiameter * Math.PI);

    return new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
  }

  public ChassisSpeeds getRobotSpeed() { // Returns ChassisSpeeds object holding forward and angular velocity of robot
  return kinematics.toChassisSpeeds(getSpeeds());
  }

  public void setTankDrive(double leftoutput, double rightoutput) { // Sets left motors to value of left joystick and right motors to value of right joystick
    leftMaster.set(ControlMode.PercentOutput, leftoutput);
    rightMaster.set(ControlMode.PercentOutput, rightoutput);
  }

  public void setTankDriveWithPID(double leftOutput, double rightOutput) {
    DifferentialDriveWheelSpeeds wheelSpeeds = getSpeeds();
    leftMaster.set(ControlMode.PercentOutput, leftPIDController.calculcate(wheelSpeeds.leftMetersPerSecond, leftOutput));
    rightMaster.set(ControlMode.PercentOutput, rightPIDController.calculate(wheelSpeeds.rightMetersPerSecond, rightOutput));
  }

  public void setArcadeDrive(double forwardPower, double turnPower) { // Move motors forward at a certain power, but changes that power to make robot turn
    leftMaster.set(ControlMode.PercentOutput, forwardPower + turnPower);
    rightMaster.set(ControlMode.PercentOutput, forwardPower - turnPower);
  }

  public void setDriveFromChassisSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    leftMaster.set(ControlMode.PercentOutput, wheelSpeeds.leftMetersPerSecond);
    rightMaster.set(ControlMode.PercentOutput, wheelSpeeds.rightMetersPerSecond);
  }

  public void setDriveShifters(boolean state) {
    shifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public boolean getShifterState() {
    return shifters.get() == DoubleSolenoid.Value.kForward;
  }

  public double powerToSpeed(double power) {
    return Constants.falconRPM * power / 60 * getGearRatio() * Constants.wheelDiameter * Math.PI;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void updateSmartDashboard() {
    SmartDashboard.putBoolean("Shifter State", getShifterState());

    SmartDashboard.putNumber("Robot x position", getRobotX());
    SmartDashboard.putNumber("Robot y position", getRobotY());

    SmartDashboard.putNumber("Robot angle", getRobotAngle());
    
    ChassisSpeeds currentSpeeds = getRobotSpeed();

    SmartDashboard.putNumber("Left motor PID error", leftPIDController.getPositionError());
    SmartDashbaord.putNumber("Right motor PID error", rightPIDController.getPositionError());
    
    SmartDashboard.putNumber("Linear velocity", currentSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Angular velocity", currentSpeeds.omegaRadiansPerSecond);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentSpeeds = getRobotSpeed();
    updateSmartDashboard();
  }
}
