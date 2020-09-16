/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  // Creating motor controllers
  private TalonSRX leftMaster = new TalonSRX(Constants.leftMaster);
  private TalonSRX leftSlave = new TalonSRX(Constants.leftSlave);
  private TalonSRX rightMaster = new TalonSRX(Constants.rightMaster);
  private TalonSRX rightSlave = new TalonSRX(Constants.rightSlave);

  // Drive shifters
  private DoubleSolenoid shifters = new DoubleSolenoid(Constants.pcmOne, Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);

  public DriveTrain() {
    // Make sure motors and not inverted
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    // Make sure slave motors are receving same power as master motors
    leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
    rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());
  }

  public void setTankDrive(double leftoutput, double rightoutput) { // Sets left motors to value of left joystick and right motors to value of right joystick
    leftMaster.set(ControlMode.PercentOutput, leftoutput);
    rightMaster.set(ControlMode.PercentOutput, rightoutput);
  }

  public void setArcadeDrive(double forwardPower, double turnPower) { // Move motors forward at a certain power, but changes that power to make robot turn
    leftMaster.set(ControlMode.PercentOutput, forwardPower + turnPower);
    rightMaster.set(ControlMode.PercentOutput, forwardPower - turnPower);
  }

  public void setDriveShifters(boolean state) {
    shifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  public boolean getShifterState() {
    return shifters.get() == DoubleSolenoid.Value.kForward;
  }

  public double powerToSpeed(double power) {
    double gearRatio = getShifterState() ? Constants.gearRatioHigh : Constants.gearRatioLow;
    return Constants.falconRPM * power / 60 * gearRatio * 2 * Constants.wheelDiameter * Math.PI;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
