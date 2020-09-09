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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private TalonSRX leftMaster = new TalonSRX(Constants.leftMaster);
  private TalonSRX leftSlave = new TalonSRX(Constants.leftSlave);
  private TalonSRX rightMaster = new TalonSRX(Constants.rightMaster);
  private TalonSRX rightSlave = new TalonSRX(Constants.rightSlave);
  private DoubleSolenoid shifters = new DoubleSolenoid(Constants.pcmOne, Constants.driveTrainShiftersForward, Constants.driveTrainShiftersReverse);

  public DriveTrain() {
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
    rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());
  }

  public void setTankDrive(double leftoutput, double rightoutput) {
    leftMaster.set(ControlMode.PercentOutput, leftoutput);
    rightMaster.set(ControlMode.PercentOutput, rightoutput);
  }

  public void setArcadeDrive(double forwardPower, double turnPower) {
    leftMaster.set(ControlMode.PercentOutput, forwardPower + turnPower);
    rightMaster.set(ControlMode.PercentOutput, forwardPower - turnPower);
  }

  public void setDriveShifters(boolean state) {
    if (state) {
      shifters.set(DoubleSolenoid.Value.kForward);
    } else {
      shifters.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public boolean getShifterState() {
    return shifters.get() == DoubleSolenoid.Value.kForward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
