/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new intake.
   */
  private CANSparkMax intakeMotor =  new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless);
  DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);
  private boolean intaking = false;
  DigitalInput intakeSensor = new DigitalInput(Constants.intakeSensor);
  public int counter = 0;
  public boolean BallShadow = false;

  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMotor.setInverted(false);

  }
  public boolean getIntakingState() {
    return intaking;
  }

  public void setIntakingState(boolean state) {
    intaking = state;
  }
  public boolean getIntakePistonExtendStatus(){
    return intakePiston.get() == DoubleSolenoid.Value.kForward ? true : false;
  }

  public void setintakePiston(boolean state){
    intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
  public void setIntakePercentOutput(double value){
    intakeMotor.set(value);
  }
  public boolean getSeeBall(){
    return intakeSensor.get();
  }
  public void updateCounter(boolean dec, boolean eject){
    if(eject){
      counter = 0;
      return;
    }
    if(dec){
      counter--;
      return;
    }
    if(getSeeBall() && !BallShadow) {
      counter++;
      BallShadow = true;
    }else{
      BallShadow = false;
    }
  }
  public int getCounter(){
    return counter;
  }

//  private void updateSmartDashboard() {
//    SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakingState());
//    SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
//  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//    updateSmartDashboard();
    updateCounter(false, false);
  }
}
