/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  public ExampleSubsystem() {
    private CANSparkMax kickerMotor =  new CANSparkMax(Constants.kickerMotor, MotorType.kBrushless);

    public Kicker() {
    kickerMotor.restoreFactoryDefaults();
    kickerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    kickerMotor.setInverted(false);
   }
public void kickit(double output) {
  kickerMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
