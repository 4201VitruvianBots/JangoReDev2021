/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  private CANSparkMax shooterL =  new CANSparkMax(Constants.shooterL, MotorType.kBrushless);
  private CANSparkMax shooterR =  new CANSparkMax(Constants.shooterR, MotorType.kBrushless);

  public Shooter() {
    shooterL.restoreFactoryDefaults();
    shooterL.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterL.setInverted(false);

    shooterR.restoreFactoryDefaults();
    shooterR.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterR.setInverted(true);

    shooterL.follow(shooterR, FollowerType.PercentOutput);
  }
  public void revup(double output) {
    shooterR.set(ControlMode.PercentOutput,output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
