/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
  private CANSparkMax turretMotor = new CANSparkMax(Constants.turretMotor, MotorType.kBrushless);

  public Turret() {
    turretMotor.restoreFactoryDefaults();
    turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    turretMotor.setInverted(false);
  }
  public void spin(double output) {
    turretMotor.set(ControlMode.PercentOutput,output);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
