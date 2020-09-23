/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.limelight;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
  private CANSparkMax turretMotor =  new CANSparkMax(Constants.turretMotor, MotorType.kBrushless);
  // create the limelight pretty please

  // yes ^^ -jacob


  public Turret() {
    turretMotor.restoreFactoryDefaults();
    turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    turretMotor.setInverted(false);

    /*
    set the camera light to on
    turn on the camera
    focus the lens
    zoom the lens
    adjust the sensitivity and exposure of the camera things. 
    */

  }

  /*
  position of the robot on the field.
  */

  /*
  the angle between the robot and the target
  */

  public double getAng(){
    return lemonlight.getAngle();
  }

  public void Zetoutput(double output) {
  turretMotor.set(output);
  }

  public void setAngle(double angle){
    while(!(angle-1 < turretMotor.getAngle && turretMotor.getAngle < angle+1)){
      Zetoutput(1);
    }
    Zetoutput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

