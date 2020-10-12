/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    /**
     * Creates a new Turret.
     */
    double kP = 0;
    double kI = 0;
    double kD = 0;

    private final CANSparkMax turretMotor = new CANSparkMax(Constants.turretMotor, MotorType.kBrushless);
    private final CANPIDController turretPID = turretMotor.getPIDController();
    // create the limelight pretty please

    // yes ^^ -jacob


    public Turret() {
        turretMotor.restoreFactoryDefaults();
        turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turretMotor.setInverted(false);

        turretPID.setP(kP);
        turretPID.setI(kI);
        turretPID.setD(kD);

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

    public double getAngle() {
        return 1;
    }

    public void getOutput(double output) {
        turretMotor.set(output);
    }

    public void setAngle(double angle) {
        while (!(angle - 1 < getAngle() && getAngle() < angle + 1)) {
            getOutput(1);
        }
        getOutput(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Applied output", turretMotor.getAppliedOutput());
        SmartDashboard.putNumber("Velocity", turretMotor.getEncoder().getVelocity());
    }
}

