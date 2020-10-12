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

public class Shooter extends SubsystemBase {
    /**
     * Creates a new Shooter.
     * (╯‵□′)╯︵┻━┻
     */
    double kP = 0.1;
    double kI = 0;
    double kD = 0;

    private final CANSparkMax shooterL = new CANSparkMax(Constants.shooterL, MotorType.kBrushless);
    private final CANSparkMax shooterR = new CANSparkMax(Constants.shooterR, MotorType.kBrushless);
    private final CANPIDController shooterPID = shooterR.getPIDController();


    public Shooter() {
        shooterL.restoreFactoryDefaults();
        shooterL.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterL.setInverted(false);

        shooterR.restoreFactoryDefaults();
        shooterR.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterR.setInverted(true);

        shooterL.follow(shooterR);

        shooterPID.setP(kP);
        shooterPID.setI(kI);
        shooterPID.setD(kD);


    }

    public void revup(double output) {
        shooterR.set(output);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Applied output", shooterR.getAppliedOutput());
        SmartDashboard.putNumber("Velocity", shooterR.getEncoder().getVelocity());
    }
}