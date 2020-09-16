/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/ 

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;
/**
 * An example command that uses an example subsystem.
 */
public class SwerveTurnAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private double start_time, m_time, m_power, m_angle, m_distance;

  /**
   * Creates a new ExampleCommand.
   *,
   * @param subsystem The subsystem used by this command.
   */
  public SwerveTurnAngle(DriveTrain drivetrain, double angle, double power) {
    m_drivetrain = drivetrain;
    m_angle = angle;
    m_power = power;
    m_distance = m_angle / 360 * 2* Math.PI * Constants.wheelDistance;
    m_time = m_distance / m_drivetrain.powerToSpeed(m_power);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start_time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setTankDrive((m_angle > 0 ? m_power : 0), (m_angle > 0 ? 0 : m_power));
    if (m_angle > 0) {
      m_drivetrain.setTankDrive(m_power, 0);
    } else {
      m_drivetrain.setArcadeDrive(0, m_power);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setTankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= start_time + m_time;
  }
}
