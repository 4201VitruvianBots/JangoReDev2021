/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;
/**
 * An example command that uses an example subsystem.
 */
public class PointTurnAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private double start_time, m_time, m_power, m_angle, m_distance, m_speed;

  /**
   * Creates a new ExampleCommand.
   *,
   * @param subsystem The subsystem used by this command.
   */
  public PointTurnAngle(DriveTrain drivetrain, double angle, double power) {
    m_drivetrain = drivetrain;
    m_angle = angle;
    m_power = power;
    m_distance = Math.PI * Constants.wheelDistance * m_angle / 360;
    m_speed = m_drivetrain.powerToSpeed(m_power);
    m_time = m_distance / m_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Angle to turn", m_angle);
    SmartDashboard.putNumber("Turning power", m_power);

    start_time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Angle turned so far", (Timer.getFPGATimestamp() - start_time) * m_speed * 360 / (Math.PI * Constants.wheelDistance));

    m_drivetrain.setArcadeDrive(0, m_power * (m_angle > 0 ? 1 : -1));
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
