/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;
import frc.robot.Constants;
/**
 * An example command that uses an example subsystem.
 */
public class FindPathToPoint extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private double m_maxPower, m_angle, m_radius, leftRadius, rightRadius, leftPower, rightPower, initialAngle; 
  private double distanceWithinPosition = 0.1; // How close robot has to be to target position for command to finish
  private double withinAngle = 3;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FindPathToPoint(DriveTrain drivetrain, double maxPower, double radius, double angle) {
    m_drivetrain = drivetrain;
    m_maxPower = maxPower;
    m_radius = radius;
    m_angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_drivetrain.getRobotAngle();
    leftRadius = m_radius + Constants.wheelDistance / 2 * (m_angle > 0 ? -1 : 1);
    rightRadius = m_radius + Constants.wheelDistance / 2 * (m_angle > 0 ? 1 : -1);
    double maxSpeed = m_drivetrain.powerToSpeed(m_maxPower);
    double maxPower = m_drivetrain.speedToPower(maxSpeed);
    double time = Math.PI * m_angle * Math.max(leftRadius, rightRadius) / 180;
    double minSpeed = Math.PI * m_angle * m_radius / 180 / time;
    double minPower = m_drivetrain.speedToPower(minSpeed);
    leftPower = m_angle > 0 ? minPower : maxPower;
    rightPower = m_angle > 0 ? maxPower : minPower;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setTankDrive(leftPower, rightPower);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getRobotAngle() - initialAngle - m_angle) < withinAngle;
  }
}
