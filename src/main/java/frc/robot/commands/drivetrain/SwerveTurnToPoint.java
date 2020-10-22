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
public class SwerveTurnToPoint extends CommandBase { // Travels in a circle from current position to a certain point
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private double m_maxPower, m_targetX, m_targetY, leftRadius, rightRadius, leftPower, rightPower, initialAngle; 
  private double distanceWithinPosition = 0.1; // How close robot has to be to target position for command to finish
  private double withinAngle = 3;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveTurnToPoint(DriveTrain drivetrain, double maxPower, double targetX, targetY) {
    m_drivetrain = drivetrain;
    m_maxPower = maxPower;
    m_targetX = targetX;
    m_targetY = targetY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_drivetrain.getRobotAngle();
    double changeInAngle = Math.atan(targetY / targetX) - initialAngle;
    double radius = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2)));
    leftRadius = radius + Constants.wheelDistance / 2 * (changeInAngle > 0 ? -1 : 1);
    rightRadius = radius + Constants.wheelDistance / 2 * (changeInAngle > 0 ? 1 : -1);
    double maxSpeed = m_drivetrain.powerToSpeed(m_maxPower);
    double maxPower = m_drivetrain.speedToPower(maxSpeed);
    double time = Math.PI * changeInAngle * Math.max(leftRadius, rightRadius) / 180;
    double minSpeed = Math.PI * changeInAngle * m_radius / 180 / time;
    double minPower = m_drivetrain.speedToPower(minSpeed);
    leftPower = changeInAngle > 0 ? minPower : maxPower;
    rightPower = changeInAngle > 0 ? maxPower : minPower;
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
    return Math.abs(m_drivetrain.getRobotAngle() - initialAngle - changeInAngle) < withinAngle;
  }
}
