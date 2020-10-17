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
public class PointTurnSensorToAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private double m_angle, m_power;
  private boolean direction; // true: clockwise, false: counterclockwise
  private double withinAngle = 3; // Angle (in degrees) that robot must be within of target angle for command to finish


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PointTurnSensorToAngle(DriveTrain drivetrain, double angle, double power) {
    m_drivetrain = drivetrain;
    m_angle = angle;
    m_power = power;
    direction = angleBetween(m_drivetrain.getRobotAngle(), m_angle) > 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  private double angleBetween(double currentAngle, double targetAngle) {
    double m_currentAngle = currentAngle % 360;
    double m_targetAngle = targetAngle % 360;
    // if(m_targetAngle < m_currentAngle) {
    //   m_targetAngle = m_targetAngle + 360;
    // }
    // if(m_targetAngle - m_currentAngle > 180) {
    //   return (m_targetAngle - m_currentAngle - 360);
    // } else {
    //   return (m_targetAngle - m_currentAngle);
    // }
    return (m_targetAngle - m_currentAngle + 180) % 360 - 180;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setArcadeDrive(0, direction ? m_power : -m_power);
    // Sets motors to joystick input as long as they're not within deadzone
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_angle - m_drivetrain.getRobotAngle()) <= withinAngle;
  }
}
