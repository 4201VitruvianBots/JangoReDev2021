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
public class SensorGetToPosition extends CommandBase { // First turns to face, then drives directly toward a certain position using sensors
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private double m_power, m_xPosition, m_yPosition; 
  private double distanceWithinPosition = 0.1; // How close robot has to be to target position for command to finish
  private double withinAngle = 5; // Robot has to be facing within this angle of target

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SensorGetToPosition(DriveTrain drivetrain, double power, double xPosition, double yPosition) {
    m_drivetrain = drivetrain;
    m_power = power;
    m_xPosition = xPosition;
    m_yPosition = yPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (getDeltaAngleToTarget() > withinAngle) {
      m_drivetrain.setTankDrive(m_power, 0); // Turning clockwise if robot angle is too small
    } else if (getDeltaAngleToTarget() < -withinAngle) { // Turning counterclockwise if robot angle is too large
      m_drivetrain.setTankDrive(0, m_power);
    } else {
      m_drivetrain.setTankDrive(m_power, m_power); // Driving straight forward
    }
  }

  private double getDeltaAngleToTarget() {
    double xDistance = m_xPosition - m_drivetrain.getRobotX(); // How far away the target is in x-direction
    double yDistance = m_yPosition - m_drivetrain.getRobotY(); // How far away the target is in y-direction
    double angleToTarget = ((Math.toDegrees(Math.atan(yDistance / xDistance)) + (xDistance < 0 ? 180 : 0)) + 180) % 360 - 180; // Angle [-180, 180] to target position
    double deltaAngle = angleToTarget - m_drivetrain.getRobotAngle(); // Angle robot must turn
    if (Math.abs(deltaAngle) > 180) {
      return 360 - deltaAngle;
    } else {
      return deltaAngle;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Finished when xy distance to target is within a certain number
    double distance = Math.sqrt(Math.pow(m_drivetrain.getRobotX() - m_xPosition, 2) + Math.pow(m_drivetrain.getRobotY() - m_yPosition, 2));
    return distance <= distanceWithinPosition;
  }
}
