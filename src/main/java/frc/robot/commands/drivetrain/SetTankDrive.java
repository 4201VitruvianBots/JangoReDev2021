/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import java.lang.Math;
import frc.robot.Constants;
/**
 * An example command that uses an example subsystem.
 */
public class SetTankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_leftJoystick;
  private final DoubleSupplier m_rightJoystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetTankDrive(DriveTrain drivetrain, DoubleSupplier leftJoystick, DoubleSupplier rightJoystick) {
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
    m_drivetrain = drivetrain;
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
    double leftOutput = 0;
    double rightOutput = 0;
    if (Math.abs(m_leftJoystick.getAsDouble()) >= Constants.deadZone) {
      leftOutput = m_leftJoystick.getAsDouble();
    }
    if (Math.abs(m_rightJoystick.getAsDouble()) >= Constants.deadZone) {
      rightOutput = m_rightJoystick.getAsDouble();
    }
    m_drivetrain.setTankDrive(leftOutput, rightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
