/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example command that uses an example subsystem.
 */
public class COngLetterLGroup extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public COngLetterLGroup(DriveTrain drivetrain) {
    double drivePower = 0.5;
    double turnPower = 0.25;
    addCommands(
      new DriveForward(drivetrain, 3, drivePower), // Drive forward for 3 seconds
      new PointTurn(drivetrain, true, 6, turnPower), // Turn left for 6 seconds
      new DriveForward(drivetrain, 3, drivePower), // Drive forward for 3 seconds
      new PointTurn(drivetrain, true, 6, turnPower), // Turn left for 6 seconds
      new DriveForward(drivetrain, 1.5, drivePower) // Drive forward for 1.5 seconds
    );
  }

}
