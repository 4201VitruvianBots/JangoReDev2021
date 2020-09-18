/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example command that uses an example subsystem.
 */
public class Dance extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Dance(DriveTrain drivetrain) {
    double power = 0.5;
    for (int i = 0; i < 2; i++) {
      addCommands(
        new DriveForward(drivetrain, 2, power), // Drive forward for 2 seconds
        new DriveBackward(drivetrain, 2, power), // Drive backward for 2 seconds
        new PointTurn(drivetrain, true, 10, power), // Turn counterclockwise for 10 seconds)
      );
    }
  }
}
