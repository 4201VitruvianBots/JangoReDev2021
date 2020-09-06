/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Enums.IntakeStates;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlledIntake extends CommandBase {
  /**
   * Creates a new ControlledIntake.
   */
  public ControlledIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Indexer m_indexer;
    private final Intake m_intake;
  
    private double intakeRPM = 5000;
    private double indexRPM = 300;
    private double timestamp, intakeTimestamp, indexerTimestamp, fourBallTimestamp;
    private boolean intaking, haveFour, haveFourTripped;
    private Joystick m_controller;
  
    private IntakeStates intakeState = IntakeStates.INTAKE_EMPTY;

  }
    public ControlledIntake(Intake intake, Indexer indexer, Joystick controller) {
    m_intake = intake;
    m_indexer = indexer;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakingState(true);
    timestamp = Timer.getFPGATimestamp();

    if(m_indexer.getIntakeSensor() && m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FIVE_BALLS;
    else if(m_indexer.getIndexerBottomSensor() && m_indexer.getIndexerTopSensor())
      intakeState = IntakeStates.INTAKE_FOUR_BALLS;
    else
      intakeState = IntakeStates.INTAKE_ONE_BALL;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
