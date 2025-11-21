// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake_Commands.Intake_WithoutSensor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.TalonFX_Intake_WithoutSensor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Roller_Intake_CMD extends Command {
  /** Creates a new Roller_Intake_CMD. */

  private TalonFX_Intake_WithoutSensor m_rollerSubsystem;

  /**
   * CMD to intake rollers without sensor
   * @param rollerSubsystem
   */
  public Roller_Intake_CMD(TalonFX_Intake_WithoutSensor rollerSubsystem) {
    m_rollerSubsystem = rollerSubsystem;
    addRequirements(m_rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rollerSubsystem.rollersIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rollerSubsystem.rollersStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
