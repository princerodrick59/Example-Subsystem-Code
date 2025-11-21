// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake_Commands.Intake_WithSensor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.TalonFX_Intake_WithSensor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Roller_Intake_WithSensor_CMD extends Command {
  /** Creates a new Roller_Intake_CMD. */

  private TalonFX_Intake_WithSensor m_rollerSubsystem;

  /**
   * CMD to intake rollers with sensor(checks sensor state to stop rollers when object is detected)
   * @param rollerSubsystem
   */
  public Roller_Intake_WithSensor_CMD(TalonFX_Intake_WithSensor rollerSubsystem) {
    m_rollerSubsystem = rollerSubsystem;
    addRequirements(m_rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_rollerSubsystem.getRollerSensor()) {
      m_rollerSubsystem.rollersStop();
    }else{
      m_rollerSubsystem.rollersIntake();
    }
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
