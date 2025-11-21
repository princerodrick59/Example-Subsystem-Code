// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.TalonFX_Elevator_DMM;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator_DMM_Setpoint_CMD extends Command {
  /** Creates a new Elevator_L1. */
  TalonFX_Elevator_DMM m_elevatorSubsystem;
  double m_elevatorSetpoint;
  public Elevator_DMM_Setpoint_CMD(TalonFX_Elevator_DMM elevatorSubsystem, double elevatorSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevatorSubsystem = elevatorSubsystem;
    m_elevatorSetpoint = elevatorSetpoint;
    addRequirements(m_elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setElevatorMotionMagic(1200, 1200, 1200);
    m_elevatorSubsystem.setElevatorPosition(m_elevatorSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}