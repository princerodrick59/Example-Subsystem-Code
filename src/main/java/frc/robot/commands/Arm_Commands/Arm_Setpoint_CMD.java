// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.TalonFX_Arm_WCPSensor_MotionMagic;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Arm_Setpoint_CMD extends Command {
  private TalonFX_Arm_WCPSensor_MotionMagic m_armSubsystem;
  private double m_armSetpoint;
  /**
   * CMD to set arm to a specific setpoint using Motion Magic
   * @param armSubsystem 
   * @param armSetpoint setpoint in units defined by the sensor-to-mechanism ratio
   */
  public Arm_Setpoint_CMD(TalonFX_Arm_WCPSensor_MotionMagic armSubsystem, double armSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    m_armSetpoint = armSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPivotPosition(m_armSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}