// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonFX_Intake_WithSensor extends SubsystemBase {
  
  private TalonFX rollerMotor;
  private TalonFXConfiguration rollerConfig;

  private DigitalInput rollerSensor;


  /**
   * Creates a new TalonFX_Rollers subsystem.
   * This involves the basic code for running the rollers of a motor.
   * This subsystem does involve the implementation of a beam break sensor that is fed through a DIO Port on the RIO.
   * You will need to adjust the motor ID, inversion, neutral mode, and current limits based on your specific mechanism and requirements.
   * This subsystem includes methods to intake, outtake, and stop the rollers.
   * 
   */
  public TalonFX_Intake_WithSensor() {

    rollerMotor = new TalonFX(10); // Replace 10 with the actual device ID

    rollerConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.CounterClockwise_Positive) //Set motor inversion based on mechanism
                                              .withNeutralMode(NeutralModeValue.Brake))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(15)); //Set current limit based on mechanism requirements

    rollerMotor.getConfigurator().apply(rollerConfig);

    rollerSensor = new DigitalInput(0); // Replace 0 with the actual DIO port number

  }

  /**
   * Method to run the rollers for intake
   */
  public void rollersIntake(){
    rollerMotor.set(1); // Set to desired speed for intake (Usually called from constants)
  }
  /**
   * Method to run the rollers for outtake
   */
  public void rollersOuttake(){
    rollerMotor.set(-1); // Set to desired speed for outtake (Usually called from constants)
  }
  /**
   * Method to stop the rollers
   */
  public void rollersStop(){
    rollerMotor.stopMotor(); // Stop the rollers
  }

  
  /**
   * Method to get the state of the roller sensor
   * @return boolean true if the sensor is triggered, false otherwise
   */
  @AutoLogOutput
  public boolean getRollerSensor(){
    return rollerSensor.get();
  }

  /**
   * Method to get the current velocity of the rollers
   * @return double velocity in RPS
   */
  @AutoLogOutput
  public double getRollersVelocity(){
    return rollerMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Method to get the supply current of the rollers
   * @return double supply current in Amps
   */
  @AutoLogOutput
  public double getRollersSupplyCurrent(){
    return rollerMotor.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Method to get the stator current of the rollers
   * @return double stator current in Amps
   */
  @AutoLogOutput
  public double getRollersStatorCurrent(){
    return rollerMotor.getStatorCurrent().getValueAsDouble();
  }


  /**
   * Method to get the voltage being applied to the rollers motor
   * @return double voltage in Volts
   */
  @AutoLogOutput
  public double getIntakeVoltage(){
    return rollerMotor.getMotorVoltage().getValueAsDouble();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
