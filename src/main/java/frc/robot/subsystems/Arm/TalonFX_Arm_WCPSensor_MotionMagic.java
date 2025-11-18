// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonFX_Arm_WCPSensor_MotionMagic extends SubsystemBase {
  /** Creates a new TalonFX_Arm_WithWCPThroughBore. */
  private TalonFX armMotor;
  private TalonFXConfiguration armConfig;

  private CANcoder armEncoder;
  private CANcoderConfiguration armEncoderConfig;

  private MotionMagicVoltage m_motionRequest;

  public TalonFX_Arm_WCPSensor_MotionMagic() {
    armEncoder = new CANcoder(12); // Replace 12 with the actual device ID

    armEncoderConfig = new CANcoderConfiguration()
                          .withMagnetSensor(new MagnetSensorConfigs()
                                            .withMagnetOffset(0) // Set based on mechanism requirements
                                            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) // set based on mechanism requirements
                                            .withAbsoluteSensorDiscontinuityPoint(1) // Set based on mechanism requirements
                                            ); 

    armEncoder.getConfigurator().apply(armEncoderConfig);



    armMotor = new TalonFX(11); // Replace 11 with the actual device ID

    
    armConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.CounterClockwise_Positive) // Set motor inversion based on mechanism
                                            .withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs()
                                        .withKP(1) // Set P values based on mechanism requirements
                                        .withKI(1) // Set I values based on mechanism requirements
                                        .withKD(1)) // Set D values based on mechanism requirements
                        .withMotionMagic(new MotionMagicConfigs()
                                            .withMotionMagicCruiseVelocity(1000) // Set cruise velocity based on mechanism requirements
                                            .withMotionMagicAcceleration(1000) // Set acceleration based on mechanism requirements
                                            .withMotionMagicJerk(1000)) // Set jerk based on mechanism requirements
                        .withFeedback(new FeedbackConfigs()
                                            .withFeedbackRemoteSensorID(12) // Replace 12 with the actual CANcoder device ID
                                            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                            .withSensorToMechanismRatio(1) // Set based on mechanism - Usually found on spec sheet - Based on where the sensor is mounted
                                            .withRotorToSensorRatio(1)) // Set based on mechanism - Usually found on spec sheet - Based on where the sensor is mounted
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(20)); // Set current limit based on mechanism requirements

    armMotor.getConfigurator().apply(armConfig);

    m_motionRequest = new MotionMagicVoltage(0).withSlot(0).withFeedForward(0); // Set feedforward based on mechanism requirements


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
