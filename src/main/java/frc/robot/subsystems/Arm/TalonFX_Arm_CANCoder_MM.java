// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TalonFX_Arm_CANCoder_MM extends SubsystemBase {
  /** Creates a new TalonFX_Arm_WithWCPThroughBore. */
  private TalonFX armMotor;
  private TalonFXConfiguration armConfig;

  private CANcoder armEncoder;
  private CANcoderConfiguration armEncoderConfig;

  private MotionMagicVoltage m_motionRequest;

  private VoltageOut m_voltageRequest;

  /**
   * Creates a new TalonFX_Arm_WithWCPThroughBore subsystem.
   * This involves the code for controlling a pivoting arm using a TalonFX motor controller with Motion Magic control mode.
   * The arm position is measured using a CANcoder through-bore encoder.
   * You will need to adjust the motor ID, encoder ID, inversion, neutral mode, PID values, motion magic parameters, feedback sensor settings, and current limits based on your specific mechanism and requirements.
   * This subsystem includes methods to pivot the arm up, pivot the arm down, stop the arm, and set the arm to a specific position using Motion Magic.
   */
  public TalonFX_Arm_CANCoder_MM() {
    
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

    m_voltageRequest = new VoltageOut(0);

    m_motionRequest = new MotionMagicVoltage(0).withSlot(0).withFeedForward(0); // Set feedforward based on mechanism requirements


  }

  
  /**
   * Method to pivot the arm up
   */
  public void armUp() {
    armMotor.set(1); // Set to desired speed for arm up (Usually called from constants)
  }

  /**
   * Method to pivot the arm down
   */
  public void armDown() {
    armMotor.set(-1); // Set to desired speed for arm down (Usually called from constants)
  }

  /**
   * Method to stop the arm
   */
  public void armStop() {
    armMotor.set(0);
  }

  /**
   * Method to set the pivot position of the arm
   * @param position The desired position in units defined by the sensor-to-mechanism ratio
   */
  public void setPivotPosition(double position){
    armMotor.setControl(m_motionRequest.withPosition(position));
  }


  // SysID definition
  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
        new SysIdRoutine.Config(
          null,            // Use default ramp rate (1 V/s)
          Volts.of(4),    // Reduce dynamic step voltage to 4 to prevent brownout
          Seconds.of(10),  // Use default timeout (10 s)
          // Log state with Phoenix SignalLogger class
          (state) -> SignalLogger.writeString("End Effector State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
          (volts) -> armMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
          null,
          this
        )
    );

  /**
   * Method to run the SysId Quasistatic routine - Ramp Speed
   * @param direction The direction of the routine (up or down)
   * @return The command to run the routine
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Method to run the SysId Quasistatic routine - Ramp Speed
   * @param direction The direction of the routine (up or down)
   * @return The command to run the routine
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * Method to get the current position of the arm
   * @return double position in units defined by the sensor-to-mechanism ratio
   */
  @AutoLogOutput
  public double getArmPosition(){
    return armMotor.getPosition().getValueAsDouble();
  }

  /**
   * Method to get the current position of the arm from the CANcoder
   * @return double position in 0-1 range
   */
  @AutoLogOutput
  public double getArmSensorPosition(){
    return armEncoder.getPosition().getValueAsDouble();
  }

  /**
   * Method to get the current setpoint of the arm
   * @return double setpoint in units defined by the sensor-to-mechanism ratio
   */
  @AutoLogOutput
  public double getArmSetpoint(){
    return m_motionRequest.Position;
  }

  /**
   * Method to check if the arm is at the setpoint
   * @return boolean true if the arm is at the setpoint, false otherwise
   */
  @AutoLogOutput
  public boolean isAtSetpoint(){
    return Math.abs(getArmPosition() - getArmSetpoint()) <= 0.05; // Replace 0.05 with acceptable error margin (Usually from constants)
  }

  /**
   * Method to get the current velocity of the arm
   * @return double velocity in RPS
   */
  @AutoLogOutput
  public double getArmVelocity(){
    return armMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Method to get the supply current of the arm
   * @return double supply current in Amps
   */
  @AutoLogOutput
  public double getArmSupplyCurrent(){
    return armMotor.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Method to get the stator current of the arm
   * @return double stator current in Amps
   */
  @AutoLogOutput
  public double getArmStatorCurrent(){
    return armMotor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Method to get the temperature of the arm motor
   * @return double temperature in Celsius
   */
  @AutoLogOutput
  public double getArmTemperature(){
    return armMotor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Method to get the voltage of the arm motor
   * @return double voltage in Volts
   */
  @AutoLogOutput
  public double getArmVoltage(){
    return armMotor.getMotorVoltage().getValueAsDouble();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
