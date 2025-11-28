// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TalonFX_Elevator_MM extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // Elevator Motors
  private TalonFX elevatorLeftLeaderMotor;
  private TalonFX elevatorRightFollowerMotor;
  // Elevator Configs
  private TalonFXConfiguration elevatorConfigs;
  // Elevator Follower
  private Follower elevatorFollower;
  // Elevator Position Request
  public MotionMagicVoltage m_elevatorPositionRequest;
  // Bottom Limit
  private DigitalInput elevatorBottonLimit;
  // Voltage Request
  private VoltageOut m_voltageRequest;



  public TalonFX_Elevator_MM() {
    // Elevator Motors
    elevatorLeftLeaderMotor = new TalonFX(15, "DriveCANivore"); // Replace 15 with the actual device ID
    elevatorRightFollowerMotor = new TalonFX(16, "DriveCANivore"); // Replace 16 with the actual device ID

    // Elevator Follower
    elevatorFollower = new Follower(15, false); // Replace 15 with the leader motor ID
    elevatorRightFollowerMotor.setControl(elevatorFollower);

    // Set Elevator Bottom Limit
    elevatorBottonLimit = new DigitalInput(1); // Replace 1 with the actual DIO port number
    
    // elevatorConfigs
    elevatorConfigs = new TalonFXConfiguration()
                          .withSlot0(new Slot0Configs()
                                        .withKP(1) // Set P values based on mechanism requirements
                                        .withKI(1) // Set I values based on mechanism requirements
                                        .withKD(1) // Set D values based on mechanism requirements
                                        .withKS(1) // Set S values based on mechanism requirements
                                        .withKV(1) // Set V values based on mechanism requirements
                                        .withKA(1) // Set A values based on mechanism requirements
                                        .withKG(1) // Set G values based on mechanism requirements
                                        .withGravityType(GravityTypeValue.Elevator_Static))
                          .withMotorOutput(new MotorOutputConfigs()
                                              .withInverted(InvertedValue.Clockwise_Positive) // Set motor inversion based on mechanism
                                              .withNeutralMode(NeutralModeValue.Brake))
                          .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(40)); //Set current limit based on mechanism requirements

    // Apply elevatorConfigs
    elevatorLeftLeaderMotor.getConfigurator().apply(elevatorConfigs);
    elevatorRightFollowerMotor.getConfigurator().apply(elevatorConfigs);

    // Elevator Position Request
    m_elevatorPositionRequest = new MotionMagicVoltage(0).withSlot(0);
    // Voltage Request
    m_voltageRequest = new VoltageOut(0.0);


    }

    /**
     * Method to move the elevator up
     */
    public void elevatorUp() {
      elevatorLeftLeaderMotor.set(1); // Set to desired speed for elevator up (Usually called from constants)
      elevatorRightFollowerMotor.setControl(elevatorFollower);
    }

    /**
     * Method to move the elevator down
     */
    public void elevatorDown() {
      elevatorLeftLeaderMotor.set(-1); // Set to desired speed for elevator down (Usually called from constants)
      elevatorRightFollowerMotor.setControl(elevatorFollower);
    }

    /**
     * Method to stop the elevator
     */
    public void elevatorStop() {
      elevatorLeftLeaderMotor.stopMotor();
      elevatorRightFollowerMotor.setControl(elevatorFollower);
    }

    /**
     * Method to set the elevator position in rotations
     * @param height The desired height in rotations
     */
    public void setElevatorPosition(double height) {
      elevatorLeftLeaderMotor.setControl(m_elevatorPositionRequest.withPosition(height));
      elevatorRightFollowerMotor.setControl(elevatorFollower);
    }

    // Defines SysID Configs
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Second), // Use default ramp rate (1 V/s)
            Volts.of(1.2), // Reduce dynamic step voltage to 4 to prevent brownout
            Seconds.of(5.3), // Use default timeout (10 s)
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("Elevator state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> elevatorLeftLeaderMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
            null,
            this));
    
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
   * Method to get the current position of the elevator
   * @return double position in rotations
   */
  @AutoLogOutput
  public double getElevatorPosition(){
    return elevatorLeftLeaderMotor.getPosition().getValueAsDouble();
  }

  /**
   * Method to get the current setpoint of the elevator
   * @return double setpoint in rotations
   */
  @AutoLogOutput
  public double getElevatorSetpoint(){
    return m_elevatorPositionRequest.Position;
  }

  /**
   * Method to check if the arm is at the setpoint
   * @return boolean true if the arm is at the setpoint, false otherwise
   */
  @AutoLogOutput
  public boolean isAtSetpoint(){
    return Math.abs(getElevatorPosition() - getElevatorSetpoint()) <= 0.05; // Replace 0.05 with acceptable error margin (Usually from constants)
  }

  /**
   * Method to get the current velocity of the elevator
   * @return double velocity in RPS
   */
  @AutoLogOutput
  public double getElevatorVelocity(){
    return elevatorLeftLeaderMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Method to get the supply current of the elevator
   * @return double supply current in Amps
   */
  @AutoLogOutput
  public double getElevatorSupplyCurrent(){
    return elevatorLeftLeaderMotor.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Method to get the stator current of the elevator
   * @return double stator current in Amps
   */
  @AutoLogOutput
  public double getElevatorStatorCurrent(){
    return elevatorLeftLeaderMotor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Method to get the temperature of the elevator motors
   * @return double temperature in Celsius
   */
  @AutoLogOutput
  public double getElevatorTemperature(){
    return elevatorLeftLeaderMotor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Method to get the voltage of the elevator motors
   * @return double voltage in Volts
   */
  @AutoLogOutput
  public double getElevatorVoltage(){
    return elevatorLeftLeaderMotor.getMotorVoltage().getValueAsDouble();
  }
  
  /**
   * Method to get the state of the elevator bottom limit switch
   * @return boolean true if the switch is triggered, false otherwise
   */
  @AutoLogOutput
  public boolean isAtBottomLimit() {
    return elevatorBottonLimit.get();
  }


    @Override
    public void periodic() {
      // set Elevator Zero Position
      if (isAtBottomLimit() && getElevatorPosition() != 0) {
        elevatorLeftLeaderMotor.setPosition(0);
      }


    }

}