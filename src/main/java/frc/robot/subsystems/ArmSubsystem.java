// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax m_motor;
  private CANSparkMax m_motor2;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
  private double m_setpoint;


  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
 // private double m_manualValue;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // create a new SPARK MAX and configure it 7 inverted 6 not
    m_motor = new CANSparkMax(Constants.Arm.kArmCanId, MotorType.kBrushless);
    //m_motor.restoreFactoryDefaults();
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kCoast);

    m_motor2 = new CANSparkMax(Constants.Arm.kArmCanId2, MotorType.kBrushless);
    //m_motor2.restoreFactoryDefaults();
    //m_motor2.setInverted(true);
    //m_motor2.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    //m_motor2.setIdleMode(IdleMode.kCoast);
   //m_motor2.follow(m_motor);
    // m_motor2.burnFlash();
   
    //m_motor.enableSoftLimit(SoftLimitDirection.kForward, true); 
    //m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    //m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.kSoftLimitForward);
   // m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.kSoftLimitReverse);

    // set up the motor encoder including conversion factors to convert to radians and radians per
    // second for position and velocity
    m_encoder = m_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    m_encoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    m_encoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);
    m_encoder.setPosition(0.0);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Arm.kArmPositionGains);

    
   

    
    m_motor.burnFlash();
   // m_motor2.burnFlash();
    m_setpoint = Constants.Arm.kScoringPosition;

    m_timer = new Timer();
    m_timer.start();

    updateMotionProfile();
  }

  /**
   * Sets the target position and updates the motion profile if the target position changed.
   *
   * @param _setpoint The new target position in radians.
   */
  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  /**
   * Update the motion profile variables based on the current setpoint and the pre-configured motion
   * constraints.
   */
  private void updateMotionProfile() {
    m_startState = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);
    m_timer.reset();
  }

  /**
   * Drives the arm to a position using a trapezoidal motion profile. This function is usually
   * wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
   *
   * <p>This function updates the motor position control loop using a setpoint from the trapezoidal
   * motion profile. The target position is the last set position with {@code setTargetPosition}.
   */
  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }

    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    m_controller.setReference(
        m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
  }

  /**
   * Drives the arm using the provided power value (usually from a joystick). This also adds in the
   * feedforward value which can help counteract gravity.
   *
   * @param _power The motor power to apply.
   */
  public void runManual(double _power) {
    // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
    // passively
    m_setpoint = m_encoder.getPosition();
    updateMotionProfile();
    // update the feedforward variable with the newly zero target velocity
    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
    // set the power of the motor
    m_motor.set(_power + (m_feedforward / 12.0));
   // m_manualValue = _power; // this variable is only used for logging or debugging if needed
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
  }

  public void intake() {
    setTargetPosition(Constants.Arm.kIntakePosition);
  }

    public void shoot() {
    setTargetPosition(Constants.Arm.kScoringPosition);
    }

    public void shootfar() {
    setTargetPosition(Constants.Arm.kScoringFarPosition);
  }
  
}
