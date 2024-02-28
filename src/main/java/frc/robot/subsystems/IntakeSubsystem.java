package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;


public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_motor;
  private CANSparkMax m_motor2;
  private CANSparkMax m_motor3;
  private CANSparkMax m_motor4;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_controller;
 
  //private final LauncherSubsystem m_launcher = new LauncherSubsystem();

  private boolean m_positionMode;
  private double m_targetPosition;
  private double m_power;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // create a new SPARK MAX and configure it
    m_motor = new CANSparkMax(Constants.Intake.kCanId, MotorType.kBrushless);
    m_motor.setInverted(true);
    m_motor.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.burnFlash();

    m_motor2 = new CANSparkMax(Constants.Intake.kCanId2, MotorType.kBrushless);
    m_motor2.setInverted(true);
    m_motor2.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor2.setIdleMode(IdleMode.kBrake);
    m_motor2.follow(m_motor);
    m_motor2.burnFlash();

    m_motor3 = new CANSparkMax(Constants.Intake.kCanId3, MotorType.kBrushless);
    m_motor3.setInverted(false);
    m_motor3.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor3.setIdleMode(IdleMode.kCoast);
    m_motor3.burnFlash();

     m_motor4 = new CANSparkMax(Constants.Intake.kCanId4, MotorType.kBrushless);
     //  m_motor4.setInverted(true);
    m_motor4.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    m_motor4.setIdleMode(IdleMode.kCoast);
    //m_motor4.follow(m_motor3);
      //m_motor4.burnFlash();

    m_encoder = m_motor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    m_controller = m_motor.getPIDController();
    PIDGains.setSparkMaxGains(m_controller, Constants.Intake.kPositionGains);

 
  
    


    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = 0.0;
  }

  /**
   * Set the power to spin the motor at. This only applies outside of position mode.
   *
   * @param _power The power to apply to the motor (from -1.0 to 1.0).
   */
  public void setPower(double _power) {
    m_positionMode = false;
    m_targetPosition = m_encoder.getPosition();
    m_power = _power;
  }

  public void setintakepower(double _power){
    m_motor.set(_power);
    //m_motor2.set(_power);

  }

  public void setfeedPower(double _power) {
    //m_positionMode = false;
    //m_targetPosition = m_encoder.getPosition();
    m_motor3.set(_power);
    m_motor4.set(_power);
  }

  public void stopfeedPower(double _power) {
    //m_positionMode = false;
    //m_targetPosition = m_encoder.getPosition();
    m_motor3.set(0);
    m_motor4.set(0);
  }
  /**
   * Constructs a command that drives the rollers a specific distance (number of rotations) from the
   * current position and then ends the command.
   *
   * @return The retract command
   */
  public Command retract() {
    Command newCommand =
        new Command() {
          @Override
          public void initialize() {
            m_positionMode = true;
            m_targetPosition = m_encoder.getPosition() + Constants.Intake.kRetractDistance;
          }

          @Override
          public boolean isFinished() {
            return isNearTarget();
          }
        };

    newCommand.addRequirements(this);

    return newCommand;
  }

  /**
   * Constructs a command that feeds a note into the launcher by running the intake for a set amount
   * of time. This command takes control of the launcher subsystem to make sure the wheels keep
   * spinning during the launch sequence.
   *
   * @param _launcher The instance of the launcher subsystem
   * @return The launch command
   */
  public Command feedLauncher(LauncherSubsystem _launcher) {
    Command newCommand =
        new Command() {
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
           // setPower(1.0);
           m_motor3.set(-1);
           m_motor4.set(-1);
            _launcher.runLauncher();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Intake.kShotFeedTime;
          }

          @Override
          public void end(boolean interrupted) {
            setPower(0.0);
            m_motor3.set(0);
            m_motor4.set(0);
          }
        };

    newCommand.addRequirements(this, _launcher);

    return newCommand;
  }

    /**
   * Constructs a command that feeds a note into the launcher by running the intake for a set amount
   * of time. This command takes control of the launcher subsystem to make sure the wheels keep
   * spinning during the launch sequence.
   *
   * @param _launcher The instance of the launcher subsystem
   * @return The launch command
   */
  public Command feedAMP(LauncherSubsystem _launcher) {
    Command newCommand =
        new Command() {
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
           // setPower(1.0);
           m_motor3.set(-.4);
           m_motor4.set(-.4);
          _launcher.runAmp();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Intake.kShotFeedTime;
          }

          @Override
          public void end(boolean interrupted) {
            setPower(0.0);
            m_motor3.set(0);
            m_motor4.set(0);
            _launcher.stopAmp();
          }
        };

    newCommand.addRequirements(this, _launcher);

    return newCommand;
  }


  @Override
  public void periodic() { // This method will be called once per scheduler run
    // if we've reached the position target, drop out of position mode
    if (m_positionMode && isNearTarget()) {
      m_positionMode = false;
      m_power = 0.0;
       ///m_launcher.stopIntake();
    }

    // update the motor power based on mode and setpoint
    if (m_positionMode) {
      m_controller.setReference(m_targetPosition, ControlType.kPosition);
     
    } else {
      m_motor.set(m_power);
     // m_launcher.runIntake();
    }
  }

  /**
   * Check if the encoder is within the position tolerance.
   *
   * @return Whether the position is within the tolerance.
   */
  public boolean isNearTarget() {
    return Math.abs(m_encoder.getPosition() - m_targetPosition)
        < Constants.Intake.kPositionTolerance;
  }

    
public void shoot () {
  setfeedPower(-.92);
}
public void stopshoot () {
  stopfeedPower(0);
}


  public void intake(){
    setPower(Constants.Intake.kIntakePower);
  }

  public void stopintake() {
    setPower(0);
  }

}



