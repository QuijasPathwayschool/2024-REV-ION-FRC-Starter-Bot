package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_topMotor;
  private CANSparkMax m_bottomMotor;
  private CANSparkMax m_LeftMotor;
  private CANSparkMax m_RightMotor;

  private boolean m_launcherRunning;
  private boolean m_launcherAmp;

  private double kPower;
  private double kbackPower;
 // private boolean m_intake;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    m_topMotor =
new CANSparkMax(Constants.Launcher.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(false);
    m_topMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_topMotor.setIdleMode(IdleMode.kCoast);

    m_topMotor.burnFlash();

    m_bottomMotor =
        new CANSparkMax(Constants.Launcher.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomMotor.restoreFactoryDefaults();
    m_bottomMotor.setInverted(true);
    m_bottomMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_bottomMotor.setIdleMode(IdleMode.kCoast);

    m_bottomMotor.burnFlash();

    m_LeftMotor =
        new CANSparkMax(Constants.Launcher.kBackLeftCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_LeftMotor.restoreFactoryDefaults();
    m_LeftMotor.setInverted(true);
    m_LeftMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_LeftMotor.setIdleMode(IdleMode.kCoast);

    m_LeftMotor.burnFlash();

    m_RightMotor =
        new CANSparkMax(Constants.Launcher.kBackRightCanId, CANSparkLowLevel.MotorType.kBrushless);
    //m_RightMotor.restoreFactoryDefaults();
    //m_RightMotor.setInverted(false);
    //m_RightMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    //m_RightMotor.setIdleMode(IdleMode.kCoast);
    //m_RightMotor.follow(m_LeftMotor);

   //m_RightMotor.burnFlash();


    
    m_launcherRunning = false;
    m_launcherAmp = false;
   // m_intake = false;
  }

  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runLauncher() {
    kPower = .95;
    kbackPower= 0;
    m_launcherRunning = true;
    
  }

   public void runbackLauncher() {
    kPower = -1;
    kbackPower= .98 ;
    m_launcherRunning = true;
    
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopAmp() {
    m_launcherAmp = false;
  }

    public void runAmp() {
    m_launcherAmp = true;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

    public void setIntake() {
      kPower = -1;
      kbackPower= 0;
      m_launcherRunning = true;
      
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopIntake() {
   m_bottomMotor.set(0);
   m_topMotor.set(0);
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      m_topMotor.set(kPower);
      m_bottomMotor.set(kPower);
      m_LeftMotor.set(kbackPower);
     // m_RightMotor.set(kbackPower);
    } else {
      m_topMotor.set(0.0);
      m_bottomMotor.set(0.0);
      m_LeftMotor.set(0 );
     // m_RightMotor.set(0);
     
    }
    if (m_launcherAmp) {
      m_topMotor.set(Constants.Launcher.kAmpTopPower);
      m_bottomMotor.set(Constants.Launcher.kAmpBottomPower);
    } else if(m_launcherRunning && m_launcherAmp) {
      m_topMotor.set(0.0);
      m_bottomMotor.set(0.0);
      m_LeftMotor.set(0 );
     // m_RightMotor.set(0);
    }


  }

  public void shoot() {
    setIntake();

  }
  public void intakein() {
      m_topMotor.set(-.9);
      m_bottomMotor.set(-.9);
  }
}
