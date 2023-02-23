// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;

//import edu.wpi.first.cameraserver.CameraServer;

//import edu.wpi.first.cscore.AxisCamera;
//import edu.wpi.first.cscore.CameraServerJNI;
/* commented out imports are not used! 
 * comment back in when ready to use!
 */
//import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//commented out until needed
//import java.util.TimerTask;
/* USING SMARTDASHBOARD, not shuffleboard --> comment back in if needed
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
 */
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  /*
   * private double kP;
   * private double kI;
   * private double kD;
   * private double integral;
   * private double previousError;
   */
  // might not need these, already exist in a preexisting class for pid loops
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // motor initialization

  // base motors
  CANSparkMax left1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax left2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax left3 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax right1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax right2 = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax right3 = new CANSparkMax(7, MotorType.kBrushless);

  // check if brushed or brushless w art

  // everything but intake is brushed, intake will have different motor controller

  CANSparkMax elevatorLeft = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax elevatorRight = new CANSparkMax(10, MotorType.kBrushless);

  // lightstrip/blinkin
  Spark lightstrip = new Spark(3);

  // controllers
  XboxController mainDriveController = new XboxController(0);
  XboxController coDriver = new XboxController(1);

  // pigeon
  PigeonIMU pigeonIMU = new PigeonIMU(19);

  /*
   * public void getYawPitchRoll(double[] ypr)
   * {
   * double[] xyz_dps = new double[3];
   * encoderImu .getRawGyro(xyz_dps);
   * 
   * ypr[0] += xyz_dps[2] * 0.02; // yaw
   * ypr[1] += xyz_dps[0] * 0.02; // pitch
   * ypr[2] += xyz_dps[1] * 0.02; // roll
   * }
   */
  /*
   * public void getYawPitchRoll(double[] ypr) {
   * double[] xyz_dps = new double[3];
   * encoderImu.getRawGyro(xyz_dps);
   * 
   * ypr[0] += xyz_dps[2] * 0.02; // yaw
   * ypr[1] += xyz_dps[0] * 0.02; // pitch
   * ypr[2] += xyz_dps[1] * 0.02; // roll
   * }
   */

  // set lightstrip colors

  boolean lights = false;
  double color_val;

  // need to determine values for kP kI and kD
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    // LEARN CALCULUS to figure this out live laugh love
  }

  private double kP;
  private double kI;
  private double kD;
  PIDController pid = new PIDController(kP, kI, kD);
  /*
   * public Robot(double kP, double kI, double kD) {
   * this.kP = kP;
   * this.kI = kI;
   * this.kD = kD;
   * }
   */

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);

    // ShuffleboardTab mainTab = Shuffleboard.getTab("Smart Dashboard");
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture();

    SmartDashboard.putNumber("Pigeon", pigeonIMU.getYaw());

    Shuffleboard.getTab("Pigeon Data")
        .add("Pigeon Data", pigeonIMU.getYaw());

    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // SmartDashboard.putNumber("Encoder", encoderImu.getEncoder());
    System.out.println("Auto selected: " + m_autoSelected);

    // SmartDashboard.putNumber("Encoder", );
    // GOOD ONE PigeonIMU encoderImu = new PigeonIMU(19);

    // create shuffleboard (for custom driving)
    // SmartDashboard.putData("Camera", (Sendable)
    // CameraServer.startAutomaticCapture());

    // set current limits
    left1.setSmartCurrentLimit(50);
    left2.setSmartCurrentLimit(50);
    left3.setSmartCurrentLimit(50);
    right1.setSmartCurrentLimit(50);
    right2.setSmartCurrentLimit(50);
    right3.setSmartCurrentLimit(50);

    elevatorLeft.setSmartCurrentLimit(50);
    elevatorRight.setSmartCurrentLimit(50);
    // invert right drive
    right1.setInverted(true);
    right2.setInverted(true);
    right3.setInverted(true);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // SmartDashboard.putNumber("Encoder", encoderImu.getEncoder());
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboallrd, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // SmartDashboard.putNumber("Encoder", encoderImu.getEncoder());
    System.out.println("Auto selected: " + m_autoSelected);

    // switch (m_autoSelected);

    /*
     * basically talking to brennan , the way to get distance is by having the motor
     * power be set to 0.5 and then having a certain amount for it to go forward and
     * then set it to zero
     */

    /*
     * while (seconds >= 5) {
     * left1.set(0);
     * left2.set(0);
     * left3.set(0);
     * right1.set(0);
     * right2.set(0);
     * right3.set(0);
     * }
     */
    // Timer timer = new Timer();
    // timer.reset();
    // timer.start();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
    }

    Timer timer = new Timer();
    timer.reset();
    timer.start();
    // double seconds = time.get();

    // initial speed
    while (timer.get() <= 1.6) {
      left1.set(.6);
      left2.set(.6);
      left3.set(.6);
      right1.set(.6);
      right2.set(.6);
      right3.set(.6);
    }
    // speed slowed down after 1.6 seconds but before 2.75 seconds
    while (timer.get() >= 1.6 && timer.get() <= 2.75) {
      left1.set(.2);
      left2.set(.2);
      left3.set(.2);
      right1.set(.2);
      right2.set(.2);
      right3.set(.2);
    }
    /*
     * left1.set(pid.calculate(90, encoderImu.getYawPitchRoll(null));
     * left2.set(pid.calculate(90, encoderImu.getDistance()));
     * left3.set(pid.calculate(90, encoderImu.getDistance()));
     * right1.set(pid.calculate(90, encoderImu.getDistance()));
     * right2.set(pid.calculate(90, encoderImu.getDistance));
     * right3.set(pid.calculate(90, encoderImu.getDistance));
     */

    while (timer.get() >= 2.75 && timer.get() <= 15) {
      left1.set(0);
      left2.set(0);
      left3.set(0);
      right1.set(0);
      right2.set(0);
      right3.set(0);
    }
    // timer.stop();
    left1.set(0);
    left2.set(0);
    left3.set(0);
    right1.set(0);
    right2.set(0);
    right3.set(0);
    timer.stop();
    // Code to run during autonomous mode goes here

    /*
     * while (timer.get() < 5.0) {
     * // Put custom auto code here
     * // case kDefaultAuto:
     * // default:
     * // Put default auto code here
     * left1.set(0.5);
     * left2.set(0.5);
     * left3.set(0.5);
     * right1.set(0.5);
     * right2.set(0.5);
     * right3.set(0.5);
     * // commented until needed for auto
     * // double time_stamp = Timer.getFPGATimestamp();
     * }
     * while (timer.get() > 5.0)
     * left1.set(0);
     * left2.set(0);
     * left3.set(0);
     * right1.set(0);
     * right2.set(0);
     * right3.set(0);
     * /*
     */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*
     * left1.set(frcController.getLeftY());
     * left2.set(frcController.getLeftY());
     * left3.set(frcController.getLeftY());
     * right1.set(frcController.getRightY());
     * right2.set(frcController.getRightY());
     * right3.set(frcController.getRightY());
     */
    left1.set(0);
    left2.set(0);
    left3.set(0);
    right1.set(0);
    right2.set(0);
    right3.set(0);

    // Talon ourTalon = new Talon(10);

    if (mainDriveController.getLeftY() >= 0.05 || mainDriveController.getLeftY() <= -0.05) {
      left1.set(mainDriveController.getLeftY());
      left2.set(mainDriveController.getLeftY());
      left3.set(mainDriveController.getLeftY());
    }
    if (mainDriveController.getRightY() >= 0.05 || mainDriveController.getRightY() <= -0.05) {
      right1.set(mainDriveController.getRightY());
      right2.set(mainDriveController.getRightY());
      right3.set(mainDriveController.getRightY());
    }

    lightstrip.set(0);
    double color = 0;
    boolean isPressed = false;
    int counter = 0;

    if (isPressed = false) {
      color = 0;
      lightstrip.set(color);
    }

    if (mainDriveController.getAButtonPressed()) {
      isPressed = true;
      counter = 1;
    }
    while (isPressed) {
      while (color == 1) {
        color = 0.69;
        lightstrip.set(color);
      }

      if (mainDriveController.getAButtonPressed()) {
        counter++;
      }

      if (counter == 2) {
        color = 0.91;
        lightstrip.set(color);
        isPressed = false;
        counter = 0;
      }

    }

    /*
     * if (mainDriveController.getAButton() == true) {
     * lights = !lights;
     * color = 0.69;
     * lightstrip.set(color);
     * }
     * if (mainDriveController.getAButtonPressed() == true) {
     * color = 0.91;
     * lightstrip.set(color);
     * lights = !lights;
     * }
     * 
     * if (lights == false) {
     * color = 0;
     * lightstrip.set(0);
     * }
     */
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    pid.setPID(kP, kI, kDefaultPeriod);
    pid.setSetpoint(5);
    if (pid.atSetpoint()) {
      left1.set(0);
      left2.set(0);
      left3.set(0);
      right1.set(0);
      right2.set(0);
      right3.set(0);
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
  /*
   * public double getDistance() {
   * GenericEntry yaw = tab.add("Yaw", 0.0).getEntry();
   * double[] ypr = new double[3];
   * encoderImu.getYawPitchRoll(ypr);
   * double distance = yaw.setValue(ypr[0]);
   * return distance;
   * 
   * 
   * double[] ypr = new double[3];
   * encoderImu.getYawPitchRoll(ypr);
   * GenericPublisher yaw;
   * double distance = yaw.setValue(ypr[0]);
   * return distance;
   * 
   * }
   */
  /**
   * @param setpoint
   * @param currentValue
   * @return
   * @return
   */
  /*
   * public double calculate(double setpoint, double currentValue) {
   * pid.enableContinuousInput(-180, 180);
   * double error = setpoint - currentValue;
   * double integral = error * 0.02;
   * double previousError;
   * double derivative = (error - previousError) / 0.02;
   * previousError = error;
   * double kP;
   * return kP * error + kI * integral + kD * derivative;
   * //currentValue = ((Robot) encoderImu).getDistance();
   * }
   */

  /*
   * 1. display the encoder values on the smartdashboard
   * 2. set to go based on rotations (w pid loop, just p) CART TEST
   * 3. if the first test is ok, test on ground and emergency stop when needed
   * 4. test generalizing p value at different distances, etc
   * if problems with precision, add in other elements NOT IF UNNECESSARY
   * INTEGRAL WHEN YOU NEED TO BUILD POWER, DERIVATIVE FOR WHEN IT ACCELERATES TOO
   * FAST
   * pid allowable error: don't go for perfect precision, find margain of error
   * if you want higher precision, that'll happen over many more actions, but will
   * be more exact
   * --> precision versus time
   */

}
/*
 * //FOR TALON
 * 
 * public class
 * 
 * //Put these imports in the import section
 * import edu.wpi.first.hal.FRCNetComm.tResourceType;
 * import edu.wpi.first.hal.HAL;
 * import edu.wpi.first.wpilibj.PWM;
 * 
 * import edu.wpi.first.wpilibj.MotorSafety;
 * import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
 * import edu.wpi.first.wpilibj.motorcontrol.Talon;
 * //ok so these are what the different values indicate in the spark class and
 * when we're doing encoder stuff
 * 
 * 2.003ms = full "forward"
 * 1.550ms = the "high end" of the deadband range
 * 1.500ms = center of the deadband range (off)
 * 1.460ms = the "low end" of the deadband range
 * 0.999ms = full "reverse"
 * 
 * 
 * public class PWMSparkMax extends PWMMotorController {
 * /**
 * //METHODS !!
 * public PWMSparkMax(final int channel) {
 * super("PWMSparkMax", channel);
 * m_pwm.setBounds(2.003, 1.55, 1.50, 1.46, 0.999);
 * m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
 * m_pwm.setSpeed(0.0);
 * m_pwm.setZeroLatch();
 * }
 * 
 * //MORE METHODS SO YOU CAN ACTUALLY USE THE STUFF ABOVE
 * 
 * public void setBounds​(double max, double deadbandMax, double center, double
 * deadbandMin, double min)
 * 
 * //Pneumatics
 * (We'll need to know where we're using them before we can use this code
 * without errors)
 * 
 * //Imports for pneumatics
 * import edu.wpi.first.wpilibj.PneumaticsModuleType;
 * import edu.wpi.first.wpilibj.Solenoid;
 * 
 * public class (whatever part we're using)
 * {
 * nameForWhateverItDoes = new Solenoid(config.pcm_ID,
 * PneumaticsModuleType.CTREPCM, config.intake_channel);
 * }
 */
