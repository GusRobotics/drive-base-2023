// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/* commented out imports are not used! 
 * comment back in when ready to use!
 */
//import edu.wpi.first.networktables.GenericEntry;
//import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.XboxController;
//commented out until needed
//import java.util.TimerTask;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

//import edu.wpi.first.wpilibj.shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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
  CANSparkMax left1 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax left2 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax left3 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax right1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax right2 = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax right3 = new CANSparkMax(7, MotorType.kBrushless);
  XboxController frcController = new XboxController(0);
  PigeonIMU encoderImu = new PigeonIMU(19);

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

  // need to determine values for kP kI and kD
  private double kP;
  private double kI;
  private double kD;
  PIDController pid = new PIDController(kP, kI, kD);

  public Robot(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    // GOOD ONE PigeonIMU encoderImu = new PigeonIMU(19);

    // create shuffleboard (for custom driving)
    // ShuffleboardTab tab = Shuffleboard.getTab("PigeonIMU");

    // set current limits
    left1.setSmartCurrentLimit(50);
    left2.setSmartCurrentLimit(50);
    left3.setSmartCurrentLimit(50);
    right1.setSmartCurrentLimit(50);
    right2.setSmartCurrentLimit(50);
    right3.setSmartCurrentLimit(50);
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
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
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
    while (timer.get() <= 3.15) {
      left1.set(-.5);
      left2.set(-.5);
      left3.set(-.5);
      right1.set(-.5);
      right2.set(-.5);
      right3.set(-.5);
    }
    /*
     * left1.set(pid.calculate(90, encoderImu.getYawPitchRoll(null));
     * left2.set(pid.calculate(90, encoderImu.getDistance()));
     * left3.set(pid.calculate(90, encoderImu.getDistance()));
     * right1.set(pid.calculate(90, encoderImu.getDistance()));
     * right2.set(pid.calculate(90, encoderImu.getDistance));
     * right3.set(pid.calculate(90, encoderImu.getDistance));
     */

    while (timer.get() >= 3.15 && timer.get() <= 15) {
      left1.set(0);
      left2.set(0);
      left3.set(0);
      right1.set(0);
      right2.set(0);
      right3.set(0);
    }
    timer.stop();
    left1.set(0);
    left2.set(0);
    left3.set(0);
    right1.set(0);
    right2.set(0);
    right3.set(0);
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

    if (frcController.getLeftY() >= 0.05 || frcController.getLeftY() <= -0.05) {
      left1.set(frcController.getLeftY());
      left2.set(frcController.getLeftY());
      left3.set(frcController.getLeftY());
    }
    if (frcController.getRightY() >= 0.05 || frcController.getRightY() <= -0.05) {
      right1.set(frcController.getRightY());
      right2.set(frcController.getRightY());
      right3.set(frcController.getRightY());
    }

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
