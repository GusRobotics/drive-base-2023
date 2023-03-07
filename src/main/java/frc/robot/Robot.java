// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.lang.Math;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  int k = 0;
  int k_new = k;

  /*
   * private static final String kDefaultAuto = "rampAuto";
   * private static final String kleftAuto = "left cube";
   * private static final String kRightAuto = "right cube";
   * private static final PneumaticsModuleType CTREPCM = null;
   */
  // private static final String kCubeAuto = "left cube";
  // private static final String krightCube = "right cube";

  private String m_autoSelected;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // motor initialization

  // base motors
  CANSparkMax left1 = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax left2 = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax left3 = new CANSparkMax(15, MotorType.kBrushless);
  CANSparkMax right1 = new CANSparkMax(10, MotorType.kBrushless);
  CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax right3 = new CANSparkMax(7, MotorType.kBrushless);

  // everything but intake is brushed, intake will have different motor controller

  CANSparkMax intake = new CANSparkMax(36, MotorType.kBrushless);

  CANSparkMax elevator = new CANSparkMax(14, MotorType.kBrushless);

  CANSparkMax arm = new CANSparkMax(13, MotorType.kBrushless);

  CANSparkMax rotIn = new CANSparkMax(12, MotorType.kBrushless);

  // lightstrip/blinkin
  Spark lightstrip = new Spark(3);

  // controllers
  XboxController mainDriveController = new XboxController(0);
  XboxController coDriver = new XboxController(1);

  // pigeon
  PigeonIMU pigeonIMU = new PigeonIMU(19);

  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  DoubleSolenoid driveShift = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 7, 3);

  // pneumatic system
  /*
   * final PneumaticsModuleType type = CTREPCM;
   * Compressor compressor = new Compressor(type);
   * Solenoid driveShift = new Solenoid(type, 0);
   */
  // set lightstrip colors

  boolean lights = false;
  double color_val;

  // PID Control Initialization
  PIDController theLoop = new PIDController(0.25, 00, 0.5);
  double startYaw = 0;
  double startRoll = 0;
  double startPitch = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    // m_chooser.setDefaultOption("Default Ramp Auto", kDefaultAuto);

    // m_chooser.addOption("Right Cube Auto", kRightAuto);
    // m_chooser.addOption("Left cube auto", kleftAuto);

    SmartDashboard.putData("Auto choices", m_chooser);
    // CameraServer.startAutomaticCapture();

    // Enable Compressor
    compressor.enableAnalog(80, 120);

    // Initialize shifting into low gear
    // Note: kReverse is Low Gear
    driveShift.set(Value.kReverse);

    // Select Autonomous
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // SmartDashboard.putNumber("Encoder", encoderImu.getEncoder());
    System.out.println("Auto selected: " + m_autoSelected);

    // SmartDashboard.putNumber("Encoder", );
    // GOOD ONE PigeonIMU encoderImu = new PigeonIMU(19);

    // set current limits
    left1.setSmartCurrentLimit(50);
    left2.setSmartCurrentLimit(50);
    left3.setSmartCurrentLimit(50);
    right1.setSmartCurrentLimit(50);
    right2.setSmartCurrentLimit(50);
    right3.setSmartCurrentLimit(50);

    // elevator.setSmartCurrentLimit(50);

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
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // SmartDashboard.putNumber("Encoder", encoderImu.getEncoder());
    // System.out.println("Auto selected: " + m_autoSelected);
    // compressor.enableAnalog(80, 120);

    SmartDashboard.putNumber("Pitch", pigeonIMU.getPitch());
    SmartDashboard.putNumber("Calculation", theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));

    SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    k++;

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
      // case kleftAuto:
    }

    SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    Timer timer = new Timer();
    timer.reset();
    timer.start();
    // double seconds = time.get();

    // initial speed
    // while (timer.get() <= 1.6) {
    // left1.set(.6);
    // left2.set(.6);
    // left3.set(.6);
    // right1.set(.6);
    // right2.set(.6);
    // right3.set(.6);
    // }
    // // speed slowed down after 1.6 seconds but before 2.75 seconds
    // while (timer.get() >= 1.6 && timer.get() <= 2.75) {
    // left1.set(.2);
    // left2.set(.2);
    // left3.set(.2);
    // right1.set(.2);
    // right2.set(.2);
    // right3.set(.2);
    // }
    // /*
    // * left1.set(pid.calculate(90, encoderImu.getYawPitchRoll(null));
    // * left2.set(pid.calculate(90, encoderImu.getDistance()));
    // * left3.set(pid.calculate(90, encoderImu.getDistance()));
    // * right1.set(pid.calculate(90, encoderImu.getDistance()));
    // * right2.set(pid.calculate(90, encoderImu.getDistance));
    // * right3.set(pid.calculate(90, encoderImu.getDistance));
    // */

    // while (timer.get() >= 2.75 && timer.get() <= 15) {
    // left1.set(0);
    // left2.set(0);
    // left3.set(0);
    // right1.set(0);
    // right2.set(0);
    // right3.set(0);
    // }
    // // timer.stop();
    // left1.set(0);
    // left2.set(0);
    // left3.set(0);
    // right1.set(0);
    // right2.set(0);
    // right3.set(0);
    // timer.stop();
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

    SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    left1.set(0);
    left2.set(0);
    left3.set(0);
    right1.set(0);
    right2.set(0);
    right3.set(0);

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

    // arm extension
    if (mainDriveController.getXButtonPressed()) {
      // arm.set(-.5);
    } else if (mainDriveController.getBButtonPressed()) {
      // arm.set(0.5);
    } else {
      // intake.set(0);
    }

    // press to shift
    // if (mainDriveController.getRightTriggerAxis() >= 0.05) {
    // driveShift.set(true);
    // } else {
    // driveShift.set(false);
    // }

    // codriver
    // intake
    if (coDriver.getLeftTriggerAxis() >= 0.05) {
      // intake.set(-0.75);
    } else if (coDriver.getRightTriggerAxis() >= 0.05) {
      // intake.set(0.75);
    } else {
      // intake.set(0);
    }
    // elevator
    if (coDriver.getLeftY() >= 0.05) {
      elevator.set(0.5);
    } else if (coDriver.getLeftY() <= -0.05) {
      elevator.set(-0.5);
    } else {
      elevator.set(0);
    }
    // arm rotation
    if (coDriver.getRightY() >= 0.05) {
      rotIn.set(0.4);
    } else if (coDriver.getRightY() <= -0.05) {
      rotIn.set(-0.4);
    } else {
      rotIn.set(0);
    }
    // lightstrip

    /*
     * lightstrip.set(0);
     * double color = 0;
     * boolean isPressed = false;
     * int counter = 0;
     * 
     * if (isPressed = false) {
     * color = 0;
     * lightstrip.set(color);
     * }
     * 
     * if (mainDriveController.getAButtonPressed()) {
     * isPressed = true;
     * counter = 1;
     * }
     * while (isPressed) {
     * while (color == 1) {
     * color = 0.69;
     * lightstrip.set(color);
     * }
     * 
     * if (mainDriveController.getAButtonPressed()) {
     * counter++;
     * }
     * 
     * if (counter == 2) {
     * color = 0.91;
     * lightstrip.set(color);
     * isPressed = false;
     * counter = 0;
     * }
     */
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

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

    SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    SmartDashboard.putNumber("right3", right3.getOutputCurrent());

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    driveShift.set(Value.kReverse);

    startYaw = pigeonIMU.getYaw();
    startPitch = pigeonIMU.getPitch();
    startRoll = pigeonIMU.getRoll();

    theLoop.setSetpoint(startPitch);
    theLoop.setTolerance(0.25);
    theLoop.enableContinuousInput(-15, 15);

    SmartDashboard.putNumber("Pitch", pigeonIMU.getPitch());
    SmartDashboard.putNumber("Calculation", theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));

    CameraServer.startAutomaticCapture();

    // compressor.enableAnalog(80, 120);

    // double position = pigeonIMU.getRoll();
    // double calculation = theLoop.calculate(position, 6.5);
    // final double output = theLoop.accept(0.5);

    // Calculates the outp
    // left1.set(theLoop.calculate(pigeonIMU.getRoll(), theLoop.getSetpoint()));
    // left2.set(theLoop.calculate(pigeonIMU.getRoll(), theLoop.getSetpoint()));
    // left3.set(theLoop.calculate(pigeonIMU.getRoll(), theLoop.getSetpoint()));
    // right1.set(theLoop.calculate(pigeonIMU.getRoll(), theLoop.getSetpoint()));
    // right2.set(theLoop.calculate(pigeonIMU.getRoll(), theLoop.getSetpoint()));
    // right3.set(theLoop.calculate(pigeonIMU.getRoll(), theLoop.getSetpoint()));
    // PIDCommand work = new PIDCommand(theLoop, pigeonIMU, position, .5, null)
    // pid command class is correct, need to group all motors and specify the
    // setpoint for parameters 4, 5

    // if (theLoop.atSetpoint()) {
    // right1.set(0);
    // right2.set(0);
    // right3.set(0);
    // left1.set(0);
    // left2.set(0);
    // left3.set(0);
    // }
    theLoop.close();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // compressor.enableAnalog(80, 120);

    SmartDashboard.putNumber("Pitch", pigeonIMU.getPitch());
    SmartDashboard.putNumber("Calculation", theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));

    /*
     * if (Math.abs(mainDriveController.getLeftY()) >= 0.05 ||
     * Math.abs(mainDriveController.getRightX()) >= 0.05) {
     * 
     * left1.set(mainDriveController.getLeftY() - .4 *
     * mainDriveController.getRightX());
     * left2.set(mainDriveController.getLeftY() - .4 *
     * mainDriveController.getRightX());
     * left3.set(mainDriveController.getLeftY() - .4 *
     * mainDriveController.getRightX());
     * right1.set(mainDriveController.getLeftY() + .4 *
     * mainDriveController.getRightX());
     * right2.set(mainDriveController.getLeftY() + .4 *
     * mainDriveController.getRightX());
     * right3.set(mainDriveController.getLeftY() + .4 *
     * mainDriveController.getRightX());
     * 
     * } else {
     * 
     * left1.set(0);
     * left2.set(0);
     * left3.set(0);
     * right1.set(0);
     * right2.set(0);
     * right3.set(0);
     * 
     * }
     * 
     */

    left1.set(0);
    left2.set(0);
    left3.set(0);
    right1.set(0);
    right2.set(0);
    right3.set(0);

    if (mainDriveController.getLeftY() >= 0.05 || mainDriveController.getLeftY() <= -0.05) {
      left1.set(mainDriveController.getLeftY());
      left2.set(mainDriveController.getLeftY());
      left3.set(mainDriveController.getLeftY());
    }
    if (mainDriveController.getRightY() >= 0.05 ||
        mainDriveController.getRightY() <= -0.05) {
      right1.set(mainDriveController.getRightY());
      right2.set(mainDriveController.getRightY());
      right3.set(mainDriveController.getRightY());
    }

    SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    // arm extension
    if (mainDriveController.getXButtonPressed()) {
      // arm.set(-.5);
    } else if (mainDriveController.getBButtonPressed()) {
      // arm.set(0.5);
    } else {
      // arm.set(0);
    }

    // press to shift
    if (mainDriveController.getLeftBumper() && k - k_new >= 50) {

      driveShift.toggle();
      k_new = k;

    }

    // codriver
    // intake
    if (coDriver.getLeftTriggerAxis() >= 0.05) {
      intake.set(-0.75);
    } else if (coDriver.getRightTriggerAxis() >= 0.05) {
      intake.set(0.75);
    } else {
      intake.set(0);
    }
    // elevator
    if (coDriver.getLeftY() >= 0.05) {
      elevator.set(0.5);
    } else if (coDriver.getLeftY() <= -0.05) {
      elevator.set(-0.5);
    } else {
      elevator.set(0);
    }
    // arm rotation
    if (coDriver.getRightY() >= 0.05) {
      rotIn.set(0.4);
    } else if (coDriver.getRightY() <= -0.05) {
      rotIn.set(-0.4);
    } else {
      rotIn.set(0);
    }

    // Small test implementation for the PID loop:
    // Activate PID Loop
    if (mainDriveController.getLeftTriggerAxis() >= 0.05) {
      left1.set(theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));
      left2.set(theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));
      left3.set(theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));
      right1.set(theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));
      right2.set(theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));
      right3.set(theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));
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

  /**
   * @param setpoint
   * @param currentValue
   * @return
   * @return
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