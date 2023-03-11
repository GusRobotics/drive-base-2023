// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;

// import java.lang.Math;

// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Map;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.Timer;

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

  // Enum for controlling which mode is passed through to the drive control

  // Enum for controlling which autonomous is run
  enum autos {
    basic
  }

  // Loop iteration variables for time control
  int k = 0;
  int k_new = k;

  private autos selectedAuto;

  private final SendableChooser<autos> m_chooser = new SendableChooser<>();

  // motor initialization

  // base motors
  CANSparkMax left1 = new CANSparkMax(9, MotorType.kBrushless);
  CANSparkMax left2 = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax left3 = new CANSparkMax(15, MotorType.kBrushless);
  CANSparkMax right1 = new CANSparkMax(10, MotorType.kBrushless);
  CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax right3 = new CANSparkMax(7, MotorType.kBrushless);

  MotorControllerGroup leftDrive = new MotorControllerGroup(left1, left2, left3);
  MotorControllerGroup rightDrive = new MotorControllerGroup(right1, right2, right3);

  // everything but intake is brushless, intake will have different motor
  // controller

  Timer timer = new Timer();
  CANSparkMax intake = new CANSparkMax(36, MotorType.kBrushless);

  CANSparkMax elevator = new CANSparkMax(14, MotorType.kBrushless);
  float elevatorLL = 0;
  float elevatorUL = 15;

  // CANSparkMax arm = new CANSparkMax(13, MotorType.kBrushless);
  // float armLL = -10000;
  // float armUL = 15;

  CANSparkMax rotIn = new CANSparkMax(12, MotorType.kBrushless);
  float wristLL = 0;
  float wristUL = 15;

  // lightstrip/blinkin
  Spark lightstrip = new Spark(3);

  // controllers
  XboxController mainDriveController = new XboxController(0);
  XboxController coDriver = new XboxController(1);

  // pigeon
  // PigeonIMU pigeonIMU = new PigeonIMU(19);

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
  double kP = 0;
  double kD = 0;
  PIDController theLoop = new PIDController(kP, 0, kD);

  // PID Control for various functions on the robot (arm, wrist, elevator)
  // Will slow the input to the motors relative to the setpoints (set as
  // softLimits)
  PIDController functionsController = new PIDController(0.75, 0, -0.25);

  double startPitch = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    // Set the digital limits for each direction of the elevator,
    // arm, and wrist (rotIn)
    elevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, elevatorUL);
    elevator.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, elevatorLL);

    // arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, armUL);
    // arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, armLL);

    // rotIn.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, wristUL);
    // rotIn.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, wristLL);

    // Set the right drive to be inverted
    rightDrive.setInverted(true);

    // Enable Compressor
    compressor.enableAnalog(80, 120);

    // Initialize shifting into low gear
    // Note: kReverse is Low Gear
    driveShift.set(Value.kReverse);

    // set current limits
    left1.setSmartCurrentLimit(80);
    left2.setSmartCurrentLimit(80);
    left3.setSmartCurrentLimit(80);
    right1.setSmartCurrentLimit(80);
    right2.setSmartCurrentLimit(80);
    right3.setSmartCurrentLimit(80);
    elevator.setSmartCurrentLimit(80);
    intake.setSmartCurrentLimit(30);
    rotIn.setSmartCurrentLimit(40);

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

    // Display relevant information to SmartDashboard.
    // SmartDashboard.putNumber("Pitch", pigeonIMU.getPitch());
    // SmartDashboard.putNumber("Calculation",
    // theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));

    SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    // Use this variable to track the number of 20 ms loops the robot has completed.
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

    timer.reset();
    timer.start();
    // Get the selected auto
    selectedAuto = m_chooser.getSelected();
    System.out.println("Auto selected:" + selectedAuto);
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // SmartDashboard.putNumber("Encoder", encoderImu.getEncoder());
    // System.out.println("Auto selected: " + m_autoSelected);

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
    // switch (m_autoSelected) {
    // // case kleftAuto:
    // }

    // SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    // SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    // SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    // SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    // SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    // SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    // double seconds = timer.get();

    // create shooting, driving auto
    // run intake for about a second at 75% power, then drive
    // initial speed

    // while (timer.get() <= 1) {
    // intake.set(-.75);
    // }
    // while (timer.get() > 1 && timer.get() <= 4) {
    // leftDrive.set(.3);
    // rightDrive.set(.3);
    // }
    // // // speed slowed down after 1.6 seconds but before 2.75 seconds
    // while (timer.get() > 4 && timer.get() <= 4.5) {
    // leftDrive.set(.1);
    // rightDrive.set(.2);
    // }

    intake.set(0);
    rightDrive.set(0);
    leftDrive.set(0);

    if (timer.get() < 1) {
      intake.set(-.9);
    } else if (timer.get() < 4) {
      leftDrive.set(.3);
      rightDrive.set(-.3);
    } else if (timer.get() < 5.5) {
      leftDrive.set(.1);
      rightDrive.set(-.2);
    } else {
      timer.stop();
      leftDrive.set(0);
      rightDrive.set(0);
    }
    // while (timer.get() <= 1.6) {
    // leftDrive.set(.3);
    // rightDrive.set(-.3);
    // }
    // // // speed slowed down after 1.6 seconds but before 2.75 seconds
    // while (timer.get() > 1.6 && timer.get() <= 3.6) {
    // leftDrive.set(.1);
    // rightDrive.set(-.2);
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

    // codriver
    // elevator
    if (coDriver.getLeftY() >= 0.05) {
      elevator.set(0.5);
    } else if (coDriver.getLeftY() <= -0.05) {
      elevator.set(-0.5);
    } else {
      elevator.set(0);
    }
    // arm rotation
    if (coDriver.getLeftBumper()) {
      rotIn.set(-0.3);
    } else if (coDriver.getRightBumper()) {
      rotIn.set(.3);
    } else {
      rotIn.set(0);
    }
    // else if (mainDriveController.getLeftBumper() && rotIn.get() == 0.4) {
    // rotIn.set(-0.4);
    // }
    // else if (mainDriveController.getLeftBumper() && rotIn.get() == -0.4) {
    // rotIn.set(0);
    // }

    // drivetrain
    // if (mainDriveController.getRightX() >= 0.05 || mainDriveController.getLeftY()
    // >= 0.05
    // || mainDriveController.getLeftY() <= -0.05 || mainDriveController.getRightX()
    // <= -0.05) {
    // driveTrain.arcadeDrive(-(mainDriveController.getRightX()),
    // -(mainDriveController.getLeftY()));

    if (mainDriveController.getLeftY() >= 0.1 || mainDriveController.getLeftY() <= -0.1) {
      leftDrive.set(mainDriveController.getLeftY() - .1);
    } else {
      leftDrive.set(0);
    }

    if (mainDriveController.getRightY() >= 0.1 || mainDriveController.getRightY() <= -0.1) {
      rightDrive.set(mainDriveController.getRightY() - .1);
    } else {
      rightDrive.set(0);
    }

    // intake sets (codriver triggers)
    if (isInputting(coDriver.getLeftTriggerAxis(), 0.05)) {
      intake.set(-0.50);
    } else if (isInputting(coDriver.getRightTriggerAxis(), 0.05)) {
      intake.set(0.50);
    } else if (coDriver.getYButton()) {
      intake.set(-.9);
    } else if (coDriver.getAButton()) {
      intake.set(-.3);
    } else if (coDriver.getBButton()) {
      intake.set(-.75);
    } else {
      intake.set(0);
    }

    // shifting one bd trigger, other trigger putting drive motors into brake
    // triggers for intake and spit out, xab whatever are for different speeds
    // shift, brake, stick, arm angles for base

    // if we need toggle
    if (mainDriveController.getRightTriggerAxis() >= 0.8 && k - k_new >= 50) {
      driveShift.toggle();
      k_new = k;
    }
    // lightstrip

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

    // Reset and start the timer
    timer.reset();
    timer.start();

    elevator.setIdleMode(IdleMode.kBrake);
    intake.setIdleMode(IdleMode.kCoast);

    dashboardInitialize();

    // Initialize gear state (Low)
    driveShift.set(Value.kReverse);

    // Initialize PID Loop & Pigeon Readings
    // startPitch = pigeonIMU.getPitch();

    // Allows a choice for kP, kD coefficients in the diagnostics tab of the
    // shuffleboard view
    GenericEntry kpentry = Shuffleboard.getTab("Diagnostics")
        .add("kP", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    GenericEntry kdentry = Shuffleboard.getTab("Diagnostics")
        .add("kD", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -2, "max", 0))
        .getEntry();

    kP = kpentry.getDouble(0);
    kD = kdentry.getDouble(0);

    theLoop.setP(kP);
    theLoop.setD(kD);

    theLoop.setSetpoint(startPitch);
    theLoop.setTolerance(5);
    theLoop.enableContinuousInput(-15, 15);

    // SmartDashboard.putNumber("Pitch", pigeonIMU.getPitch());
    // SmartDashboard.putNumber("Calculation",
    // theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));

    // Is this necessary???
    // theLoop.close();

    // Initialize shuffleboard
    // shuffleboardInitialize();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // dashboardUpdate();

    // driverControls(driveMode.twinArcade);

    // Small test implementation for the PID loop:
    // Activate PID Loop
    // if (mainDriveController.getLeftTriggerAxis() >= 0.05) {
    // leftDrive.set(0.75 * theLoop.calculate(pigeonIMU.getPitch(),
    // theLoop.getSetpoint()));
    // rightDrive.set(0.75 * theLoop.calculate(pigeonIMU.getPitch(),
    // theLoop.getSetpoint()));
    // }

    // intake

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

  // Method which contains all of the driver control information,

  // private void driverControls(driveMode mode) {

  // // Drive Controls
  // // Switch cases to accomodate for different driving input modes (tank,
  // arcade)
  // switch (mode) {
  // case tank:
  // driveTrain.tankDrive(adjustForDeadband(mainDriveController.getLeftY(), 0.05),
  // adjustForDeadband(mainDriveController.getRightY(), 0.05));
  // break;
  // case twinArcade:
  // driveTrain.arcadeDrive(-0.625 *
  // adjustForDeadband(mainDriveController.getRightX(), 0.05),
  // -0.625 * adjustForDeadband(mainDriveController.getLeftY(),
  // 0.05));
  // break;
  // }

  // // Gear shifting
  // if (mainDriveController.getLeftBumper() && k - k_new >= 50) {
  // driveShift.toggle();
  // k_new = k;
  // }

  // // intake
  // if (isInputting(coDriver.getLeftTriggerAxis(), 0.05)) {
  // intake.set(-0.40);
  // } else if (isInputting(coDriver.getRightTriggerAxis(), 0.05)) {
  // intake.set(0.40);
  // } else {
  // intake.set(0);
  // }

  // // arm extension
  // // if (mainDriveController.getXButtonPressed()) {
  // // arm.set(-0.5);
  // // } else if (mainDriveController.getBButtonPressed()) {
  // // arm.set(0.5);
  // // } else {
  // // arm.set(0);
  // // }

  // }

  // Runs a motor until it gets to the setpoint using a PID Loop.
  // private void toSetpoint(double setPoint, CANSparkMax toControl) {

  // toControl.set(functionsController.calculate(setPoint -
  // toControl.getEncoder()));

  // }

  // A simple method to determine whether or not a particular axis is meant to
  // input
  private boolean isInputting(double inputAxis, double deadband) {
    if (adjustForDeadband(inputAxis, deadband) != 0) {
      return true;
    }
    return false;
  }

  // Adjusts an input for a particular deadband. Use this to combat input
  // (controller, joystick, etc.) drift.
  private double adjustForDeadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    }
    return input;
  }

  // Used for SmartDashboard, periodically update entries.
  // This may not be necessary depending on whether or not SmartDashboard
  // already updates items in the background...
  private void dashboardInitialize() {

    SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    // SmartDashboard.putNumber("Pitch", pigeonIMU.getPitch());
    // SmartDashboard.putNumber("Calculation",
    // theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));

  }

  // Initializes Shuffleboard entries
  // todo: FIX THIS! -> Shuffleboard calls PID loop before initialization when put
  // where it should be (RobotInit)
  // private void shuffleboardInitialize() {

  // // Sets up options for autonomous
  // m_chooser.setDefaultOption("Basic Auto", autos.basic);
  // // m_chooser.addOption("Basic Auto", autos.basic);

  // // This tab contains all data particular to diagnosing problems on the robot
  // // Shuffleboard.getTab("SmartDashboard")
  // // .addPersistent("Auto Choice", m_chooser);

  // Shuffleboard.getTab("Diagnostics")
  // .add("Pitch", pigeonIMU.getPitch());

  // Shuffleboard.getTab("Diagnostics")
  // .add("Calculation", theLoop.calculate(pigeonIMU.getPitch(),
  // theLoop.getSetpoint()));

  // Shuffleboard.getTab("Diagnostics")
  // .add("left1", left1.getOutputCurrent());

  // Shuffleboard.getTab("Diagnostics")
  // .add("left2", left2.getOutputCurrent());

  // Shuffleboard.getTab("Diagnostics")
  // .add("left3", left3.getOutputCurrent());

  // Shuffleboard.getTab("Diagnostics")
  // .add("right1", right1.getOutputCurrent());

  // Shuffleboard.getTab("Diagnostics")
  // .add("right2", right2.getOutputCurrent());

  // Shuffleboard.getTab("Diagnostics")
  // .add("right3", right3.getOutputCurrent());

  // Shuffleboard.getTab("Diagnostics")
  // .add("Arm Rotations", arm.getEncoder());

  // Shuffleboard.getTab("Diagnostics")
  // .add("Elevator Rotations", elevator.getEncoder());

  // Shuffleboard.getTab("Diagnostics")
  // .add("Wrist Rotations", rotIn.getEncoder());

  // // Runs the camera server
  // Shuffleboard.getTab("SmartDashboard").add(CameraServer.startAutomaticCapture(1));

  // Shuffleboard.startRecording();

  // }

}
/*
 * 
 * 
 * 2.003ms = full "forward"
 * 1.550ms = the "high end" of the deadband range
 * 1.500ms = center of the deadband range (off)
 * 1.460ms = the "low end" of the deadband range
 * 0.999ms = full "reverse"
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