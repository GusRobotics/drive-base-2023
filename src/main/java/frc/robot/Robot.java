// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;

// import java.lang.Math;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid;

// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// //import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

// import java.util.Map;

// import com.ctre.phoenix.sensors.BasePigeon;
// import com.ctre.phoenix.sensors.Pigeon2;
// import com.ctre.phoenix.sensors.PigeonIMU;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;

//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

//import edu.wpi.first.wpilibj.Solenoid;
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
  // enum autos {
  //   basic
  // }

  // Loop iteration variables for time control
  // int k = 0;
  // int k_new = k;

  // private autos selectedAuto;

  // private final SendableChooser<autos> m_chooser = new SendableChooser<>();

  // // motor initialization

  // // base motors
  // CANSparkMax left1 = new CANSparkMax(9, MotorType.kBrushless);
  // CANSparkMax left2 = new CANSparkMax(11, MotorType.kBrushless);
  // CANSparkMax left3 = new CANSparkMax(15, MotorType.kBrushless);
  // CANSparkMax right1 = new CANSparkMax(10, MotorType.kBrushless);
  // CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);
  // CANSparkMax right3 = new CANSparkMax(7, MotorType.kBrushless);

  // MotorControllerGroup leftDrive = new MotorControllerGroup(left1, left2, left3);
  // MotorControllerGroup rightDrive = new MotorControllerGroup(right1, right2, right3);

  // Timer timer = new Timer();

  // // CANSparkMax arm = new CANSparkMax(13, MotorType.kBrushless);
  // float armLL = -10;
  // float armUL = 15;

  // // CANSparkMax rotIn = new CANSparkMax(12, MotorType.kBrushless);
  // float wristLL = 0;
  // float wristUL = 9;

  // // lightstrip/blinkin
  // Spark lightstrip = new Spark(2);

  // // controllers
  XboxController mainDriveController = new XboxController(0);
  // XboxController coDriver = new XboxController(1);

  // // module refers to pcm --> always 2 unless we need a second one ig
  // Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
  // DoubleSolenoid driveShift = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 7, 3);
  // // DoubleSolenoid shooter = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 1,
  // // 2);
  // // DoubleSolenoid intakeDrop = new DoubleSolenoid(2, PneumaticsModuleType.REVPH,
  // // 4, 5);

  // PigeonIMU pigeon = new PigeonIMU(10);

  // CANSparkMax intake = new CANSparkMax(36, MotorType.kBrushless);

  // set lightstrip colors

  // boolean lights = false;
  // double color_val;

  // // PID Control Initialization
  // double kP = 0;
  // double kD = 0;
  // PIDController theLoop = new PIDController(kP, 0, kD);

  // // PID Control for various functions on the robot (arm, wrist, elevator)
  // // Will slow the input to the motors relative to the setpoints (set as
  // // softLimits)
  // PIDController functionsController = new PIDController(0.75, 0, -0.25);

  // AnalogInput distSensor = new AnalogInput(0);
  // double startPitch = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

   CANSparkMax top = new CANSparkMax(12, MotorType.kBrushless);
   CANSparkMax bottom = new CANSparkMax(36, MotorType.kBrushless);

  @Override
  public void robotInit() {
    // // Set the right drive to be inverted
    // rightDrive.setInverted(true);

    // // Enable Compressor
    // compressor.enableAnalog(100, 120);

    // // Initialize shifting into low gear
    // // Note: kReverse is Low Gear
    // driveShift.set(Value.kReverse);

    // // set current limits
    // left1.setSmartCurrentLimit(50);
    // left2.setSmartCurrentLimit(50);
    // left3.setSmartCurrentLimit(50);
    // right1.setSmartCurrentLimit(50);
    // right2.setSmartCurrentLimit(50);
    // right3.setSmartCurrentLimit(50);

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

    // SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    // SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    // SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    // SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    // SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    // SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    // // Use this variable to track the number of 20 ms loops the robot has completed.
    // k++;

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

  }

  // spin at tend and do it when we grab the cube
  // leave arm down and we raise it as we come back

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if(mainDriveController.getLeftBumperPressed()){
    //   top.set(0.5);
    //   bottom.set(0.5);
    // } else {
    //   top.set(0);
    //   bottom.set(0);
    // }
    if(mainDriveController.getRightY() > 0.1){
      top.set(-0.75);
      bottom.set(-0.7);
    } else{
      bottom.set(0);
      top.set(0);
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

    // Reset and start the timer
    // timer.reset();
    // timer.start();

    // elevator.setIdleMode(IdleMode.kBrake);
    // intake.setIdleMode(IdleMode.kCoast);

    dashboardInitialize();

    // Initialize gear state (Low)
    // driveShift.set(Value.kReverse);

    // Initialize PID Loop & Pigeon Readings
    // startPitch = pigeonIMU.getPitch();

    // Allows a choice for kP, kD coefficients in the diagnostics tab of the
    // shuffleboard view
    // GenericEntry kpentry = Shuffleboard.getTab("Diagnostics")
    //     .add("kP", 0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", 0, "max", 1))
    //     .getEntry();

    // GenericEntry kdentry = Shuffleboard.getTab("Diagnostics")
    //     .add("kD", 0)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("min", -2, "max", 0))
    //     .getEntry();

    // kP = kpentry.getDouble(0);
    // kD = kdentry.getDouble(0);

    // theLoop.setP(kP);
    // theLoop.setD(kD);

    // theLoop.setSetpoint(startPitch);
    // theLoop.setTolerance(5);
    // theLoop.enableContinuousInput(-15, 15);

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

  // Used for SmartDashboard, periodically update entries.
  // This may not be necessary depending on whether or not SmartDashboard
  // already updates items in the background...
  private void dashboardInitialize() {

    // SmartDashboard.putNumber("left1", left1.getOutputCurrent());
    // SmartDashboard.putNumber("left2", left2.getOutputCurrent());
    // SmartDashboard.putNumber("left3", left3.getOutputCurrent());
    // SmartDashboard.putNumber("right1", right1.getOutputCurrent());
    // SmartDashboard.putNumber("right2", right2.getOutputCurrent());
    // SmartDashboard.putNumber("right3", right3.getOutputCurrent());

    // SmartDashboard.putNumber("Pitch", pigeonIMU.getPitch());
    // SmartDashboard.putNumber("Calculation",
    // theLoop.calculate(pigeonIMU.getPitch(), theLoop.getSetpoint()));

  }

}
