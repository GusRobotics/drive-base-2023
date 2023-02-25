// package frc.robot;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

package frc.robot;

//import edu.wpi.first.wpilibj.MotorSafety;
//import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;

//import edu.wpi.first.wpilibj.motorcontrol;
//import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.hal.HAL;
//import edu.wpi.first.wpilibj.PWM;

/*
 * Value meaning2.037ms = full "forward"
 * 1.539ms = the "high end" of the deadband range
 * 1.513ms = center of the deadband range (off)
 * 1.487ms = the "low end" of the deadband range
 * 0.989ms = full "reverse"
 */

// Red to white
// Green to black

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Cross the Road Electronics (CTRE) Talon and Talon SR Motor Controller.
 * <p>Note that the Talon uses the following bounds for PWM values. These values
 * should work
 * reasonably well for most controllers, but if users experience issues such as
 * asymmetric behavior
 * around the deadband or inability to saturate the controller in either
 * direction, calibration is
 * recommended. The calibration procedure can be found in the Talon User Manual
 * available from CTRE.
 */

/*
 * <ul>
 * <li>2.037ms = full "forward"
 * <li>1.539ms = the "high end" of the deadband range
 * <li>1.513ms = center of the deadband range (off)
 * <li>1.487ms = the "low end" of the deadband range
 * <li>0.989ms = full "reverse"
 * </ul>
 */
/*
 * public class Talon extends PWMMotorController {
 * 
 * 
 * @param channel The PWM channel that the Talon is attached to. 0-9 are
 * on-board, 10-19 are on the MXP port the MXP port
 * 
 * public Talon(final int channel) {
 * super("Talon", channel);
 * 
 * m_pwm.setBounds(2.037, 1.539, 1.513, 1.487, 0.989);
 * m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
 * m_pwm.setSpeed(0.0);
 * m_pwm.setZeroLatch();
 * 
 * HAL.report(tResourceType.kResourceType_Talon, getChannel() + 1);
 * }
 * 
 * public class PWMMotorController {
 * 
 * }
 * }
 */

// New talon code
public class talon {
    public talon(int channel) {
        new TalonSRX(channel);
    }
}