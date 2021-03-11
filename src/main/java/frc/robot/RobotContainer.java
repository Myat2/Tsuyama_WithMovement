/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleCmd;
import frc.robot.commands.auto.MoveOnce;
import frc.robot.commands.gamepad.OI;
import frc.robot.subsystems.OmniDrive;

public class RobotContainer {

  /**
   * Create the subsystems and gamepad objects
   */
  public final static OmniDrive m_omnidrive = new OmniDrive();
  public final static OI m_oi = new OI();
  public final static TeleCmd m_teleCmd = new TeleCmd();
  public final static MoveOnce m_move_once = new MoveOnce();
  public RobotContainer()
  {
      //Create new instances


      //Set the default command for the hardware subsytem
      m_omnidrive.setDefaultCommand(m_teleCmd);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_teleCmd;
  }
}
