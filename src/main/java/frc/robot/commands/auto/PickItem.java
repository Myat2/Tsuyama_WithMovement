package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import the commands

//WPI imports


/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to pick up objects
 */
public class PickItem extends SequentialCommandGroup
{
    
    public PickItem()
    {
      
      super(
        new MoveArm(0.4, 0.4, 0.4),
        new MoveArm(0.5, 0.5, 0.4)
      

      );
      
    }
}