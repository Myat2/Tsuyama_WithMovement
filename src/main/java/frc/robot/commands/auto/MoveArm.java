package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//WPI imports
import edu.wpi.first.wpilibj2.command.CommandBase;
//RobotContainer import
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
//Subsystem imports


/**
 * SimpleDrive class
 * <p>
 * This class drives a motor 
 */
public class MoveArm extends CommandBase
{
    //Grab the subsystem instance from RobotContainer
    private final static Arm m_arm = RobotContainer.m_arm;
    private double dT = 0.02;
    private boolean m_endFlag = false;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private TrapezoidProfile m_profile;

    private double tgt_x, tgt_y, cur_x, cur_y, start_x, start_y;
    private double tgt_dist;
    private double m_dx, m_dy;    

    /**
     * This command moves the robot a certain distance following a trapezoidal speed profile.
     * <p>
     * 
     * @param pos - target position
     * @param maxSpeed - max speed of robot
     */
    //This move the robot a certain distance following a trapezoidal speed profile.
    public MoveArm(double x, double y, double maxSpeed)
    {
        m_constraints = new TrapezoidProfile.Constraints(maxSpeed, maxSpeed);
        tgt_x = x;
        tgt_y = y;
        //addRequirements(m_arm); // Adds the subsystem to the command

    }

    public void SetTargetPos(double x, double y) {
        tgt_x = x;
        tgt_y = y;
    }
    /**
     * Runs before execute
     */
    @Override
    public void initialize()
    {   
        start_x = m_arm.getArmPosX();
        start_y = m_arm.getArmPosY();
        m_dx = tgt_x - start_x;
        m_dy = tgt_y - start_y;
        tgt_dist = Math.sqrt(m_dx*m_dx + m_dy*m_dy);

        m_goal = new TrapezoidProfile.State(tgt_dist, 0);
        m_setpoint = new TrapezoidProfile.State(0, 0);
        m_endFlag = false;
        if ((m_dx==0) && (m_dy==0)) {
            m_endFlag = true;
        }
    }
    /**
     * Condition to end speed profile
     */
    public boolean endCondition()
    {
        return false;
    }
    /**
     * Called continously until command is ended
     */
    @Override
    public void execute()
    {
        //Create a new profile to calculate the next setpoint(speed) for the profile

        m_profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);

        m_setpoint = m_profile.calculate(dT);

        cur_x = start_x + m_setpoint.position*m_dx/tgt_dist;
        cur_y = start_y + m_setpoint.position*m_dy/tgt_dist;
        m_arm.setArmPos(cur_x, cur_y);

        if ((m_profile.isFinished(dT)) ) {
            //distance reached End the command
            m_arm.setArmPos(tgt_x, tgt_y);
            m_endFlag = true;
        }
    }

    /**
     * Called when the command is told to end or is interrupted
     */
    @Override
    public void end(boolean interrupted)
    {

    }

    /**
     * Creates an isFinished condition if needed
     */
    @Override
    public boolean isFinished()
    {
        return m_endFlag;
    }

}