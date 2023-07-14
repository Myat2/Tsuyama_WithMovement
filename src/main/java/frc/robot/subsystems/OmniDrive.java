package frc.robot.subsystems;


//Java imports

//Vendor imports
import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class OmniDrive extends SubsystemBase
{
    //Creates all necessary hardware interface here for omni-drive

    //Motors and encoders
    private final TitanQuad[] motors;

    private double curHeading, targetHeading;
    private double[] motorOuts;
    
    
    //DigitalOutput debugOut = new DigitalOutput(8);


    // Sensors
    private final AHRS gyro;

    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("OmniDrive");

    private final NetworkTableEntry D_navYaw = tab.add("Nav Yaw", 0).getEntry();
    private final NetworkTableEntry D_curHeading = tab.add("curHeading", 0).getEntry();
    private final NetworkTableEntry D_tgtHeading = tab.add("tgtHeading", 0).getEntry();
    private final NetworkTableEntry D_inputW = tab.add("inputW", 0).getEntry();


    //Subsystem for omnidrive
    public OmniDrive() {


        //Omni drive motors
        motors = new TitanQuad[Constants.MOTOR_NUM];
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            motors[i] = new TitanQuad(Constants.TITAN_ID, i);
            motors[i].setInverted(true);   //CCW is positive
        }


        motorOuts = new double[Constants.MOTOR_NUM];

        // gyro for rotational heading control
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.zeroYaw();
        curHeading = targetHeading = getYawRad();

    }



    public double getYawRad() {
        return -gyro.getYaw()*Math.PI/180;
    }

    /**
     * Call for the current angle from the internal NavX
     * <p>
     * 
     * @return yaw angle in degrees range -180° to 180°
     */
    public double getYaw() {
        //return gyro.getYaw();
        return gyro.getRawGyroZ();
    }

    /**
     * Resets the yaw angle back to zero
     */
    public void resetGyro() {
        gyro.zeroYaw();
    }

    public void resetHeading() {
        curHeading = targetHeading = getYawRad();
    }
    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setMotorSpeedAll(final double speed)
    {
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            motors[i].set(speed);
        }
        
    }
    // CCW is positive
    public void setMotorOut0123(double s0, double s1, double s2, double s3)
    {
        
        motors[0].set(s0);
        motors[1].set(s1);
        motors[2].set(s2);
        motors[3].set(s3);
        
    }
    
    /**
     * Sets the Output power of the 4 motors according to robot x, y, and z speed
     * This will result in open loop control of the robot speed
     * This is for appreciation only. No real use for this function.
     * <p>
     * 
     * @param x range -1 to 1 (0 stop)
     * @param y range -1 to 1 (0 stop)
     * @param z range -1 to 1 (0 stop)
     */
    public void setRobotSpeedXYW_Open(double x, double y, double w)
    {

        // Follow Studica titan convention
        // Left   Right
        // M2     M0   -  Front
        // M3     M1   -  Back
        //
        // M0 = [-1    1     1]              //Right-front wheel
        // M1 = [+1    1     1]              //Right-back wheel
        // M2 = [-1   -1     1] * [x y w]'   //Left-front wheel
        // M3 = [+1   -1     1]              //Left-back wheel

        motorOuts[0] = (-x +y +w);
        motorOuts[1] = ( x +y +w);
        motorOuts[2] = (-x -y +w);
        motorOuts[3] = ( x -y +w);

        //Limit output to -1.0 to 1.0 as PID outputs may be greater then 1.0
        double max=1.0;
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            max = Math.max(max, Math.abs(motorOuts[i]));
        }

        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            motors[i].set(motorOuts[i]/max);
            ///////////////////////////////////////////////////////////
            //motors[i].set(0);   //off motor to test encoders manually
        } 

        setMotorOut0123(motorOuts[0], motorOuts[1], motorOuts[2], motorOuts[3]);
    }

    
    /**
     * Code that runs once every robot loop
     */
    int initCnt=0;
    @Override
    public void periodic()
    {
        System.out.print("obj");
        if (initCnt<1) {
            initCnt++;
            gyro.zeroYaw();
            curHeading = targetHeading = getYawRad();
            return;
        }


        //Use feedback signal. Should be more accurate?

        /**
         * Updates for outputs to the shuffleboard
         */

        //D_curHeading.setDouble(curHeading);
        D_curHeading.setDouble(curHeading*180/Math.PI);
        D_tgtHeading.setDouble(targetHeading*180/Math.PI);
        D_navYaw.setDouble(-gyro.getYaw());

  
    }
}