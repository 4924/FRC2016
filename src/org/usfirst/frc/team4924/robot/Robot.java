package org.usfirst.frc.team4924.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	Button button1; 
	double direction = 1;
	boolean direction_bool = true;
	int autoLoopCounter;
	SmartDashboard dash;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	public void onepress() {
		direction = direction * -1;
	}
	
    public void robotInit() {
    	myRobot = new RobotDrive(0,1);
    	stick = new Joystick(0);
    	button1 = new JoystickButton(stick, 5);
    	dash = new SmartDashboard();
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	if(stick.getRawButton(5)&&direction_bool) {
    		direction_bool = !direction_bool;
    		direction = direction * -1;
    	} else if(stick.getRawButton(5)&&!direction_bool)  {
    		
    	} else if(!direction_bool) {
    		direction_bool = !direction_bool;
    	} else {
    		
    	}
        myRobot.arcadeDrive(stick.getY()*direction, stick.getX()*-1);
        SmartDashboard.putNumber("Directions", direction);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
