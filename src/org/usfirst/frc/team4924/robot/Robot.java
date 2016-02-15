package org.usfirst.frc.team4924.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TalonSRX;
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
	Preferences prefs;
	RobotDrive myRobot;
	Joystick stick;
	Joystick pstick;
	Button button1; 
	double direction = 1;
	double calX = 0.5;
	double calY = 0.5;
	boolean direction_bool = true;
	boolean arm_bool = true;
	boolean cal_bool = true;
	boolean comp_bool = true;
	boolean comp_on_bool = true;
	boolean arm = true;
	int autoLoopCounter;
	SmartDashboard dash;
    Compressor comp;
	Solenoid sol1;
    Solenoid sol2;
    Servo camY;
    double camdX;
    Servo camX;
    double camdY;
    CANTalon motor1;
    double motor1speed;
    double motor2speed;
    CANTalon motor2;
    CANTalon motor3;

	
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
    	pstick = new Joystick(1);
    	button1 = new JoystickButton(stick, 5);
    	dash = new SmartDashboard();
    	comp = new Compressor();
    	sol1 = new Solenoid(0);
        sol2 = new Solenoid(1);
    	sol1.set(true);
    	sol2.set(false);
        comp.start();
        camY = new Servo(4);
        camX = new Servo(3);
        motor1 = new CANTalon(2);
        motor2 = new CANTalon(3);
        motor3 = new  CANTalon(4);
        motor1speed = prefs.getDouble("motor 1 Speed", 0);
        motor2speed = prefs.getDouble("Motor 2 Speed", 0);
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
    	
    	if(stick.getRawButton(1)&&arm_bool) {
    		sol1.set(!sol1.get());
    		sol2.set(!sol2.get());
    		arm_bool = !arm_bool;
    	} else if(stick.getRawButton(1)&&!arm_bool)  {
    		
    	} else if(!arm_bool) {
    		arm_bool = !arm_bool;
    	} else {
    		
    	}
    	
    	if(pstick.getRawButton(1)&&cal_bool) {
    		cal_bool = !cal_bool;
    		calX = camdX;
    		calY = camdY;
    	} else if(pstick.getRawButton(1)&&!cal_bool)  {
    		
    	} else if(!cal_bool) {
    		cal_bool = !cal_bool;
    	} else {
    		
    	}
        SmartDashboard.putNumber("Direction", direction);
        myRobot.arcadeDrive(stick.getY()*direction, stick.getX()*-1);
        
        if(pstick.getX()>=0) {
        	camdX = pstick.getX()*(1-calX)+calX;
            camX.set(pstick.getX()*(1-calX)+calX);
        } else {
        	camdX = pstick.getX()*calX+calX;
            camX.set(pstick.getX()*calX+calX);        	
        }

        if(pstick.getY()>=0) {
        	camdY = pstick.getY()*(1-calY)+calY;
            camY.set(pstick.getY()*(1-calY)+calY);
        } else {
        	camdY = pstick.getY()*calY+calY;
            camY.set(pstick.getY()*calY+calY);        	
        }
        
    	if(stick.getRawButton(8)&&comp_bool) {
    		comp_bool = !comp_bool;
    		if(comp_on_bool) {
    			comp.stop();
    			comp_on_bool = false;
    		} else {
    			comp_on_bool = true;    			
    			comp.start();
    		}
    	} else if(stick.getRawButton(8)&&!comp_bool)  {
    		
    	} else if(!comp_bool) {
    		comp_bool = !comp_bool;
    	} else {
    		
    	}
    	
    	if(stick.getRawButton(2)) {
    		motor1.set(motor1speed);
    		motor2.set(motor2speed);
    	}
    	
    	if(stick.getRawButton(3)) {
    		motor3.set(0.5);
    	}
    	
    	if(pstick.getRawButton(2)) {
    		calX = 0.5;
    	    calY = 0.5;
    	}
        

        SmartDashboard.putNumber("CamX", camX.get());
        SmartDashboard.putNumber("CamY", camY.get());
        SmartDashboard.putNumber("CalX", calX);
        SmartDashboard.putNumber("CalY", calY);
        SmartDashboard.putBoolean("Compessor", comp_on_bool);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
