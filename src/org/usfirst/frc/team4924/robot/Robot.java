package org.usfirst.frc.team4924.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
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
	boolean arm_bool1 = true;
	boolean arm_bool2 = true;
	boolean where_arm = true;
	boolean cal_bool = true;
	boolean comp_bool = true;
	boolean comp_on_bool = true;
	boolean where_finger = true;
	boolean arm = true;
	int autoLoopCounter;
	SmartDashboard dash;
    Compressor comp;
	Solenoid sol1;
    Solenoid sol2;
	Solenoid sol3;
    Solenoid sol4;
	Solenoid sol5;
    Solenoid sol6;
    Servo camY;
    double camdX;
    int intake_num = 0;
    Servo camX;
    double camdY;
    CANTalon motor1;
    double motor1speed;
    double motor2speed;
    CANTalon motor2;
    CANTalon motor3;
    DigitalInput arm_hal;
    AnalogInput psi;
    AnalogInput distance;

	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	public void onepress() {
		direction = direction * -1;
	}
	
    public void robotInit() {
    	prefs = Preferences. getInstance();
    	myRobot = new RobotDrive(0,1);
    	stick = new Joystick(0);
    	pstick = new Joystick(1);
    	button1 = new JoystickButton(stick, 5);
    	dash = new SmartDashboard();
    	comp = new Compressor();
    	sol1 = new Solenoid(0);
        sol2 = new Solenoid(1);
    	sol3 = new Solenoid(2);
        sol4 = new Solenoid(3);
    	sol5 = new Solenoid(4);
        sol6 = new Solenoid(5);        
    	sol1.set(where_arm);
    	sol2.set(!where_arm);
    	sol3.set(true);
    	sol4.set(false);
    	sol5.set(false);
    	sol6.set(true);
    	comp.start();
        camY = new Servo(4);
        camX = new Servo(3);
        motor1 = new CANTalon(2);
        motor2 = new CANTalon(4);
        motor3 = new  CANTalon(3);
        arm_hal = new DigitalInput(1);
        psi = new AnalogInput(3);
        distance = new AnalogInput(0);
        motor1speed = prefs.getDouble("Motor 1 Speed", 0);
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
    	/*
    	 * 1 Ball Shooter
    	 * 2 Arm Toggle
    	 * 3 Ball Intake
    	 * 4 Ball Shooter Solenoid
    	 * 5 Change Directions
    	 * 6 Finger Solenoid
    	 * 8 Comp Toggle
    	 * 9 Ball Intake Direction
    	 * 11 Ball Intake Direction
    	 */
    	//CHANGE DIRECTION
    	if(stick.getRawButton(5)&&direction_bool) {
    		direction_bool = !direction_bool;
    		direction = direction * -1;
    	} else if(stick.getRawButton(5)&&!direction_bool)  {
    		
    	} else if(!direction_bool) {
    		direction_bool = !direction_bool;
    	} else {
    		
    	}
    	
    	//ARM CONTROL
    	if(stick.getRawButton(2)&&arm_bool) {
    		where_arm = !where_arm;
    		sol1.set(where_arm);
    		sol2.set(!where_arm);
    		arm_bool = !arm_bool;
    	} else if(stick.getRawButton(2)&&!arm_bool)  {
    		
    	} else if(!arm_bool) {
    		arm_bool = !arm_bool;
    	} else {
    		
    	}
    	
    	if(where_arm==false&&arm_hal.get()==false) {
    		sol1.set(false);
    		sol2.set(false);
    	}
    	
    	//SHOOTER SOLENOID CONTROL
    	if(stick.getRawButton(4)&&arm_bool1) {
    		sol3.set(!sol3.get());
    		sol4.set(!sol4.get());
    		arm_bool1 = !arm_bool1;
    	} else if(stick.getRawButton(4)&&!arm_bool1)  {
    		
    	} else if(!arm_bool1) {
    		arm_bool1 = !arm_bool1;
    	} else {
    		
    	}
    	
    	//FINGER CONTROL
    	if(stick.getRawButton(6)&&arm_bool2) {
    		sol5.set(!where_finger);
    		sol6.set(where_finger);
    		arm_bool2 = !arm_bool2;
    		where_finger = !where_finger;
    	} else if(stick.getRawButton(6)&&!arm_bool2)  {
    		
    	} else if(!arm_bool2) {
    		arm_bool2 = !arm_bool2;
    		sol5.set(false);
    		sol6.set(false);
    	} else {
    		
    	}
    	
    	//COMP CONTROL
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
    	}
    	
    	//BALL SHOOTER FIRE CONTROL
    	if(stick.getRawButton(1)) {
    		motor1.set(motor1speed);
    		motor2.set(motor2speed);
    	}else{
    		motor1.set(0);
    		motor2.set(0);
    	}
    	
    	//BALL SHOOTER INTAKE CONTROL
    	if(intake_num == -1) {
    	} else if(stick.getRawButton(3)&&intake_num==0) {
    		intake_num = 1;
    		motor3.set(-0.5);
    	} else if(stick.getRawButton(3)&&intake_num==1)  {
    		
    	} else if(intake_num < 20) {
    		motor3.set(0.5);
    		intake_num += 1;
    	} else if(intake_num > 20) {
    		intake_num = 0;
    		motor3.set(0);
    	}
    	
    	if(stick.getRawButton(9)&&intake_num==0) {
    		intake_num = -1;
    		motor3.set(0.5);
    	} else if(stick.getRawButton(11)&&intake_num==0) {
    		intake_num = -1;
    		motor3.set(-0.5);
    	} else if(intake_num==-1) {
    		intake_num = 0;
    		motor3.set(0);
    	}
    	
    	//CAMERA
    	if(pstick.getRawButton(1)&&cal_bool) {
    		cal_bool = !cal_bool;
    		calX = camdX;
    		calY = camdY;
    	} else if(pstick.getRawButton(1)&&!cal_bool)  {
    		
    	} else if(!cal_bool) {
    		cal_bool = !cal_bool;
    	} 
    	        
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
    	
    	if(pstick.getRawButton(2)) {
    		calX = 0.5;
    	    calY = 0.5;
    	}
        //CAMERA
    	
    	
    	
    	
    	motor1speed = prefs.getDouble("Motor 1 Speed", 0);
        motor2speed = prefs.getDouble("Motor 2 Speed", 0);
        SmartDashboard.putNumber("CamX", camX.get());
        SmartDashboard.putNumber("CamY", camY.get());
        SmartDashboard.putNumber("CalX", calX);
        SmartDashboard.putNumber("CalY", calY);
        SmartDashboard.putNumber("motor1ball", motor1speed);
        SmartDashboard.putNumber("motor2ball", motor2speed);
        SmartDashboard.putNumber("Distance", distance.getValue());
        SmartDashboard.putNumber("Distance V", (5.000/4096.000)*distance.getValue());
        SmartDashboard.putNumber("Distance mm", ((((5.000/4096.000)*distance.getValue())*1000.000)/4.880)*5.000);
        SmartDashboard.putNumber("Pressure", psi.getValue());
        SmartDashboard.putNumber("Pressure1", (5.000/4096.000) * psi.getValue());
        SmartDashboard.putNumber("Pressure2", 50.000*((5.000/4096.000) * psi.getValue())-25.000 );
        SmartDashboard.putBoolean("Compessor", comp_on_bool);
        SmartDashboard.putBoolean("Sensor", arm_hal.get());
        SmartDashboard.putNumber("Direction", direction);
        myRobot.arcadeDrive(stick.getY()*direction, stick.getX()*-1);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
