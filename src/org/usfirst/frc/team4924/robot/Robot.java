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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
   Preferences prefs;
   RobotDrive myRobot;
   Joystick stick;

   //CAMERA
   Joystick pstick;
   double calX = 0.5;
   double calY = 0.5;
   boolean cal_bool = true;
   Servo camX;
   double camdY;
   Servo camY;
   double camdX;
   //CAMERA


   //Toggle Controls


    //direction
    //1 is forward
    double direction = 1;
    boolean direction_bool = true;

    //arm
    //starts down
    boolean arm_bool = true;
    //or true is down
    boolean where_arm = true;

    //finger
    //starts in
    boolean finger_bool = true;
    //or true is in
    boolean where_finger = true;

    //comp
    //starts running
    boolean comp_bool = true;
    //or true is running
    boolean where_comp = true;

    //launcher
    //true is down (once again only a toggle value)
    boolean launcher_bool = true;
    //we only really need a toggle value to control a solenoid because of blank.set(!blanck.get())
    //the reason all the others have location values is because of the anti-bummper-quick-release
    //seems to me now that those are not even necessary
    //Toggle Control


    //Pneumatics
    Compressor comp;
    Solenoid sol1;
    Solenoid sol2;
    Solenoid sol3;
    Solenoid sol4;
    Solenoid sol5;
    Solenoid sol6;
    //Pneumatics


    //Launcher
    //Launcher status
    int intake_num = 0;
    //Launcher motors
    CANTalon motor1;
    CANTalon motor2;
    CANTalon motor3;
    //Launcher


    //Presets
    //Dashboard
    SmartDashboard dash;
    double motor1speed;
    double motor2speed;
    //hardcoded
    double pullback;
    //Presets


    //Sensors
    DigitalInput arm_hal;
    AnalogInput psi;
    AnalogInput distance;
    //Sensors


    //autonomous
    double autoNum;
    int autoLoopCounter;
    //autonomous

    /**
     * robotInit runs once at startup
     */

    public void robotInit() {
    	prefs = Preferences. getInstance();
    	myRobot = new RobotDrive(0,1);
    	stick = new Joystick(0);
    	pstick = new Joystick(1);
    	dash = new SmartDashboard();
    	comp = new Compressor();
    	comp.start();


    	//Solenoid

    	/**
    	 * sol1 is arm_down and starts True
    	 * sol2 is arm_up and starts False
    	 * sol3 is launcher_down and starts False
    	 * sol4 is launcher_up and starts True
    	 * sol5 is finger_out and starts False
    	 * sol6 is finger_in and starts True
    	 */

    	sol1 = new Solenoid(0);
        sol2 = new Solenoid(1);
    	sol3 = new Solenoid(2);
        sol4 = new Solenoid(3);
    	sol5 = new Solenoid(4);
        sol6 = new Solenoid(5);

        //arm (starts down)
        //where_arm initialises as true so
        //sol1 is arm_down (starts true)
    	sol1.set(where_arm);
    	//sol2 is arm_up (starts false)
    	sol2.set(!where_arm);

    	//launcher (starts up)
    	//sol3 is launcher_down
    	sol3.set(false);
    	//sol4 is launcher_up
    	sol4.set(true);

    	//Finger (starts in)
    	//sol5 is finger_out
    	sol5.set(false);
    	//sol6 is finger_in
    	sol6.set(true);
    	//Solenoid


        //Camera
        camY = new Servo(4);
        camX = new Servo(3);
        //Camera


        //Non-drive Motors
        //firing motors
        motor1 = new CANTalon(2);
        motor2 = new CANTalon(4);
        //intake motor
        motor3 = new  CANTalon(3);
        //Non-drive Motors


        //Sensors
        arm_hal = new DigitalInput(1);
        psi = new AnalogInput(3);
        distance = new AnalogInput(0);
        //Sensors
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

    	if(autoLoopCounter < 3) {
        	sol3.set(false);
        	sol4.set(true);
        	if(stick.getRawAxis(3)>0){
        		autoNum = -0.8;
        	} else {
        		autoNum = 0;
        	}
        	autoLoopCounter++;
    	} else if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(autoNum, 0.0); 	// drive forwards half speed
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
    	 * 12 Reverse Motor
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
    		sol1.set(!where_arm);
    		sol2.set(where_arm);
    		arm_bool = !arm_bool;
    	} else if(stick.getRawButton(2)&&!arm_bool)  {

    	} else if(!arm_bool) {
    		arm_bool = !arm_bool;
    	} else {

    	}



    	//SHOOTER SOLENOID CONTROL
    	if(stick.getRawButton(4)&&launcher_bool) {
    		sol3.set(!sol3.get());
    		sol4.set(!sol4.get());
    		launcher_bool = !launcher_bool;
    	} else if(stick.getRawButton(4)&&!launcher_bool)  {

    	} else if(!launcher_bool) {
    		launcher_bool = !launcher_bool;
    	} else {

    	}

    	//FINGER CONTROL
    	if(stick.getRawButton(6)&&finger_bool) {
    		if(!where_arm&&!where_finger) {

    		} else {
    			sol5.set(!where_finger);
    			sol6.set(where_finger);
    			finger_bool = !finger_bool;
    			where_finger = !where_finger;
    		}
    	} else if(stick.getRawButton(6)&&!finger_bool)  {

    	} else if(!finger_bool) {
    		finger_bool = !finger_bool;
    		sol5.set(false);
    		sol6.set(false);
    	} else {

    	}

    	//COMP CONTROL
    	if(stick.getRawButton(8)&&comp_bool) {
    		comp_bool = !comp_bool;
    		if(where_comp) {
    			comp.stop();
    			where_comp = false;
    		} else {
    			where_comp = true;
    			comp.start();
    		}
    	} else if(stick.getRawButton(8)&&!comp_bool)  {

    	} else if(!comp_bool) {
    		comp_bool = !comp_bool;
    	}

    	//BALL SHOOTER FIRE CONTROL
    	if(stick.getRawButton(12)) {
    		motor1.set(-0.6);
    		motor2.set(0.6);
    	}else{
    		motor1.set(0);
    		motor2.set(0);
    	}

    	if(stick.getRawButton(1)) {
    		motor1.set(0.6);
    		motor2.set(-0.6);
    	} else {
    		motor1.set(0);
    		motor2.set(0);
    	}

    	//BALL SHOOTER INTAKE CONTROL
    	if(intake_num == -1) {
    	} else if(stick.getRawButton(3)&&intake_num==0) {
    		intake_num = 1;
    		motor3.set(-0.5);
    	} else if(stick.getRawButton(3)&&intake_num==1)  {

    	} else if(intake_num < pullback&&intake_num != 0) {
    		motor3.set(0.5);
    		intake_num += 1;
    	} else if(intake_num >= pullback&&intake_num != 0) {
    		intake_num = 0;
    		motor3.set(0);
    	}

    	if(stick.getRawButton(9)&&intake_num==0) {
    		intake_num = -1;
    		motor3.set(1);
    	} else if(stick.getRawButton(11)&&intake_num==0) {
    		intake_num = -1;
    		motor3.set(-1);
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
        pullback = prefs.getDouble("pullback", 0);
        SmartDashboard.putNumber("CamX", camX.get());
        SmartDashboard.putNumber("CamY", camY.get());
        SmartDashboard.putNumber("CalX", calX);
        SmartDashboard.putNumber("CalY", calY);
        SmartDashboard.putNumber("motor1ball", motor1speed);
        SmartDashboard.putNumber("motor2ball", motor2speed);
        SmartDashboard.putNumber("pullback", pullback);
        SmartDashboard.putNumber("Distance mm", ((((5.000/4096.000)*distance.getValue())*1000.000)/4.880)*5.000);
        SmartDashboard.putNumber("Pressure", 50.000*((5.000/4096.000) * psi.getValue())-25.000 );
        SmartDashboard.putBoolean("Compessor", where_comp);
        SmartDashboard.putBoolean("Sensor", arm_hal.get());
        SmartDashboard.putNumber("Direction", direction);
        if(sol3.get()==true) {
        	myRobot.arcadeDrive(stick.getY()*direction*0.7, stick.getX()*-1);
        } else {
        	myRobot.arcadeDrive(stick.getY()*direction, stick.getX()*-1);
        }
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }

}
