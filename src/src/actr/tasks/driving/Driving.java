package actr.tasks.driving;

import java.awt.BorderLayout;
import java.io.*;
import java.io.File; 

// import java.util.Arrays;
// import java.io.BufferedWriter;
// import java.io.FileWriter;
// import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Vector;
import javax.swing.JLabel;

import actr.model.Model;
import actr.task.Result;
import actr.task.Task;

/**
 * The main Driving task class that sets up the simulation and starts periodic updates.
 *  
 * @author Dario Salvucci
 */

/* TODO
java internal width of lanes smaller
different lane widths even within condition
smaller lanes
block switching + additional nback levels in array 4	2	1	3	2	0	3	4	0	1
accelbrake verhogen bij 100+ snelheden, trekt niet meer op

act-r
add (un)safe
continuous and single rehearsal
check 4backs


the get-visual-location can fire multiple times, even after get-visual wrong-kind never fires now (buffer always busy?)
wait for vision to be available for check or just force it?

order of just-check (add to add)
latency most recent productions, some might need to be updated in other models	


For every new n-back task, participants were instructed to stay at the 
speed of the first sign until they passed ‘n’ successive speed signs
before they could begin with the n-back task.


*/

public class Driving extends actr.task.Task
{
	static Simulator simulator = null;

	Simulation simulation;
	JLabel nearLabel, farLabel, speedsign, instructions, warning, speedometer;

	final double scale = .6; // .85
	final double steerFactor_dfa = (16 * scale);
	final double steerFactor_dna = (4 * scale);
	final double steerFactor_na  = (3 * scale);
	final double steerFactor_fa  = (0 * scale);
	final double accelFactor_thw  = (1 * .40);
	final double accelFactor_dthw = (3 * .40);
	final double steerNaMax = .07;
	final double thwFollow = 1.0;
	final double thwMax = 4.0;

	double startTime=0, endTime=3630;					// STOPS EVERYTHING
	double accelBrake=0, speed=0;

	static int minX=174, maxX=(238+24), minY=94, maxY=(262+32);
	static int centerX=(minX+maxX)/2, centerY=(minY+maxY)/2; 

	//make it bigger if the experiment is longer than 60s or signs appear more often
	static String currentLimit = "60"; //starting speed
	String previousLimit = "60";
	int nback_count = 0;
	double signOnset = 0; 
	double instructionsOnset = -5;		
	double warningOnset = 0;
	boolean signShowing = false; 
	boolean signSeen = false; 
	int signReadded = 0; 
	static boolean instructionsShowing = false;
	static boolean warningSeen = false;
	boolean instructionsSeen = false; 
	int instrReadded = 0; 
	static int speedI = 0;
	static Coordinate signPos;
	static String currentNBack = "";
	String[] nBack_list = {"4back", "3back", "0back", "1back", "4back", "0back", "3back", "4back", "1back", "2back"};
	String[] nBack_list_constr = {"2back", "3back", "0back", "1back", "4back", "0back", "3back", "4back", "1back", "2back"};
	double sign_count = 0;
	int rehearsal_count = 0;
	static String imaginedSpeedlimit = "60";
	List<String> output = new ArrayList<String>();	
	double mentalSpeed = 8; 
	double roughSpeed; 
	double prevTime = 0; 
	double curTime; 
	double prevDist = 0; 
	double curDist; 	
	boolean newCSV = true; 
	String[] blocks = {"normal", "construction", "normal", "construction", "normal", "construction", "normal", "construction"}; 
	String curBlock = "normal"; 
	int[] speedSigns = {0, 0, 0, 0, 0}; 	// adds most recent speed at [4]
	int speedSignsAdded = 0; 
	int correctNbackSpeed = 0;
	int numSpeedoChecks = 0; 
	double freqSpeedoChecks = 0; 
	double timeStartBlock = 0; 
	double envTime[] = {0};
	double blockTime[] = new double[envTime.length]; 
	String prevCorSpeed = ""; 

	
	public Driving ()
	{
		super ();
		nearLabel = new JLabel (".");
		farLabel = new JLabel ("X");
		speedsign = new JLabel ("*");
		instructions = new JLabel (",");
		warning = new JLabel (";");
		speedometer = new JLabel("~");
	}

	public void start ()
	{
		simulation = new Simulation (getModel());
		
		if (getModel().getRealTime())
		{
			curBlock = blocks[nback_count]; 			
			
			setLayout (new BorderLayout());
			if (simulator == null) simulator = new Simulator ();
			add (simulator, BorderLayout.CENTER);
			simulator.useSimulation (simulation);

		} else
		{
			curBlock = blocks[nback_count]; 
			
			add (nearLabel);
			nearLabel.setSize (20, 20);
			nearLabel.setLocation (250, 250);
			add (farLabel);
			farLabel.setSize (20, 20);
			farLabel.setLocation (250, 250);
			add (speedsign);
			speedsign.setSize (20, 20);
			speedsign.setLocation (250, 250);
			add (instructions);
			instructions.setSize(20,20);
			instructions.setLocation(200, 100);
			add (warning);
			warning.setSize(20, 20);
			warning.setLocation(200, 50);
			add(speedometer);
			speedometer.setSize(20,20);
			speedometer.setLocation(300, 350);
		}

		getModel().runCommand ("(set-visual-frequency near .1)");
		getModel().runCommand ("(set-visual-frequency far .1)");

		//they don't reset otherwise if you run the model multiple times in a row
		accelBrake = 0;
		speed = 0;
		previousLimit = "60";
		currentLimit = "60";
		currentNBack = ""; 
		signPos = null; 
		signOnset = 0;
		signShowing = false; 
		instructionsOnset = -5;
		instructionsShowing = false; 
		warningSeen = false;
		mentalSpeed = 8; 		
		speedSignsAdded = 0; 
		correctNbackSpeed = 0; 
		Arrays.fill(speedSigns, 0);  	// make all 0 again
		
		getModel().getVision().addVisual ("near", "near", "near", nearLabel.getX(), nearLabel.getY(), 1, 1, 10);
		getModel().getVision().addVisual ("far", "far", "far", farLabel.getX(), farLabel.getY(), 1, 1, 100);
		getModel().getVision().addVisual ("speedometer", "speedometer", "speedometer", 150, 315, 1, 1, 1);
		addPeriodicUpdate (Env.sampleTime);
		
	}

	public void update (double time)
	{
		if (time <= endTime)
		{
			simulation.env.time = time - startTime; 
			updateVisuals();
			if(simulation.env.time>10) //only start after 10s
			{
				updateSign(simulation.env.time);
			}
			updateInstructions(simulation.env.time);			// $$$$$$$$$$$$$$$$$$$$$$
			simulation.update();	
			updateSpeedometer();
		}
		else 
		{
			getModel().stop();
			output("myData", simulation.samples);
		}

	}

	//	save to file
	void output(String filename, Vector<Sample> samples)
	{
		for (int i = 1 ; i < samples.size(); i++)
		{
			Sample s = samples.elementAt(i);
			if(i == 1) {
				output.add(s.listVars() + System.lineSeparator());
				output.add(s.toString() + System.lineSeparator());
			}else
				output.add(s.toString() + System.lineSeparator());
			}
		Model.print(output, "_driving_");
	}

	void updateInstructions(double time)
	{

		//reverse order randomly for participants
		if (nback_count == 0 && Math.round(Math.random()) == 1)
		{
			//nBack_list_constr = new String[] {"2back", "3back", "0back",  "1back", "4back",  "0back", "3back",  "4back",  "1back",   "2back"}; 
			//keep the old order
		} else if(nback_count == 0)
		{
			//NBACKLIST ARRAY #2
			nBack_list = new String[] {"2back", "1back", "4back", "3back", "0back", "4back", "1back", "0back", "3back", "2back"};
			//nBack_list_constr = new String[] {"2back", "1back", "4back", "3back", "0back", "4back", "1back", "0back", "3back", "2back"};
		}

		//3 min speed signs, stop, instructions, show speed signs again
		// offset by 10 seconds always
		
		//add instructions
		// now at 3 seconds and every 60 sec		 
		if(!instructionsShowing && ((time > 5 && time < 7) || (time > 1 && ((time%90)+10) < 1)))
		{
			// reset some variables
			numSpeedoChecks = 0; 
			freqSpeedoChecks = 0;
			// make a deep copy of env time
			double envTime[] = {simulation.env.time};
			double blockTime[] = new double[envTime.length]; 
			blockTime[0] = envTime[0]; 
			timeStartBlock = blockTime[0];
				
			instructionsShowing = true;
			double timeInt = time; 
			currentNBack = nBack_list[nback_count];
			// Clear speeds array and correct speed
			prevCorSpeed = Integer.toString(correctNbackSpeed); 
			correctNbackSpeed = 0; 
			System.out.println("prev: " + prevCorSpeed + " curr: " + correctNbackSpeed); 
			for (int i = 0; i < speedSigns.length; i++)
				speedSigns[i] = 0;
			
						getModel().getVision().addVisual("instructions", "instructions", currentNBack.substring(0,1), 320, 45, 50, 50, 0);
			instructionsOnset = time;
			nback_count++;
			System.out.println("nbackcount: " + nback_count); 
		} else if (time - instructionsOnset < 3 && instructionsOnset >= 0 && instructionsShowing)
		{
			// artificially giving the model more time to notice instructions by re-adding them
			if (!instructionsSeen && instrReadded < 40)
			{	//System.out.println("instructions: " + currentNBack + " now"); 
				//System.out.println(instrReadded + " Readded instructions at " + time); 
				getModel().getVision().removeVisual("instructions");
				getModel().getVision().addVisual("instructions", "instructions", currentNBack.substring(0,1), 320, 45, 50, 50, 0);
				//instructionsSeen = true; 
				instrReadded++; 
			}
				
		} else if((time - instructionsOnset >= 3) && instructionsOnset >= 0 && instructionsShowing)	// 2
		{
			getModel().getVision().removeVisual("instructions");
			instructionsShowing = false;
			instructionsSeen = false;
			instrReadded = 0; 		
		}
	}

	void updateSign(double time)
	{
		String[] speedlimits = {"60", "70", "80", "90", "100", "110", "120", "130", "140"};		
		Env env = simulation.env;
		Simcar simcar = simulation.env.simcar;
		
		Random r = new Random((long)time); 
		double noise = r.nextGaussian() * 0.25;	
		
		// time%20 before!	added some temporal jitter			
		if((int)(time+noise)%20 == 0 && !signShowing && (speedI < speedlimits.length) )
		{	
			if (env.simcar.nearPoint != null)
			{	System.out.println("jitter: " + noise); 
				//pick a random speed with delta<30
				previousLimit = currentLimit;
				while (Math.abs(Integer.parseInt(currentLimit) - Integer.parseInt(previousLimit)) > 30 || (currentLimit == previousLimit))
				{
					int rnd = new Random().nextInt(speedlimits.length);
					currentLimit = speedlimits[rnd];
				} 
				// keeps an array with the recent speeds to get the right one for the current nback
				int newSpeed = Integer.parseInt(currentLimit); 				
				if (speedSignsAdded > 0) 
					shiftLeft(); 
				speedSignsAdded++;
				speedSigns[4] = newSpeed; 
				
				// determine what speed it should be driving at
				int nback = Character.getNumericValue(currentNBack.charAt(0)); 	
				if (speedSignsAdded > nback)
				{	prevCorSpeed = Integer.toString(correctNbackSpeed); 
					correctNbackSpeed = speedSigns[4 - nback];		
				}
				System.out.println(String.format("%.2f",time) + Arrays.toString(speedSigns) + " nback: " + nback + " corspeed: " + correctNbackSpeed); 		
				
				// display speed sign
				Position newLoc = Road.location(env.simcar.fracIndex + 20 , 4);		// env.simcar.fracIndex + 20 , 4
				newLoc.y = 0.0;
				signPos = env.world2image(newLoc);
				signShowing = true;
				//getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, (int)signPos.x, (int)signPos.y, 1, 1, 100);
				getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, 605, 235, 1, 1, 100);
				//speedsign.setLocation(605, 235); //speedsign.setLocation((int)signPos.x, (int)signPos.y);
				signOnset = time;
			} 
				
		// move the speed sign
		} else if (signShowing && (time - signOnset < 2) && signOnset > 0) 
		{ 
			// artificially giving the model more time to notice sign by re-adding it
			if (!signSeen && signReadded < 40)
			{
				//System.out.println(signReadded + " Readded speed sign at " + time); 
				getModel().getVision().removeVisual("speedsign");
				getModel().getVision().addVisual ("speedsign", "speedsign", currentLimit, 605, 235, 1, 1, 100);
				signReadded++;
			}
//			signPos.x += 5;
//			signPos.y += 1;

		} else if(signShowing && (time - signOnset >= 2) && signOnset > 0)
		{
			getModel().getVision().removeVisual("speedsign");
			signPos = null; 
			signShowing = false;
			signSeen = false; 
			signReadded = 0; 
		}   
	}

	void updateSpeedometer()
	{
		Simcar simcar = simulation.env.simcar;
		String speed = Integer.toString((int)Utilities.mps2mph(Utilities.mph2kph(simcar.speed)));
		getModel().getVision().changeValue("speedometer", speed);
	}

	void updateVisuals ()
	{
		Env env = simulation.env;
		if (env.simcar.nearPoint != null)
		{
			Coordinate cn = env.world2image (env.simcar.nearPoint);
			Coordinate cc = env.world2image (env.simcar.farPoint);
			//Coordinate cc = env.world2image (env.simcar.carPoint);

			if (cn == null || cc == null) env.done = true;
			else
			{
				nearLabel.setLocation (cn.x, cn.y);
				farLabel.setLocation (cc.x, cc.y);
				getModel().getVision().moveVisual ("near", cn.x, cn.y);
				getModel().getVision().moveVisual ("far", cc.x, cc.y);
				//				speed = env.simcar.speed;

			}
		}
	}

	double minSigned (double x, double y)
	{
		return (x>=0) ? Math.min (x,y) : Math.max (x,-y);
	}

	void doSteer (double na, double dna, double dfa, double dt)
	{
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0)
		{
			double dsteer = (dna * steerFactor_dna)
					+ (dfa * steerFactor_dfa)
					+ (minSigned (na, steerNaMax) * steerFactor_na * dt);
			dsteer *= simulation.driver.steeringFactor;
			simcar.steerAngle += dsteer;
		}
		else simcar.steerAngle = 0;
	}

	void doAccelerate (double fthw, double dthw, double dt, double tlimit)
	{
		Simcar simcar = simulation.env.simcar;
		if (simcar.speed >= 10.0)
		{
			double dacc = (dthw * accelFactor_dthw)
					+ (dt * (fthw - thwFollow) * accelFactor_thw);
			accelBrake += dacc;
			accelBrake = minSigned (accelBrake, 1.0);
		}
		else
		{
			accelBrake = .65 * (simulation.env.time / 3.0);
			accelBrake = minSigned (accelBrake, .65);
		}
		simcar.accelerator = (accelBrake >= 0) ? accelBrake : 0;
		simcar.brake = (accelBrake < 0) ? -accelBrake : 0;
	}
	
	void keepLimit(double slimit)	// slimit is nbackspeed from imaginal
	{	
		makeCSVwithData(simulation.env.simcar.speed, roughSpeed, mentalSpeed);
		imaginedSpeedlimit = Integer.toString((int)slimit); 
				
		Simcar simcar = simulation.env.simcar;	
		double realSpeed = simcar.speed; 
		slimit = Utilities.mph2mps(Utilities.kph2mph(slimit));

		// calculate the difference between what the model thinks it should be driving and what it should be driving
		double deltaSpeed; 
		if (correctNbackSpeed != 0)
			deltaSpeed = Utilities.mph2mps(Utilities.kph2mph(correctNbackSpeed)) - realSpeed; 	
		 else
		 {	// So after new instructions it keeps following the speed it was driving at
			 deltaSpeed = Utilities.mph2mps(Utilities.kph2mph(Double.parseDouble(currentLimit))) - realSpeed;
		 }
		double time = simulation.env.time - startTime;

		//warning if the speed limit is not kept (method says 6 seconds (3 before and after))
		if( Math.abs(deltaSpeed)>1.39 && !warningSeen && signOnset != 0 && (time - signOnset) > 3) //1.39 m/s is roughly 5 km/h
		{	
			warningOnset = time;
			getModel().getVision().addVisual("warning", "warning", "warning", 200, 100, 50, 50);
			warning.setLocation(200, 50);
			warningSeen = true;
		} else if (warningSeen && Math.abs(deltaSpeed)<=1.39) // && (time - warningOnset) > 3) 
		{
			getModel().getVision().removeVisual("warning");
			//warningOnset = time;
			warningSeen = false;
		}
		

		//System.out.println("\t\tcorspeed: " + String.format("%.2f",slimit) + " (" + String.format("%.2f",(slimit*3.6)) + " kmh)" + "\tmentalspeed: " + String.format("%.2f",mentalSpeed) + "\tsimcarspeed: " + String.format("%.2f",simcar.speed)); 
		
		// acceleration is based on the MENTAL speed, not (necessarily) what the car is actually driving at
		double diff = slimit - mentalSpeed;
		if (Math.abs(diff) > 0)
		{		
			if(diff > 2) {	
				double dacc = (diff * 1.2) + (0.25 * diff * 0.4); 
				accelBrake += dacc;
				accelBrake = minSigned (accelBrake, 1.0); // 
			} else
			{
				double dacc = (diff* 1.2) + (0.25 * diff * 0.4);
				accelBrake += dacc;
				accelBrake = minSigned (accelBrake, 0.65);	
			}	
		}
		
		simcar.accelerator = (accelBrake >= 0) ? 1.5*accelBrake : 0;
		if (diff < 2.77) // 10 km/h
			accelBrake /= 200; 
		simcar.brake = (accelBrake < 0) ? -(accelBrake) : 0;		// braking is way too strong
		//System.out.println("accelbrake: " + 	accelBrake); 
	}

	void makeCSVwithData(double real, double rough, double mental)
	{
		try {
	        FileWriter fw = new FileWriter("speeds.csv", true);
	        BufferedWriter bw = new BufferedWriter(fw);
	        PrintWriter pw = new PrintWriter(bw);	
	        if (newCSV)
	        {
	        	pw.println("\nTime, Real, Rough, Mental, CorrectSpeed, Real-Rough, Real-Mental, Correct-Real, FreqSpeedo, Nback, Road");
	        	newCSV = false; 
	        }
	        double corrSpeed; 
	        if (correctNbackSpeed != 0)
	        	corrSpeed = Utilities.mph2mps(Utilities.kph2mph(correctNbackSpeed)); 
	        else
	        {	// So after new instructions it keeps following the speed it was driving at
	        	if (nback_count < 2) 
	        	{
	        		corrSpeed = Utilities.mph2mps(Utilities.kph2mph(Integer.parseInt(currentLimit))); 
	        		System.out.println("count < 2, corrspeed: " + corrSpeed);
	        	} else {
	        		corrSpeed = Utilities.mph2mps(Utilities.kph2mph(Integer.parseInt(prevCorSpeed))); 
	        		System.out.println("count <> 2, corrspeed: " + corrSpeed);
	        	}
	        }
	        String timing  = String.valueOf(simulation.env.time);
	        pw.println(timing + "," + 3.6*real + "," + 3.6*rough + "," + 3.6*mental + "," + 3.6*corrSpeed + "," + (real-rough) + "," + (real-mental) + "," + (corrSpeed-real) + "," + freqSpeedoChecks + "," + currentNBack + "," + curBlock);
	        pw.flush();
	        pw.close();
	        
	    } catch (FileNotFoundException e) {
	        System.out.println(e.getMessage());
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}
	
	boolean isCarStable (double na, double nva, double fva)
	{
		double f = 2.5;		// is this lane width? 
		return (Math.abs(na) < .025*f) && (Math.abs(nva) < .0125*f) && (Math.abs(fva) < .0125*f);
	}
	
	// na = near point, fva = far?? nva = ??		velocity??

	double image2angle (double x, double d)		// x = near/far angle, d = distance
	{
		Env env = simulation.env;
		double px = env.simcar.p.x + (env.simcar.h.x * d);
		double pz = env.simcar.p.z + (env.simcar.h.z * d);
		Coordinate im = env.world2image (new Position (px, pz));
		try { return Math.atan2 (.5*(x-im.x), 450); }		
		//  counterclockwise angle to the POSITIVE (right side) X axis. Y value first
		//  math.atan2(y,x) * 180 / Math.PI = normal degrees ie 2.2 = 45
//				|	/<_
//				|  /   \^
//				| /	    \ 
//				|/	     |^
//		-------------------x	math.atan2(y, x)
//				|
//				|
//				|
//				y
		
		
		catch (Exception e) { return 0; }
	}

	public void eval (Iterator<String> it)
	{
		it.next();
		String cmd = it.next();
		if (cmd.equals ("do-steer"))
		{
			double na = Double.valueOf (it.next());
			double dna = Double.valueOf (it.next()); // crash
			double dfa = Double.valueOf (it.next());
			double dt = Double.valueOf (it.next());
			doSteer (na, dna, dfa, dt);
		}
		else if (cmd.equals ("do-accelerate"))
		{
			double fthw = Double.valueOf (it.next());
			double dthw = Double.valueOf (it.next());
			double dt = Double.valueOf (it.next());
			double tlimit = Double.valueOf(it.next());
			doAccelerate (fthw, dthw, dt, tlimit);
		}
		else if (cmd.equals ("keep-limit"))
		{
			// get the (imaginal) speed of the car
			double tlimit = Double.valueOf (it.next());
			keepLimit (tlimit);
		}
		else if (cmd.equals ("placeholder"))
		{
			// estimate the current speed based on how much distance was crossed since the last check
		} else if (cmd.equals("set-rough-speed"))
		{
			// fix the near and far points swerving so much
			
			// only check rough speed every once in a while but do keep adding noise every time
			Simcar simcar = simulation.env.simcar;
			
			Random r = new Random(); 
			double noise = r.nextGaussian() * 0.1;		// 1 mps is 3.6 km/h		0.3 = 1.2 km/h
			curTime = simulation.env.time; 						
			curDist = simcar.getDistance();
			
			// difference mental - (rough + noise)			
			double diffDist = curDist - prevDist; 
			double dTime = curTime - prevTime;
			if (diffDist != 0 && dTime != 0)
				roughSpeed = diffDist / dTime + noise; 
			else 
				roughSpeed = noise; 
			
			mentalSpeed = (mentalSpeed + roughSpeed)/2; 
			
			prevTime = curTime; 
			prevDist = curDist; 
						
		} else if (cmd.equals("set-speedo-speed"))
		{					
			// calculate frequency of speedo checks
			if (currentNBack != "")
				numSpeedoChecks++; 
			double deltaTime = simulation.env.time - timeStartBlock;
			freqSpeedoChecks = numSpeedoChecks / deltaTime; 
			
			// updates mentalspeed to speedo value
			double speedoSpeed = Double.valueOf (it.next());
			mentalSpeed = Utilities.mph2mps(Utilities.kph2mph(speedoSpeed)); 
			
		} else if (cmd.equals("saw-speed-sign"))
		{	System.out.println("attended sign"); 
			signSeen = true; 
			
		} else if (cmd.equals("saw-instructions"))
		{	System.out.println("attended instructions"); 
			instructionsSeen = true; 
		}
	}

	public boolean evalCondition (Iterator<String> it)
	{
		it.next();
		String cmd = it.next();
		if (cmd.equals ("is-car-stable") || cmd.equals ("is-car-not-stable"))
		{
			double na = Double.valueOf (it.next());	
			double nva = Double.valueOf (it.next());
			double fva = Double.valueOf (it.next());
			boolean b = isCarStable(na,nva,fva);
			return cmd.equals("is-car-stable") ? b : !b;
		} else if (cmd.equals("didnt-see-sign"))
			return !signSeen ? true : false;
		else if (cmd.equals("didnt-see-instructions"))
			return !instructionsSeen? true : false; 
		else return false;
	}

	public double bind (Iterator<String> it)
	{
		try
		{
			it.next(); 
			String cmd = it.next();
			if (cmd.equals ("image->angle"))
			{
				double x = Double.valueOf (it.next());		// near/far point
				double d = Double.valueOf (it.next());		// distance
				return image2angle (x, d);
			}
			else if (cmd.equals ("mp-time")) return simulation.env.time;
			else if (cmd.equals ("get-thw"))
			{
				double fd = Double.valueOf (it.next());
				double v = Double.valueOf (it.next());
				double thw = (v==0) ? 4.0 : fd/v;
				System.out.println("fthw: " + thw); 
				return Math.min (thw, 4.0);
			}
			else if (cmd.equals ("get-velocity")) return speed;
			else if (cmd.equals ("get-chunk-id"))
			{
				//every chunk needs a unique id. Time is unique so I just used that
				double cid = (int)simulation.env.time;
				return cid;
			}
			else if (cmd.equals ("get-num-sign"))
			{
				//number of signs passed
				sign_count += 1;
				return sign_count;
			}
			else if (cmd.equals ("reset-num-sign"))
			{
				sign_count = 0;
				return sign_count;
			}
			else if (cmd.equals ("get-num-rehearsal"))
			{
				//number of rehearsals per iteration
				rehearsal_count += 1;
				return rehearsal_count;
			}
			else if (cmd.equals ("reset-rehearsal"))
			{
				//number of rehearsals in a loop
				rehearsal_count = 0;
				return rehearsal_count;
			} 
			// ADDED for timing 
			else if (cmd.equals("eval-speed"))
			{
	
				// 0 = early, 1 = safe, 2 = late
				double speedometerSpeed = Double.valueOf (it.next());
				double imaginedSpeed = Double.valueOf (it.next());
				double diff = Math.abs(speedometerSpeed - imaginedSpeed); 
				System.out.println("\tspeedo: " + speedometerSpeed + "(" + String.format("%.2f", Utilities.mph2mps(Utilities.kph2mph(speedometerSpeed))) + ") "+ " imagined: " + imaginedSpeed + "(" + String.format("%.2f", Utilities.mph2mps(Utilities.kph2mph(imaginedSpeed))) + ")"); 
				
				if (diff <= 2.5)	
					return 0;	// early
				else if (diff > 2.5 && diff <= 5) 
					return 1; 	// safe
				else if (diff > 5)
					return 2;	// late 
				else 
					return 0; 
				
			// ADDED for range of similar experiences
			} else if (cmd.equals("get-mintick"))
			{
				double minT = Double.valueOf (it.next());
				double deduction = Double.valueOf (it.next());
				minT -= deduction; 
				return (minT < 0) ? 0: minT; 
			} 
			else return 0;
			
		}
		catch (Exception e)
		{
			System.out.println("exception!"); 
			e.printStackTrace();
			System.exit(1);
			return 0;
		}
	}

	void incNum(Integer rehearsal_count, boolean startover)
	{
		if (startover)
			rehearsal_count = 0;
		else
		{
			rehearsal_count += 1;
		}
	}


	public int numberOfSimulations () { return 1; }

	public Result analyze (Task[] tasks, boolean output)
	{
		return null;
	}
	
	void shiftLeft()
	{
		for (int i = 0; i<4; i++)
		{
			speedSigns[i] = speedSigns[i+1]; 
		}
		speedSigns[4] = 0; 	
		
	}

	double[] copyByValue(double time)
	{
		double envTime[] = {time};

		 double blockTime[] = new double[envTime.length]; 
		 blockTime[0] = envTime[0]; 
		
		 return blockTime;
	}
}
