package actr.tasks.tutorial;

import actr.task.*;

/**
 * Tutorial Unit 2: Demo Task
 * 
 * @author Dario Salvucci
 */
public class U2Demo extends actr.task.Task
{
	TaskLabel label;

	public U2Demo ()
	{
		super();
		label = new TaskLabel ("A", 100, 100, 40, 20);
		add (label);
	}
	
	public void start ()
	{
		label.setText("a");
		processDisplay();
	}
	
	public void typeKey (char c)
	{
		label.setText("-");
	}

	public Result analyze (Task[] tasks, boolean output)
	{
		boolean ok = (getModel().getProcedural().getLastProductionFired().getName().getString().contains("respond"));
		return new Result ("U2Demo", ok);
	}
}
