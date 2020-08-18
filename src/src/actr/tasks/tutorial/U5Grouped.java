package actr.tasks.tutorial;

import java.util.Iterator;
import actr.task.*;

/**
 * Tutorial Unit 5: Grouped Task
 * 
 * @author Dario Salvucci
 */
public class U5Grouped extends Task
{
	String responses = "";
	
	public void start ()
	{
		responses = "";
	}

	public void eval (Iterator<String> it)
	{
		it.next(); // (
		it.next(); // record-response
		String response = it.next().replaceAll("\"", "");
		it.next(); // )
		responses += response;
	}
	
	public Result analyze (Task[] tasks, boolean output)
	{
		if (output)
			getModel().output ("\n-----\n\n'" + responses + "'");

		boolean ok = (getModel().getTime()>1.0 && responses.length()>0);
		return new Result ("U5Grouped", ok);
	}
}
