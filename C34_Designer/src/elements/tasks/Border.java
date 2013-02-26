package elements.tasks;

import java.awt.Color;
import java.awt.Graphics2D;

public abstract class Border {
	protected Task _task;
	
	public Border(Task task) {
		this._task = task;
	}
	
	public abstract void paint(Graphics2D g);

	public void setBackgroundColor(Graphics2D g) {
		if (_task.isRunning())
			g.setPaint(new Color(152, 251, 152, 200));
		else
			g.setPaint(Color.white);
	}

	public void setBackgroundColor(Graphics2D g, Color bgmain) {
		if (_task.isRunning())
			g.setPaint(new Color(152, 251, 152, 200));
		else
			g.setPaint(bgmain);
	}
}