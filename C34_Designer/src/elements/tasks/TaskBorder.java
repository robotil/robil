package elements.tasks;

import java.awt.Color;
import java.awt.Graphics2D;

import elements.GElement.GraphProp;

class TaskBorder extends Border {
	int x, y, w, h;
	boolean _failure = false;
	
	public TaskBorder(Task task, int _x, int _y, int _w, int _h, boolean failure) {
		super(task);
		this.x = _x;
		this.y = _y;
		this.w = _w;
		this.h = _h;
		this._failure = failure;
	}

	@Override
	public void paint(Graphics2D g) {
		setBackgroundColor(g, new Color(230, 230, 250, 200));

		g.fillRect(this.x, this.y, this.w, this.h);

		g.setPaint(Color.black);
		g.drawRect(this.x, this.y, this.w, this.h);

		if (this._failure) {
			GraphProp gp = new GraphProp(g);
			g.setPaint(Color.red);
			g.drawRect(this.x - 1, this.y - 1, this.w + 2, this.h + 2);
			gp.restore();
		}
	}
}