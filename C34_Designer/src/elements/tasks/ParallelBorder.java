package elements.tasks;

import java.awt.Color;
import java.awt.Graphics2D;

import elements.Vec;

class ParallelBorder extends Border {
	public int[] _x;
	public int[] _y;

	public ParallelBorder(Task task, int x, int y, int w, int h) {
		super(task);
		Vec t = new Vec(5, 3).scale(_task.getView().zoom);
		this._x = new int[] { x + t.getIntX(), x + w + t.getIntX(),
				x + w - t.getIntX(), x - t.getIntX() };
		this._y = new int[] { y, y, y + h, y + h };
	}

	@Override
	public void paint(Graphics2D g) {
		setBackgroundColor(g);
		// g.drawString("P", _x[0]+ size()/2, _y[0]-5);
		// g.setPaint(new Color(127,255,212));
		g.fillPolygon(this._x, this._y, size());

		g.setPaint(Color.black);
		g.drawPolygon(this._x, this._y, size());
	}

	public int size() {
		return this._x.length;
	}
}