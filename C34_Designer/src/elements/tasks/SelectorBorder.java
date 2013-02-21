package elements.tasks;

import java.awt.Color;
import java.awt.Graphics2D;

import elements.Vec;

class SelectorBorder extends Border {
	int x, y, w, h;

	public SelectorBorder(Task task, int _x, int _y, int _w, int _h) {
		super(task);
		this.x = _x;
		this.y = _y;
		this.w = _w;
		this.h = _h;
	}

	@Override
	public void paint(Graphics2D g) {
		Vec t = new Vec(20, 20).scale(_task.getView().zoom);
		setBackgroundColor(g);
		g.fillRoundRect(this.x, this.y, this.w, this.h, t.getIntX(),
				t.getIntY());
		g.setPaint(Color.black);
		g.drawRoundRect(this.x, this.y, this.w, this.h, t.getIntX(),
				t.getIntY());
	}
}