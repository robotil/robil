package elements.tasks;

import java.awt.Color;
import java.awt.Graphics2D;

import elements.Vec;

class SequenceBorder extends Border {
	public int[] _x;
	public int[] _y;

	public SequenceBorder(Task task, int x, int y, int w, int h) {
		super(task);
		Vec t = new Vec(12, 3).scale(_task.getView().zoom);
		this._x = new int[] { x, (int) (x + w - t.x), (int) (x + w - t.x),
				x + w, (int) (x + w - t.x), (int) (x + w - t.x), x, x };
		this._y = new int[] { (int) (y + t.y), (int) (y + t.y), y,
				(int) (y + h / 2.0), y + h, (int) (y + h - t.y),
				(int) (y + h - t.y), (int) (y + t.y) };
	}

	@Override
	public void paint(Graphics2D g) {
		setBackgroundColor(g);

		g.fillPolygon(this._x, this._y, size());

		g.setPaint(Color.black);
		g.drawPolygon(this._x, this._y, size());
	}

	public int size() {
		return this._x.length;
	}
}