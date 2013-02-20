package elements.tasks;

import java.awt.Color;
import java.awt.Graphics2D;

import elements.Vec;

class SwitchBorder extends Border {
	int x, y, w, h;

	public SwitchBorder(Task task, int _x, int _y, int _w, int _h) {
		super(task);
		this.x = _x;
		this.y = _y;
		this.w = _w;
		this.h = _h;
	}

	@Override
	public void paint(Graphics2D g) {
		Vec t = new Vec(5, 5).scale(_task.getView().zoom);
		if (t.y > 5)
			t.y = 5;
		setBackgroundColor(g);
		g.fillRect(this.x, this.y, this.w, this.h);
		// g.setStroke (new BasicStroke(
		// 2f,
		// BasicStroke.CAP_ROUND,
		// BasicStroke.JOIN_ROUND,
		// 2f,
		// new float[] {6f},
		// 0f));
		g.setPaint(Color.black);
		g.drawRect(this.x, this.y, this.w, this.h);

		int sw, k, d = 0;
		if (this.w >= 5 * 5) {
			int kk, nn;
			for (kk = 3, nn = 3; this.w / kk >= 5; nn = kk, kk += 2)
				;
			k = Math.abs(this.w % kk) > Math.abs(this.w % nn) ? nn : kk;
			d = this.w % k;
		} else {
			k = 5;
		}
		sw = this.w / k;
		int di = 1;// (d/(k/2));

		int[] _x = new int[k * 2];
		int[] _y = new int[k * 2];
		_x[0] = this.x;
		for (int i = 1; i < k; i += 1) {
			_x[i * 2] = _x[i * 2 - 2] + sw;
			if (d > 0) {
				_x[i * 2] += di;
				d -= di;
			}
		}
		for (int i = 0; i < k - 1; i += 1) {
			_x[1 + i * 2] = _x[1 + i * 2 + 1];
		}
		for (int i = 0; i < k; i += 1) {
			_y[i * 2] = i % 2 == 0 ? this.y + this.h : this.y + this.h
					+ (int) (t.y);
		}
		for (int i = 0; i < k - 1; i += 1) {
			_y[1 + i * 2] = _y[1 + i * 2 - 1];
		}
		_x[k * 2 - 1] = this.x + this.w;
		_y[k * 2 - 1] = this.y + this.h;

		setBackgroundColor(g);
		g.fillPolygon(_x, _y, _x.length);
		g.setPaint(Color.black);
		g.drawPolygon(_x, _y, _x.length);
	}
}