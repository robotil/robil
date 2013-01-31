package elements;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.Map;

import javax.swing.Icon;
import javax.swing.JOptionPane;

import elements.Arrow.ArrayElement;

public class Decorator extends GElement implements ArrayElement,
		View.ChangesListener {

	private abstract class Border {
		public abstract void paint(Graphics2D g);
	}

	static public class Creator extends GElement.Creator {
		Arrow array = null;

		@Override
		public void add(GElement selectedElement) {
			if (selectedElement instanceof Arrow == false)
				return;
			this.array = (Arrow) selectedElement;
		}

		@Override
		public boolean createOnEmptyPlace() {
			return false;
		}

		@Override
		public Icon getIcon() {
			return null;
		}

		@Override
		public String getToolbarName() {
			return "Decorator";
		}

		@Override
		public GElement newInstance() {
			Decorator c = new Decorator();
			this.array.add(this.array.lastSelectedSegmentId, c);
			this.array = null;
			return c;
		}

		@Override
		public boolean ready() {
			return this.array != null;
		}

		@Override
		public String toolTip() {
			return "Create a Decorator. Select point on array where you want to place a decorator.";
		}
	}

	private class Dec extends Border {
		public int[] _x;
		public int[] _y;

		public Dec(int x, int y, int w, int h) {
			Vec t = new Vec(5, 3).scale(Decorator.this.view.zoom);
			this._x = new int[] { x, (int) (x + t.x), (int) (x + w - t.x),
					(int) (x + w), (int) (x + w - t.x), (int) (x + t.x), x };
			this._y = new int[] { (int) (y + h / 2.0), y, y,
					(int) (y + h / 2.0), y + h, y + h, (int) (y + h / 2.0) };
		}

		@Override
		public void paint(Graphics2D g) {
			g.setPaint(Color.white);
			g.fillPolygon(this._x, this._y, size());

			g.setPaint(Color.black);
			g.drawPolygon(this._x, this._y, size());
		}

		public int size() {
			return this._x.length;
		}
	}

	public String text = "Noname";
	public Font font = new Font("sansserif", Font.BOLD, 10);

	public Arrow array = null;

	public Decorator() {
		this.property.size.set(new Vec(10, 10));
	}

	public Decorator(double x, double y) {
		this.property.size.set(new Vec(10, 10));
		this.property.loc.set(new Vec(x, y).sub(this.property.size.scale(0.5)));
	}

	@Override
	public void add(Arrow a) {
		this.array = a;
	}

	@Override
	public GElement clone() {
		Decorator n = new Decorator();
		cloneInit(n);
		n.array = this.array;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {
		this.array = (Arrow) link.get(this.array);
	}

	public void drawString(Graphics2D g, String t, int x, int y) {
		g.drawString(this.text, x, y + getTextSize(g, this.text).height);
	}

	@Override
	public Arrow getArray() {
		return this.array;
	}

	@Override
	public void modify() {
		String ip = JOptionPane.showInputDialog("Set Decorator", this.text);
		if (ip != null && ip.trim().length() > 0)
			this.text = ip;
		onViewChange();
	}

	@Override
	public void onViewChange() {
		this.property.size = new Vec(getTextSize(this.view.graphics, this.font,
				this.text)).add(new Vec(10, 10));
	}

	@Override
	public void paint(Graphics2D g) {
		GraphProp gp = new GraphProp(g);

		Point loc = getLocation().getPoint();
		Dimension size = getSize().getDimension();

		if (this.property.leftClicked) {
			g.setStroke(new BasicStroke(3));
		}

		(new Dec(loc.x, loc.y, size.width, size.height)).paint(g);

		Font f = new Font(this.font.getFamily(), this.font.getStyle(),
				(int) (this.font.getSize() * this.view.zoom));
		g.setFont(f);
		Dimension tdim = new Vec(getTextSize(g, this.text)).scale(0.5)
				.getDimension();
		Point cnt = getCenter().getPoint();
		drawString(g, this.text, cnt.x - tdim.width, cnt.y - tdim.height);

		gp.restore();
	}

	@Override
	public GElement underMouse(Point p) {
		Point lt = getLocation().getPoint();
		Point rb = getLocation().add(getSize()).getPoint();
		if (lt.x <= p.x && p.x <= rb.x && lt.y <= p.y && p.y <= rb.y)
			return this;
		return null;
	}
}
