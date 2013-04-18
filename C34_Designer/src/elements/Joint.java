package elements;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.Map;

import javax.swing.Icon;

public class Joint extends GElement implements Arrow.ArrayElement {

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
			return "Joint";
		}

		@Override
		public GElement newInstance() {
			Joint c = new Joint();
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
			return "Create a Joint point. Select point on array where you want to place a joint";
		}
	}

	public Arrow array = null;

	public Joint() {
		this.property.size.set(new Vec(10, 10));
	}

	public Joint(double x, double y) {
		this.property.size.set(new Vec(10, 10));
		this.property.location.set(new Vec(x, y).sub(this.property.size.scale(0.5)));
	}

	@Override
	public void add(Arrow a) {
		this.array = a;
	}

	@Override
	public GElement clone() {
		Joint n = new Joint();
		cloneInit(n);
		n.array = this.array;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {
		this.array = (Arrow) link.get(this.array);
	}

	@Override
	public Arrow getArray() {
		return this.array;
	}

	@Override
	public Vec getSize() {
		return super.getSize().scale(1 / this.view.zoom);
	}

	@Override
	public void modify() {

	}

	@Override
	public void paint(Graphics2D g) {
		GraphProp gp = new GraphProp(g);
		g.setColor(Color.darkGray);
		if (this.property.leftClicked) {
			Point loc = getLocation().getPoint();
			Dimension size = getSize().getDimension();
			g.setStroke(new BasicStroke(3));

			g.fillOval(loc.x, loc.y, size.width, size.height);
		} else {

			Point loc = getCenter().sub(new Vec(5, 5).scale(0.5)).getPoint();
			Dimension size = new Vec(5, 5).getDimension();

			g.fillOval(loc.x, loc.y, size.width, size.height);
		}

		gp.restore();
	}

	@Override
	public GElement underMouse(Point p) {
		Vec c = getCenter();
		Vec n = c.sub(new Vec(p));
		if (n.len() <= getSize().scale(0.5).len())
			return this;
		return null;
	}

}
