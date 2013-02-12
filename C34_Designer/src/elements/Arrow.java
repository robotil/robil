package elements;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Map;

import javax.swing.Icon;

public class Arrow extends GElement {

	public interface ArrayElement {
		public void add(Arrow array);

		public Arrow getArray();
	}

	static public class Creator extends GElement.Creator {
		ArrayList<GElement> els = new ArrayList<GElement>();

		@Override
		public void add(GElement selectedElement) {
			if (selectedElement instanceof Task == false)
				return;
			if (this.els.size() == 1 && this.els.get(0) == selectedElement)
				return;
			if (this.els.size() < 2)
				this.els.add(selectedElement);
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
			return "Arrow";
		}

		@Override
		public GElement newInstance() {
			if (ready() == false)
				return null;
			Arrow a = new Arrow(this.els.get(0), this.els.get(1));
			this.els.clear();
			return a;
		}

		@Override
		public boolean ready() {
			return this.els.size() == 2;
		}

		@Override
		public String toolTip() {
			return "Create an Arrow between to tasks. Fist, select source task and then select distination task.";
		}
	}

	static public class Reconector extends GElement.Creator {
		ArrayList<GElement> els = new ArrayList<GElement>();
		Arrow inst = null;

		public Reconector(Arrow inst) {
			this.inst = inst;
		}

		@Override
		public void add(GElement selectedElement) {
			if (selectedElement instanceof Task == false)
				return;
			if (this.els.size() == 1 && this.els.get(0) == selectedElement)
				return;
			if (this.els.size() < 2)
				this.els.add(selectedElement);
		}

		@Override
		public boolean createOnEmptyPlace() {
			return false;
		}

		@Override
		public Icon getIcon() {
			return null;
		}

		public GElement getInstance() {
			return this.inst;
		}

		@Override
		public String getToolbarName() {
			return "Arrow Reconector";
		}

		@Override
		public GElement newInstance() {
			if (ready() == false)
				return null;
			// Arrow a = new Arrow(els.get(0), els.get(1));
			this.inst.source = this.els.get(0);
			this.inst.targets.remove(this.inst.getTarget());
			this.inst.add(this.els.get(1));
			this.els.clear();
			return this.inst;
		}

		@Override
		public boolean ready() {
			return this.els.size() == 2;
		}

		@Override
		public String toolTip() {
			return "Reconect selected arrow. Fist, select source task and then select distination task.";
		}
	}

	static double line_x(Vec v1, Vec v2, double y) {
		return (y - v1.y) * (v2.x - v1.x) / (v2.y - v1.y) + v1.x;
	}

	static double line_y(Vec v1, Vec v2, double x) {
		return (v2.y - v1.y) / (v2.x - v1.x) * (x - v1.x) + v1.y;
	}

	public GElement source = null;

	public ArrayList<GElement> targets = new ArrayList<GElement>();
	public int lastSelectedSegmentId = -1;

	private Arrow() {
		this.source = null;
	}

	public Arrow(GElement str, GElement end) {
		this.source = str;
		if (end != null)
			this.targets.add(end);
	}

	public void add(GElement e) {
		this.targets.add(e);
		if (e instanceof ArrayElement) {
			((ArrayElement) e).add(this);
		}
	}

	public void add(int i, GElement e) {
		this.targets.add(i, e);
		if (e instanceof ArrayElement) {
			((ArrayElement) e).add(this);
		}
	}

	@Override
	public GElement clone() {
		Arrow n = new Arrow();
		cloneInit(n);
		n.source = this.source;
		n.targets.addAll(this.targets);
		n.lastSelectedSegmentId = this.lastSelectedSegmentId;
		return n;
	}
	
	public Arrow clone(GElement newSource, ArrayList<GElement> newTargets) {
		Arrow n = new Arrow();
		cloneInit(n);
		n.source = newSource;
		n.targets.addAll(newTargets);
		n.lastSelectedSegmentId = this.lastSelectedSegmentId;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {
		this.source = link.get(this.source);
		ArrayList<GElement> old = this.targets;
		this.targets = new ArrayList<GElement>();
		for (GElement e : old)
			this.targets.add(link.get(e));
	}

	@Override
	Vec getLocation() {
		return this.source.getCenter();
	}

	@Override
	Vec getSize() {
		if (this.targets.size() < 1)
			return new Vec(0, 0);
		return this.targets.get(this.targets.size() - 1).getCenter()
				.sub(this.source.getCenter());
	}

	public GElement getSource() {
		return this.source;
	}

	public GElement getTarget() {
		if (this.targets.size() == 0)
			return null;
		return this.targets.get(this.targets.size() - 1);
	}

	@Override
	public void modify() {
		// TODO Auto-generated method stub

	}

	@Override
	public void paint(Graphics2D g) {
		if (this.source == null || this.targets.size() < 1)
			return;
		GraphProp gp = new GraphProp(g);

		if (this.property.leftClicked) {
			g.setStroke(new BasicStroke(3));
		}

		GElement s = this.source;
		for (int i = 0; i < this.targets.size() - 1; i++) {
			paintSegment(g, s, this.targets.get(i));
			s = this.targets.get(i);
		}
		paintLastSegment(g, s, this.targets.get(this.targets.size() - 1));
		gp.restore();
	}

	public void paintLastSegment(Graphics2D g, GElement source, GElement target) {
		GraphProp gp = new GraphProp(g);

		g.setPaint(Color.black);

		Vec vloc = source.getCenter();
		Vec vctar = target.getCenter();
		// Vec border = target.getSize().scale(0.5);

		double ang = vctar.sub(vloc).ang();
		double dx = Math.abs(target.getSize().scale(0.5).x / Math.cos(ang));
		double dy = Math.abs(target.getSize().scale(0.5).y / Math.sin(ang));
		double d = dx <= target.getSize().scale(0.5).len() ? dx : dy;
		Vec border = new Vec(0, d);

		Vec vtar = vctar.sub(vloc).offsetLen(-border.len()).add(vloc);
		Vec vtar1 = vctar.sub(vloc)
				.offsetLen(-border.len() - 5 * this.view.zoom).add(vloc);

		Point loc = vloc.getPoint();
		Point tar = vtar.getPoint();
		Point tar1 = vtar1.getPoint();
		g.drawLine(loc.x, loc.y, tar1.x, tar1.y);

		Vec v = vloc.sub(vtar).changeLen(10 * this.view.zoom);
		Point s1 = v.rotate(Vec.d2r(+20)).add(vtar).getPoint();
		Point s2 = v.rotate(Vec.d2r(-20)).add(vtar).getPoint();
		g.fillPolygon(new int[] { tar.x, s1.x, s2.x }, new int[] { tar.y, s1.y,
				s2.y }, 3);
		gp.restore();
	}

	public void paintSegment(Graphics2D g, GElement source, GElement target) {
		GraphProp gp = new GraphProp(g);

		g.setPaint(Color.black);

		Vec vloc = source.getCenter();
		Vec vctar = target.getCenter();

		Point loc = vloc.getPoint();
		Point tar = vctar.getPoint();
		g.drawLine(loc.x, loc.y, tar.x, tar.y);

		gp.restore();
	}

	public boolean segmentUnderMouse(GElement s, GElement e, Vec m) {
		Vec c1 = s.getCenter();
		Vec c2 = e.getCenter();
		double mix = Math.min(c1.x, c2.x);
		// double miy = Math.min(c1.y, c2.y);
		double max = Math.max(c1.x, c2.x); 
		// double may = Math.max(c1.y, c2.y);
		
		if (!(mix <= m.x && m.x <= max /* && miy<=m.y && m.y<=may */))
			return false;
		return Vec.distance(c1, c2, m) < 4;
	}

	@Override
	public GElement underMouse(Point p) {
		Vec v = new Vec(p);
		GElement s = this.source;
		for (int i = 0; i < this.targets.size(); i++) {
			if (segmentUnderMouse(s, this.targets.get(i), v)) {
				this.lastSelectedSegmentId = i;
				return this;
			}
			s = this.targets.get(i);
		}

		return null;
	}
}
