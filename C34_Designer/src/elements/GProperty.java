package elements;

import org.w3c.dom.Element;

import document.Parameters;

public class GProperty {

	public Vec loc = new Vec(0, 0);
	public double heading = 0;
	public Vec size = new Vec(0, 0);
	public boolean leftClicked = false;
	public boolean rightClicked = false;
	public boolean collapsed = false;

	public int test_time = Parameters.test_time;
	public boolean test_result = Parameters.test_result;

	public boolean running = false;

	@Override
	public GProperty clone() {
		GProperty p = new GProperty();
		p.loc = new Vec(this.loc);
		p.heading = this.heading;
		p.size = new Vec(this.size);
		p.leftClicked = this.leftClicked;
		p.collapsed = this.collapsed;
		p.test_result = this.test_result;
		p.test_time = this.test_time;
		return p;
	}

	public void setCenter(Vec p) {
		this.loc = p.sub(this.size.scale(0.5));
	}

	public void setXml(Element xml) {

	}

	@Override
	public String toString() {

		return "x=\"" + this.loc.x + "\" y=\"" + this.loc.y + "\"";
	}

	public String toString(View v) {
		Vec l = this.loc.add(v.loc.scale(1 / v.zoom));
		return "x=\"" + l.x + "\" y=\"" + l.y + "\"";
	}

	public String toStringForSTask(View v) {
		Vec l = this.loc.add(v.loc.scale(1 / v.zoom));
		return "x=\"" + l.x + "\" y=\"" + l.y + "\"" + " collapsed=\""
				+ (this.collapsed ? "true" : "false") + "\"";
	}

	public String toStringForTask(View v) {
		Vec l = this.loc.add(v.loc.scale(1 / v.zoom));
		return "x=\"" + l.x + "\" y=\"" + l.y + "\"" + " test_time=\""
				+ this.test_time + "\" test_result=\""
				+ (this.test_result ? "true" : "false") + "\"";
	}

}
