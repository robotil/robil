package elements;

import org.w3c.dom.Element;

import document.Parameters;

public class GProperty {

	public Vec location = new Vec(0, 0);
	public double heading = 0;
	public Vec size = new Vec(0, 0);
	public boolean leftClicked = false;
	public boolean rightClicked = false;
	public boolean collapsed = false;
	public boolean isRoot = false;
	public int testTime = Parameters.test_time;
	public boolean testResult = Parameters.test_result;

	public boolean running = false;

	@Override
	public GProperty clone() {
		GProperty p = new GProperty();
		p.location = new Vec(this.location);
		p.heading = this.heading;
		p.size = new Vec(this.size);
		p.leftClicked = this.leftClicked;
		p.collapsed = this.collapsed;
		p.testResult = this.testResult;
		p.testTime = this.testTime;
		p.isRoot = this.isRoot;
		return p;
	}

	public void setCenter(Vec p) {
		this.location = p.sub(this.size.scale(0.5));
	}

	public void setXml(Element xml) {

	}

	@Override
	public String toString() {

		return "x=\"" + this.location.x + "\" y=\"" + this.location.y + "\"";
	}

	public String toString(View v) {
		Vec l = this.location.add(v.loc.scale(1 / v.zoom));
		return "x=\"" + l.x + "\" y=\"" + l.y + "\"";
	}

	public String toStringForSTask(View v) {
		Vec l = this.location.add(v.loc.scale(1 / v.zoom));
		return "x=\"" + l.x + "\" y=\"" + l.y + "\"" + " collapsed=\""
				+ (this.collapsed ? "true" : "false") + "\"";
	}

	public String toStringForTask(View v) {
		Vec l = this.location.add(v.loc.scale(1 / v.zoom));
		return "x=\"" + l.x + "\" y=\"" + l.y + "\"" + " test_time=\""
				+ this.testTime + "\" test_result=\""
				+ (this.testResult ? "true" : "false") + "\"";
	}

}
