package elements;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Paint;
import java.awt.Point;
import java.awt.Stroke;
import java.util.Map;
import java.util.UUID;

import javax.swing.Icon;

import org.w3c.dom.Element;

public abstract class GElement {

	static abstract public class Creator {
		public void add(GElement selectedElement) {

		}

		public boolean createOnEmptyPlace() {
			return true;
		}

		public Icon getIcon() {
			return null;
		}

		public String getToolbarName() {
			return "None";
		}

		public GElement newInstance() {
			return null;
		}

		public boolean ready() {
			return false;
		}

		public String toolTip() {
			return "";
		}
	}

	static protected class GraphProp {
		Graphics2D g;
		Paint paint;
		Stroke strok;
		Font font;

		public GraphProp(Graphics2D g) {
			this.g = g;
			this.paint = g.getPaint();
			this.strok = g.getStroke();
			this.font = g.getFont();
		}

		public void restore() {
			this.g.setPaint(this.paint);
			this.g.setStroke(this.strok);
			this.g.setFont(this.font);
		}
	}

	public static UUID getRandomUUID() {
		return UUID.randomUUID();
	}

	static Dimension getTextSize(Graphics graphics, Font font, String text) {
		if (graphics == null || font == null) {
			System.out.println("EXECPTION");
		}
		FontMetrics metrics = graphics.getFontMetrics(font);
		int hgt = metrics.getHeight();
		int adv = metrics.stringWidth(text);
		Dimension size = new Dimension(adv, hgt);
		return size;
	}

	static Dimension getTextSize(Graphics graphics, String text) {
		return getTextSize(graphics, graphics.getFont(), text);
	}

	public UUID id = getRandomUUID();

	protected View view = new View();
	protected Element xmlElement = null;
	protected GProperty property = new GProperty();

	public boolean isVisiable = true;

	@Override
	abstract public GElement clone();

	protected void cloneInit(GElement n) {
		n.property = this.property.clone();
		n.view = (n.view.clone());
		n.xmlElement = this.xmlElement;
	}

	abstract public void cloneReconnect(Map<GElement, GElement> link);

	Vec getCenter() {
		return getLocation().add(getSize().scale(0.5));
	}

	Vec getLocation() {
		return this.property.loc.scale(this.view.zoom).add(this.view.loc);
	}

	public GProperty getProperty() {
		return this.property;
	}

	public String getPropertyToString() {
		return this.property.toString();
	}

	Vec getSize() {
		return this.property.size.scale(this.view.zoom);
	}

	public View getView() {
		return this.view;
	}

	public Element getXmlElement() {
		return this.xmlElement;
	}

	abstract public void modify();

	public abstract void paint(Graphics2D g);

	public void paintElement(Graphics2D g) {
		if (this.isVisiable)
			paint(g);
	}

	public void setProperty(GProperty p) {
		this.property = p;
	}

	public void setPropertyXml(Element xml) {
		this.property.setXml(xml);
	}

	public void setView(View view) {
		this.view = view;
		if (this instanceof View.ChangesListener) {
			((View.ChangesListener) this).onViewChange();
		}
	}

	public void setXmlElement(Element xml) {
		this.xmlElement = xml;
	}

	public GElement underMouse(Point p) {
		return null;
	}
}
