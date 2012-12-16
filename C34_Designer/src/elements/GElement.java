package elements;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Paint;
import java.awt.Point;
import java.awt.Stroke;
import java.util.UUID;

import javax.management.remote.TargetedNotification;
import javax.swing.Icon;

import org.w3c.dom.Element;

public abstract class GElement {
	
	public UUID id = UUID.randomUUID();
	
	static protected class GraphProp{
		Graphics2D g;
		Paint paint;
		Stroke strok;
		Font font;
		public GraphProp(Graphics2D g) {
			this.g = g;
			paint = g.getPaint();
			strok = g.getStroke();
			font = g.getFont();
		}
		public void restore(){
			g.setPaint(paint);
			g.setStroke(strok);
			g.setFont(font);
		}
	}
	
	protected View view = new View();
	public void setView(View view){
		this.view = view;
		if(this instanceof View.ChangesListener){
			((View.ChangesListener) this).onViewChange();
		}
	}
	public View getView(){
		return view;
	}
	
	protected Element xmlElement = null;
	public Element getXmlElement(){
		return xmlElement;
	}
	public void setXmlElement(Element xml){
		xmlElement = xml;
	}

	protected GProperty property = new GProperty();
	public String getPropertyToString(){
		return property.toString();
	}
	public void setPropertyXml(Element xml){
		property.setXml(xml);
	}
	public GProperty getProperty(){
		return property;
	}
	public void setProperty(GProperty p){
		property = p;
	}

	static abstract public class Creator{
		public GElement newInstance(){
			return null;
		}
		public Icon getIcon(){
			return null;
		}
		public boolean ready(){
			return false;
		}
		public boolean createOnEmptyPlace(){
			return true;
		}
		public void add(GElement selectedElement) {
			
		}
		public String getToolbarName(){
			return "None";
		}
		public String toolTip(){
			return "";
		}
	}
	
	public GElement underMouse(Point p){
		return null;
	}
	
	public abstract void paint(Graphics2D g);
	
	Vec getLocation(){
		return property.loc.scale(view.zoom).add(view.loc);
	}
	Vec getSize(){
		return property.size.scale(view.zoom);
	}
	Vec getCenter(){
		return getLocation().add(getSize().scale(0.5));
	}
	
	static Dimension getTextSize(Graphics graphics, Font font, String text){
		FontMetrics metrics = graphics.getFontMetrics(font);
		int hgt = metrics.getHeight();
		int adv = metrics.stringWidth(text);
		Dimension size = new Dimension(adv, hgt);
		return size;
	}
	static Dimension getTextSize(Graphics graphics, String text){
		return getTextSize(graphics, graphics.getFont(), text);
	}
	abstract public void modify() ;
}
