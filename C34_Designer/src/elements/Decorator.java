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

import org.w3c.dom.css.ViewCSS;

import elements.Arrow.ArrayElement;
import elements.GElement.GraphProp;

public class Decorator extends GElement implements ArrayElement, View.ChangesListener{
	
	public String text = "Noname";
	public Font font = new Font("sansserif", Font.BOLD, 10);

	
	public Arrow array = null;
	public void add(Arrow a){ array = a; }
	public Arrow getArray() {
		return array;
	}

	public Decorator(){
		property.size.set(new Vec(10,10));
	}
	public Decorator(double x, double y){
		property.size.set(new Vec(10,10));
		property.loc.set(new Vec(x, y).sub(property.size.scale(0.5)));
	}
	
	static public class Creator extends GElement.Creator{
		Arrow array = null;
		public GElement newInstance(){
			Decorator c = new Decorator();
			array.add(array.lastSelectedSegmentId, c);
			array = null;
			return c;
		}
		public Icon getIcon(){
			return null;
		}
		public boolean ready(){
			return array!=null;
		}
		public boolean createOnEmptyPlace(){
			return false;
		}
		public void add(GElement selectedElement) {
			if(selectedElement instanceof Arrow == false) return;
			array = (Arrow) selectedElement;
		}
		public String getToolbarName() {
			return "Decorator";
		}
		@Override
		public String toolTip() {
			return "Create a Decorator. Select point on array where you want to place a decorator.";
		}
	}
	
	@Override
	public void modify() {
		String ip = JOptionPane.showInputDialog("Set Decorator", text);
		if(ip!=null && ip.trim().length()>0) text = ip;
		onViewChange();
	}
	
	public GElement underMouse(Point p){
		Point lt = getLocation().getPoint();
		Point rb = getLocation().add(getSize()).getPoint();
		if(lt.x<=p.x && p.x<=rb.x && lt.y<=p.y && p.y<=rb.y)
			return this;
		return null;
	}
	
	private abstract class Border{
		public abstract void paint(Graphics2D g);
	}
	private class Dec extends Border{
		public int[] _x;
		public int[] _y;
		public int size(){ return _x.length; }
		public Dec(int x, int y, int w, int h){
			Vec t = new Vec(5,3).scale(view.zoom);
			_x = new int[]{
					x, (int)(x+t.x), (int)(x+w-t.x), (int)(x+w), (int)(x+w-t.x), (int)(x+t.x), x	
					};
			_y = new int[]{
					(int)(y+h/2.0), y, y, (int)(y+h/2.0),  y+h, y+h, (int)(y+h/2.0)	
					};
		}
		public void paint(Graphics2D g){
			g.setPaint(Color.white);
			g.fillPolygon(_x, _y, size());
			
			g.setPaint(Color.black);
			g.drawPolygon(_x, _y, size());
		}
	}
	
	public void paint(Graphics2D g){
		GraphProp gp = new GraphProp(g);
		
		Point loc = getLocation().getPoint();
		Dimension size = getSize().getDimension();
		
		if(property.selected)
		{
			g.setStroke(new BasicStroke(3));
		}
		
		(new Dec(loc.x, loc.y, size.width, size.height)).paint(g);
		
		Font f = new Font(font.getFamily(), font.getStyle(), (int)(font.getSize()*view.zoom));
		g.setFont(f);
		Dimension tdim = new Vec(getTextSize(g, text)).scale(0.5).getDimension();
		Point cnt = getCenter().getPoint();
		drawString(g, text, cnt.x-tdim.width, cnt.y-tdim.height);
		
		gp.restore();
	}
	
	public void drawString(Graphics2D g, String t, int x, int y){
		g.drawString(text, x, y + getTextSize(g, text).height);
	}

	@Override
	public void onViewChange() {
		property.size = new Vec(getTextSize(view.graphics, font, text)).add(new Vec(10,10));
	}
	
	@Override
	public GElement clone() {
		Decorator n = new Decorator();
		cloneInit(n);
		n.array = array;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {
		array = (Arrow) link.get(array);
	}
}
