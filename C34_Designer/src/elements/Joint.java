package elements;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.Map;

import javax.swing.Icon;

public class Joint extends GElement implements Arrow.ArrayElement{
	
	public Arrow array = null;
	public void add(Arrow a){ array = a; }

	public Joint(){
		property.size.set(new Vec(10,10));
	}
	public Joint(double x, double y){
		property.size.set(new Vec(10,10));
		property.loc.set(new Vec(x, y).sub(property.size.scale(0.5)));
	}
	
	static public class Creator extends GElement.Creator{
		Arrow array = null;
		public GElement newInstance(){
			Joint c = new Joint();
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
			return "Joint";
		}
		@Override
		public String toolTip() {
			return "Create a Joint point. Select point on array where you want to place a joint";
		}
	}
	
	public GElement underMouse(Point p){
		Vec c = getCenter();
		Vec n = c.sub(new Vec(p));
		if(n.len()<=getSize().scale(0.5).len())
			return this;
		return null;
	}
	
	public void paint(Graphics2D g){
		GraphProp gp = new GraphProp(g);
		g.setColor(Color.darkGray);
		if(property.selected)
		{
			Point loc = getLocation().getPoint();
			Dimension size = getSize().getDimension();
			g.setStroke(new BasicStroke(3));
		
			g.fillOval(loc.x, loc.y, size.width, size.height);
		} else {
			
			Point loc = getCenter().sub(new Vec(5,5).scale(0.5)).getPoint();
			Dimension size = new Vec(5,5).getDimension();

			g.fillOval(loc.x, loc.y, size.width, size.height);
		}
		
		gp.restore();
	}
	
	public Vec getSize(){
		return super.getSize().scale(1/view.zoom);
	}

	@Override
	public Arrow getArray() {
		return array;
	}

	@Override
	public void modify() {
		
	}

	@Override
	public GElement clone() {
		Joint n = new Joint();
		cloneInit(n);
		n.array = array;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {
		array = (Arrow) link.get(array);
	}
	
	
}
