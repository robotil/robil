package elements;

import java.awt.AlphaComposite;
import java.awt.Color;
import java.awt.Composite;
import java.awt.Graphics2D;
import java.util.Map;

public class Tooltip extends GElement {
	
	private String _title;
	private String _message;
	private GElement _parent;
	
	private Vec _textPosition = new Vec(0, 0);
	
	public Tooltip(GElement parent) {
		super();
		_parent = parent;
		_title = "";
		_message = "";
	}
	
	public void setMessage(String title, String message) {
		_title = title;
		_message = message;
	}
	
	public void updatePosition(Graphics2D g) {
		property.loc.x = _parent.getLocation().getX();
		property.loc.y = _parent.getLocation().getY() + 27 * _parent.getView().zoom;
		
		property.size.x = (getMaxLineWidth(g, _message) + 20) * _parent.getView().zoom;
		property.size.y = 50 * _parent.getView().zoom;
		
		_textPosition.x = property.loc.x + 10 * _parent.getView().zoom;
		_textPosition.y = property.loc.y + 15 * _parent.getView().zoom;
	}

	private int getMaxLineWidth(Graphics2D g, String string) {
		double maxWidth = 0;
		
		for (String line : string.split("\n")) {
			if (maxWidth < getTextSize(g, line).getWidth())
				maxWidth = getTextSize(g, line).getWidth();
		}
		
		return (int)maxWidth;
	}
	
	private void drawMultiLineString(Graphics2D g, String string, int x, int y) {
		for (String line : string.split("\n"))
			g.drawString(line, x, y += g.getFontMetrics().getHeight());
	}
	
	@Override
	public void paint(Graphics2D g) {
		updatePosition(g);
		
		Composite oldComposite = g.getComposite();
		
        AlphaComposite ac = java.awt.AlphaComposite.getInstance(AlphaComposite.SRC_OVER,0.7F);
        g.setComposite(ac);
	        
		g.setColor(Color.BLACK);
		g.fillRoundRect((int)property.loc.x, (int)property.loc.y, (int)property.size.x, (int)property.size.y, 10, 10);
		g.setColor(Color.WHITE);
		drawMultiLineString(g, _message, (int)_textPosition.x, (int)_textPosition.y);
		g.setComposite(oldComposite);
	}

	@Override
	public void modify() {
	}

	@Override
	public GElement clone() {
		return null;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {

	}

}
