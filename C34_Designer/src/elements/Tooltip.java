package elements;

import java.awt.AlphaComposite;
import java.awt.Color;
import java.awt.Composite;
import java.awt.Graphics2D;
import java.util.Map;

import elements.tasks.Task;

public class Tooltip extends GElement {

	public enum ToolTipDesign {
		Default,
		DebugInfo,
		RuntimeInfo,
		PlanExecutionInfo
	}
	
	public enum Position {
		Bottom,
		Right,
		Top
	}
	
	private String _title;
	private String _message;
	private Position _position; 
	private final GElement _parent;
	private final ToolTipDesign _design;

	private Vec _textPosition = new Vec(0, 0);
	
	public Tooltip(GElement parent) {
		this(parent, ToolTipDesign.Default);
	}
	
	public Tooltip(GElement parent, ToolTipDesign design) {
		this(parent, design, Position.Bottom);
	}
	
	public Tooltip(GElement parent, ToolTipDesign design, Position position) {
		this._parent = parent;
		this._title = "";
		this._message = "";
		this._design = design;
		this._position = position;
	}
	
	public GElement clone(Task parent) {
		Tooltip tooltip = new Tooltip(parent, this._design);
		this.cloneInit(tooltip);
		tooltip._textPosition = new Vec(this._textPosition);
		tooltip._message = new String(this._message);
		tooltip._title = new String(this._title);
		tooltip._position = this._position;
		return tooltip;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) { }

	private void drawMultiLineString(Graphics2D g, String string, int x, int y) {
		for (String line : string.split("\n")) {
			Color originalColor = g.getColor();
			
			if (line.contains("$RED$")) 
				g.setColor(Color.RED);
			
			if (line.contains("$GREEN$"))
				g.setColor(Color.GREEN);
			
			if (line.contains("$BLUE$"))
				g.setColor(Color.BLUE);
			
			// Replace colors declarations:
			// 		$RED$, $BLUE$, $GREEN$
			g.drawString(line.replaceAll("\\$.*?\\$", ""), x, y += g.getFontMetrics().getHeight());
			
			g.setColor(originalColor);
		}
	}

	/**
	 * Removes $COLOR$ from string
	 * @param str
	 * @return
	 */
	private String getEscapedString(String str) {
		return str.replaceAll("\\$.*?\\$", "");
	}
	
	private int getLinesCountHeight(String string) {
		return string.split("\n").length;
	}

	private int getMaxLineWidth(Graphics2D g, String string) {
		double maxWidth = 0;

		for (String line : string.split("\n")) {
			if (maxWidth < getTextSize(g, getEscapedString(line)).getWidth())
				maxWidth = getTextSize(g, getEscapedString(line)).getWidth();
		}

		return (int) maxWidth;
	}
	
	private double getTextHeight(Graphics2D g) {
		return getTextSize(g, this._message).getHeight() * this._message.split("\n").length;
	}

	@Override
	public void modify() {
	}

	@Override
	public void paint(Graphics2D g) {
		
		if (this._message.trim().equals(""))
			return;
		
		updatePosition(g);

		Composite oldComposite = g.getComposite();

		AlphaComposite ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.8F);
		g.setComposite(ac);
		
		switch (_design) {
		case Default:
			g.setColor(Color.BLACK);
			g.fillRoundRect((int) this.property.location.x, (int) this.property.location.y, (int) this.property.size.x, (int) this.property.size.y, 10, 10);
			g.setColor(Color.WHITE);
			drawMultiLineString(g, this._message, (int) this._textPosition.x, (int) this._textPosition.y);
			break;
		case DebugInfo:
			g.setColor(Color.WHITE);
			g.fillRoundRect((int) this.property.location.x, (int) this.property.location.y, (int) this.property.size.x, (int) this.property.size.y, 10, 10);
			g.setColor(Color.BLACK);
			drawMultiLineString(g, this._message, (int) this._textPosition.x, (int) this._textPosition.y);
			break;
		case RuntimeInfo:
			g.setColor(Color.YELLOW);
			g.fillRoundRect((int) this.property.location.x, (int) this.property.location.y, (int) this.property.size.x, (int) this.property.size.y, 10, 10);
			g.setColor(Color.BLACK);
			drawMultiLineString(g, this._message, (int) this._textPosition.x, (int) this._textPosition.y);
			break;
		case PlanExecutionInfo:
			g.setColor(new Color(217,250,221));
			g.fillRoundRect((int) this.property.location.x, (int) this.property.location.y, (int) this.property.size.x, (int) this.property.size.y, 10, 10);
			g.setColor(new Color(0,50,0));
			drawMultiLineString(g, this._message, (int) this._textPosition.x, (int) this._textPosition.y);
			break;
		default:
			break;
		}
		
		g.setComposite(oldComposite);
	}

	public void setMessage(String title, String message) {
		this._title = "";
		this._message = (title.trim().length() > 0 ? title + ":\n" : "") + message;
	}
	
	public void setMessage(String message) {
		this._title = "";
		this._message = message;
	}
	
	public void updatePosition(Graphics2D g) {
		// Box location & dimensions
		
		switch (this._position) {
		case Bottom:
			this.property.location.x = this._parent.getLocation().getX();
			this.property.location.y = this._parent.getLocation().getY() + 27 * this._parent.getView().zoom;
			break;
		case Right:
			this.property.location.x = this._parent.getLocation().getX() + this._parent.getProperty().size.x + 10 * this._parent.getView().zoom;
			this.property.location.y = this._parent.getLocation().getY();
			break;
		case Top:
			this.property.location.x = this._parent.getLocation().getX();
			this.property.location.y = this._parent.getLocation().getY() - getTextHeight(g) - 15 * this._parent.getView().zoom;
			break;
		default:
			this.property.location.x = 0;
			this.property.location.y = 0;
		}
		
		this.property.size.x = getMaxLineWidth(g, this._message) + 20
				* this._parent.getView().zoom;
		this.property.size.y = getTextSize(g, this._message).getHeight()
				* getLinesCountHeight(this._message) + 10
				* this._parent.getView().zoom;

		// Text location
		this._textPosition.x = this.property.location.x + 10 * this._parent.getView().zoom;
		this._textPosition.y = this.property.location.y + 2  * this._parent.getView().zoom;
	}

	@Override
	public GElement clone() {
		return null;
	}

}
