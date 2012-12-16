package elements;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.font.FontRenderContext;
import java.awt.font.TextLayout;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.GroupLayout;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SpringLayout;

public class Task extends GElement implements View.ChangesListener{

	public final static String TYPE_task = "task";
	public final static String TYPE_selector = "selector";
	public final static String TYPE_sequenser = "sequenser";
	public final static String TYPE_parallel = "parallel";
	
	public String text = "Noname";
	public String type = TYPE_task;
	public Font font = new Font("sansserif", Font.BOLD, 10);
	public int seqNumber=0;
	
	public Task(){
		property.size = new Vec(100,100);
	}
	
	public String toString(){
		return ""+type+"{"+text+"}";
	}
	
	final int shortTextLen = 25;
	public String getText(){
		if(property.selected) return text;
		if(text.length()>shortTextLen) return text.substring(0,shortTextLen-3)+"...";
		return text;
	}
	public String getShortText(){
		if(text.length()>shortTextLen) return text.substring(0,shortTextLen-3)+"...";
		return text;
	}
	
	static public class Creator extends GElement.Creator{
		public GElement newInstance(){
			return new Task();
		}
		public Icon getIcon(){
			return null;
		}
		public boolean ready(){
			return true;
		}
		public boolean createOnEmptyPlace(){
			return true;
		}
		public String getToolbarName() {
			return "Task";
		}
		@Override
		public String toolTip() {
			return "Create a Node (Seq, Sel, Par, Task). Select a point on the document to place a task";
		}
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
		public void setBackgroundColor(Graphics2D g){
			if(getProperty().running)
				g.setPaint(new Color(152, 251, 152, 200));
			else 
				g.setPaint(Color.white);
		}
		public void setBackgroundColor(Graphics2D g, Color bgmain){
			if(getProperty().running)
				g.setPaint(new Color(152, 251, 152, 200));
			else 
				g.setPaint(bgmain);
		}		
	}
	
	private class Seq extends Border{
		public int[] _x;
		public int[] _y;
		public int size(){ return _x.length; }
		public Seq(int x, int y, int w, int h){
			Vec t = new Vec(12,3).scale(view.zoom);
			_x = new int[]{
					x, (int)(x+w-t.x), (int)(x+w-t.x), x+w, (int)(x+w-t.x), (int)(x+w-t.x), x, x	
					};
			_y = new int[]{
					(int)(y+t.y), (int)(y+t.y), y, (int)(y+h/2.0),  y+h, (int)(y+h-t.y), (int)(y+h-t.y), (int)(y+t.y)	
					};
		}
		public void paint(Graphics2D g){
			setBackgroundColor(g);
			
			g.fillPolygon(_x, _y, size());
			
			g.setPaint(Color.black);
			g.drawPolygon(_x, _y, size());
		}
	}
	private class Par extends Border{
		public int[] _x;
		public int[] _y;
		public int size(){ return _x.length; }
		public Par(int x, int y, int w, int h){
			Vec t = new Vec(5,3).scale(view.zoom);
			_x = new int[]{
					x+t.getIntX(), x+w+t.getIntX(), x+w-t.getIntX(), x-t.getIntX()
					};
			_y = new int[]{
					y, y, y+h, y+h	
					};
		}
		public void paint(Graphics2D g){
			setBackgroundColor(g);
			g.drawString("P", _x[0]+ size()/2, _y[0]-5);
			//g.setPaint(new Color(127,255,212));
			g.fillPolygon(_x, _y, size());
			
			g.setPaint(Color.black);
			g.drawPolygon(_x, _y, size());
			}
	}
	private class Sel extends Border{
		int x,  y,  w,  h;
		public Sel(int _x, int _y, int _w, int _h){
			x=_x; y=_y; w=_w; h=_h;
		}
		public void paint(Graphics2D g){
			Vec t = new Vec(20,20).scale(view.zoom);
			setBackgroundColor(g);
			g.fillRoundRect(x, y, w, h, t.getIntX(), t.getIntY());
			g.setStroke (new BasicStroke(
				      2f, 
				      BasicStroke.CAP_ROUND, 
				      BasicStroke.JOIN_ROUND, 
				      2f, 
				      new float[] {6f}, 
				      0f));
			g.setPaint(Color.black);
			g.drawRoundRect(x, y, w, h, t.getIntX(), t.getIntY());
		}
	}
	private class Tsk extends Border{
		int x,  y,  w,  h;
		public Tsk(int _x, int _y, int _w, int _h){
			x=_x; y=_y; w=_w; h=_h;
		}
		public void paint(Graphics2D g){
			setBackgroundColor(g, new Color(230,230,250, 200));
			
			g.fillRect(x, y, w, h);
			
			g.setPaint(Color.black);
			g.drawRect(x, y, w, h);

			if(getProperty().dbg_result==false){
				GraphProp gp = new GraphProp(g);
				g.setPaint(Color.red);
				g.drawRect(x-1, y-1, w+2, h+2);
				gp.restore();
			}
		}
	}
	
	
	public void paint(Graphics2D g){
		onViewChange();
		GraphProp gp = new GraphProp(g);
		
		Point loc = getLocation().getPoint();
		Dimension size = getSizeInternal().getDimension();
		
		if(property.selected)
		{
			g.setStroke(new BasicStroke(3));
		}
		
		Border border = null;
		if(type==TYPE_selector) border = new Sel(loc.x, loc.y, size.width, size.height);
		if(type==TYPE_sequenser) border = new Seq(loc.x, loc.y, size.width, size.height);
		if(type==TYPE_task) border = new Tsk(loc.x, loc.y, size.width, size.height);
		if(type==TYPE_parallel) border = new Par(loc.x, loc.y, size.width, size.height);
		border.paint(g);

		int fontsize=font.getSize();
		Font f = new Font(font.getFamily(), font.getStyle(), (int)(fontsize*view.zoom));
		g.setFont(f);
		Dimension tdim = new Vec(getTextSize(g, getText())).scale(0.5).getDimension();
		Point cnt = getCenterInternal().getPoint();
		drawString(g, getText(), cnt.x-tdim.width, cnt.y-tdim.height);
		
		if(seqNumber>0){
			f = new Font(font.getFamily(), font.getStyle(), (int)(fontsize*0.8*view.zoom));
			g.setFont(f);
			g.setPaint(Color.blue);
			tdim = new Vec(getTextSize(g, ""+seqNumber)).scale(0.5).getDimension();
			cnt = getLocation().getPoint();
			drawString(g, ""+seqNumber, cnt.x-tdim.width, cnt.y-tdim.height);
			//g.fillOval(cnt.x-tdim.width, cnt.y-tdim.height, tdim.width, tdim.height);
		}	
		
		Vec typesize = new Vec(20,20).scale(view.zoom);
		Vec typeloc = getLocation().sub(typesize.scale(0.5));
		
		
		gp.restore();
	}
	
	public void drawString(Graphics2D g, String t, int x, int y){
		g.drawString(t, x, y + (int)(getTextSize(g, getText()).height*0.8));
	}

	Vec getSize(){
		onViewChange();
		return super.getSize();
	}
	Vec getSizeInternal(){
		return new Vec(getTextSize(view.graphics, font, getText())).add(new Vec(10,10)).scale(view.zoom);
	}
	Vec getCenterInternal(){
		return getLocation().add(getSizeInternal().scale(0.5));
	}	
	@Override
	public void onViewChange() {
		if(view.graphics==null) return;
		property.size = new Vec(getTextSize(view.graphics, font, getShortText())).add(new Vec(10,10));
	}


	@Override
	public void modify() {
		ModifyDialog dlg = new ModifyDialog();
		dlg.setVisible(true);
		onViewChange();
	}
	
	
	class ModifyDialog extends JDialog {

	    public ModifyDialog() {

	        initUI();
	    }
	    JTextField txtName = null;
	    JComboBox cType = null;
	    JTextField txtDbgTime = null;
	    JComboBox txtDbgResult = null;
	    
	    public final void initUI() {

	    	setLayout(null);
	    	
	    	JLabel lbl1 = new JLabel("Name ");
	    	JLabel lbl2 = new JLabel("Type ");
	    	JLabel lbl3 = new JLabel("Dbg-Time   ");
	    	JLabel lbl4 = new JLabel("Dbg-Result ");
	    	
	    	txtName = new JTextField(text); txtName.selectAll();
	    	cType = new JComboBox(new String[]{TYPE_sequenser,TYPE_selector,TYPE_task, TYPE_parallel});
	    	cType.setSelectedItem(type);
	    	txtDbgTime = new JTextField(""+getProperty().dbg_time); 
	    	txtDbgResult = new JComboBox(new String[]{"true","false"});
	    	txtDbgResult.setSelectedItem(""+getProperty().dbg_result);
	    	
        
	        JButton close = new JButton("Close");
	        close.addActionListener(new ActionListener() {
	        	
	        	public void actionPerformed(ActionEvent event) {
	        		dispose();
	        	}
	        });
	        	        
	        JButton OK = new JButton("OK");
	        OK.addActionListener(new ActionListener() {
	        	
	        	public void actionPerformed(ActionEvent event) {
	        		text = txtName.getText();
	        		type = (String) cType.getSelectedItem();
	        		try{
	        			getProperty().dbg_time = Integer.parseInt(txtDbgTime.getText());
	        			getProperty().dbg_result = Boolean.parseBoolean((String) txtDbgResult.getSelectedItem());
	        		}catch(Exception e){
	        			e.printStackTrace();
	        		}
	        		dispose();
	        	}
	        });

	        
	        add(lbl1);
	        add(txtName);
	        add(lbl2);
	        add(cType);
	        add(lbl3);
	        add(txtDbgTime);
	        add(lbl4);
	        add(txtDbgResult);
	        
	        add(close);
	        add(OK);
	        
	        lbl1.setBounds(10,10, 100, 30);
	        txtName.setBounds(120,10, 170, 30);
	        lbl2.setBounds(10,50, 100, 30);
	        cType.setBounds(120,50, 170, 30);
	        lbl3.setBounds(10,90, 100, 30);
	        txtDbgTime.setBounds(120,90, 170, 30);
	        lbl4.setBounds(10,130, 100, 30);
	        txtDbgResult.setBounds(120,130, 170, 30);
	        
	        close.setBounds(120,170, 80, 30);
	        OK.setBounds(210,170, 80, 30);

	        setModalityType(ModalityType.APPLICATION_MODAL);

	        setTitle("Change Task");
	        setDefaultCloseOperation(DISPOSE_ON_CLOSE);
	        setLocationRelativeTo(null);
	        setSize(300, 230);
	    }
	}
	
}
