package elements;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.font.FontRenderContext;
import java.awt.font.TextLayout;
import java.util.ArrayList;
import java.util.Map;

import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.GroupLayout;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.SpringLayout;
import javax.swing.UIManager;

import org.w3c.dom.Element;

import document.description.TaskDescription;

public class Task extends GElement implements View.ChangesListener{

	public final static String TYPE_task = "task";
	public final static String TYPE_selector = "selector";
	public final static String TYPE_sequenser = "sequenser";
	public final static String TYPE_parallel = "parallel";
	public final static String TYPE_switch = "switch";
	
	public String text = "Noname";
	public String type = TYPE_task;
	public Font font = new Font("sansserif", Font.BOLD, 10);
	public int seqNumber=0;
	
	private TaskDescription taskDescriptionProvider;
	
	private final Tooltip _tooltip;
	
	public Task(){
		property.size = new Vec(100,100);
		_tooltip = new Tooltip(this);
	}
	
	public String toString(){
		return ""+type+"{"+text+"}";
	}
	
	public void setTaskDescriptionProvider(TaskDescription provider) {
		taskDescriptionProvider = provider;
		
		// TaskDescription.Task testTask = new TaskDescription.Task();
		// testTask.algorithm = text;
		// taskDescriptionProvider.put(getNameWithoutParameters(), testTask);
	}

	public String getNameWithoutParameters() {
		return getNameWithoutParameters(text);
	}

	public String getNameWithoutParameters(String name) {
		name = name.replaceAll("\\(.*\\)", "");
		return name.replaceAll("\\(.*", "");
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
			//g.drawString("P", _x[0]+ size()/2, _y[0]-5);
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
//			g.setStroke (new BasicStroke(
//				      2f, 
//				      BasicStroke.CAP_ROUND, 
//				      BasicStroke.JOIN_ROUND, 
//				      2f, 
//				      new float[] {6f}, 
//				      0f));
			g.setPaint(Color.black);
			g.drawRoundRect(x, y, w, h, t.getIntX(), t.getIntY());
		}
	}
	
	private class Swi extends Border{
		int x,  y,  w,  h;
		public Swi(int _x, int _y, int _w, int _h){
			x=_x; y=_y; w=_w; h=_h;
		}
		public void paint(Graphics2D g){
			Vec t = new Vec(5,5).scale(view.zoom);
			if(t.y>5)t.y=5;
			setBackgroundColor(g);
			g.fillRect(x, y, w, h);
//			g.setStroke (new BasicStroke(
//				      2f, 
//				      BasicStroke.CAP_ROUND, 
//				      BasicStroke.JOIN_ROUND, 
//				      2f, 
//				      new float[] {6f}, 
//				      0f));
			g.setPaint(Color.black);
			g.drawRect(x, y, w, h);
			
			int sw, k, d=0;
			if( w >= 5*5){
				int kk, nn; for( kk=3, nn=3; w/kk >=5; nn=kk,kk+=2 );
				k = Math.abs(w%kk)>Math.abs(w%nn) ? nn : kk;
				d = w%k;
			}else{
				k=5;
			}
			sw = w/k;
			int di=1;//(d/(k/2));
						
			int[] _x = new int[k*2];	int[] _y = new int[k*2];
			_x[0]=x;
			for(int i=1;i<k;i+=1){	_x[i*2]=_x[i*2-2]+sw;	if(d>0){ _x[i*2]+= di; d-=di; }	}
			for(int i=0;i<k-1;i+=1){	_x[1+i*2]=_x[1+i*2+1];	}
			for(int i=0;i<k;i+=1){	_y[i*2]=i%2==0?y+h:y+h+(int)(t.y);	}
			for(int i=0;i<k-1;i+=1){	_y[1+i*2]=_y[1+i*2-1];	}
			_x[k*2-1]=x+w;	_y[k*2-1]=y+h;
			
			setBackgroundColor(g);
			g.fillPolygon(_x, _y, _x.length);
			g.setPaint(Color.black);
			g.drawPolygon(_x, _y, _x.length);
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

			if(getProperty().test_result==false){
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
		if(type==TYPE_switch) border = new Swi(loc.x, loc.y, size.width, size.height);
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
		
		if(property.collapsed){
			f = new Font(font.getFamily(), font.getStyle(), (int)(fontsize*0.8*view.zoom));
			g.setStroke(new BasicStroke(1));
			cnt = getLocation().add(getSizeInternal()).getPoint();
			Vec dim = new Vec(10,10).scale(view.zoom);
			g.setPaint(Color.white);
			g.fillRect(cnt.x, cnt.y, dim.getIntX(),dim.getIntY());
			g.setPaint(Color.blue);
			g.drawRect(cnt.x, cnt.y, dim.getIntX(),dim.getIntY());
			g.drawLine(cnt.x+dim.getIntX()/2, cnt.y+(int)(view.zoom*2), cnt.x+dim.getIntX()/2, cnt.y+dim.getIntY()-(int)(view.zoom*2));
			g.drawLine(cnt.x+(int)(view.zoom*2), cnt.y+dim.getIntY()/2, cnt.x+dim.getIntX()-(int)(view.zoom*2), cnt.y+dim.getIntY()/2);
		}
		
		// ADDED Draw tooltip
		if (type.equals(TYPE_task) && property.selected && taskDescriptionProvider != null) {
			String cleanName = getNameWithoutParameters();
			
			TaskDescription.Task taskDesc = taskDescriptionProvider.get(cleanName);
			if (taskDesc != null) {
				String message = String.format("Description:\n\t- %s", taskDesc.algorithm);
				
				_tooltip.setMessage("", message);
				_tooltip.paint(g);
			}
		}
		
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
	public GElement clone() {
		Task n = new Task();
		cloneInit(n);
		n.text = text;
		n.type = type;
		n.seqNumber = seqNumber;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {
		
	}
	
	@Override
	public void modify() {
		ModifyDialog dlg = new ModifyDialog();
		dlg.setVisible(true);
		onViewChange();
	}
	
	class ModifyDialog extends JDialog {
		private static final long serialVersionUID = 1739783395697186997L;

		public ModifyDialog() {

	        initUI();
	    }
		
		Java2sAutoTextField txtName = null;
	    JComboBox cType = null;
	    JTextField txtDbgTime = null;
	    JComboBox txtDbgResult = null;
	    JCheckBox chkCollapse = null;
	    JTextArea txtTaskDescAlgoritm;
	    
	    // Java2sAutoTextField txtNameAC;
	    
	    public final void initUI() {

	    	setLayout(null);
	    	UIManager.put("TextArea.margin", new Insets(10,10,10,10));
	    	
	    	JLabel lbl1 = new JLabel("Name ");
	    	JLabel lbl2 = new JLabel("Type ");
	    	JLabel lbl3 = new JLabel("Dbg-Time   ");
	    	JLabel lbl4 = new JLabel("Dbg-Result ");
	    	
	    	// Task description
	    	JLabel lbl5 = new JLabel("Description ");
	    	
	    	
	    	if (taskDescriptionProvider != null)
	    		txtName = new Java2sAutoTextField(taskDescriptionProvider.getNames()); // txtName.selectAll();
	    	else
	    		txtName = new Java2sAutoTextField(new ArrayList<String>()); // txtName.selectAll();
	    	
	    	txtName.setEditable(true);
	    	txtName.setEnabled(true);
	    	txtName.setStrict(false);
	    	txtName.setText(text);
	    	
	    	cType = new JComboBox(new String[]{TYPE_sequenser,TYPE_selector,TYPE_task, TYPE_parallel, TYPE_switch});
	    	cType.setSelectedItem(type);
	    	txtDbgTime = new JTextField(""+getProperty().test_time); 
	    	txtDbgResult = new JComboBox(new String[]{"true","false"});
	    	txtDbgResult.setSelectedItem(""+getProperty().test_result);
	    	
	    	txtTaskDescAlgoritm = new JTextArea();

	    	chkCollapse = new JCheckBox("Collapse");
	    	chkCollapse.setSelected(getProperty().collapsed);
	    	
        
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
	        			getProperty().test_time = Integer.parseInt(txtDbgTime.getText());
	        			getProperty().test_result = Boolean.parseBoolean((String) txtDbgResult.getSelectedItem());
	        			getProperty().collapsed = chkCollapse.isSelected();
	        			
						if (type == TYPE_task && taskDescriptionProvider != null) {
							TaskDescription.Task updateTask = new TaskDescription.Task();
							updateTask.algorithm = txtTaskDescAlgoritm.getText();
							taskDescriptionProvider.put(getNameWithoutParameters(), updateTask);
						}
	        		}catch(Exception e){
	        			e.printStackTrace();
	        		}
	        		dispose();
	        	}
	        });
	        
	        txtName.addKeyListener(new KeyAdapter() {
	            public void keyReleased(KeyEvent e) {
	            	if (taskDescriptionProvider == null)
	            		return;
	            	
	            	String typedText = getNameWithoutParameters(txtName.getText());
	            	TaskDescription.Task taskDesc = taskDescriptionProvider.get(typedText);
	            	
	            	if (taskDesc != null)
	            		txtTaskDescAlgoritm.setText(taskDesc.algorithm);
	            	else 
	            		txtTaskDescAlgoritm.setText("");
	            }
	 
	            public void keyTyped(KeyEvent e) {
	            	
	            }
	 
	            public void keyPressed(KeyEvent e) {
	                
	            }
	        });
	        
	        add(lbl1); add(txtName);
	        add(lbl2); add(cType);
	        
	        if(type!=TYPE_task){
	        	add(chkCollapse);
	        }else{
	        	add(lbl3); add(txtDbgTime);
	        	add(lbl4); add(txtDbgResult);
	        	
	        	if (taskDescriptionProvider != null) {
		        	TaskDescription.Task taskDesc = taskDescriptionProvider.get(getNameWithoutParameters());
		        	
		        	if (taskDesc != null)
		        		txtTaskDescAlgoritm.setText(taskDesc.algorithm);
		        	
		        	txtTaskDescAlgoritm.setBorder(BorderFactory.createLineBorder(Color.DARK_GRAY));
		        	
			        // Task description
		        	add(lbl5); add(txtTaskDescAlgoritm);
	        	}
	        }
	        
	        add(close); add(OK);
	        
	        lbl1.setBounds(10,10, 100, 30);
	        txtName.setBounds(160,10, 370, 30);
	        lbl2.setBounds(10,50, 100, 30);
	        cType.setBounds(160,50, 370, 30);
	        
	        lbl3.setBounds(10,90, 130, 30);
	        txtDbgTime.setBounds(160,90, 370, 30);
	        lbl4.setBounds(10,130, 130, 30);
	        txtDbgResult.setBounds(160,130, 370, 30);
	        
	        lbl5.setBounds(10, 170, 100, 30);
	        txtTaskDescAlgoritm.setBounds(160, 170, 370, 150);
	        
	        chkCollapse.setBounds(5,200, 100, 30);
	        
	        close.setBounds(10,340, 80, 30);
	        OK.setBounds(440,330, 100, 30);

	        setModalityType(ModalityType.APPLICATION_MODAL);

	        setTitle("Change Task");
	        setDefaultCloseOperation(DISPOSE_ON_CLOSE);
	        setLocationRelativeTo(null);
	        //setSize(300, 230);
	        setSize(550, 400);
	    }
	}
	
}
