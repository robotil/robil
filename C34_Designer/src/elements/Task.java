package elements;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.Map;
import java.util.Vector;

import javax.swing.Icon;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.ScrollPaneConstants;
import javax.swing.UIManager;

import document.description.TaskDescription;

public class Task extends GElement implements View.ChangesListener {

	private abstract class Border {
		public abstract void paint(Graphics2D g);

		public void setBackgroundColor(Graphics2D g) {
			if (getProperty().running)
				g.setPaint(new Color(152, 251, 152, 200));
			else
				g.setPaint(Color.white);
		}

		public void setBackgroundColor(Graphics2D g, Color bgmain) {
			if (getProperty().running)
				g.setPaint(new Color(152, 251, 152, 200));
			else
				g.setPaint(bgmain);
		}
	}

	static public class Creator extends GElement.Creator {
		@Override
		public boolean createOnEmptyPlace() {
			return true;
		}

		@Override
		public Icon getIcon() {
			return null;
		}

		@Override
		public String getToolbarName() {
			return "Task";
		}

		@Override
		public GElement newInstance() {
			return new Task();
		}

		@Override
		public boolean ready() {
			return true;
		}

		@Override
		public String toolTip() {
			return "Create a Node (Seq, Sel, Par, Task). Select a point on the document to place a task";
		}
	}

	class ModifyDialog extends JDialog {
		private static final long serialVersionUID = 1739783395697186997L;

		JAutoSuggestComboBox txtName = null;

		JComboBox<String> cType = null;;

		JTextField txtDbgTime = null;
		JComboBox<String> txtDbgResult = null;
		JCheckBox chkCollapse = null;
		JTextArea txtTaskDescAlgoritm;
		JScrollPane txtTaskDescScroll;
		Boolean descriptionChanged = false;
		Boolean descriptionLocked = false;

		public ModifyDialog() {
			initUI();
		}

		@Override
		public void dispose() {
			super.dispose();

		}

		public final void initUI() {

			setLayout(null);
			UIManager.put("TextArea.margin", new Insets(10, 10, 10, 10));

			JLabel lbl1 = new JLabel("Name ");
			JLabel lbl2 = new JLabel("Type ");
			JLabel lbl3 = new JLabel("Dbg-Time   ");
			JLabel lbl4 = new JLabel("Dbg-Result ");

			// Task description
			JLabel lbl5 = new JLabel("Description ");

			if (Task.this.taskDescriptionProvider != null)
				this.txtName = new JAutoSuggestComboBox(
						Task.this.taskDescriptionProvider.getNamesVector()); // txtName.selectAll();
			else
				this.txtName = new JAutoSuggestComboBox(new Vector<String>()); // txtName.selectAll();

			this.txtName.setEditable(true);
			this.txtName.setEnabled(true);
			// txtName.setStrict(false);
			this.txtName.setText(Task.this.text);

			this.cType = new JComboBox<String>(new String[] { TYPE_sequenser,
					TYPE_selector, TYPE_task, TYPE_parallel, TYPE_switch });
			this.cType.setSelectedItem(Task.this.type);
			this.txtDbgTime = new JTextField("" + getProperty().test_time);
			this.txtDbgResult = new JComboBox<String>(new String[] { "true",
					"false" });
			this.txtDbgResult.setSelectedItem("" + getProperty().test_result);

			this.txtTaskDescAlgoritm = new JTextArea();
			// txtTaskDescAlgoritm.setAutoscrolls(true);
			this.txtTaskDescScroll = new JScrollPane(this.txtTaskDescAlgoritm);

			this.chkCollapse = new JCheckBox("Collapse");
			this.chkCollapse.setSelected(getProperty().collapsed);

			JButton close = new JButton("Close");
			close.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent event) {
					dispose();
				}
			});

			JButton OK = new JButton("OK");
			OK.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent event) {
					Task.this.text = ModifyDialog.this.txtName.getText();
					Task.this.type = (String) ModifyDialog.this.cType
							.getSelectedItem();
					try {
						getProperty().test_time = Integer
								.parseInt(ModifyDialog.this.txtDbgTime
										.getText());
						getProperty().test_result = Boolean
								.parseBoolean((String) ModifyDialog.this.txtDbgResult
										.getSelectedItem());
						getProperty().collapsed = ModifyDialog.this.chkCollapse
								.isSelected();

						if (Task.this.type.equalsIgnoreCase(TYPE_task)
								&& Task.this.taskDescriptionProvider != null) {
							TaskDescription.Task updateTask = new TaskDescription.Task();
							updateTask.algorithm = ModifyDialog.this.txtTaskDescAlgoritm
									.getText();
							Task.this.taskDescriptionProvider.put(
									getNameWithoutParameters(), updateTask);
						}
					} catch (Exception e) {
						e.printStackTrace();
					}
					dispose();
				}
			});

			this.txtName.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent arg0) {
					if (Task.this.taskDescriptionProvider == null
							|| ModifyDialog.this.descriptionLocked)
						return;

					String typedText = getNameWithoutParameters(ModifyDialog.this.txtName
							.getText());
					TaskDescription.Task taskDesc = Task.this.taskDescriptionProvider
							.get(typedText);

					if (ModifyDialog.this.descriptionChanged
							&& !ModifyDialog.this.txtTaskDescAlgoritm.getText()
									.trim().equals("")) {

						// InputBox
						if (taskDesc != null) {
							int dialogResult = JOptionPane.showConfirmDialog(
									null, "Save description?",
									"Description has changed",
									JOptionPane.YES_NO_OPTION);

							if (dialogResult == JOptionPane.YES_OPTION)
								ModifyDialog.this.descriptionLocked = true;
							else {
								ModifyDialog.this.txtTaskDescAlgoritm
										.setText(taskDesc.algorithm);
							}
						}

					} else {
						if (taskDesc != null)
							ModifyDialog.this.txtTaskDescAlgoritm
									.setText(taskDesc.algorithm);
						else
							ModifyDialog.this.txtTaskDescAlgoritm.setText("");
					}
				}
			});

			this.txtName.addKeyListener(new KeyAdapter() {
				@Override
				public void keyReleased(KeyEvent e) {

				}
			});

			this.txtTaskDescAlgoritm.addKeyListener(new KeyAdapter() {
				@Override
				public void keyReleased(KeyEvent e) {
					ModifyDialog.this.descriptionChanged = true;
				}
			});

			// Handle escape key press
			KeyboardFocusManager.getCurrentKeyboardFocusManager()
					.addKeyEventDispatcher(new KeyEventDispatcher() {
						@Override
						public boolean dispatchKeyEvent(KeyEvent e) {
							if (e.getID() == KeyEvent.KEY_PRESSED) {
								if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
									dispose();

								}
							}
							return false;
						}
					});

			add(lbl1);
			add(this.txtName);
			add(lbl2);
			add(this.cType);

			if (!Task.this.type.equalsIgnoreCase(TYPE_task)) {
				add(this.chkCollapse);
			} else {
				add(lbl3);
				add(this.txtDbgTime);
				add(lbl4);
				add(this.txtDbgResult);

				if (Task.this.taskDescriptionProvider != null) {
					TaskDescription.Task taskDesc = Task.this.taskDescriptionProvider
							.get(getNameWithoutParameters());

					if (taskDesc != null)
						this.txtTaskDescAlgoritm.setText(taskDesc.algorithm);

					// Task description
					add(lbl5);
					add(this.txtTaskDescScroll);
				}
			}

			add(close);
			add(OK);

			lbl1.setBounds(10, 10, 100, 30);
			this.txtName.setBounds(160, 10, 370, 30);
			lbl2.setBounds(10, 50, 100, 30);
			this.cType.setBounds(160, 50, 370, 30);

			lbl3.setBounds(10, 90, 130, 30);
			this.txtDbgTime.setBounds(160, 90, 370, 30);
			lbl4.setBounds(10, 130, 130, 30);
			this.txtDbgResult.setBounds(160, 130, 370, 30);

			lbl5.setBounds(10, 170, 100, 30);
			// txtTaskDescAlgoritm.setBounds(160, 170, 370, 150);
			this.txtTaskDescScroll.setPreferredSize(new Dimension(370, 150));
			this.txtTaskDescScroll.setBounds(160, 170, 370, 150);
			this.txtTaskDescScroll
					.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
			this.txtTaskDescAlgoritm.setLineWrap(true);
			// txtTaskDescAlgoritm.setBorder(BorderFactory.createLineBorder(Color.DARK_GRAY));

			this.chkCollapse.setBounds(5, 200, 100, 30);

			close.setBounds(10, 340, 80, 30);
			OK.setBounds(440, 330, 100, 30);

			setModalityType(ModalityType.APPLICATION_MODAL);

			setTitle("Change Task");
			setDefaultCloseOperation(DISPOSE_ON_CLOSE);
			setLocationRelativeTo(null);
			// setSize(300, 230);
			setSize(550, 400);
		}
	}

	private class Par extends Border {
		public int[] _x;
		public int[] _y;

		public Par(int x, int y, int w, int h) {
			Vec t = new Vec(5, 3).scale(Task.this.view.zoom);
			this._x = new int[] { x + t.getIntX(), x + w + t.getIntX(),
					x + w - t.getIntX(), x - t.getIntX() };
			this._y = new int[] { y, y, y + h, y + h };
		}

		@Override
		public void paint(Graphics2D g) {
			setBackgroundColor(g);
			// g.drawString("P", _x[0]+ size()/2, _y[0]-5);
			// g.setPaint(new Color(127,255,212));
			g.fillPolygon(this._x, this._y, size());

			g.setPaint(Color.black);
			g.drawPolygon(this._x, this._y, size());
		}

		public int size() {
			return this._x.length;
		}
	}

	private class Sel extends Border {
		int x, y, w, h;

		public Sel(int _x, int _y, int _w, int _h) {
			this.x = _x;
			this.y = _y;
			this.w = _w;
			this.h = _h;
		}

		@Override
		public void paint(Graphics2D g) {
			Vec t = new Vec(20, 20).scale(Task.this.view.zoom);
			setBackgroundColor(g);
			g.fillRoundRect(this.x, this.y, this.w, this.h, t.getIntX(),
					t.getIntY());
			g.setPaint(Color.black);
			g.drawRoundRect(this.x, this.y, this.w, this.h, t.getIntX(),
					t.getIntY());
		}
	}

	private class Seq extends Border {
		public int[] _x;
		public int[] _y;

		public Seq(int x, int y, int w, int h) {
			Vec t = new Vec(12, 3).scale(Task.this.view.zoom);
			this._x = new int[] { x, (int) (x + w - t.x), (int) (x + w - t.x),
					x + w, (int) (x + w - t.x), (int) (x + w - t.x), x, x };
			this._y = new int[] { (int) (y + t.y), (int) (y + t.y), y,
					(int) (y + h / 2.0), y + h, (int) (y + h - t.y),
					(int) (y + h - t.y), (int) (y + t.y) };
		}

		@Override
		public void paint(Graphics2D g) {
			setBackgroundColor(g);

			g.fillPolygon(this._x, this._y, size());

			g.setPaint(Color.black);
			g.drawPolygon(this._x, this._y, size());
		}

		public int size() {
			return this._x.length;
		}
	}

	private class Swi extends Border {
		int x, y, w, h;

		public Swi(int _x, int _y, int _w, int _h) {
			this.x = _x;
			this.y = _y;
			this.w = _w;
			this.h = _h;
		}

		@Override
		public void paint(Graphics2D g) {
			Vec t = new Vec(5, 5).scale(Task.this.view.zoom);
			if (t.y > 5)
				t.y = 5;
			setBackgroundColor(g);
			g.fillRect(this.x, this.y, this.w, this.h);
			// g.setStroke (new BasicStroke(
			// 2f,
			// BasicStroke.CAP_ROUND,
			// BasicStroke.JOIN_ROUND,
			// 2f,
			// new float[] {6f},
			// 0f));
			g.setPaint(Color.black);
			g.drawRect(this.x, this.y, this.w, this.h);

			int sw, k, d = 0;
			if (this.w >= 5 * 5) {
				int kk, nn;
				for (kk = 3, nn = 3; this.w / kk >= 5; nn = kk, kk += 2)
					;
				k = Math.abs(this.w % kk) > Math.abs(this.w % nn) ? nn : kk;
				d = this.w % k;
			} else {
				k = 5;
			}
			sw = this.w / k;
			int di = 1;// (d/(k/2));

			int[] _x = new int[k * 2];
			int[] _y = new int[k * 2];
			_x[0] = this.x;
			for (int i = 1; i < k; i += 1) {
				_x[i * 2] = _x[i * 2 - 2] + sw;
				if (d > 0) {
					_x[i * 2] += di;
					d -= di;
				}
			}
			for (int i = 0; i < k - 1; i += 1) {
				_x[1 + i * 2] = _x[1 + i * 2 + 1];
			}
			for (int i = 0; i < k; i += 1) {
				_y[i * 2] = i % 2 == 0 ? this.y + this.h : this.y + this.h
						+ (int) (t.y);
			}
			for (int i = 0; i < k - 1; i += 1) {
				_y[1 + i * 2] = _y[1 + i * 2 - 1];
			}
			_x[k * 2 - 1] = this.x + this.w;
			_y[k * 2 - 1] = this.y + this.h;

			setBackgroundColor(g);
			g.fillPolygon(_x, _y, _x.length);
			g.setPaint(Color.black);
			g.drawPolygon(_x, _y, _x.length);
		}
	}

	private class Tsk extends Border {
		int x, y, w, h;

		public Tsk(int _x, int _y, int _w, int _h) {
			this.x = _x;
			this.y = _y;
			this.w = _w;
			this.h = _h;
		}

		@Override
		public void paint(Graphics2D g) {
			setBackgroundColor(g, new Color(230, 230, 250, 200));

			g.fillRect(this.x, this.y, this.w, this.h);

			g.setPaint(Color.black);
			g.drawRect(this.x, this.y, this.w, this.h);

			if (getProperty().test_result == false) {
				GraphProp gp = new GraphProp(g);
				g.setPaint(Color.red);
				g.drawRect(this.x - 1, this.y - 1, this.w + 2, this.h + 2);
				gp.restore();
			}
		}
	}

	public final static String TYPE_task = "task";

	public final static String TYPE_selector = "selector";

	public final static String TYPE_sequenser = "sequenser";

	public final static String TYPE_parallel = "parallel";

	public final static String TYPE_switch = "switch";

	public String text = "Noname";

	public String type = TYPE_task;

	public Font font = new Font("sansserif", Font.BOLD, 10);

	public int seqNumber = 0;

	private TaskDescription taskDescriptionProvider;

	private final Tooltip _tooltip;

	final int shortTextLen = 25;

	public Task() {
		this.property.size = new Vec(100, 100);
		this._tooltip = new Tooltip(this);
	}

	@Override
	public GElement clone() {
		Task n = new Task();
		cloneInit(n);
		
		if (this.text != null)
			n.text = new String(this.text);
		
		if (this.type != null)
			n.type = new String(this.type);
		
		n.seqNumber = this.seqNumber;
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {

	}

	public void drawString(Graphics2D g, String t, int x, int y) {
		g.drawString(t, x, y + (int) (getTextSize(g, getText()).height * 0.8));
	}

	Vec getCenterInternal() {
		return getLocation().add(getSizeInternal().scale(0.5));
	}

	public String getNameWithoutParameters() {
		return getNameWithoutParameters(this.text);
	}

	public String getNameWithoutParameters(String name) {
		name = name.replaceAll("\\(.*\\)", "");
		return name.replaceAll("\\(.*", "");
	}

	public String getShortText() {
		if (this.text.length() > this.shortTextLen)
			return this.text.substring(0, this.shortTextLen - 3) + "...";
		return this.text;
	}

	@Override
	Vec getSize() {
		onViewChange();
		return super.getSize();
	}

	Vec getSizeInternal() {
		return new Vec(getTextSize(this.view.graphics, this.font, getText()))
				.add(new Vec(10, 10)).scale(this.view.zoom);
	}

	public String getText() {
		if (this.property.leftClicked)
			return this.text;
		if (this.text.length() > this.shortTextLen)
			return this.text.substring(0, this.shortTextLen - 3) + "...";
		return this.text;
	}

	@Override
	public void modify() {
		ModifyDialog dlg = new ModifyDialog();
		dlg.setVisible(true);
		onViewChange();
	}

	@Override
	public void onViewChange() {
		if (this.view.graphics == null)
			return;
		this.property.size = new Vec(getTextSize(this.view.graphics, this.font,
				getShortText())).add(new Vec(10, 10));
	}

	@Override
	public void paint(Graphics2D g) {
		onViewChange();
		GraphProp gp = new GraphProp(g);

		Point loc = getLocation().getPoint();
		Dimension size = getSizeInternal().getDimension();

		if (this.property.leftClicked) {
			g.setStroke(new BasicStroke(3));
		}

		Border border = null;
		if (this.type.equalsIgnoreCase(TYPE_selector))
			border = new Sel(loc.x, loc.y, size.width, size.height);
		if (this.type.equalsIgnoreCase(TYPE_sequenser))
			border = new Seq(loc.x, loc.y, size.width, size.height);
		if (this.type.equalsIgnoreCase(TYPE_task))
			border = new Tsk(loc.x, loc.y, size.width, size.height);
		if (this.type.equalsIgnoreCase(TYPE_parallel))
			border = new Par(loc.x, loc.y, size.width, size.height);
		if (this.type.equalsIgnoreCase(TYPE_switch))
			border = new Swi(loc.x, loc.y, size.width, size.height);
		border.paint(g);

		int fontsize = this.font.getSize();
		Font f = new Font(this.font.getFamily(), this.font.getStyle(),
				(int) (fontsize * this.view.zoom));
		g.setFont(f);
		Dimension tdim = new Vec(getTextSize(g, getText())).scale(0.5)
				.getDimension();
		Point cnt = getCenterInternal().getPoint();
		drawString(g, getText(), cnt.x - tdim.width, cnt.y - tdim.height);

		if (this.seqNumber > 0) {
			f = new Font(this.font.getFamily(), this.font.getStyle(),
					(int) (fontsize * 0.8 * this.view.zoom));
			g.setFont(f);
			g.setPaint(Color.blue);
			tdim = new Vec(getTextSize(g, "" + this.seqNumber)).scale(0.5)
					.getDimension();
			cnt = getLocation().getPoint();
			drawString(g, "" + this.seqNumber, cnt.x - tdim.width, cnt.y
					- tdim.height);
			// g.fillOval(cnt.x-tdim.width, cnt.y-tdim.height, tdim.width,
			// tdim.height);
		}

		// Vec typesize = new Vec(20, 20).scale(this.view.zoom);
		// Vec typeloc = getLocation().sub(typesize.scale(0.5));

		if (this.property.collapsed) {
			f = new Font(this.font.getFamily(), this.font.getStyle(),
					(int) (fontsize * 0.8 * this.view.zoom));
			g.setStroke(new BasicStroke(1));
			cnt = getLocation().add(getSizeInternal()).getPoint();
			Vec dim = new Vec(10, 10).scale(this.view.zoom);
			g.setPaint(Color.white);
			g.fillRect(cnt.x, cnt.y, dim.getIntX(), dim.getIntY());
			g.setPaint(Color.blue);
			g.drawRect(cnt.x, cnt.y, dim.getIntX(), dim.getIntY());
			g.drawLine(cnt.x + dim.getIntX() / 2, cnt.y
					+ (int) (this.view.zoom * 2), cnt.x + dim.getIntX() / 2,
					cnt.y + dim.getIntY() - (int) (this.view.zoom * 2));
			g.drawLine(cnt.x + (int) (this.view.zoom * 2),
					cnt.y + dim.getIntY() / 2, cnt.x + dim.getIntX()
							- (int) (this.view.zoom * 2), cnt.y + dim.getIntY()
							/ 2);
		}

		// ADDED Draw tooltip
		if (this.type.equalsIgnoreCase(TYPE_task) && this.property.leftClicked
				&& this.taskDescriptionProvider != null) {
			String cleanName = getNameWithoutParameters();

			TaskDescription.Task taskDesc = this.taskDescriptionProvider
					.get(cleanName);
			if (taskDesc != null) {
				String message = String.format("Description:\n%s",
						taskDesc.algorithm);

				this._tooltip.setMessage("", message);
				this._tooltip.paint(g);
			}
		}

		gp.restore();
	}

	public void setTaskDescriptionProvider(TaskDescription provider) {
		this.taskDescriptionProvider = provider;

		// TaskDescription.Task testTask = new TaskDescription.Task();
		// testTask.algorithm = text;
		// taskDescriptionProvider.put(getNameWithoutParameters(), testTask);

	}

	@Override
	public String toString() {
		return "" + this.type + "{" + this.text + "}";
	}

	@Override
	public GElement underMouse(Point p) {
		Point lt = getLocation().getPoint();
		Point rb = getLocation().add(getSize()).getPoint();
		if (lt.x <= p.x && p.x <= rb.x && lt.y <= p.y && p.y <= rb.y)
			return this;
		return null;
	}

}
