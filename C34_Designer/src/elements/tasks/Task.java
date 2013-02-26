package elements.tasks;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Dialog.ModalityType;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Map;

import javax.swing.JFrame;

import logger.Log;

import terminal.communication.StackStreamMessage;
import terminal.communication.StackStreamMessage.ChangeType;
import terminal.communication.StackStreamMessage.TaskFinishReason;

import document.BTDesigner;
import document.Document;
import document.description.TaskDescription;
import elements.GElement;
import elements.Tooltip;
import elements.Vec;
import elements.View;
import elements.Tooltip.ToolTipDesign;

public class Task extends GElement implements View.ChangesListener {

	public final static String TYPE_task = "task";
	public final static String TYPE_selector = "selector";
	public final static String TYPE_sequenser = "sequenser";
	public final static String TYPE_parallel = "parallel";
	public final static String TYPE_switch = "switch";
	
	public int seqNumber = 0;
	public String text = "Noname";
	public String type = TYPE_task;
	public Font font = new Font("sansserif", Font.BOLD, 10);
	private TaskDescription taskDescriptionProvider;
	private Tooltip _tooltip;	
	private Tooltip _debugInfo;
	private Tooltip _runtimeInfo;
	private Document _document;
	private TaskResultCollection _resultsHistory = new TaskResultCollection();
	private Date _runStart;
	private Date _runFinish;
	private boolean _runFailed;
	private boolean _isRunning;
	

	final int shortTextLen = 25;

	public Task() {
		this.property.size = new Vec(100, 100);
		this._tooltip = new Tooltip(this);
		this._debugInfo = new Tooltip(this, ToolTipDesign.DebugInfo);
		this._runtimeInfo = new Tooltip(this, ToolTipDesign.RuntimeInfo);
	}

	private boolean isDebugViewEnabled() {
		return !(this._document == null) && this._document.isDebugViewEnabled();
	}
	
	private boolean isRuntimeViewEnabled() {
		return !(this._document == null) && this._document.isRuntimeViewEnabled();
	}
	
	@Override
	public GElement clone() {
		Task n = new Task();
		n.setDocument(this._document);
		cloneInit(n);
		
		if (this.text != null)
			n.text = new String(this.text);
		
		if (this.type != null)
			n.type = new String(this.type);
		
		n.view = this.getView();
		n.seqNumber = this.seqNumber;
		
		n._debugInfo = (Tooltip)this._debugInfo.clone(n);
		n._runtimeInfo = (Tooltip)this._runtimeInfo.clone(n);
		
		n._resultsHistory = this._resultsHistory;
		n._runFailed = this._runFailed;
		n._runFinish = this._runFinish;
		n._runStart = this._runStart;
		n._isRunning = this._isRunning;
		
		return n;
	}

	@Override
	public void cloneReconnect(Map<GElement, GElement> link) {

	}

	public void drawString(Graphics2D g, String t, int x, int y) {
		g.drawString(t, x, y + (int) (getTextSize(g, getText()).height * 0.8));
	}

	@Override
	public void modify() {
		ModifyDialog dlg = new ModifyDialog(this);
		dlg.setModal(true);
		dlg.setModalityType(ModalityType.APPLICATION_MODAL);
		dlg.setLocation(100, 100);
		dlg.setVisible(true);
		// dlg.setAlwaysOnTop(true);
		onViewChange();
	}

	@Override
	public void paint(Graphics2D g) {
		onViewChange();
		
		GraphProp gp = new GraphProp(g);
		Point loc = getLocation().getPoint();
		Dimension size = getSizeInternal().getDimension();
		
		
		// *******************************************************************************
		// Bold border when selected *****************************************************
		// *******************************************************************************
		if (this.property.leftClicked)
			g.setStroke(new BasicStroke(3));
		
		// *******************************************************************************
		// Draw border *******************************************************************
		// *******************************************************************************
		Border border = null;
		if (this.type.equalsIgnoreCase(TYPE_selector))
			border = new SelectorBorder(this, loc.x, loc.y, size.width, size.height);
		if (this.type.equalsIgnoreCase(TYPE_sequenser))
			border = new SequenceBorder(this, loc.x, loc.y, size.width, size.height);
		if (this.type.equalsIgnoreCase(TYPE_task))
			border = new TaskBorder(this, loc.x, loc.y, size.width, size.height, isLastRunFailed());
		if (this.type.equalsIgnoreCase(TYPE_parallel))
			border = new ParallelBorder(this, loc.x, loc.y, size.width, size.height);
		if (this.type.equalsIgnoreCase(TYPE_switch))
			border = new SwitchBorder(this, loc.x, loc.y, size.width, size.height);
		border.paint(g);

		// *******************************************************************************
		// Draw text *********************************************************************
		// *******************************************************************************
		int fontsize = this.font.getSize();
		Font f = new Font(this.font.getFamily(), this.font.getStyle(), (int) (fontsize * this.view.zoom));
		g.setFont(f);
		Dimension tdim = new Vec(getTextSize(g, getText())).scale(0.5).getDimension();
		Point cnt = getCenterInternal().getPoint();
		drawString(g, getText(), cnt.x - tdim.width, cnt.y - tdim.height);

		
		// *******************************************************************************
		// Draw tooltips *****************************************************************
		// *******************************************************************************
		if (this.type.equalsIgnoreCase(TYPE_task) && this.property.leftClicked && this.taskDescriptionProvider != null) {
			this._tooltip.setView(this.getView());
			this._tooltip.paint(g);
		}
		else if (this.type.equalsIgnoreCase(TYPE_task) && isDebugViewEnabled()) {
			// Draw debug info tooltip
			this._debugInfo.setView(this.getView());
			this._debugInfo.setMessage(String.format("Duration: %d\n%sDebug: %b", this.property.test_time, !this.property.test_result ? "$RED$" : "",this.property.test_result));
			this._debugInfo.paint(g);
		} else if (this.type.equalsIgnoreCase(TYPE_task) && isRuntimeViewEnabled()) {
			// this._runtimeInfo.setMessage(getRunResultsString());
			this._runtimeInfo.setView(this.getView());
			this._runtimeInfo.paint(g);
		}
		
		// *******************************************************************************
		// Draw sequence number **********************************************************
		// *******************************************************************************		
		if (this.seqNumber > 0) {
			f = new Font(this.font.getFamily(), this.font.getStyle(),
					(int) (fontsize * 0.8 * this.view.zoom));
			g.setFont(f);
			g.setPaint(Color.blue);
			tdim = new Vec(getTextSize(g, "" + this.seqNumber)).scale(0.5)
					.getDimension();
			cnt = getLocation().getPoint();
			drawString(g, "" + this.seqNumber, cnt.x - tdim.width, cnt.y - tdim.height);
		}

		// *******************************************************************************
		// Draw collapse icon ************************************************************
		// *******************************************************************************
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
		
		gp.restore();
	}


	// *******************************************************************************
	// Getters & setters *************************************************************
	// *******************************************************************************
	
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
	protected Vec getSize() {
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

	public String getRunResultsString(boolean addColorFormat) {
		if (_resultsHistory.size() == 0)
			return "";
		
		StringBuilder output = new StringBuilder();
		
		for (TaskResult result : _resultsHistory) {
			if (result.isFailure() && addColorFormat)
				output.append("$RED$");
			
			output.append(result);
			output.append("\n");
		}
		
		return output.toString();
	}
	
	public boolean hasFailedRunResult() {
		for (TaskResult result : _resultsHistory)
			if (result.isFailure())
				return true;
		
		return false;
	}
	
	public boolean isLastRunFailed() {
		return _runFailed;
	}
	
	public boolean isRunning() {
		return _isRunning || getProperty().running;
	}

	public void setDocument(Document document) {
		this._document = document;
	}
	
	public Document getDocument() {
		return this._document;
	}
	
	public void setTaskDescriptionProvider(TaskDescription provider) {
		this.taskDescriptionProvider = provider;

		String cleanName = getNameWithoutParameters();
		TaskDescription.TaskInfo taskDesc = this.taskDescriptionProvider.get(cleanName);
		if (taskDesc != null)
			this._tooltip.setMessage("Description", taskDesc.algorithm);
	}
	
	public TaskDescription getTaskDescriptionProvider() {
		return this.taskDescriptionProvider;
	}
	
	public TaskResultCollection getRunResults() {
		return _resultsHistory;
	}
	
	@Override
	public GElement underMouse(Point p) {
		Point lt = getLocation().getPoint();
		Point rb = getLocation().add(getSize()).getPoint();
		if (lt.x <= p.x && p.x <= rb.x && lt.y <= p.y && p.y <= rb.y)
			return this;
		return null;
	}
	
	@Override
	public String toString() {
		return "" + this.type + "{" + this.text + "}";
	}

	public void clearRunning() {
		getProperty().running = false;
		_isRunning = false;
		_runFailed = false;
		_runtimeInfo.setMessage("");
	}
	
	// *******************************************************************************
	// Events ************************************************************************
	// *******************************************************************************
	
	public void onMessageReceive(StackStreamMessage message) {
		TaskResult taskResult = new TaskResult(message.getTaskResultCode(), message.getTaskResultDescription());
		
		// _results.add(taskResult);
		
		if (message.getChangeType() == ChangeType.TaskStarted)
			onStart();
		else
			onStop(taskResult, message);
		
	}
	
	public void onPlanRun() {
		clearRunning();
	}
	
	public void onStart() {
		_isRunning = true;
		_runStart = new Date();
		_runFinish = new Date();
		_runFailed = false;
		
		this._runtimeInfo.setMessage(
				String.format("[%s] %s", new SimpleDateFormat("HH:mm:ss").format(_runStart), "Running..."));
	}
	
	public void onStop(TaskResult taskResult, StackStreamMessage message) {
		_isRunning = false;
		_runFinish = taskResult.getFinishTime();
		_runFailed = taskResult.isFailure();
		taskResult.setStartTime(_runStart);
		
		if (message.getTaskFinishReason() == null) {
			Log.e("Something is null");
		}
		
		this._runtimeInfo.setMessage(String.format(
				"%s[%s-%s] %s(%d):%s",
				_runFailed ? "$RED$" : "",
				new SimpleDateFormat("HH:mm:ss").format(_runStart),
				new SimpleDateFormat("HH:mm:ss").format(_runFinish),
				message.getTaskFinishReason(),
				taskResult.getCode(),
				taskResult.getDescription().length() > 15 ? taskResult.getDescription().subSequence(0, 14) + "..." : taskResult.getDescription()
			));

		_resultsHistory.add(taskResult);
	}

	@Override
	public void onViewChange() {
		if (this.view.graphics == null)
			return;
		
		this.property.size = new Vec(getTextSize(this.view.graphics, this.font,getShortText())).add(new Vec(10, 10));
	}

	public void showHistory(JFrame parent) {
		new TaskRunHistoryDialog(parent, this);
	}
}
