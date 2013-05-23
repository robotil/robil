package document;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.xml.parsers.DocumentBuilderFactory;

import logger.Log;

import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import terminal.communication.StackStreamMessage;
import terminal.communication.StopStreamMessage;
import terminal.communication.StopStreamMessage.PlanFinishReason;
import windows.PlanEditor;
import windows.PlanExecutionHistoryDialog;
import windows.designer.BTDesigner;
import windows.designer.BTDesigner.DesignerTab;

import document.actions.Dialogs;
import document.description.TaskDescription;
import document.history.HistoryManager;
import document.history.HistoryManagerNotReadyException;
import document.ui.Toolbar;
import elements.Arrow;
import elements.Decorator;
import elements.GElement;
import elements.Joint;
import elements.Modifier;
import elements.Vec;
import elements.View;
import elements.tasks.Task;
import elements.tasks.TaskCreator;

public class Document extends JPanel {

	private static final long serialVersionUID = 2195280758622734696L;

	enum TreeChangeType {
		Add, AddArrow, Remove, RemoveArrow, SubTreeRemove, TreeCopy, ArrowModify, Modify
	}

	public static final String tabulation = "   ";

	public final boolean cleanToolSelectionAfterUse = false;
	GElement selectedElement = null;
	Point mousePressed = null;
	Process BTExecuter = null;
	public GElement.Creator creator = null;
	public boolean removeElement = false;
	public boolean removeSubElements = false;
	public boolean copyElement = false;
	public boolean reconectArrow = false;
	public Modifier modifier = null;
	public JLabel tip = new JLabel();
	public ArrayList<GElement> arrays = new ArrayList<GElement>();
	public ArrayList<GElement> elements = new ArrayList<GElement>();
	public View view = new View();
	private TaskDescription _taskDescription = null;
	private LookupTable _lookupTable;
	private boolean _documentChanged = false;
	private boolean _shouldBeSavedAs = false;
	private double lastX = 0;
	private double lastY = 0;
	private String _taskDescriptionFilename;
	private String _taskDescriptionFilenameOriginal;
	private Boolean _taskDescriptionExists = false;
	private Boolean _buildTime = false;
	private String absoluteFilePath = "plan.xml";
	private Vec _mousePosition = new Vec(0, 0);
	private HistoryManager _historyManager = new HistoryManager();
	private Set<String> savedIds = new HashSet<String>();
	private PlanExecutionCollection _executionResults = new PlanExecutionCollection();
	
	public BTDesigner mainWindow = null;

	private Map<String, GElement> loadedElements = new HashMap<String, GElement>();

	/**
	 * Creates new empty, unsaved document
	 * @param mw Parent BTDesginer window
	 */
	public Document(BTDesigner mainWindow) {

		try {
			this.absoluteFilePath = new File(Parameters.path_to_plans, getTempFileName()).getCanonicalFile().getAbsolutePath();
		} catch (IOException e) {
			Log.e(e);
		}
		
		initializeDocument(mainWindow);
		
		this._taskDescriptionFilename = parsePlanPath(Parameters.path_to_description);
		try {
			this._taskDescription = new TaskDescription(this._taskDescriptionFilename);
			Log.d("Task descriptions file loaded from " + this._taskDescriptionFilename + ", descriptions count = " + this._taskDescription.getNames().size());
		} catch (Exception ex) {
			Log.e(ex);
		}
		
		try {
			this._historyManager.init(this);
		} catch (HistoryManagerNotReadyException e) {
			Log.e(e);
		}
		
		_shouldBeSavedAs = true;
	}
	
	/**
	 * Load specified document
	 * @param mw Parent BTDesginer window
	 * @param fileName Plan to load
	 */
	public Document(BTDesigner mainWindow, String fileName) {
		try {
			this.absoluteFilePath = new File(fileName).getCanonicalFile().getAbsolutePath();
		} catch (IOException e) {
			Log.e(e);
		}
		
		initializeDocument(mainWindow);
		
		loadPlan(fileName);
		
		try {
			_historyManager.init(this);
		} catch (HistoryManagerNotReadyException e) {
			Log.e(e);
		}
		
		_shouldBeSavedAs = false;
	}

	private void initializeDocument(BTDesigner mainWindow) {
		this.mainWindow = mainWindow;
		this.view.loc = new Vec(0, 0);
		this.view.zoom = 1;

		this._lookupTable = new LookupTable(Parameters.path_to_lookup);
		
		DocumentMouseHandler mouseHander = new DocumentMouseHandler(this);
		addMouseListener(mouseHander);
		addMouseMotionListener(mouseHander);
		addMouseWheelListener(mouseHander);
	}
	
	private String getTempFileName() {
		SimpleDateFormat dateFormat = new SimpleDateFormat("dd-MM-yy-HHmmssSS");
		return "plan" + dateFormat.format(new Date()) + ".xml";
	}
	
	public void activate() {
		updateUndoRedoButtonsState();
	}
	
	public void setBuildTime(boolean value) {
		this._buildTime = value;
	}
	
	public Vec getMousePos() {
		return this._mousePosition;
	}
	
	public void setMousePosition(Vec vec) {
		this._mousePosition = vec;
	}
	
	public void createTask() {
		GElement el = new TaskCreator().newInstance();
		el.setView(Document.this.view);
		
		// if (el.isTask()) {
		el.getAsTask().setDocument(this);
		el.getAsTask().setTaskDescriptionProvider(_taskDescription);
		// }
		
		if (el instanceof View.ChangesListener)
			((View.ChangesListener) el).onViewChange();
		
		el.getProperty().setCenter(getMousePos());
		el.modify();
		
		add(el);
		repaint();
	}
	
	public void createDecorator(GElement element) {
		Decorator.Creator creator = new Decorator.Creator();
		creator.add(element);
		GElement el = creator.newInstance();
		
		el.setView(Document.this.view);
		if (el instanceof View.ChangesListener)
			((View.ChangesListener) el).onViewChange();
		el.getProperty().setCenter(getMousePos());
		el.modify();
		add(el);
		
		repaint();
	}
	
	public void createJoint(GElement element) {
		Joint.Creator creator = new Joint.Creator();
		creator.add(element);
		GElement el = creator.newInstance();
		
		el.setView(Document.this.view);
		if (el instanceof View.ChangesListener)
			((View.ChangesListener) el).onViewChange();
		el.getProperty().setCenter(getMousePos());
		el.modify();
		add(el);
		
		repaint();
	}
	
	public void add(GElement el) {
		

		el.setView(this.view);
		if (el instanceof Arrow) {
			onBeforeTreeChange(TreeChangeType.AddArrow, el);
			this.arrays.add(el);
			onTreeChange(TreeChangeType.AddArrow, el);
		} else {
			onBeforeTreeChange(TreeChangeType.Add, el);
			this.elements.add(el);

			if (el instanceof Task && ((Task) el).type.equals("task"))
				((Task) el).setTaskDescriptionProvider(this._taskDescription);
			
			onTreeChange(TreeChangeType.Add, el);
		}

		
	}

	private String addTextToFileName(String file, String text) {
		ArrayList<String> s = new ArrayList<String>();
		for (String f : file.split("\\."))
			s.add(f);
		s.add(s.size() - 1, text);
		String r = "";
		for (int i = 0; i < s.size() - 1; i++)
			r = r.concat(s.get(i)) + ".";
		r = r.concat(s.get(s.size() - 1));
		return r;
	}

	public void cleanRunning() {
		for (GElement e : this.elements) {
			e.getProperty().running = false;
		}
		repaint();
	}

	public void compile() {
		compile(getCurrentWorkingFile(), true, true);
	}
	
	public void save() {
		String fileName = getCurrentWorkingFile();
		
		if (_shouldBeSavedAs) {
			fileName = Dialogs.saveFile("Save XML", "xml", getShortFilePath().replace("*", ""), Parameters.path_to_plans);
			if (!fileName.equals("")) {
				fileName = getCurrentWorkingFile();
				_shouldBeSavedAs = false;
			}
		}
		
		compile(fileName, true, true);
	}
	
	public void save(String fileName) {
		compile(fileName, true, true);
	}

	public boolean compile(String destination, boolean updateCurrentWorkingFile, boolean createJustNamesXml) {
		// if (updateCurrentWorkingFile)
		String originalDestination = this.absoluteFilePath;
		boolean saved = false;
		
		setCurrentWorkingFile(destination);
		
		try {
		
			toolSelectionClean();
			ArrayList<GElement> root = getRoot();
			if (root.size() != 1) {
				this.tip.setText("ERROR: Behavior TREE has to have ONE and only ONE root node");
				return false;
			}
			for (GElement ea : this.arrays) {
				if (((Task) ((Arrow) ea).getSource()).type.equalsIgnoreCase(Task.TYPE_task)) {
					this.tip.setText("ERROR: Task in BT is a TERMINAL node");
					return false;
				}
			}
			for (GElement ea : this.elements)
				if (ea.isTask()) {
					if (!ea.isTaskType() && getArrow(ea, null).size() == 0) {
						this.tip.setText("ERROR: Sequenser, Parallel or Selector in BT are a NON TERMINAL nodes");
						return false;
					}
				}
			Task rootTask = (Task) root.get(0);
	
			String xml = null;
			try {
	
				xml = createXml(rootTask, tabulation);
				// Log.d("XML : \n"+"<plan>\n"+xml+"\n</plan>");
				this.tip.setText("Compilation is OK");
	
			} catch (StackOverflowError e) {
				this.tip.setText("ERROR: Behavior TREE has not to have CYCLES");
				return false;
			}
	
			// Write task descriptions file
			try {
				this._taskDescription.save(_taskDescriptionFilename);
				Log.d("Task descriptions file successfully saved to "
						+ _taskDescriptionFilename);
			} catch (Exception e) {
				Log.e("Task descriptions save failed. Filename = "
						+ _taskDescriptionFilename);
			}
	
			saved = saveXmlToFile(xml, getCurrentWorkingFile(), true);
			if (saved && createJustNamesXml)
				saveXmlToFile(createXml(rootTask, tabulation, true),
						getCurrentWorkingFileForXmlWithJustNames(), false);		
			
			return saved;
		} finally {
			
			if (!updateCurrentWorkingFile)
				this.absoluteFilePath = originalDestination;
			
			onDocumentSave(saved);
		}
	}

	public void copyTree(GElement el) {
		onBeforeTreeChange(TreeChangeType.TreeCopy, el);

		ArrayList<GElement> sourceElements = new ArrayList<GElement>();
		ArrayList<GElement> sourceArrows  = arrays;
		ArrayList<GElement> outElements = new ArrayList<GElement>();
		ArrayList<GElement> outArrows = new ArrayList<GElement>();
		
		// Take only tasks
		for (GElement element : searchAllSubelements(el))
			if (element.isTask())
				sourceElements.add(element);
		
		sourceElements.add(el);
		
		cloneElements(sourceElements, sourceArrows, outElements, outArrows);
		
		for (GElement arrow : outArrows) {
			// add(arrow);
			add(arrow);
			arrow.setView(this.view);
		}
		
		for (GElement element : outElements) {
			// add(element);
			add(element);
			element.setView(this.view);
			element.getProperty().location.x+=10;
			element.getProperty().location.y+=10;
		}
		
		onTreeChange(TreeChangeType.TreeCopy, el);
	}
	
	public void copyTree(GElement el, Document sourceDocument) {
		onBeforeTreeChange(TreeChangeType.TreeCopy, el);

		ArrayList<GElement> sourceElements = new ArrayList<GElement>();
		ArrayList<GElement> sourceArrows  = sourceDocument.arrays;
		ArrayList<GElement> outElements = new ArrayList<GElement>();
		ArrayList<GElement> outArrows = new ArrayList<GElement>();
		
		// Take only tasks
		for (GElement element : sourceDocument.searchAllSubelements(el))
			if (element.isTask())
				sourceElements.add(element);
		
		sourceElements.add(el);
		
		cloneElements(sourceElements, sourceArrows, outElements, outArrows);
		
		for (GElement arrow : outArrows) {
			// add(arrow);
			add(arrow);
			arrow.setView(this.view);
		}
		
		for (GElement element : outElements) {
			// add(element);
			add(element);
			element.setView(this.view);
		}

		onTreeChange(TreeChangeType.TreeCopy, el);
	}
	
	public void cloneElements(ArrayList<GElement> outElements, ArrayList<GElement> outArrows) {
		
		HashMap<GElement, GElement> clonedElements = new HashMap<GElement, GElement>();

		GElement clonedElement;
		
		// Copy elements
		for (GElement element : elements) {
			clonedElement = element.clone();
			clonedElements.put(element, clonedElement);
			
			if (clonedElement.isTaskType())
				clonedElement.getAsTask().setTaskDescriptionProvider(_taskDescription);
			
			outElements.add(clonedElement);
		}
		
		for (GElement arrow : arrays) {
			ArrayList<GElement> clonedTargets = new ArrayList<GElement>();
			
			for (GElement target : arrow.getAsArrow().targets)
				clonedTargets.add(clonedElements.get(target));
			
			Arrow clonedArrow = arrow.getAsArrow().clone(
					clonedElements.get(arrow.getAsArrow().source), 
					clonedTargets);
			
			outArrows.add(clonedArrow);
		}
	}
	
	public void cloneElements(ArrayList<GElement> sourceElements, ArrayList<GElement> sourceArrows, ArrayList<GElement> outElements, ArrayList<GElement> outArrows) {
		
		HashMap<GElement, GElement> clonedElements = new HashMap<GElement, GElement>();
	
		GElement clonedElement;
		
		// Copy elements
		for (GElement element : sourceElements) {
			
			if (element.isTaskType())
				clonedElement = element.getAsTask().clone();
			else
				clonedElement = element.clone();
			
			clonedElements.put(element, clonedElement);
			
			if (clonedElement.isTaskType())
				clonedElement.getAsTask().setTaskDescriptionProvider(_taskDescription);
			
			outElements.add(clonedElement);
		}
		
		// Arrows
		for (GElement arrow : sourceArrows) {
			ArrayList<GElement> clonedTargets = new ArrayList<GElement>();
			
			if (!clonedElements.containsKey(arrow.getAsArrow().source))
				continue;
			
			GElement decorator = null;
			
			for (GElement target : arrow.getAsArrow().targets)
				if (clonedElements.containsKey(target))
					clonedTargets.add(clonedElements.get(target));
				else if (target.isDecorator())
					decorator = target.clone();
				else
					continue;
			
			Arrow clonedArrow = arrow.getAsArrow().clone(
				clonedElements.get(arrow.getAsArrow().source), 
				clonedTargets);
			
			if (decorator != null)
				clonedArrow.add(0, decorator);
			
			outArrows.add(clonedArrow);
		}
	}


	public String createXml(Task root, String tab) {
		String res = createXml(root, tab, false);
		this.savedIds.clear();
		return res;
	}

	public String createXml(Task root, String tab, boolean justNames) {
		if (root.type.equalsIgnoreCase(Task.TYPE_task))
			return tab + "<" + xmlNameOfTask(root.type) + " name=\""
					+ root.text + "\""
					+ (justNames ? "" : " " + strTaskProperties(root)) + " />";

		String xml = tab + "<" + xmlNameOfTask(root.type) + " name=\""
				+ root.text + "\""
				+ (justNames ? "" : " " + strTaskProperties(root)) + "> ";
		ArrayList<GElement> subel = getSubElements(root);
		String ntab = tab + tabulation;
		for (GElement el : subel) {
			xml += "\n";
			Arrow arr = (Arrow) getArrow(root, el).get(0);
			ArrayList<GElement> decor = arr.targets;// getDecorators(arr);
			String d_xml_p = "", d_xml_s = "", nntab = ntab;
			for (GElement ed : decor) {
				if (ed instanceof Decorator) {
					Decorator d = (Decorator) ed;
					d_xml_p = d_xml_p + nntab + "<dec name=\"" + d.text + "\""
							+ (justNames ? "" : " " + strDecProperties(d))
							+ "> \n";
					d_xml_s = "\n" + nntab + "</dec>" + d_xml_s;
				} else if (ed instanceof Joint) {
					Joint d = (Joint) ed;
					d_xml_p = d_xml_p + nntab + "<jnt "
							+ d.getProperty().toString(this.view)
							+ (justNames ? "" : " " + strJointProperties(d))
							+ "> \n";
					d_xml_s = "\n" + nntab + "</jnt>" + d_xml_s;
				}
				nntab += tabulation;
			}
			String t_xml = createXml((Task) el, nntab, justNames);
			xml += d_xml_p + t_xml + d_xml_s;
		}
		xml += "\n" + tab + "</" + xmlNameOfTask(root.type) + ">";
		return xml;
	}

	public void exportTasks(String fileName) {
		StringBuilder stringBuilder = new StringBuilder();
		String taskDescString = "";

		int maxTaskNameLength = 0;

		for (GElement ea : this.elements)
			if (ea instanceof Task && ((Task) ea).type.equals(Task.TYPE_task))
				if (((Task) ea).text.length() > maxTaskNameLength)
					maxTaskNameLength = ((Task) ea).text.length();

		int i = 1;
		for (GElement ea : this.elements)
			if (ea instanceof Task && ((Task) ea).type.equals(Task.TYPE_task)) {
				TaskDescription.TaskInfo taskDesc = this._taskDescription
						.get(((Task) ea).getNameWithoutParameters());
				if (taskDesc != null)
					taskDescString = taskDesc.algorithm;
				else
					taskDescString = "";

				stringBuilder.append(String.format(" %3d\t%-"
						+ maxTaskNameLength + "s\t%s\n", i++, ((Task) ea).text,
						taskDescString));
			}

		FileWriter fw;
		try {
			fw = new FileWriter(fileName);
			fw.write(stringBuilder.toString());
			fw.close();
			this.tip.setText("Tasks exported to " + fileName);
		} catch (IOException e) {
			Log.e(e);
			this.tip.setText("Failed to export tasks, see log for more information");
		}

	}

	public ArrayList<String> extractIds(String text) {
		ArrayList<String> ids = new ArrayList<String>();
		String[] lines = text.split("\\[id=");
		int l = 1;
		for (String line : lines) {
			if (l-- < 1) {
				String linesp = line.split("\\]")[0];
				if (linesp.contains(","))
					for (String line1 : linesp.split(","))
						ids.add(line1);
				else
					ids.add(linesp);
			}
		}
		return ids;
	}

	public String getAbsoluteFilePath() {
		if (this.absoluteFilePath == null) {
			return null;
		}
		return new String(this.absoluteFilePath);
	}

	public ArrayList<GElement> getArrow(GElement es, GElement et) {
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for (GElement e : this.arrays) {
			Arrow arr = (Arrow) e;
			if ((arr.getSource() == es || es == null)
					&& (arr.getTarget() == et || et == null))
				subels.add(arr);
		}
		return uniq(subels);
	}
	
	public Arrow getArrow(GElement source) {
		for (GElement arrow : arrays) 
			if (arrow.getAsArrow().source == source)
				return arrow.getAsArrow();
		
		return null;
	}

	private String getCurrentWorkingFile() {
		return this.absoluteFilePath;
	}

	private String getCurrentWorkingFileForXmlWithJustNames() {
		return addTextToFileName(this.absoluteFilePath, "jn");
	}

	public ArrayList<GElement> getDecorators(Arrow arr) {
		ArrayList<GElement> dec = new ArrayList<GElement>();
		for (GElement ae : arr.targets) {
			if (ae instanceof Decorator) {
				dec.add(ae);
			}
		}
		return uniq(dec);
	}

	public ArrayList<GElement> getDecorators(GElement el) {
		ArrayList<GElement> dec = new ArrayList<GElement>();
		for (GElement e : this.arrays) {
			Arrow arr = (Arrow) e;
			for (GElement ae : arr.targets) {
				if (ae instanceof Decorator) {
					dec.add(ae);
				}
			}
		}
		return uniq(dec);
	}

	public ArrayList<GElement> getReversed(ArrayList<GElement> original) {
		ArrayList<GElement> copy = new ArrayList<GElement>(original);
		Collections.reverse(copy);
		return copy;
	}

	public ArrayList<GElement> getRoot() {
		ArrayList<GElement> root = new ArrayList<GElement>();
		for (GElement e : this.elements)
			if (e instanceof Task) {
				if (getArrow(null, e).size() == 0)
					root.add(e);
			}
		
		return root;
	}

	public String getShortFilePath() {
		if (this.absoluteFilePath == null) {
			return null;
		}

		String[] splitted = this.absoluteFilePath.split("/");
		
		if (splitted.length == 0)
			return null;
		
		return (_documentChanged ? "*" : "") + new String(splitted[splitted.length - 1]);
	}
	
	public String getFileNameOnly() {
		return getShortFilePath().replace("*", "");
	}

	public ArrayList<GElement> getSubElements(GElement el) {
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for (GElement e : this.arrays) {
			Arrow arr = (Arrow) e;
			if (arr.getSource() == el)
				subels.add(arr.getTarget());
		}
		Collections.sort(subels, new Comparator<GElement>() {
			@Override
			public int compare(GElement a, GElement b) {
				return (int) (a.getProperty().location.x - b.getProperty().location.x);
			}
		});
		return uniq(subels);
	}

	public ArrayList<GElement> getSubElementsOfOneLevel(GElement el) {
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for (GElement e : this.arrays) {
			Arrow arr = (Arrow) e;
			if (arr.getSource() == el) {
				subels.add(arr);
				subels.addAll(arr.targets);
			}else{
				boolean found = false;
				for( int i=0; i<arr.targets.size(); i++ ){
					if(arr.targets.get(i) == el){
						found = true;
					}
					if( found ){
						subels.add(arr.targets.get(i));
					}
				}
			}
		}
		return uniq(subels);
	}
	
	public boolean isDebugViewEnabled() {
		return this.mainWindow.getMenubar().getDebugViewMenuItem().isSelected();
	}
	
	public boolean isRuntimeViewEnabled() {
		return this.mainWindow.getMenubar().getRuntimeViewMenuItem().isSelected();
	}

	public ArrayList<GElement> getSuperElements(GElement el) {
		ArrayList<GElement> subels = new ArrayList<GElement>();
		for (GElement e : this.arrays) {
			Arrow arr = (Arrow) e;
			if (arr.getTarget() == el)
				subels.add(arr.getSource());
		}
		Collections.sort(uniq(subels), new Comparator<GElement>() {
			@Override
			public int compare(GElement a, GElement b) {
				return (int) (a.getProperty().location.x - b.getProperty().location.x);
			}
		});
		return subels;
	}

	private void hideCollapsed() {
		for (GElement e : this.elements)
			e.isVisiable = true;
		for (GElement e : this.arrays)
			e.isVisiable = true;
		for (GElement e : this.elements) {
			if (e.getProperty().collapsed) {
				for (GElement c : searchAllSubelements(e))
					c.isVisiable = false;
			}
		}
	}

	private void loadPlan(Element el, GElement rge) {
		// _buildTime = true;
		this.lastY += 40;
		NodeList nl = el.getChildNodes();
		if (nl != null && nl.getLength() > 0) {
			for (int i = 0; i < nl.getLength(); i++) {
				GElement ge = rge;
				if (nl.item(i).getNodeType() != Node.ELEMENT_NODE)
					continue;
				Element e = (Element) nl.item(i);
				if (ge instanceof Task) {
					Arrow a = new Arrow(ge, null);
					add(a);
					ge = a;
				}
				GElement nge = null;
				if (Parameters.enableLinkConnection
						&& e.hasAttribute("id")
						&& this.loadedElements.containsKey(UUID.fromString(
								e.getAttribute("id")).toString())) {
					nge = this.loadedElements.get(UUID.fromString(
							e.getAttribute("id")).toString());
					if (ge == null) {
						loadPlan(e, nge);
					} else if (ge instanceof Arrow) {
						GElement se = null;
						Arrow a = (Arrow) ge;
						a.add(nge);
						se = nge;
						loadPlan(e, se);
					}
				} else {
					String nodeName = e.getNodeName().toLowerCase();
					if (ge == null) {
						if (nodeName.equals("seq")
								|| nodeName.equals("sequenser")) {
							nge = new Task();
							((Task) nge).type = Task.TYPE_sequenser;
							((Task) nge).text = e.getAttribute("name");
						} else if (nodeName.equals("sel")
								|| nodeName.equals("selector")) {
							nge = new Task();
							((Task) nge).type = Task.TYPE_selector;
							((Task) nge).text = e.getAttribute("name");
						} else if (nodeName.equals("swi")
								|| nodeName.equals("switch")) {
							nge = new Task();
							((Task) nge).type = Task.TYPE_switch;
							((Task) nge).text = e.getAttribute("name");
						} else if (nodeName.equals("par")
								|| nodeName.equals("parallel")) {
							nge = new Task();
							((Task) nge).type = Task.TYPE_parallel;
							((Task) nge).text = e.getAttribute("name");
						} else if (nodeName.equals("tsk")
								|| nodeName.equals("task")) {
							nge = new Task();
							((Task) nge).type = Task.TYPE_task;
							((Task) nge).text = e.getAttribute("name");
						}
						if (e.hasAttribute("x") && e.hasAttribute("y")) {
							this.lastX = nge.getProperty().location.x = Double
									.parseDouble(e.getAttribute("x"));
							this.lastY = nge.getProperty().location.y = Double
									.parseDouble(e.getAttribute("y"));
						} else {
							this.lastX = nge.getProperty().location.x = this.lastX + 20;
							this.lastY = nge.getProperty().location.y = this.lastY;
						}
						if (e.hasAttribute("collapsed")) {
							nge.getProperty().collapsed = Boolean
									.parseBoolean(e.getAttribute("collapsed"));
						} else {
							nge.getProperty().collapsed = false;
						}
						if (e.hasAttribute("test_time")) {
							nge.getProperty().testTime = Integer.parseInt(e
									.getAttribute("test_time"));
						}
						if (e.hasAttribute("test_result")) {
							nge.getProperty().testResult = Boolean
									.parseBoolean(e.getAttribute("test_result"));
						}
						if (e.hasAttribute("id")) {
							nge.id = UUID.fromString(e.getAttribute("id"));
							this.loadedElements.put(nge.id.toString(), nge);
						}
						
						if (nge.isTask())
							nge.getAsTask().setDocument(this);
						
						add(nge);
						loadPlan(e, nge);
					} else if (ge instanceof Arrow) {
						GElement se = null;
						if (nodeName.equals("jnt") || nodeName.equals("joint")) {
							Arrow a = (Arrow) ge;
							nge = new Joint();
							a.add(nge);
							loadPlan(e, ge);
						} else if (nodeName.equals("dec")
								|| nodeName.equals("decorator")) {
							Arrow a = (Arrow) ge;
							nge = new Decorator();
							((Decorator) nge).text = e.getAttribute("name");
							a.add(nge);
							se = ge;
						} else if (nodeName.equals("seq")
								|| nodeName.equals("sequenser")) {
							Arrow a = (Arrow) ge;
							nge = new Task();
							((Task) nge).type = Task.TYPE_sequenser;
							((Task) nge).text = e.getAttribute("name");
							a.add(nge);
							se = nge;
						} else if (nodeName.equals("sel")
								|| nodeName.equals("seletor")) {
							Arrow a = (Arrow) ge;
							nge = new Task();
							((Task) nge).type = Task.TYPE_selector;
							((Task) nge).text = e.getAttribute("name");
							a.add(nge);
							se = nge;
						} else if (nodeName.equals("swi")
								|| nodeName.equals("switch")) {
							Arrow a = (Arrow) ge;
							nge = new Task();
							((Task) nge).type = Task.TYPE_switch;
							((Task) nge).text = e.getAttribute("name");
							a.add(nge);
							se = nge;
						} else if (nodeName.equals("par")
								|| nodeName.equals("parallel")) {
							Arrow a = (Arrow) ge;
							nge = new Task();
							((Task) nge).type = Task.TYPE_parallel;
							((Task) nge).text = e.getAttribute("name");
							a.add(nge);
							se = nge;
						} else if (nodeName.equals("tsk")
								|| nodeName.equals("task")) {
							Arrow a = (Arrow) ge;
							nge = new Task();
							((Task) nge).type = Task.TYPE_task;
							((Task) nge).text = e.getAttribute("name");
							a.add(nge);
							se = nge;
						}
						
						if (nge.isTask())
								nge.getAsTask().setDocument(this);
						
						add(nge);
						if (e.hasAttribute("x") && e.hasAttribute("y")) {
							this.lastX = nge.getProperty().location.x = Double
									.parseDouble(e.getAttribute("x"));
							this.lastY = nge.getProperty().location.y = Double
									.parseDouble(e.getAttribute("y"));
						} else {
							this.lastX = nge.getProperty().location.x = this.lastX + 20;
							this.lastY = nge.getProperty().location.y = this.lastY;
						}
						if (e.hasAttribute("collapsed")) {
							nge.getProperty().collapsed = Boolean
									.parseBoolean(e.getAttribute("collapsed"));
						} else {
							nge.getProperty().collapsed = false;
						}
						if (e.hasAttribute("test_time")) {
							nge.getProperty().testTime = Integer.parseInt(e
									.getAttribute("test_time"));
						}
						if (e.hasAttribute("test_result")) {
							nge.getProperty().testResult = Boolean
									.parseBoolean(e.getAttribute("test_result"));
						}
						if (e.hasAttribute("id")) {
							nge.id = UUID.fromString(e.getAttribute("id"));
							this.loadedElements.put(nge.id.toString(), nge);
						}
						loadPlan(e, se);
					}
				}
			}
		}

		// _buildTime = false;
	}

	public void loadPlan(String fileName, boolean updateCurrentWorkingFile) {
		String originalFilename = this.absoluteFilePath;
		
		if (!updateCurrentWorkingFile) {
			loadPlan(fileName);
			this.absoluteFilePath = originalFilename;
		} else 
			loadPlan(fileName);
	}
	
	public void loadPlan(String fileName) {
		this._buildTime = true;
		onBeforeDocumentLoad(fileName);

		setCurrentWorkingFile(fileName);
		File file = new File(fileName);
		org.w3c.dom.Document doc = null;
		try {
			doc = DocumentBuilderFactory.newInstance().newDocumentBuilder()
					.parse(file);
		} catch (javax.xml.parsers.ParserConfigurationException ex) {
			Log.e(ex);
		} catch (SAXException ex) {
			Log.e(ex);
		} catch (IOException ex) {
			Log.e(ex);
		}	

		toolSelectionClean();
		if (doc == null) {
			this.tip.setText("ERROR: Can't load plan file : "
					+ file.getAbsolutePath());
			this._buildTime = false;
			return;
		}
		this.elements.clear();
		this.arrays.clear();
		if (doc.getDocumentElement().getNodeName() != "plan") {
			this.tip.setText("ERROR: Xml file of plan does not contain plan tag");
			this._buildTime = false;
			return;
		}
		this.view.loc = new Vec(0, 0);
		this.view.zoom = 1;
		this.lastX = 0;
		this.lastY = 0;
		this.loadedElements.clear();

		this._taskDescriptionFilename = null;
		this._taskDescriptionFilenameOriginal = null;
		this._taskDescriptionExists = false;

		if (doc.getDocumentElement().hasAttribute("descriptions")) {
			try {
				// Descriptions file path is relative to select plan file
				this._taskDescriptionExists = true;
				this._taskDescriptionFilename = doc.getDocumentElement()
						.getAttribute("descriptions");
				this._taskDescriptionFilenameOriginal = this._taskDescriptionFilename;

				Log.d("Plan has a descriptions attribute = "
						+ this._taskDescriptionFilenameOriginal);

				// Relative path, need to combine with current path
				if (!this._taskDescriptionFilename.startsWith(".")
						&& !this._taskDescriptionFilename.startsWith("/"))
					this._taskDescriptionFilename = new File(
							new File(fileName).getParent(),
							this._taskDescriptionFilename).getPath();

				this._taskDescription = new TaskDescription(this._taskDescriptionFilename);
				
				Log.d("Task descriptions loaded from "
						+ this._taskDescriptionFilename);
			} catch (Exception e) {
				Log.d("WARNING: Can't find or open task description file. It's not a critical error.");
				Log.e("NOT CRITICAL : Print stack and exception name (for debug purposes only): ");
				this._taskDescription = new TaskDescription();
			}
		} else {

			try {
				Log.d("Plan has no descriptions attribute");

				this._taskDescriptionFilename = parsePlanPath(Parameters.path_to_description);

				if (!this._taskDescriptionFilename.startsWith(".")
						&& !this._taskDescriptionFilename.startsWith("/"))
					this._taskDescriptionFilename = new File(
							new File(fileName).getParent(),
							this._taskDescriptionFilename).getPath();

				this._taskDescription = new TaskDescription(
						this._taskDescriptionFilename);
				Log.d("Descriptions loaded from "
						+ this._taskDescriptionFilename);
			} catch (Exception e) {
				Log.d("WARNING: Can'tpaint find or open task description file. It's not a critical error.");
				Log.e("NOT CRITICAL : Print stack and exception name (for debug purposes only): ");
				Log.e(e);
				this._taskDescription = new TaskDescription();
				Log.d("Default task descriptions file not found, empty created.");
			}
		}
		loadPlan(doc.getDocumentElement(), null);
		repaint();
		this._buildTime = false;
		onDocumentLoad(fileName);
	}

	// **********************************************************************************
	// Events
	// **********************************************************************************
	
	boolean _treeChangeEvent = false;
	int _treeChangeNestingCounter = 0;
	private void onBeforeDocumentLoad(String fileName) {

	}

	void onBeforeTreeChange(TreeChangeType changeType, GElement element) {
		// Disable nested calls
		_treeChangeNestingCounter++;
		
		if (!_treeChangeEvent) 
			_treeChangeEvent = true;
	}
	
	void onTreeChange(TreeChangeType changeType, GElement element) {
		_treeChangeNestingCounter--;
		if (_treeChangeNestingCounter == 0)	
			_treeChangeEvent = false;
		
		if (_historyManager.isReady() && !_buildTime && !_treeChangeEvent)
			try {
				// if (!element.isArrow() || (element.isArrow() && _treeChangeNestingCounter == 0))
				_historyManager.createSnapshot();
			} catch (HistoryManagerNotReadyException e) {
				this.tip.setText("History manager create snapshot exception");
				Log.e("Snapshot create failed");
			}
		
		if (!_buildTime) {
			_documentChanged = true;
			updateTabTitle();
			updateRootElement();
		}
		
		/**
		 * Update lookup table overrides
		 */
		if (element.isTaskType()) {
			if (_lookupTable.containsTask(element.getAsTask().getNameWithoutParameters()))
				element.getAsTask().overrideTask(_lookupTable.getByTaskName(element.getAsTask().getNameWithoutParameters()));
		}
		
		updateUndoRedoButtonsState();
	}
	
	private void onTreeChange() {
		updateOverrides();
	}
	
	private void onBeforeTreeChange() {
		
	}

	private void onDocumentLoad(String fileName) {
		_documentChanged = false;
		updateTabTitle();
		updateRootElement();
		updateOverrides();		
		repaint();
	}
	
	private void onDocumentSave(boolean successfuly) {
		if (successfuly)
			_documentChanged = false;
		
		updateTabTitle();
	}
	
	public void onMessageReceive(StackStreamMessage message) {
		Task task = findTaskById(message.getTaskId());
		
		if (task == null)
			return;
		
		task.onMessageReceive(message);
		repaint();
	}
	
// 	@Deprecated
//	private void updateTabId() {
//		DesignerTab tab = this.mainWindow.getTabByDocument(this);
//		GElement rootElement;
//		if (this.getRoot().size() > 0 && tab != null) {
//			rootElement = this.getRoot().get(0);
//			tab.setID(rootElement.id.toString());
//			Log.d("Tab id = " + rootElement.id.toString());
//		}
//			
//	}
	
	private void updateOverrides() {
		for (GElement element : elements)
			if (element.isTaskType() && _lookupTable.containsTask(element.getAsTask().getNameWithoutParameters()))
				element.getAsTask().overrideTask(_lookupTable.getByTaskName(element.getAsTask().getNameWithoutParameters()));
	}
	
	PlanExecution _currentPlanExecution;
	public void onRun() {
		this.mainWindow.getTabByDocument(this).updateId();
		Log.d("Tab id = " + this.mainWindow.getTabByDocument(this).getID());
		
		for (GElement element : elements)
			if (element.isTaskType())
				element.getAsTask().onPlanRun();

		_planExecutionMessage = String.format("[%s] Running", new SimpleDateFormat("HH:mm:ss").format(new Date()) );
		_currentPlanExecution = new PlanExecution();
		
		repaint();
	}
	
	public void onPause() {
		
	}
	
	public void onStop() {
		Log.i("Plan execution aborted");
		_currentPlanExecution.stop(null, null, false, true);
		_executionResults.add(_currentPlanExecution);
		// _isRunning = false;
	}
	
	public void onStop(StopStreamMessage message) {
		Log.i("STOPSTREAM", "Plan execution finished [" + message.getFinishReason() + "]");
		
		if (_currentPlanExecution == null)
			_currentPlanExecution = new PlanExecution();
		
		_currentPlanExecution.stop(
				findTaskById(message.getTargetTaskId()),
				findTasksById(message.getTasksTree()),
				message.getFinishReason() == PlanFinishReason.Failure,  
				false);
	
		_executionResults.add(_currentPlanExecution);
		

		_planExecutionMessage = _currentPlanExecution.toString() + String.format("(%d)", message.getFinishCode());
		
		if (!message.getFinishReasonDescription().equals("")) 
			_planExecutionMessage += ":" + message.getFinishReasonDescription();
		
		if (_currentPlanExecution.isFailure())
			_planExecutionMessage = "$RED$" + _planExecutionMessage;
		
		repaint();
	}
	
	// **********************************************************************************
	// ** Misc **************************************************************************
	// **********************************************************************************
	
	private String _planExecutionMessage = "Idle";
	public String getPlanExecutionMessage() {
		return _planExecutionMessage;
	}
	

	public PlanExecutionCollection getPlanExecutionResults() {
		return _executionResults;
	}
	
	public PlanExecution getCurrentPlanExecution() {
		return _currentPlanExecution;
	}
	
	public TaskDescription getTaskDescriptionProvider() {
		return _taskDescription;
	}
	
	public void updateRootElement() {
		for (GElement element : elements)
			element.getProperty().isRoot = false;
		
		for (GElement element : getRoot())
			element.getProperty().isRoot = true;
	}
	
	public Task findTaskById(String taskId) {
		for (GElement e : elements)
			if (e.isTask())
				if (e.id.toString().equalsIgnoreCase(taskId))
					return e.getAsTask();
		
		return null;
	}
	
	public ArrayList<Task> findTasksById(ArrayList<String> ids) {
		ArrayList<Task> tasks = new ArrayList<Task>();
		
		for (String id : ids) {
			Task task = findTaskById(id);
			
			if (task != null)
				tasks.add(task);
		}
		
		return tasks;
	}
	
	public void undo() {
		onBeforeTreeChange();
		if (_historyManager.hasUndo())
			try {
				_historyManager.undo();
			} catch (Exception e) {
				Log.e(e);
				return; 
			} finally {
				onTreeChange();
			}
		
		_documentChanged = true;
		
		updateUndoRedoButtonsState();
		updateTabTitle();
	}
	
	public void redo() {
		onBeforeTreeChange();
		if (_historyManager.hasRedo())
			try {
				_historyManager.redo();
			} catch (Exception e) {
				Log.e(e);
				return; 
			} finally {
				onTreeChange();
			}
		
		_documentChanged = true;
		
		updateUndoRedoButtonsState();
		updateTabTitle();
	}

	private void updateUndoRedoButtonsState() {
		this.mainWindow.toolbar.setUndoButtonState(_historyManager.hasUndo());
		this.mainWindow.toolbar.setRedoButtonState(_historyManager.hasRedo());
	}
	
	private void updateTabTitle() {
		this.mainWindow.setTabName(this, getShortFilePath());
	}
	
	public void showHistory(JFrame designer) {
		new PlanExecutionHistoryDialog(designer, this);
	}
	
	@Override
	public void paint(Graphics g) {
		g.setColor(Color.lightGray);
		g.fillRect(0, 0, getWidth(), getHeight());
		Graphics2D g2d = (Graphics2D) g;
		g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
				RenderingHints.VALUE_ANTIALIAS_ON);
		if (this.view.graphics == null && g2d != null) {
			this.view.graphics = g2d;
			riceOnViewChange();
		}
		
		renumberElements(this.elements);
		hideCollapsed();
		paintElement(g2d, this.arrays);
		paintElement(g2d, this.elements);
		if (this.selectedElement != null)
			this.selectedElement.paint(g2d);

	}

	public void paintElement(Graphics2D g, ArrayList<GElement> elements) {
		for (GElement el : (elements))
			el.paintElement(g);
	}

	private String parsePlanPath(String pattern) {

		String planFileName = new File(getAbsoluteFilePath()).getName();
		planFileName = planFileName.replaceAll("\\.[^\\.]+$", "");
		return pattern.replace("{PLANFILENAME}", planFileName);
	}

	@SuppressWarnings("unchecked")
	public void remove(GElement el) {
		onBeforeTreeChange(TreeChangeType.Remove, el);

		if (el instanceof Task) {
			for (GElement e : (ArrayList<GElement>) this.arrays.clone()) {
				Arrow a = (Arrow) e;
				if (a.getTarget() == el || a.getSource() == el) {
					remove(e);
				}
			}
			this.elements.remove(el);
		}
		if (el instanceof Arrow) {
			Arrow a = (Arrow) el;
			ArrayList<GElement> forrem = new ArrayList<GElement>();
			for (int i = 0; i < a.targets.size() - 1; i++) {
				forrem.add(a.targets.get(i));
			}
			for (GElement e : forrem)
				remove(e);
			this.arrays.remove(el);
		}
		if (el instanceof Arrow.ArrayElement) {
			this.elements.remove(el);
			Arrow.ArrayElement a = (Arrow.ArrayElement) el;
			a.getArray().targets.remove(el);
		}

		onTreeChange(TreeChangeType.Remove, el);
	}

	public void removeSubTree(GElement el) {
		onBeforeTreeChange(TreeChangeType.SubTreeRemove, el);

		ArrayList<GElement> targets = searchAllSubelements(el);
		targets.add(el);
		for (GElement t : targets) {
			if (this.elements.contains(t) || this.arrays.contains(t))
				remove(t);
		}

		onTreeChange(TreeChangeType.SubTreeRemove, el);
	}

	public void renumberElements(ArrayList<GElement> elements) {
		for (GElement el : elements) {
			if (el instanceof Task) {
				((Task) el).seqNumber = 0;
			}
		}
		for (GElement el : elements) {
			if (el instanceof Task) {
				Task t = ((Task) el);
				if (t.type.equalsIgnoreCase(Task.TYPE_sequenser)
						|| t.type.equalsIgnoreCase(Task.TYPE_selector)) {
					int i = 1;
					for (GElement e : getSubElements(t)) {
						if (e instanceof Task) {
							((Task) e).seqNumber = i++;
						}
					}
				} else if (t.type.equalsIgnoreCase(Task.TYPE_switch)) {
					int i = 0;
					for (GElement e : getSubElements(t)) {
						if (e instanceof Task) {
							((Task) e).seqNumber = i++;
						}
					}
				}
			}
		}
	}

	private void riceOnViewChange() {
		for (GElement e : this.elements)
			if (e instanceof View.ChangesListener)
				((View.ChangesListener) e).onViewChange();
		for (GElement e : this.arrays)
			if (e instanceof View.ChangesListener)
				((View.ChangesListener) e).onViewChange();
	}

	private boolean saveXmlToFile(String xml, String fileName,
			boolean withNotifications) {
		String descriptionFileAttribute = "";

		// Add descriptions filename attribute
		if (this._taskDescriptionFilename != null
				&& !this._taskDescriptionFilename.equals("")
				&& this._taskDescriptionExists)
			descriptionFileAttribute = "descriptions=\""
					+ this._taskDescriptionFilenameOriginal + "\"";
		else
			// Plan hadn't description attribute specified
			descriptionFileAttribute = "descriptions=\""
					+ parsePlanPath(Parameters.path_to_description) + "\"";

		Log.d("description attribute added to plan = "
				+ descriptionFileAttribute);

		xml = "<plan"
				+ (descriptionFileAttribute.equals("") ? "" : " "
						+ descriptionFileAttribute) + ">\n" + xml + "\n</plan>";

		FileWriter w = null;
		File file = null;
		try {
			file = new File(fileName);
			w = new FileWriter(file);
			w.write(xml);
			w.close();
			if (withNotifications)
				this.tip.setText(this.tip.getText() + "; file created: "
						+ file.getAbsolutePath());
		} catch (IOException e) {
			if (withNotifications)
				this.tip.setText(this.tip.getText()
						+ "; ERROR: writing to file is fault. file: "
						+ file.getAbsolutePath() + ", " + e.toString());
			return false;
		}
		return true;
	}

	public ArrayList<GElement> searchAllSubelements(GElement el) {
		ArrayList<GElement> ret = uniq(searchAllSubelements(el,	new ArrayList<GElement>()));
		ret.remove(el);
		return ret;
	}

	public ArrayList<GElement> searchAllSubelements(GElement el,ArrayList<GElement> subels) {
		ArrayList<GElement> subels_onelevel = getSubElementsOfOneLevel(el);
		for (GElement e : subels_onelevel)
			if (subels.contains(e) == false) {
				subels.add(e);
				searchAllSubelements(e, subels);
			}
		return subels;
	}

	public ArrayList<GElement> searchAllSubelements(GElement el,boolean removeTopElement) {
		ArrayList<GElement> ret = uniq(searchAllSubelements(el,new ArrayList<GElement>()));
		if (removeTopElement)
			ret.remove(el);
		return ret;
	}

	public void setCurrentWorkingFile(String absoluteFilePath) {
		this.absoluteFilePath = absoluteFilePath;
	}

	public void setRunning(ArrayList<String> ids) {
		if(Parameters.log_print_running_tasks_id)
			Log.d("Select running tasks:");
		
		for (GElement e : this.elements){
			
			if(Parameters.log_print_running_tasks_id)
				Log.d("  select as running : "+ids+". ");
			
			if (ids.contains(e.id.toString())) {
				if(Parameters.log_print_running_tasks_id)
					Log.d("id="+e.id+" found.");
				
				e.getProperty().running = true;
			}else{
				if(Parameters.log_print_running_tasks_id)
					Log.d("not found.");
			}
		}
		repaint();
	}

	public String strDecProperties(Decorator d) {
		return d.getProperty().toString(this.view);
	}

	public String strJointProperties(Joint d) {
		return "";
	}

	public String strTaskProperties(Task root) {
		String rootId = root.id.toString();
		if (Parameters.enableTaskIdRegeneration) {
			if (this.savedIds.contains(rootId)) {
				rootId = GElement.getRandomUUID().toString();
			}
			this.savedIds.add(rootId);
		}
		if (root.type.equals(Task.TYPE_task))
			return root.getProperty().toStringForTask(this.view) + " id=\""
					+ rootId + "\"";
		return root.getProperty().toStringForSTask(this.view) + " id=\""
				+ rootId + "\"";
	}

	public void test() {
		if (this.BTExecuter != null) {
			this.BTExecuter.destroy();
			return;
		}
		compile();
		if (this.tip.getText().startsWith("ERROR"))
			return;
		(new Thread() {
			@Override
			public void run() {
				ArrayList<String> cmd = new ArrayList<String>();
				mainWindow.getMenubar().setDebugView();
				if (OSValidator.isUnix()) {
					String[] f1 = new String[] { "./BTExecuter-lin.exe",
							"./BTExecuter-lin.bin", "./BTExecuter-lin.a",
							"./BTExecuter-lin.run", "./BTExecuter" };
					File f = null;
					for (String ff : f1) {
						f = new File(ff);
						if (f.exists())
							break;
					}
					cmd.add(f.getAbsolutePath());
				} else if (OSValidator.isWindows())
					cmd.add("./BTExecuter-win.exe");
				// cmd.add("--test");
				cmd.add("-bt");
				cmd.add(Document.this.absoluteFilePath);
				cmd.add("-lu");
				cmd.add((new File(Parameters.path_to_lookup)).getAbsolutePath());
				cmd.add("--address");
				cmd.add((new File(Parameters.path_to_address))
						.getAbsolutePath());
				ProcessBuilder pb = new ProcessBuilder(cmd);
				try {
					Process p = pb.start();
					Document.this.BTExecuter = p;
					BufferedReader stdout = new BufferedReader(
							new InputStreamReader(p.getInputStream()));
					BufferedReader stderr = new BufferedReader(
							new InputStreamReader(p.getErrorStream()));
					String line;
					String plan = "";
					int c = 0;
					Document.this.tip
							.setText("Execution running... (press \"Run\" again for stop running)");
					while ((line = stdout.readLine()) != null) {
						Log.d("BTExecuter STDOUT: " + line);
						if (line.contains("plan{")) {
							plan += line;
							c++;
						} else if (plan.length() > 0) {
							plan += line;
							if (line.contains("{"))
								c++;
							if (line.contains("}"))
								c--;
							if (c == 0) {
								cleanRunning();
								setRunning(extractIds(plan));
								plan = "";
							}
						}
					}
					while ((line = stderr.readLine()) != null) {
						Log.d("BTExecuter STDERR: " + line);
					}
				} catch (IOException e) {
					Log.e(e);
					Document.this.tip.setText("ERROR: Can not run BTExecuter");
				}
				Document.this.BTExecuter = null;
				cleanRunning();
				Log.d("BTExecuter DONE");
				Document.this.tip.setText("Execution done.");
			}
		}).start();
	}

	public void toolSelectionClean() {
		this.creator = null;
		this.removeElement = false;
		this.removeSubElements = false;
		this.copyElement = false;
		this.reconectArrow = false;
		this.modifier = null;
		if (this.tip != null)
			this.tip.setText(Toolbar.TIP_move);
	}

	public ArrayList<GElement> uniq(ArrayList<GElement> arr) {
		for (int s = 0; s < arr.size(); s++) {
			for (int p = s + 1; p < arr.size(); p++) {
				if (arr.get(s) == arr.get(p)) {
					arr.remove(p);
					p--;
				}
			}
		}
		return arr;
	}

	public String xmlNameOfTask(String tname) {
		if (tname.equalsIgnoreCase(Task.TYPE_selector))
			return "sel";
		if (tname.equalsIgnoreCase(Task.TYPE_sequenser))
			return "seq";
		if (tname.equalsIgnoreCase(Task.TYPE_task))
			return "tsk";
		if (tname.equalsIgnoreCase(Task.TYPE_parallel))
			return "par";
		if (tname.equalsIgnoreCase(Task.TYPE_switch))
			return "swi";
		return "UNKNOWN_TASK_TYPE";
	}

	public boolean close() {
		if (_documentChanged) {
			int dialogResult = JOptionPane.showConfirmDialog(
					this.mainWindow, "Document '" + getShortFilePath().replace("*", "") + 
					"' has unsaved changes, save document before close?",
					"Document has unsaved changes",
					JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE);

			switch (dialogResult) {
			case JOptionPane.YES_OPTION:
				compile();
				return true;
			case JOptionPane.NO_OPTION:
				return true;
			case JOptionPane.CANCEL_OPTION:
				return false;
			}
		}
		
		return true;
	}

	public void showEditor() {
		new PlanEditor(this.mainWindow, this);
	}

}
