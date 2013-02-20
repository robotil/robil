package document;

//ColorPane.java
//A simple extension of JTextPane that allows the user to easily append
//colored text to the document.
//

import java.awt.Color;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;

import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.text.AttributeSet;
import javax.swing.text.SimpleAttributeSet;
import javax.swing.text.StyleConstants;
import javax.swing.text.StyleContext;

import logger.LogManager;

public class LogConsoleWindow extends JTextPane {

	private static final long serialVersionUID = 7243421902696150132L;
	
	private static LogConsoleWindow pane = null;

	public static void show(String logFile) {
		// logFile = LogManager.getOutputFileName();

		if(pane==null){
			System.out.println("Create Log Window");
			pane = new LogConsoleWindow();
		}
		pane.frame.setVisible(true);
	
	}

	private String logFile;

	@SuppressWarnings("unused")
	private JScrollPane scroll;
	private JFrame frame;
	Thread thread = null;
	
	public LogConsoleWindow() {
		JScrollPane scroll = new JScrollPane(this);
		this.setScrollPane(scroll);

		frame = new JFrame("Log Console");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(scroll);
		frame.setSize(600, 400);
		frame.addWindowListener(new WindowListener() {
			private boolean isClosed = true;
			public void windowOpened(WindowEvent arg0) {}
			public void windowIconified(WindowEvent arg0) {}
			public void windowDeiconified(WindowEvent arg0) {}
			public void windowDeactivated(WindowEvent arg0) {}
			public void windowClosing(WindowEvent arg0) {
				System.out.println("Log Window Closed");
				isClosed = true;
				thread = null;
			}
			public void windowClosed(WindowEvent arg0) {}
			public void windowActivated(WindowEvent arg0) {
				if(isClosed){
					System.out.println("Log Window Open");
					isClosed = false;
					runThread();
				}
			}
		});
	}
	
	private void runThread(){
		new Thread() {
			@Override
			public void run() {
				thread = this;
				LogManager.LineCounter lastline = new LogManager.LineCounter(0);
				long vlines=0;
				while (true) {
					if(thread==null) return;
					long ll = lastline.number;
					String outputContent = LogManager.getOutputContent(lastline);
				
					if(outputContent.length()!=0){
						String currentContent = getText();
						currentContent = currentContent+outputContent;
						vlines+= lastline.number-ll;
						int linelimit = Parameters.log_preview_lines_limit;
						if(vlines > linelimit){
							int c=0, p=currentContent.indexOf('\n', 0), l=p;
							while(c<(vlines-linelimit) && p>=0){
								l=p;
								c++;
								p=currentContent.indexOf('\n', l+1);
							}
							vlines = vlines - c;
							currentContent = currentContent.substring(l+1);
						}
						setText(currentContent);
						
						setCaretPosition(getDocument().getLength());
					}

					try {
						//System.out.println("lastline "+(lastline.number));
						Thread.sleep(100);
					} catch (InterruptedException e) {
					}
				}
			}
		}.start();
	}

	private void append(Color c, String s, Object style) { // better
															// implementation--uses
		// StyleContext
		StyleContext sc = StyleContext.getDefaultStyleContext();
		AttributeSet aset = sc.addAttribute(SimpleAttributeSet.EMPTY, style, c);

		int len = getDocument().getLength(); // same value as
												// getText().length();
		setCaretPosition(len); // place caret at the end (with no selection)
		setCharacterAttributes(aset, false);
		replaceSelection(s); // there is no selection, so inserts at caret
	}

	public void appendNaive(Color c, String s) { // naive implementation
		// bad: instiantiates a new AttributeSet object on each call
		SimpleAttributeSet aset = new SimpleAttributeSet();
		StyleConstants.setForeground(aset, c);

		int len = getText().length();
		setCaretPosition(len); // place caret at the end (with no selection)
		setCharacterAttributes(aset, false);
		replaceSelection(s); // there is no selection, so inserts at caret
	}

	private void processData() {
		BufferedReader reader = null;
		try {
			reader = new BufferedReader(new InputStreamReader(
					new FileInputStream(this.logFile)));

			String line = null;
			while ((line = reader.readLine()) != null) {
				line.charAt(0);
			}

		} catch (FileNotFoundException ex) {
			JOptionPane.showMessageDialog(null, "Log file " + this.logFile
					+ " cannot be found", "Open Log Console",
					JOptionPane.ERROR_MESSAGE);
		} catch (IOException ex) {
			System.err.println("Error in reading logfile " + ex.getMessage());
		} finally {
			try {
				if (reader == null) {
					return;
				}

				reader.close();
			} catch (IOException ex) {
				System.err.println(ex.getMessage());
			}
		}
	}

	public void refreshData() {
		this.setText("");

		processData();
	}

	public void setScrollPane(JScrollPane scrollPane) {
		this.scroll = scrollPane;
	}

	@SuppressWarnings("unused")
	private void writeError(String txt) {
		append(Color.RED, txt + "\n", StyleConstants.Foreground);
	}

	@SuppressWarnings("unused")
	private void writeInformation(String txt) {
		append(Color.GREEN, txt + "\n", StyleConstants.Foreground);
	}

	@SuppressWarnings("unused")
	private void writeNormal(String txt) {
		append(Color.BLACK, txt + "\n", StyleConstants.Foreground);
	}

	@SuppressWarnings("unused")
	private void writePlanRun(String txt) {
		append(Color.YELLOW, txt + "\n", StyleConstants.Background);
	}

}