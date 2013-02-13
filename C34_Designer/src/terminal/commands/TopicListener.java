package terminal.commands;

import java.io.IOException;
import java.util.ArrayList;

import terminal.lineprocessors.BatchLineProcessor;
import terminal.lineprocessors.ConsoleWriterLineProcessor;
import terminal.lineprocessors.LineProcessor;


import terminal.communication.RosPipe.RosTargets;

import terminal.Terminal;

public class TopicListener extends RosCommand {
//	Process process;

	public TopicListener(Terminal terminal) {
		super(terminal, "listener");

	}

	public boolean isAutoCompleteAvailable(String command) {
		return  isPartOfName(command, name)
				|| command.startsWith(name);
	}

	public ArrayList<String> autocomplete(String command) {
		
		String[] args = command.split(" ");
		String topic = "";
		ArrayList<String> ret = new ArrayList<String>();
		
		if (args.length > 1) {
			topic = args[1];
		}
		
		if (name.startsWith(command) && !command.equals(name)) {
			ret.add(name);
			return ret;
		}
		

		try {
			this.thread = new Thread();
			
			BatchLineProcessor processor = new BatchLineProcessor();
			initPipe(RosTargets.Topic, processor, "list");
			pipe.sendAndReceive();
			ret = processor.getLines(command, name+" ","");
		} catch (IOException ex) {
			sys.println("ERROR: " + ex.getMessage());
		} finally {
			this.thread = null;
		}
		
		return ret;
	}
	
//	public ArrayList<String> autocomplete1(String command) {
//		String[] args = command.split(" ");
//		ArrayList<String> ret = new ArrayList<String>();
//		
//		if (name.startsWith(command) && !command.equals(name)) {
//			ret.add(name);
//			return ret;
//		}
//
//		final String roscommand = "rostopic";
//		final String echo = "list";
//		String topic = "";
//		if (args.length > 1)
//			topic = args[1];
//		ProcessBuilder pb = new ProcessBuilder(roscommand, echo);
//		try {
//			process = pb.start();
//		} catch (IOException e) {
//			sys.println("ERROR: Can not open topic " + topic + ": "
//					+ e.getMessage());
//			for (StackTraceElement element : e.getStackTrace()) {
//				sys.println(String.format("\t%s", element));
//			}
//			process = null;
//		}
//		if (process == null) {
//			ret.add("listener");
//			return ret;
//		}
//
//		BufferedReader proc_stdout = new BufferedReader(new InputStreamReader(
//				process.getInputStream()));
//		String line = null;
//		try {
//			while ((line = proc_stdout.readLine()) != null) {
//				if (line.startsWith(topic))
//					ret.add("listener " + (line));
//			}
//		} catch (IOException e) {
//			sys.println("ERROR: reading problem " + topic + "\n"
//					+ e.getMessage());
//
//		} finally {
//			process = null;
//		}
//		return ret;
//	}

	public void stop() {
		super.stop();
		//pipe.stop();
	}

	public void execute(String command) {
		if (pipe.isAlreadyRunning()) {
			sys.println("ERROR: The previous process is running");
			return;
		}
		String[] args = command.split(" ");
		if (args.length < 2) {
			sys.println("ERROR: name of topic is missing.");
			return;
		}
		
		if (thread.isInterrupted()) {
			return;
		}
		
		final String roscommand = "rostopic";
		final String echo = "echo";
		final String topic = args[1];
		
		LineProcessor processor = new ConsoleWriterLineProcessor(this.sys);
		
		if (thread.isInterrupted()) {
			return;
		}

		try {
			initPipe(RosTargets.Topic, processor, "echo", topic);
			pipe.sendAndReceive();
		} catch (IOException ex) {
			sys.println(ex.getMessage());
		}
		
//		if (thread.isInterrupted())
//			return -1000;
//		BufferedReader proc_stdout = new BufferedReader(new InputStreamReader(
//				process.getInputStream()));
//		String line = null;
//		try {
//			while ((line = proc_stdout.readLine()) != null) {
//				if (thread.isInterrupted())
//					return -1000;
//				sys.println(line);
//			}
//		} catch (IOException e) {
//			sys.println("ERROR: reading problem " + topic + "\n"
//					+ e.getMessage());
//
//		} finally {
//			process = null;
//			try {
//				proc_stdout.close();
//			} catch (IOException e) {
//			}
//		}

		return;
	}
	
//	public int main1(String command) {
//		if (pipe.isAlreadyRunning()) {
//			sys.println("ERROR: The previous process is running");
//			return -1;
//		}
//		String[] args = command.split(" ");
//		if (args.length < 2) {
//			sys.println("ERROR: name of topic is missing.");
//			return 0;
//		}
//		
//		if (thread.isInterrupted()) {
//			return -1000;
//		}
//		
//		final String roscommand = "rostopic";
//		final String echo = "echo";
//		final String topic = args[1];
//		ProcessBuilder pb = new ProcessBuilder(roscommand, echo, topic);
//		if (thread.isInterrupted())
//			return -1000;
//		try {
//			process = pb.start();
//		} catch (IOException e) {
//			sys.println("ERROR: Can not open topic " + topic + ": "
//					+ e.getMessage());
//			for (StackTraceElement element : e.getStackTrace()) {
//				sys.println(String.format("\t%s", element));
//			}
//			// sys.println("ERROR: Can not open topic "+topic+"\n"+e.getMessage());
//			process = null;
//		}
//		if (process == null)
//			return -1;
//
//		if (thread.isInterrupted())
//			return -1000;
//		BufferedReader proc_stdout = new BufferedReader(new InputStreamReader(
//				process.getInputStream()));
//		String line = null;
//		try {
//			while ((line = proc_stdout.readLine()) != null) {
//				if (thread.isInterrupted())
//					return -1000;
//				sys.println(line);
//			}
//		} catch (IOException e) {
//			sys.println("ERROR: reading problem " + topic + "\n"
//					+ e.getMessage());
//
//		} finally {
//			process = null;
//			try {
//				proc_stdout.close();
//			} catch (IOException e) {
//			}
//		}
//
//		return 0;
//	}

}
