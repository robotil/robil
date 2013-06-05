package terminal.commands;

import java.io.IOException;
import java.util.ArrayList;

import logger.Log;

import terminal.Terminal;
import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.BatchLineProcessor;
import terminal.lineprocessors.ConsoleWriterLineProcessor;
import terminal.lineprocessors.LineProcessor;

public class ServiceCaller extends RosCommand {

	public ServiceCaller() {
		super(null, "call");
	}

	public ServiceCaller(Terminal terminal) {
		super(terminal, "call");

	}

	@Override
	public ArrayList<String> autocomplete(String command) {

		String[] args = command.split(" ");
		String service = "";
		ArrayList<String> ret = new ArrayList<String>();

		if (this.name.startsWith(command) && !command.equals(this.name)) {
			ret.add(this.name);
			return ret;
		}

		if (args.length >= 1) {
			try {
				this.thread = new Thread();

				// get available services list
				BatchLineProcessor processor = new BatchLineProcessor();

				initPipe(RosTargets.Service, processor, "list");
				this.pipe.sendAndReceive();
				String com = "";
				if (args.length > 0)
					com += args[0];
				if (args.length > 1)
					com += " " + args[1];
				ret = processor.getLines(com, this.name + " ", "");
				this.thread = null;

				for (String s : ret)
					Log.d(s);

			} catch (IOException ex) {
				this.sys.println("ERROR: " + ex.getMessage());
			} finally {
				this.thread = null;
			}

			if (args.length > 1) {
				boolean found = false;
				String com = args[0] + " " + args[1];
				for (String op : ret) {
					if (op.equals(com)) {
						found = true;
						break;
					}
				}
				if (found) {
					service = args[1];

					try {
						String[] type = new String[1];
						ArrayList<String> dataType = getServicesDataType(
								service, type);
						this.sys.println("\n\tData structure for service "
								+ service + "( " + type[0] + " )");
						for (String l : dataType) {
							this.sys.println("\t" + l);
						}
						if (args.length > 2) {
							ret.clear();
							ret.add(command);
						}
					} catch (IOException ex) {
						this.sys.println("ERROR: " + ex.getMessage());
					}

				}
			}

			// }
			// if (args.length > 2) {
		}

		return ret;
	}

	@SuppressWarnings("unused")
	public ArrayList<String> callService(String serviceName, String... args) {
		ArrayList<String> ret = new ArrayList<String>();

		if (serviceName == null) {
			return ret;
		}

		String cmdArguments = "";
		String spacer = "";
		String[] newargs = new String[args.length + 2];
		newargs[0] = "call";
		newargs[1] = serviceName;
		for (int i = 0, j = 2; i < args.length; i++, j++) {
			newargs[j] = args[i];
		}

		this.thread = new Thread();

		try {
			BatchLineProcessor processor = new BatchLineProcessor();
			// initPipe(RosTargets.Service, processor, "call",
			// serviceName, cmdArguments);
			initPipe(RosTargets.Service, processor, newargs);
			this.pipe.sendAndReceive();
			ret.addAll(processor.getLines());
		} catch (IOException ex) {
			Log.e(ex.getMessage());
		} finally {
			this.thread = null;
		}

		return ret;
	}

	@Override
	public void execute(String command) {
		if (isAlreadyRunning()) {
			this.sys.println("ERROR: The previous process is running");
			return;
		}
		String[] args = command.split(" ");
		if (args.length < 2) {
			this.sys.println("ERROR: name of service is missing.");
			return;
		}

		if (this.thread.isInterrupted()) {
			return;
		}

		String cmdArguments = "";
		String spacer = "";
		for (int i = 2; i < args.length; i++) {
			cmdArguments += spacer + args[i];
			spacer = " ";
		}
		String serviceName = args[1];

		LineProcessor processor = new ConsoleWriterLineProcessor(this.sys);

		if (this.thread.isInterrupted()) {
			return;
		}

		try {
			initPipe(RosTargets.Service, processor, "call", serviceName,
					cmdArguments);
			this.pipe.sendAndReceive();
		} catch (IOException ex) {
			this.sys.println(ex.getMessage());
		}

		return;
	}

	private ArrayList<String> getServicesDataType(String service,
			String[] ref_type) throws IOException {
		ArrayList<String> ret = new ArrayList<String>();

		// get available services list
		BatchLineProcessor processor = new BatchLineProcessor();

		this.thread = new Thread();

		// TODO: performed in two executions, can be done by one
		initPipe(RosTargets.Service, processor, "type", service);
		this.pipe.sendAndReceive();

		Log.d("DEBUG: services received: ");
		// translate type of service
		for (String type : processor.getLines()) {
			Log.d("DEBUG: " + type);
			ref_type[0] = type;
			BatchLineProcessor p = new BatchLineProcessor();
			initPipe(RosTargets.Rossrv, p, "show", type);
			this.pipe.sendAndReceive();

			for (String line : p.getLines()) {
				ret.add(line);
			}
			break;
		}

		this.thread = null;

		// get available services' type of input
		return ret;
	}

	@Override
	public boolean isAutoCompleteAvailable(String command) {
		return isPartOfName(command, this.name)
				|| command.startsWith(this.name);
	}

	@Override
	public void stop() {
		super.stop();
	}

}
