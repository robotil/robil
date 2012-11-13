/*
 * BTExecuter.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "BTExecuter.h"
#include "StringOperations.h"
#include "TaskProxyTableXML.h"


	Params::Params(int n, char** v){
		std::string key="";
		std::string def="#";
		for(int i=0;i<n;i++){
			std::string value(v[i]);
			if(v[i][0]!='-' && key.size()>0){ m[key]=value; key=""; }
			else if(v[i][0]!='-'){ m[def]=value; def+="#"; }
			else if(value.size()==1){ key="-"; m[key]="true"; }
			else if(v[i][1]=='-'){ key=std::string(v[i]+2); m[key]="true";}
			else { key=std::string(v[i]+1); m[key]="true";}
		}
	}
	bool Params::contains(const std::string& key){
		return m.find(key)!=m.end();
	}



	BTExecuter::BTExecuter(int an, char** av)
	:params(an,av), name("console"), stack(new ExeStack("plan")), finish_callback(0)
	{
		std::string & _executer_id = name;
		if(params.contains("bt")==false){
			log<<"BTExecuter: there is no -bt parameter";
			return;
		}

		if(params.get<std::string>("test","false")=="true"){
			log<<"BT="<<params.get<std::string>("bt");
			if(params.contains("lu")){
				log<<"LOOKUP="<<params.get<std::string>("lu");
			}
			return;
		}

		energy=ExeEnergy::Ref( new ExeEnergy() );
		energy->setContinuously(1000);

		taskenergy=ExeEnergy::Ref( new ExeEnergy() );
		taskenergy->setContinuously(0);

		std::string bt_file;
		bt_file = params.get<std::string>("bt");

		BT bt( bt_file );
		if(params.contains("lu")){
			lookup= Lookup::Ref(new Lookup(params.get<std::string>("lu")));
		}else{
			lookup= Lookup::Ref(new Lookup());
		}

		debug = true;

		node = Node::createNode(lookup, bt);
		node->setExecuterId(name);
		node->setLookup(lookup);

		node->setOutputDebugStream(debug);
		node->setEnergy(energy);
		node->setEnergyForTask(taskenergy);

		node->setStack(stack);
	}

	BTExecuter::BTExecuter(int an, char** av, TaskProxyTable::Ref tp)
	:params(an,av), name("console"), stack(new ExeStack("plan")), finish_callback(0)
	{
		std::string & _executer_id = name;
		if(params.contains("bt")==false){
			log<<"BTExecuter: there is no -bt parameter";
			return;
		}

		if(params.get<std::string>("test","false")=="true"){
			log<<"BT="<<params.get<std::string>("bt");
			if(params.contains("lu")){
				log<<"LOOKUP="<<params.get<std::string>("lu");
			}
			return;
		}

		energy=ExeEnergy::Ref( new ExeEnergy() );
		energy->setContinuously(1000);

		taskenergy=ExeEnergy::Ref( new ExeEnergy() );
		taskenergy->setContinuously(0);

		std::string bt_file;
		bt_file = params.get<std::string>("bt");

		BT bt( bt_file );
		if(params.contains("lu")){
			lookup= Lookup::Ref(new Lookup(params.get<std::string>("lu")));
		}else{
			lookup= Lookup::Ref(new Lookup());
		}

		debug = true;

		node = Node::createNode(lookup, bt);
		node->setExecuterId(name);
		node->setLookup(lookup);
		node->setTaskProxy(tp);

		node->setOutputDebugStream(debug);
		node->setEnergy(energy);
		node->setEnergyForTask(taskenergy);

		node->setStack(stack);
	}

	BTExecuter::BTExecuter(double e, double te, Lookup::Ref l, TaskProxyTable::Ref tp, BT bt, std::string name)
	:params(0,0), name(name), stack(new ExeStack("plan")), finish_callback(0)
	{
		std::string & _executer_id = name;
		energy=ExeEnergy::Ref( new ExeEnergy() );
		energy->setContinuously(e);
		//energy->setStepMode();

		taskenergy=ExeEnergy::Ref( new ExeEnergy() );
		taskenergy->setContinuously(te);

		lookup = l;
		taskproxy = tp;

		node = Node::createNode(lookup, bt);
		node->setExecuterId(name);
		node->setLookup(lookup);
		node->setTaskProxy(taskproxy);

		node->setOutputDebugStream(debug);
		node->setEnergy(energy);
		node->setEnergyForTask(taskenergy);

		node->setStack(stack);
	}

	void BTExecuter::start(){
		threads.add_thread(new boost::thread( boost::bind(&BTExecuter::run, this) ));
	}

	void BTExecuter::run(){
		result = node->run();
		node->removeFromStack();
		{Logger l(name); l<<"BT "<< name <<" is finished \n";
		stack->printDown(l);}
		std::stringstream res; result->print(res);
		if(finish_callback) finish_callback(name, res.str());
	}

	void BTExecuter::wait(){
		std::string & _executer_id = name;

		log<<"wait while tree has been stopped.";
		threads.join_all();

		{Logger l(name); l<<"Result: \n"; result->print(l);}
	}


using namespace std;


	BTServer::BTServer(){
		lookup = Lookup::Ref(new Lookup());
		finishCallback = 0;
	}
	void BTServer::start(){}
	void BTServer::run(string id, string file){
		if(exists(id)) return;
		{logID(id)<<"run "<<id;}
		executers[id]=BTExecuter::Ref( new BTExecuter(100,0, lookup, taskproxy, file, id) );
		if(finishCallback) executers[id]->finish_callback = finishCallback;
		if(stackCallback ) executers[id]->stack->setOnChangeNotification(id, stackCallback);
		executers[id]->energy->setStepMode();/*<<<<< for debuging */
		executers[id]->start();
	}
	void BTServer::stop(string id){
		if(!exists(id)) return;
		{logID(id)<<"stop "<<id;}
		executers[id]->node->terminate("custom");
		executers[id]->energy->setContinuously(0);
		executers[id]->energy->wake();
		executers[id]->wait();
		executers.erase(id);
	}
	void BTServer::pause(string id){
		if(!exists(id)) return;
		executers[id]->energy->setStepMode();
		{logID(id)<<"pause of "<<id;}
	}
	void BTServer::resume(string id){
		if(!exists(id)) return;
		{logID(id)<<"resume "<<id;}
		executers[id]->energy->setContinuously(100);
		executers[id]->energy->wake();
	}
	void BTServer::step(string id){
		if(!exists(id)) return;
		executers[id]->energy->setStep();
		{logID(id)<<"step for "<<id;}
	}
	void BTServer::stack(string id){
		if(!exists(id)) return;
		{Logger l(id); l<<"stack print for "<<id<<"\n";executers[id]->stack->printDown(l);}
	}

	void BTServer::dump(string id){
		if(!exists(id)) return;
		executers[id]->debug = !(executers[id]->debug);
		{logID(id)<<"dump for "<<id<<" is "<<(!(executers[id]->debug)?"off":"on");}
	}


ServerActions::ServerActions(BTServer& serv):server(serv){
	cdir=boost::filesystem::initial_path();
}

std::string ServerActions::help(){
	std::stringstream s;
	s<<"Help:";
	s<<"\n\t"<<"pwd - get current directory";
	s<<"\n\t"<<"cd FOLDER - change directory to FOLDER. FOLDER is a relative path to new location (may be ..)";
	s<<"\n\t"<<"ls - list of file in current directory";
	s<<"\n\t"<<"lookup FILE - load lookup file";
	s<<"\n\t"<<"address FILE - load file with <task name, task address> map";
	s<<"\n\t"<<"run ID FILE - run BT in FILE with ID";
	s<<"\n\t"<<"pause ID - pause BT execution";
	s<<"\n\t"<<"resume ID - continue execution of BT";
	s<<"\n\t"<<"step ID - make step of BT executer";
	s<<"\n\t"<<"stop ID - stop BT execution and print result";
	s<<"\n\t"<<"dump ID - turn off/on debug printing of BT";
	s<<"\n\t"<<"stack ID - print stack state of BT";
	s<<"\n\t"<<"show_lookup - print content of lookup table";
	s<<"\n\t"<<"show_address - print content of task name-address association table";
	s<<"\n";
	//s<<"\t"<<"-------------\n\t Copyright Cogniteam @ 2012 for ROBIL project\n";
	return s.str();
}

std::string ServerActions::pwd(){
	std::stringstream s; s<<cdir; return s.str();
}

std::string ServerActions::cd(std::string pathToFolder){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	path orig = cdir;

	if(pathToFolder==".."){
		cdir = cdir.parent_path();
	}else if(pathToFolder=="."){

	}else if(pathToFolder[0]=='/'){
		cdir = path(pathToFolder);
	}else{
		cdir = cdir / pathToFolder;
	}

	if(exists(cdir)==false) cdir = orig;

	return pwd();
}

std::string ServerActions::ls(){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;
	stringstream cout;
	path p = cdir;
	try{
	    if (exists(p)){
	      if (is_regular_file(p))  cout << p << " size is " << file_size(p) << '\n';
	      else if (is_directory(p)){
	        cout << p << " is a directory containing:\n";
	        typedef vector<path> vec;             // store paths,
	        vec v;                                // so we can sort them later
	        copy(directory_iterator(p), directory_iterator(), back_inserter(v));
	        sort(v.begin(), v.end());             // sort, since directory iteration
	        for (vec::const_iterator it (v.begin()); it != v.end(); ++it){
	          cout << "   " << it->filename() << '\n';
	        }
	      }
	      else{
	    	  stringstream err;
	    	  err << p << " exists, but is neither a regular file nor a directory";
	    	  throw err.str();
	      }
	    }
	    else{
	    	stringstream err;
	    	err << p << " does not exist";
	    	throw err.str();
	    }
	}catch (const filesystem_error& ex){
		throw std::string(ex.what());
	}
	return cout.str();
}

void ServerActions::lookup(std::string filename){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;
	if(string_operations::startWith(filename, "<lookup") || string_operations::startWith(filename, "<?xml")){
		std::stringstream xml; xml << filename;
		server.lookup = Lookup::Ref(new Lookup(xml));
	}else{
		path p(filename);
		if(filename[0]!='/') p = cdir / p;
		if(exists(p)==false || is_regular_file(p)==false){
			stringstream err; err<<"error: "<<p<<" is not file";
			throw err.str();
		}
		server.lookup = Lookup::Ref(new Lookup(p.string()));
	}
}

void ServerActions::address(std::string filename, BTTaskProxyCreator::Ref creator){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;
	if(string_operations::startWith(filename, "<tasks") || string_operations::startWith(filename, "<?xml")){
		std::cout<<"--- ServerActions::address. from stream ---"<<std::endl;
		std::stringstream xml; xml << filename;
		server.taskproxy = TaskProxyTable::Ref(new TaskProxyTableXML(xml, creator));
	}else{
		std::cout<<"--- ServerActions::address. from file ---"<<std::endl;
		path p(filename);
		if(filename[0]!='/') p = cdir / p;
		if(exists(p)==false || is_regular_file(p)==false){
			stringstream err; err<<"error: "<<p<<" is not file";
			throw err.str();
		}
		server.taskproxy = TaskProxyTable::Ref(new TaskProxyTableXML(p.string(), creator));
	}
}

std::string ServerActions::show_lookup(){
	if(server.lookup.get()){
		return server.lookup->str();
	}
	return "LOOKUP EMPTY";
}
std::string ServerActions::show_address(){
	if(server.taskproxy.get()){
		return server.taskproxy->str();
	}
	return "ADDRESS TABLE EMPTY";
}

void ServerActions::run(std::string id, std::string filename){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;
	if(string_operations::startWith(filename, "<lookup") || string_operations::startWith(filename, "<?xml")){
		server.run(id,filename);
	}else{
		path p(filename);
		if(filename[0]!='/') p = cdir / p;
		if(exists(p)==false || is_regular_file(p)==false){
			stringstream err; err<<"error: "<<p<<" is not file";
			throw err.str();
		}
		server.run(id,p.string());
	}
}

std::string ServerActions::stop(std::string id){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	server.stop(id);
	return logSystem.get();
}
void ServerActions::pause(std::string id){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	server.pause(id);
}
void ServerActions::step(std::string id){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	server.step(id);
}
void ServerActions::resume(std::string id){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	server.resume(id);
}
void ServerActions::dump(std::string id){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	server.dump(id);
}
std::string ServerActions::stack(std::string id){
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	server.stack(id);
	return logSystem.get();
}

#include "TestTaskProxyTable.h"

int BTMain(int argn, char** argv){
	Params params(argn,argv);
	if(params.contains("bt")){

		BTExecuter::Ref executer;
		if(params.contains("proxy")){
			executer = BTExecuter::Ref(
					new BTExecuter( argn, argv, TaskProxyTable::Ref(new TestTaskProxyTable()) )
			);
		}else{
			executer = BTExecuter::Ref(new BTExecuter(argn,argv));
		}
		executer->start();
		executer->wait();
		return 0;

	}

	BTServer server;
	server.start();

	std::string line;
	boost::filesystem::path cdir=boost::filesystem::initial_path();
	namespace fs = boost::filesystem;
	using namespace std;
	using namespace fs;

	ServerActions actions(server);
#ifdef log
#undef log
#endif
#define log std::cout
	while(line!="exit"){
		std::getline(std::cin, line);
		if(line=="pwd"){
			try{
				std::string res = actions.pwd(); log<<res;
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(line=="help"){
			try{
				std::string res = actions.pwd(); log<<res;
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"cd ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write new folder name"; continue; }
			try{
				std::string res = actions.cd(par[1]); log<<res;
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(line=="ls"){
			try{
				std::string res = actions.ls(); log<<res;
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"lookup ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write lookup table"; continue; }
			try{
				actions.lookup(par[1]);
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"run ")){
			stringlist par = split(line);
			if(par.size()<3){ log<<"error: write behavior tree id and file"; continue; }
			try{
				actions.run(par[1], par[2]);
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"stop ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write behavior tree id"; continue; }
			try{
				std::string res = actions.stop(par[1]); log<<res;
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"pause ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write behavior tree id"; continue; }
			try{
				actions.pause(par[1]);
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"step ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write behavior tree id"; continue; }
			try{
				actions.step(par[1]);
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"resume ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write behavior tree id"; continue; }
			try{
				actions.resume(par[1]);
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"dump ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write behavior tree id"; continue; }
			try{
				actions.dump(par[1]);
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
		if(startWith(line,"stack ")){
			stringlist par = split(line);
			if(par.size()<2){ log<<"error: write behavior tree id"; continue; }
			try{
				std::string res = actions.stack(par[1]); log<<res;
			}catch(std::string& err){
				log<<"ERROR: "<<err;
			}
			continue;
		}
	}
}

