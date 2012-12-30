/*
 * BTExecuter.h
 *
 *  Created on: Sep 27, 2012
 *      Author: dan
 */

#ifndef BTEXECUTER_H_
#define BTEXECUTER_H_

#include "Logger.h"

#include "Node.h"
#include "BT.h"
#include "Result.h"
#include "ExecuterStatus.h"

#include "ExeStack.h"
#include "ExeEnergy.h"
#include "Lookup.h"
#include "TaskProxyTable.h"

#include <sstream>
#include <boost/thread.hpp>

#include <boost/filesystem.hpp>


class Params{
	std::map<std::string,std::string> m;
public:
	Params(int n, char** v);
	template<class T> T get(const std::string& key){
		if(contains(key)==false) return T();
		std::stringstream s;
		s<<m[key]; T v; s>>v;
		return v;
	}
	bool contains(const std::string& key);
	template<class T> T get(const std::string& key, T dv){
		if(contains(key)==false) return dv;
		std::stringstream s;
		s<<m[key]; T v; s>>v;
		return v;
	}
};

class BTExecuter{
public:
	typedef boost::shared_ptr<BTExecuter> Ref;
	typedef void (*FINISH_CALLBACK)(std::string name, std::string result);
	typedef void (*STACK_CALLBACK)(std::string sid, int code, std::string node, std::string state);

private:
	Params params;
	std::string name;
	boost::thread_group threads;

public:
	Node::Ref node;
	Result::Ref result;

	ExeEnergy::Ref energy;
	ExeEnergy::Ref taskenergy;
	Lookup::Ref lookup;
	ExeStack::Ref stack;
	TaskProxyTable::Ref taskproxy;
	bool debug;

	ExecuterStatus::Ref executerStatus;

	FINISH_CALLBACK finish_callback;

public:
	BTExecuter(int an, char** av);
	BTExecuter(int an, char** av, TaskProxyTable::Ref tp);

	BTExecuter(double e, double te, Lookup::Ref l, TaskProxyTable::Ref tp, BT bt, std::string name);

	void start();

	void run();

	void wait();
};

#include <map>
using namespace std;

class BTServer{

	map<string,BTExecuter::Ref> executers;
	bool exists(string id){ return executers.find(id)!=executers.end(); }

public:
	Lookup::Ref lookup;
	TaskProxyTable::Ref taskproxy;
	BTExecuter::FINISH_CALLBACK finishCallback;
	BTExecuter::STACK_CALLBACK stackCallback;

	BTServer();
	void start();
	void run(string id, string file);
	void stop(string id);
	void pause(string id);
	void resume(string id);
	void step(string id);
	void stack(string id);
	void dump(string id);
	std::string whoIsRunning();
};

class ServerActions{
	boost::filesystem::path cdir;
	BTServer& server;
public:
	ServerActions(BTServer& server);
	std::string pwd();
	std::string cd(std::string pathToNewFolder);
	std::string ls();
	void lookup(std::string filename);
	void address(std::string filename, BTTaskProxyCreator::Ref creator);
	void saveFile(std::string filename, std::string filetext);
	std::string readFile(std::string filename);
	void run(std::string id, std::string filename);
	std::string stop(std::string id);
	void pause(std::string id);
	void step(std::string id);
	void resume(std::string id);
	std::string stack(std::string id);
	void dump(std::string id);
	std::string help();
	std::string show_lookup();
	std::string show_address();
	std::string whoIsRunning();
	std::string version();
};


inline bool startWith(const std::string& line, const std::string& t){
	if(t.size()>line.size()) return false;
	if(t.size()==line.size()) return t==line;
	for(int i=0;i<t.size();i++){
		if(line[i]!=t[i]) return false;
	}
	return true;
}

#include <vector>
#include <algorithm>
#include <iterator>
typedef std::vector<std::string> stringlist;
inline stringlist split(std::string&  line){
	using namespace std;
	vector<string> tokens;
	istringstream iss(line);
	copy(istream_iterator<string>(iss),
			 istream_iterator<string>(),
			 back_inserter<vector<string> >(tokens));
	return tokens;
}

int BTMain(int argn, char** argv);

#endif /* BTEXECUTER_H_ */
