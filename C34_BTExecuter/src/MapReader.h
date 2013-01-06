/*
 * MapReader.h
 *
 *  Created on: Oct 6, 2012
 *      Author: dan
 */

#ifndef MAPREADER_H_
#define MAPREADER_H_

#include <string>
#include <map>

class MapReader {
public:
	typedef std::map<std::string,std::string> MAP;
	MapReader(std::string fname, std::string root, std::string items, std::string key, std::string value);
	MapReader(std::istream& fname, std::string root, std::string items, std::string key, std::string value);
	virtual ~MapReader();

	const MAP& getMap()const { return map; }

protected:
	MAP map;
};


#endif /* MAPREADER_H_ */
