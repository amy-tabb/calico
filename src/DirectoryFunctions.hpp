/*
 * DirectoryFunctions.hpp
 *
 *  Created on: Dec 20, 2011
 *      Author: Amy Tabb
 */

#ifndef DIRECTORYFUNCTIONS_HPP_
#define DIRECTORYFUNCTIONS_HPP_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <vector>
#include <iostream>

using std::vector;
using std::string;


void ReadDirectory(string dir_name, vector<string>& content_names);

bool IsDirectory(string dir_name);

bool SortbyLength(string s0, string s1);


#endif /* DIRECTORYFUNCTIONS_HPP_ */
