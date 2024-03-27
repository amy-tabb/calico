/*
 * helper.hpp
 *
 *  Created on: Jul 31, 2019
 *      Author: atabb
 */

#ifndef HELPER_HPP_
#define HELPER_HPP_

#include "Includes.hpp"

class profileInfo{
public:
    int doProfile = 0;
    string file = "temp.txt";
    ofstream stream;

    profileInfo();
    profileInfo(string f);

};

extern profileInfo profileClass;

template<class T>
std::string FormatWithCommas(T value)
{
    if (profileClass.doProfile){
          #pragma omp critical
                  {
                      profileClass.stream << "PROFILE  helper.hpp, line " << __LINE__ << endl;
                  }
              }
	uint64_t uvalue = value;
	bool negative = false;
	if (value < 0){
		negative = true;
		uvalue = -value;
	}


	string s;
	int cnt = 0;
	do
	{
		s.insert(0, 1, char('0' + uvalue % 10));
		uvalue /= 10;
		if (++cnt == 3 && uvalue)
		{
			s.insert(0, 1, ',');
			cnt = 0;
		}
	} while (uvalue);

	if (negative){
		s = "-" + s;
	}
	return s;
}


string FindValueOfFieldInFile(const string& filename, const string& fieldTag, bool separator, bool kill_if_not_found);

void EnsureDirHasTrailingBackslash(string& write_directory);

bool CheckExistenceOfDirectory(string write_directory);

void TestStream(const ifstream& in, const string& filename);

void CopyFile(const string& source, const string& destination);

void TestStream(const ofstream& out, const string& filename);


#endif /* HELPER_HPP_ */
