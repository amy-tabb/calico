/*
 * helper.hpp
 *
 *  Created on: Jul 31, 2019
 *      Author: atabb
 */

#ifndef HELPER_HPP_
#define HELPER_HPP_

#include "Includes.hpp"

template<class T>
std::string FormatWithCommas(T value)
{
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

string FindMultipleStringValueOfFieldInFile(const string& filename, const string& fieldTag, bool separator, bool kill_if_not_found, int number_items);

void EnsureDirHasTrailingBackslash(string& write_directory);

bool sort_by_x( const pair<Point2f, Point2f>& p0, const pair<Point2f, Point2f>& p1);

void TestStream(const ifstream& in, const string& filename);

void CopyFile(const string& source, const string& destination);

void TestStream(const ofstream& out, const string& filename);


#endif /* HELPER_HPP_ */
