/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <behavior_tree.h>
#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <unistd.h>

std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree");
    try
    {
        int TickPeriod_milliseconds = 1000;
	//std::cout << getexepath() << std::endl;	
	std::ifstream I("/home/safoex/catkin_ws/devel/lib/behavior_tree_core/input.txt");
	std::string s;
	I >> s;
	std::cout << s << std::endl <<std::endl;
	std::cout << "1234566" << std::endl << std::endl;
        BT::ROSAction* action = new BT::ROSAction("action");
        BT::ROSCondition* condition = new BT::ROSCondition("condition");

	
    	std::cout << "HERE123" << std::endl;
        BT:: SequenceNode* sequence1 = new BT::SequenceNode("seq1");

        sequence1->AddChild(condition);
        sequence1->AddChild(action);
	
        Execute(sequence1, TickPeriod_milliseconds);  // from BehaviorTree.cpp
	
    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
