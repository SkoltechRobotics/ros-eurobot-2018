#include <std_msgs/String.h>
#include <behavior_tree.h>
#include <iostream>
#include <fstream>
//#include <std_msgs/String.h>
#include <sstream>
#include <map>



std::map<std::string, int> bt_node_types;

void assign_bt_node_types() {
    int i = 0;
    for(auto s : {"action", "condition", "sequence"} )
        bt_node_types[s] = i++;
    
}

enum class NODE_TYPE {
    ACTION,
    CONDITION,
    SEQUENCE
};


void* createBTreeNode(std::string node_name, std::string node_type) {
    switch(bt_node_types[node_type]) {
        case (int)NODE_TYPE::ACTION:       return new BT::ROSAction(node_name);    break;
        case (int)NODE_TYPE::CONDITION:    return new BT::ROSCondition(node_name); break;
        case (int)NODE_TYPE::SEQUENCE:     return new BT::SequenceNode(node_name); break;
        //TODO :: other types
    }
        
}


std::pair<std::map<std::string, void*>, std::string> 
createBTree(const std::string& msg) {
    //first - just create BT
    
    std::map<std::string, void* > named_tree_nodes;

    std::istringstream description(msg);
    std::string tree_name;
    description >> tree_name;
    std::string root_name("NOT FOUND");
    std::cout << tree_name << std::endl; 
    std::string node_name, node_type, parent_name;
    for (; description >> node_name >> node_type >> parent_name ;) {
        
        
        std::cout << "nn : " << node_name << "nt : " << node_type << "pn : " << parent_name << std::endl;
        named_tree_nodes[node_name] = createBTreeNode(node_name, node_type);
        if(parent_name == "__ROOT__") {
            root_name = node_name;
        }
        else {
            if(named_tree_nodes.count(parent_name)) {
                BT::ControlNode* casted_parent = (BT::ControlNode*)named_tree_nodes[parent_name];
                void* not_casted_child = named_tree_nodes[node_name];
                switch(bt_node_types[node_type]){
                    case (int)NODE_TYPE::ACTION: casted_parent->AddChild((BT::ROSAction*)not_casted_child); break;
                    case (int)NODE_TYPE::CONDITION: casted_parent->AddChild((BT::ROSCondition*)not_casted_child); break;
                    case (int)NODE_TYPE::SEQUENCE: casted_parent->AddChild((BT::SequenceNode*)not_casted_child); break;
                }
                //TODO :: other types, proper C++ cast
            }
            else {
                //TODO :: re-write as exceptions
                return make_pair(named_tree_nodes, std::string("ERRORS FOUND"));
            }
        }
    }
    
    return make_pair(named_tree_nodes, root_name);
}


void startBTree(const std_msgs::String::ConstPtr& msg) {
    auto tree = createBTree(msg->data);
    std::cout << "I've done the tree" << std::endl;
    std::cout << tree.second << std::endl;
    int TickPeriod_milliseconds = 1000;
    if(tree.second != "NOT FOUND" && tree.second != "ERRORS FOUND") {
        Execute((BT::ControlNode*)tree.first[tree.second], TickPeriod_milliseconds); // from BehaviorTree.cpp
    }
}

int main(int argc, char **argv){ 
    
    assign_bt_node_types();

    ros::init(argc, argv, "TreeMaker");
    
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("bt_maker", 1000, startBTree);
    
    ros::spin();
    return 0;
}
