#include <catenary_checker/catenary_checker_manager.h>

int main(int argc, char **argv)
{
    std::string node_name = "catenary_checker_manager_node";

	ros::init(argc, argv, node_name);

    std::cout << "Initializing node: " << node_name << std::endl;
    CatenaryCheckerManager checker(node_name);

	ros::Rate loop_rate(5);

    while(ros::ok()){

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
 