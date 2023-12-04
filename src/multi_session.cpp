#include "multi-session/Incremental_mapping.h"

std::string sessions_dir;
std::string central_sess_name;
std::string query_sess_name;
std::string save_directory;

int iteration;  // >= 4

int main(int argc, char** argv){
    ros::init(argc, argv, "multi-session");
    ros::NodeHandle nh;

    nh.param<std::string>("multi_session/sessions_dir", sessions_dir, " ");
    nh.param<std::string>("multi_session/central_sess_name", central_sess_name, " ");
    nh.param<std::string>("multi_session/query_sess_name", query_sess_name, " ");
    nh.param<std::string>("multi_session/save_directory", save_directory, " ");
    nh.param<int>("multi_session/iteration", iteration, 5);

    ROS_INFO("\033[1;32m----> multi-sessionstarts.\033[0m");
    
    // fsmkdir(sessions_dir);
    // fsmkdir(std::string(sessions_dir + central_sess_name));
    // fsmkdir(std::string(sessions_dir + query_sess_name));

    fsmkdir(std::string(sessions_dir + save_directory));

    MultiSession::IncreMapping multi_session(sessions_dir, central_sess_name, query_sess_name, save_directory);

    std::cout << "----------  current estimate -----------" << std::endl;
    multi_session.optimizeMultisesseionGraph(true, 0); // optimize the graph with existing edges 
    multi_session.writeAllSessionsTrajectories(std::string("bfr_intersession_loops"));

    multi_session.detectInterSessionSCloops(); // detectInterSessionRSloops was internally done while sc detection 
    multi_session.addSCloops();

    int t = 1;
    ros::Rate rate(0.2);
    bool status = ros::ok();
    while(status){
        ros::spinOnce();
        
        if(t <= iteration){
            std::cout << "----------  sc estimate -----------" << std::endl;
            multi_session.optimizeMultisesseionGraph(true, t); // optimize the graph with existing edges + SC loop edges
            multi_session.publish();
        }

        if(t > iteration){
            std::cout << "----------  rs estimate -----------" << std::endl;
            bool toOpt = multi_session.addRSloops();
            if(toOpt && (t <= iteration * 2)){
                multi_session.optimizeMultisesseionGraph(toOpt, t); // optimize the graph with existing edges + SC loop edges + RS loop edges
                multi_session.publish();
            }
        }

        t = t + 1;
        rate.sleep();
    }    
        
    multi_session.writeAllSessionsTrajectories(std::string("aft_intersession_loops"));

    return 0;
}