#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include "ros/time.h"
#include "path_follower/SemCamera.h"
#include <iostream>
#include <fstream>
#include <string>
#include "path_follower/approx_marginals.h"

#include <sstream>

#define BUFFER 100

geometry_msgs::Pose g_initial_pose;
message_filters::Cache<geometry_msgs::PoseStamped> g_pose_cache(BUFFER);
message_filters::Cache<path_follower::SemCamera> g_camera_cache(BUFFER);
path_follower::SemCamera g_camera;
static const int min_quality = 138; // max 425 



/** Counts the number of occurences of a substring in a string.
    @param str String 
    @param sub Substring
    @return The number of occurences.
*/
int visible_objects(std::string str, std::string sub)
{
        int sl = sub.length();
        if (str.length() == 0) return 0;
        int count = 0;
        for (size_t offset = str.find(sub); offset != std::string::npos;
             offset = str.find(sub, offset + sl))
        {
                ++count;
        }
        return count;
}

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
#ifdef DEBUG
        geometry_msgs::Point coord = msg->pose.position; 
       
        ROS_INFO("Oldest time cached is %f", g_pose_cache.getOldestTime().toSec());
        ROS_INFO("Last time received is %f", g_pose_cache.getLatestTime().toSec());
        ROS_INFO("Current position: (%g, %g, %g) \n", coord.x, coord.y, coord.z);
#endif
        g_pose_cache.add(msg);      
}


// NB: should be implemented as a cache with std_msgs::String::ConstPtr
// or with the msg SemCamera, which has an header
// but it keeps telling me that it cannot add such an object to
// the cache, so I dunno how to do that.
void cameraCallback(const std_msgs::String& msg)
{
#ifdef DEBUG
        ROS_INFO("Camera output: %s", msg.data.data());
#endif

         path_follower::SemCamera cmsg;
         cmsg.data = msg.data;
         cmsg.num = visible_objects( msg.data, std::string("Weed"));
        // cmsg.header.stamp = ros::Time::now();
        // cmsg.data = msg->data.data();

        // geometry_msgs::Point coord;
        // coord.x = position.at(0);
        // coord.y = position.at(1);
        // coord.z = position.at(2);

//        geometry_msgs::Pose msg;
//        msg.position = coord;
        //      return new geometry_msgs::Pose(msg);

        // visible_objects();
        
           g_camera =  cmsg;
//      g_camera_cache.add(cmsg);
      
}



geometry_msgs::Pose* get_pose(std::string s)
{

        //        
        std::vector<double>position;
        std::string delimiter = ",";
        size_t pos = 0;
        std::string token;

        while ((pos = s.find(delimiter)) != std::string::npos) {
                token = s.substr(0, pos);

                position.push_back( atof(token.c_str()) );
                // std::cout << atof(token.c_str())  << std::endl;

                s.erase(0, pos + delimiter.length());
        }

        geometry_msgs::Point coord;
        coord.x = position.at(0);
        coord.y = position.at(1);
        coord.z = position.at(2);

        geometry_msgs::Pose msg;
        msg.position = coord;
        return new geometry_msgs::Pose(msg);
}

/**
   Stores the waypoints in a vector.
   @param filename File containing the waypoints list.
   @param realist A vector to store the waypoints.
*/
void read_path(const char* filename, std::vector<geometry_msgs::Pose>& readlist)
{
        std::string current_line;
        std::ifstream infile;
	infile.open (filename);
        while(getline(infile,current_line))
        {
                 // Saves the line in current_line.
                ROS_INFO("Reading position: (%s)", current_line.c_str()); // Prints out
                readlist.push_back( *get_pose(current_line) );
        }
	infile.close();
//	system ("pause");
}


/**
   Checks if the specified waypoint has been reached.
   @param w The waypoint to be reached.
   @return True if current position is at w
*/
bool at_waypoint(geometry_msgs::Pose& w)
{
        const float e = 1.0;

        if (g_pose_cache.getElemAfterTime( g_pose_cache.getOldestTime() ) != NULL) 
        {
                boost::shared_ptr<const geometry_msgs::PoseStamped> p = g_pose_cache.getElemAfterTime( g_pose_cache.getOldestTime() );
#ifdef DEBUG
                ROS_INFO("        at position: (%g, %g, %g)", p->pose.position.x, p->pose.position.y, p->pose.position.z);
                ROS_INFO("        going to position: (%g, %g, %g)", w.position.x, w.position.y, w.position.z);
#endif               
                return ( (abs(p->pose.position.x - w.position.x) < e) && (abs(p->pose.position.y - w.position.y) < e) && (abs(p->pose.position.z - w.position.z) < e));
        }
        return false;
}



/**
   Prints command line usage.
   @param exec the command
*/
void usage(char *exec)
{
        //        std::cout << "Approximate inference algorithms  by A.Albore. Implementation of Libdai." << std::endl;
#ifdef MANY_ALGORITMS
        std::cout << "Usage: " << exec << " <-w|-c> <file> [algorithm]" << std::endl;
        std::cout << "-w : waypoint mode, specify waypoints file. (def. false)" << std::endl;
        std::cout << "-c : camera mode, specify fg and algo files." << std::endl;
        std::cout << "<file>: file of the waypoints or Factor Graph." << std::endl;
        std::cout << "[algorithm]: algorithm used (def. bp)." << std::endl;
#else
        std::cout << "Usage: " << exec << " <-w|-c> <filename>"  << std::endl; 
        std::cout << "   -w : waypoint mode, specify waypoints file. (def. false)" << std::endl;
        std::cout << "   -c : camera mode, specify .fg file." << std::endl;
        std::cout << "   <file>: file of the waypoints or Factor Graph." << std::endl;
#endif
        std::cout << std::endl;
        exit(0);
}


/** 
    Calculates the trajectory based on an adaptive sampling algorithm.
    * The parameter k
    @param a The algorithm to calculate the marginals, next sampling Pose and the Gibbs samples.
    @return The number of waypoints visited.
*/
int adaptive_sampling(ApproxMarginals& a, ros::Publisher &motion, ros::Rate& loop_rate )
{
        /**
         * A count of how many messages we have sent. This is used to create
         * a unique string for each message.
         */
        int count = 0;
        int timelimit = 50;
        geometry_msgs::Pose msg = g_initial_pose; //NB: can pus as well an initial elevate position
        std_msgs::String semantic_camera;

        // assert(!waypoints_mode);

        ROS_INFO("Ready");

        while (ros::ok())
        {
                /* AA
                 * here 1. gets the observation
                 * 2. updates the FG
                 * 3. gets the next waypoint and sets msg=next_waypoint()
                 */
                
                // AA: Here can initialize the FG with some samplings taken from file or from movements arount the initial position.

                //        ROS_INFO("%s", msg.data.c_str());
                /**
                 * The publish() function is how you send messages. The parameter
                 * is the message object. The type of this object must agree with the type
                 * given as a template parameter to the advertise<>() call, as was done
                 * in the constructor above.
                 */
                ROS_INFO("Moving toward: (%g, %g, %g)", msg.position.x, msg.position.y, msg.position.z);
                motion.publish(msg);

                while ( !at_waypoint(msg) ) 
                {
                        ros::spinOnce();
                        loop_rate.sleep();
                }

                ROS_INFO("Reached position: (%g, %g, %g)", msg.position.x, msg.position.y, msg.position.z, count);
                ++count;
                ros::spinOnce();
                loop_rate.sleep();

                // Using semantic camera
                
                //Get the time and store it in the time variable.
                ros::Time time = ros::Time::now();
                //Wait a duration of 3 seconds to stabilize the camera.
                ros::Duration d = ros::Duration(3, 0);
                d.sleep();

                ROS_INFO("Camera output: %s", g_camera.data.data());
                ROS_INFO("Camera output: %d", g_camera.num);

                a.evidence(msg, g_camera.num);

                cout << "Quality: " <<  a.quality() << " - Avg quality: " <<  a.quality()/a.fg().nrVars() << endl;

                msg = a.next_waypoint();

                /* Minimal quality for the reconstructed map. */
                // const int min_quality = 138; // max 425 

                // Finishing the path
                if (a.quality() >= min_quality) //AA: and max_time?
                {
                        a.print_Gibbs_sample(1000);
                        ROS_INFO("Moving toward: (%g, %g, %g)", 
                                 g_initial_pose.position.x, g_initial_pose.position.y, g_initial_pose.position.z);
                        motion.publish(g_initial_pose);
                        // Wait here?
                        break;
                }
        }
        return count;
}




/** 
    Method that sends messages corresponding to the Pose of the a waypoints path.
    @param path The vector containing the  @return Tumber of steps executed. 
    @return The number of steps in the path executed.
*/
int follow_waypoints(std::vector<geometry_msgs::Pose> &path, ros::Publisher &motion, ros::Rate& loop_rate)
{
        int count = 0;
        int timelimit = 50;
        ROS_INFO("Ready");
        geometry_msgs::Pose msg;
        std_msgs::String semantic_camera;

        while (ros::ok())
        {
                /**
                 * This is a message object. You stuff it with data, and then publish it.
                 */
                // If drone around waypoint, gets next, else, waits

                msg = path.at(count);

//        ROS_INFO("%s", msg.data.c_str());
                /**
                 * The publish() function is how you send messages. The parameter
                 * is the message object. The type of this object must agree with the type
                 * given as a template parameter to the advertise<>() call, as was done
                 * in the constructor above.
                 */
                ROS_INFO("Moving toward: (%g, %g, %g)", msg.position.x, msg.position.y, msg.position.z);

                motion.publish(msg);

                while ( !at_waypoint(msg) ) 
                {
                        ros::spinOnce();
                        loop_rate.sleep();
                }

                ROS_INFO("Reached position: (%g, %g, %g)", msg.position.x, msg.position.y, msg.position.z, count);
                ros::spinOnce();
                loop_rate.sleep();


                // Finishing the path in the initial position
                if (path.size() <= ++count) 
                {
                        ROS_INFO("Moving toward: (%g, %g, %g)", 
                                 g_initial_pose.position.x, g_initial_pose.position.y, g_initial_pose.position.z);
                        motion.publish(g_initial_pose);
                        // Wait here?
                        break;
                }
        }
        return count;
}



int main(int argc, char **argv)
{
        ros::init(argc, argv, "plan_node");
        std::cout << argv[1] << std::endl;

        // True if reading a file with the waypoints
        bool waypoints_mode = false;


        /* Reads Factor Graph from input file */
        FactorGraph network;

        /* Gets the waypoints from input file */
        std::vector<geometry_msgs::Pose> path;

        ROS_INFO("started with: %s %s %s",argv[0], argv[1], argv[2] );

        // Reads command line
        if (argc > 1)
        {
                // First option should be -w or -c
                if (argv[1] == std::string("-w"))
                        waypoints_mode = true;
                else if  (argv[1] == std::string("-c"))
                        ;
                else  usage(argv[0]);
                try {
                        if (waypoints_mode)
                                read_path(argv[2], path);
                        else 
                                network.ReadFromFile( argv[2] );
                } catch (int e) {
                        cout << "An exception occurred. Cannot read from file." << endl; 
                        ROS_FATAL("Input file is missing!");
                        usage(argv[0]);
                }
        }
        else
                usage(argv[0]);
  

/**
 * NodeHandle is the main access point to communications with the ROS system.
 * The first NodeHandle constructed will fully initialize this node, and the last
 * NodeHandle destructed will close down the node.
 */
        ros::NodeHandle n;

/**
 * The advertise() function is how you tell ROS that you want to
 * publish on a given topic name.
 */
        ros::Publisher motion = n.advertise<geometry_msgs::Pose>("/bee/waypoint", 1000);

        // subscribes to stream
        ros::Subscriber sub = n.subscribe("/bee/pose", 1000, positionCallback);
        ROS_INFO("subscribed to: %s", sub.getTopic().c_str() );
        ros::Subscriber subC = n.subscribe("/bee/camera", 1000, cameraCallback);

        // message_filters::Subscriber<std_msgs::String> subO(n, "/bee/camera", 1);
        // message_filters::Cache<std_msgs::String> cache(subO, 100);
        // message_filters::Subscriber<geometry_msgs::PoseStamped> sub(n, "/bee/pose", 1);
        // g_pose_cache.connectInput(sub);
        // g_pose_cache.registerCallback(boost::bind(&chatterCallback, _1));
        ros::Rate loop_rate(5);

        while (ros::ok())
        {
                /* Gets the initial pose of the drone */
                if (g_pose_cache.getElemAfterTime( g_pose_cache.getOldestTime() ) != NULL)
                { 
                        boost::shared_ptr<const geometry_msgs::PoseStamped> orig = g_pose_cache.getElemAfterTime( g_pose_cache.getOldestTime() );
                        g_initial_pose = (orig->pose);
                        ROS_INFO("Initial position: (%g, %g, %g)", g_initial_pose.position.x, g_initial_pose.position.y, g_initial_pose.position.z);
                        break;
                }
                else  ROS_INFO("Error in identifying Initial position\n");
                
                ros::spinOnce();
                loop_rate.sleep();
        }



        int steps = 0;
        if (waypoints_mode)
        {
                steps = follow_waypoints(path, motion, loop_rate);
                path.erase(path.begin(), path.end());
        }
        else 
        {
                cout << "Network read from input file" << endl;
                // we assume that all vars have the same nr of states
                int num_classes = network.var(1).states();

                /* Gets an approximate marginals algorithm */
                ApproxMarginals marginals(network, num_classes);

                // Output some information about the factorgraph
                cout << marginals.fg().nrVars() << " variables" << endl;
                cout << marginals.fg().nrFactors() << " factors" << endl;
                cout << marginals.num_classes() << " states" << endl;

                steps = adaptive_sampling( marginals, motion, loop_rate );
        }

        ROS_INFO("Algorithm terminated after %d steps.\n\n", steps); 

     return 0;
 }
