#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <global_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>

#define UP 1
#define DOWN 2
#define FU 3
#define FD 4


#define V_W 300
#define V_H 300
#define V_SIZE V_H * V_W
#define VRANGE 5
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

rm::Pose mypose_;
nav_msgs::OccupancyGrid map_;
nav_msgs::OccupancyGrid localmap_;
nav_msgs::OccupancyGrid potential_;
vector<int> l_p;

std::vector<std::pair<int,std::pair<int,int> > > node_vc;
std::queue<int> rbque;
ros::Publisher vmap_pub;
ros::Publisher node_pose_pub;
ros::Publisher cmdvel_pub;
ros::Publisher potential_pub;
double py_;
int st_;


// 이동하면서 후보노드 탐색하는 벡터생성
vector<std::pair<int,int> > candi_left, candi_right;
int visited[V_SIZE];


void poseCallback(const nav_msgs::Odometry::ConstPtr& odom){
  mypose_ = odom->pose.pose;
  //ROS_INFO("%f",odom->pose.pose.position.x);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
  map_ = *map;

  potential_.data.clear();
  int size = map_.data.size();

  ROS_INFO("%d", size);
  for(int i=0;i<size;i++){
    potential_.data.push_back(map_.data[i]);
  }
  
  potential_.header.frame_id = "map";
  potential_.header.stamp = ros::Time::now();
    
  potential_.info.width = map_.info.width;
  potential_.info.height = map_.info.height;
 
  potential_.info.resolution = map_.info.resolution;
  potential_.info.origin.position.x = map_.info.origin.position.x;
  potential_.info.origin.position.y = map_.info.origin.position.y; 
}

void localmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& localmap){
  localmap_ = *localmap;

  l_p.clear();
  int size = localmap_.data.size();
  for(int i=0;i<size;i++){
   l_p.push_back(localmap_.data[i]);
  }
}
void mapToWorld(double mx, double my, double& wx, double& wy) {
     wx = map_.info.origin.position.x + (mx+0.5) * map_.info.resolution;
     wy = map_.info.origin.position.y + (my+0.5) * map_.info.resolution;
}
void mapToWorld_v(double mx, double my, double& wx, double& wy) {
     wx = map_.info.origin.position.x + (mx+0.5) * 0.1;
     wy = map_.info.origin.position.y + (my+0.5) * 0.1;
}

bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) {
  double origin_x = map_.info.origin.position.x, origin_y = map_.info.origin.position.y;
  double resolution = map_.info.resolution;

  if (wx < origin_x || wy < origin_y)
      return false;

  mx = (wx - origin_x) / resolution - 0.5;
  my = (wy - origin_y) / resolution - 0.5;

      if (mx < map_.info.width && my < map_.info.height)
          return true;

      return false;
}

bool isInBox(int tx, int ty, int ix, int iy){
  int box = 2;
  if(tx - box < ix && tx + box > ix && ty - box < iy && ty + box > iy){
    return true;
  }
  return false;
}
bool worldToMap_v(double wx, double wy, unsigned int& mx, unsigned int& my) {
  double origin_x = map_.info.origin.position.x, origin_y = map_.info.origin.position.y;
  double resolution = 0.1;

  if (wx < origin_x || wy < origin_y)
      return false;

  mx = (wx - origin_x) / resolution - 0.5;
  my = (wy - origin_y) / resolution - 0.5;

      if (mx < V_W && my < V_H)
          return true;

      return false;
}

int abs(int a){
  if(a<0)
    return -a;
  return a;
}

void publishvmap(){
  nav_msgs::OccupancyGrid vmap;

  vmap.info.origin.position.x = map_.info.origin.position.x;
  vmap.info.origin.position.y = map_.info.origin.position.y;
  vmap.info.origin.orientation.w = 1.0;

  vmap.info.resolution = 0.1;

  vmap.info.width = V_W;
  vmap.info.height = V_H;

  vmap.header.frame_id = "map";
  vmap.header.stamp = ros::Time::now();

  
  for(int i=0;i<V_SIZE;i++)
    vmap.data.push_back(visited[i]);

  vmap_pub.publish(vmap);
}
void visitedUpdate(unsigned int x, unsigned int y, int num){
  int r = 2;
  for(int dx=x-r;dx<x+r;dx++){
    for(int dy=y-r;dy<y+r;dy++){
      if(visited[dy*V_W + dx] != 2)
        visited[dy*V_W + dx] = num;
    }
  }
}
int getState(int x,int y){
    return map_.data[y*map_.info.width + x];
}
int getVState(int x,int y){
   return visited[y*V_W + x];
}
void make_node(int v_mx, int v_my, int st){
  node_vc.push_back(std::make_pair(st,std::make_pair(v_mx,v_my)));
}
void send_goal(MoveBaseClient* ac, double x, double y){
  move_base_msgs::MoveBaseGoal tempgoal;

  tempgoal.target_pose.header.frame_id = "map";
  tempgoal.target_pose.header.stamp = ros::Time::now();
  tempgoal.target_pose.pose.position.x = x;
  tempgoal.target_pose.pose.position.y = y;
  tempgoal.target_pose.pose.orientation.w = 1.0;

  ac->sendGoal(tempgoal);
  ROS_INFO("Sending goal");
}
void send_goal_map(MoveBaseClient* ac, unsigned int mx, unsigned int my){
  double x,y;
  mapToWorld(mx,my,x,y);

  move_base_msgs::MoveBaseGoal tempgoal;

  tempgoal.target_pose.header.frame_id = "map";
  tempgoal.target_pose.header.stamp = ros::Time::now();
  tempgoal.target_pose.pose.position.x = x;
  tempgoal.target_pose.pose.position.y = y;
  tempgoal.target_pose.pose.orientation.w = 1.0;

  ac->sendGoal(tempgoal);
  ROS_INFO("Sending goal");
}
void publish_node_pose(){
  if(!node_vc.empty()){
    sensor_msgs::PointCloud tmp;
    tmp.header.stamp = ros::Time::now();
    tmp.header.frame_id = "map";
    double wx,wy;
    double offx, offy;
//
//    offx = map_.info.origin.position.x;
//    offy = map_.info.origin.position.y;
//
    for(int i=0;i<node_vc.size();i++){
      rm::Point32 point;
      mapToWorld_v(node_vc[i].second.first, node_vc[i].second.second, wx, wy);
      point.x = wx;// - offx;
      point.y = wy;// - offy; 
      point.z = 0;
      tmp.points.push_back(point);
    }
    node_pose_pub.publish(tmp);

  }
}
// 나중에 생성된 노드로 병합

void merge(int dic,int mx,int my){
    int tmpdic;
    if(dic==FU){
        tmpdic = DOWN;
    }else if(dic==FD){
        tmpdic = UP;
    }
/*
    for (int i=0;i<candi.size();i++){
      ROS_INFO("[%d %d]",candi[i].first, i);
    }
  */  

  // 병합을 없애고 visited 그리드마다 노드가된다.
  // 지나간자리에 노드가 있을경우 노드를 삭제한다.
  /*
    // 바로 위아래있는거끼리 병합한다.
    for(int i=0;i<candi_left.size()-1;i++){
      if(candi_left[i].second == candi_left[i+1].second){
        if(abs(candi_left[i].first - candi_left[i+1].first) <= 1){
          candi_left.erase(candi_left.begin()+i);
          i--;
        }
      }
    }
    for(int i=0;i<candi_right.size()-1;i++){
      if(candi_right[i].second == candi_right[i+1].second){
        if(abs(candi_right[i].first - candi_right[i+1].first) <= 1){
          candi_right.erase(candi_right.begin()+i);
          i--;
        }
      }
    }
*/
    //오른쪽을 마지막에넣어서 pop_back 할때 오른쪽이 다음노드되게
    for(int i=0;i<candi_left.size();i++){
      make_node(candi_left[i].first, candi_left[i].second,tmpdic);
    }
    for(int i=0;i<candi_right.size();i++){
      make_node(candi_right[i].first, candi_right[i].second,tmpdic);
    }
    candi_left.clear();
    candi_right.clear();
}
bool getStatebool(int x,int y){

    // Lidar 센서로 대체인식 예정.
  int m = map_.data[y*map_.info.width + x];
  if(m == 100){
    return true;
  }
  return false;
}
void eraseNode(int v_mx, int v_my){
  for(int i=0;i<node_vc.size();i++){
    if(isInBox(v_mx,v_my,node_vc[i].second.first, node_vc[i].second.second)){
      node_vc.erase(node_vc.begin()+i);
    }
  }
}
void go_front(int mx,int my){
  //publish(cmd_vel);
}
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
  ROS_INFO("Waiting for the move_base action server to come up");
  }
  ros::NodeHandle private_nh;
  ros::Subscriber pose_sub_ = private_nh.subscribe<nav_msgs::Odometry>("odom", 100, &poseCallback);
  ros::Subscriber map_sub_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("map", 100, &mapCallback);
  ros::Subscriber local_map_sub = private_nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 100, &localmapCallback);
  vmap_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("vmap", 100); 
  node_pose_pub = private_nh.advertise<sensor_msgs::PointCloud>("node_pose",100);
  cmdvel_pub = private_nh.advertise<rm::Twist>("cmd_vel",100);
  potential_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("potential",100);
  ros::Rate loop_rate(5);
  bool okgo = false;


    
// pair <int : 위방향 청소 1 아래방향 청소 2 // >
  
  while(ros::ok()){
   
    unsigned int mx, my;
    unsigned int v_mx, v_my;
    double wx = mypose_.position.x, wy=mypose_.position.y;
    worldToMap(wx,wy,mx,my);
    worldToMap_v(wx,wy,v_mx,v_my);
    //ROS_INFO("%d %d",v_mx,v_my);
    // 현재 위치를 visit 배열에 업데이트
    visitedUpdate(v_mx,v_my,2);
   // visit 맵 publish
    publishvmap();
    publish_node_pose();

    int lpsize = l_p.size();
    int offset = mx-localmap_.info.width/2+(my-localmap_.info.height/2)*localmap_.info.width; 
    for(int i=0;i<lpsize;i++){
       potential_.data[offset+i] = l_p[i];
    }

    potential_pub.publish(potential_);
    // 위치정보 토픽을 제대로 입력받으면 mx가 0이아님
    if(mx != 0){

      // 아무 수행이나 노드없으면 아래 청소노드 추가
      if(rbque.empty() && node_vc.empty()){
        ROS_INFO("first node added");
        make_node(v_mx, v_my, DOWN);
      }

      // 수행목표지점에 도착했을때
      if(rbque.empty()){
        // 목표 노드에 도착했으면 해당노드제거 후 수행명령큐에 명령삽입
          

          int tx, ty;
          tx = node_vc.back().second.first;
          ty = node_vc.back().second.second;
          
            ROS_INFO("node send goal : %d",node_vc.size());
            double dx,dy;
            mapToWorld_v(tx,ty,dx,dy);
            send_goal(&ac, dx, dy);
            rbque.push(5);
          
      }else{
          int st = rbque.front();
          if(st == FD || st == FU){
              // 현재위치에 해당되는 노드가 노드벡터안에 있을경우 해당노드삭제
              eraseNode(v_mx,v_my);
        
            // 벡터에 왼쪽부터 들어가므로 맨마지막에 들어가는 건 오른쪽임.
              ROS_INFO("%d, %f",okgo, mypose_.orientation.z);
              // 왼쪽 탐색
                if(getState(mx,my-10) == 0 && getVState(v_mx,v_my-VRANGE) == 0){
                    visitedUpdate(v_mx,v_my-VRANGE,1);
                    candi_left.push_back(std::make_pair(v_mx,v_my-VRANGE));
                    
                }
              // 오른쪽 탐색
                if(getState(mx,my+10) == 0 && getVState(v_mx,v_my+VRANGE) == 0){
                    visitedUpdate(v_mx,v_my+VRANGE,1);
                    candi_right.push_back(std::make_pair(v_mx,v_my+VRANGE));
                    
                }

              if(st == FD){
               
                rm::Twist cmd;

                ROS_INFO("///%d///", mx);
                if(okgo){
                    cmd.linear.x = 0.5;
                    cmd.linear.y = 0;
                    cmd.linear.z = 0;
                    cmd.angular.z = py_-wy;
                    for(int i=mx;i<mx+30;i++){
                        if(getStatebool(i,my)){
                             ROS_INFO("kiallalalla");
                             cmd.linear.x = 0;
                             cmd.linear.y = 0;
                             cmd.linear.z = 0;
                             cmd.angular.z = 0.4;
                             merge(st, mx, my);
                             rbque.pop();
                             okgo = false;                    
                            break;
                        }
                    }
                }else{
                    if(mypose_.orientation.z > 0.1){
                        cmd.angular.z = -0.5;
                    }else{
                         okgo = true;
                    }
                }
                
                cmdvel_pub.publish(cmd);
              }else if(st == FU){
                
                rm::Twist cmd;

                if(okgo){
                    cmd.linear.x = 0.8;
                    cmd.linear.y = 0;
                    cmd.linear.z = 0;
                    cmd.angular.z = py_-wy;
                    for(int i=mx;i>mx-30;i--){
                        if(getStatebool(i,my)){
                             ROS_INFO("kiallalalla");
                             cmd.linear.x = 0;
                             cmd.linear.y = 0;
                             cmd.linear.z = 0;
                             cmd.angular.z = 0;
                             merge(st, mx, my);
                             rbque.pop();
                             okgo = false;                    
                            break;
                        }
                    }
                }else{
                    //if(mypose_.orientation.z > 1.1){
                    //    cmd.angular.z = 0.4;
                    if(mypose_.orientation.z < 0.9){
                        cmd.angular.z = 0.4;
                    }else{
                         okgo = true;
                    }
                }
                
                cmdvel_pub.publish(cmd);
              }
          }else if(st==DOWN || st==UP){
              // 수행목표가 벽찾음이고 해당 수행목표지점에 도착했을 경우.
              if(st == DOWN){                    
                py_ = wy;
                rbque.pop();
                rbque.push(FD);
              }else if(st == UP){
                py_ = wy;
                rbque.pop();
                rbque.push(FU);
              }
          }else{
              int tx, ty;
              tx = node_vc.back().second.first;
              ty = node_vc.back().second.second;
ROS_INFO("gf");
              if(isInBox(tx, ty, v_mx, v_my)){
                ac.cancelAllGoals();
                ROS_INFO("pop back [%d]",node_vc.back().first);
                rbque.pop();
                rbque.push(node_vc.back().first);
                node_vc.pop_back();
              }
          }
        
        }
    }
    ros::spinOnce();
    loop_rate.sleep();

  }

return 0;
}
