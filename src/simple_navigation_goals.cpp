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
#include <actionlib_msgs/GoalStatusArray.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <keyboard/Key.h>




#define UP 1
#define DOWN 2
#define FU 3
#define FD 4
#define GG 5
#define GSG 6
#define HUMAN 1
#define ROBOT 2
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))  
#define DISF 0.7
#define DISS 0.7
#define PI 3.14
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

rm::Point32 foots[2];

std::vector<std::pair<int,std::pair<int,int> > > node_vc;
std::queue<int> rbque;
ros::Publisher vmap_pub;
ros::Publisher node_pose_pub;
ros::Publisher cmdvel_pub;
ros::Publisher potential_pub;
ros::Publisher bang_pub;
double py_;
double dis_f_ = 999999, dis_r_, dis_l_;
int st_;
int mystatus_;
int keydown[5] = {0,0,0,0,0};
// 이동하면서 후보노드 탐색하는 벡터생성
vector<std::pair<int,int> > candi_left, candi_right;
int visited[V_SIZE];
int mode = HUMAN;

void poseCallback(const nav_msgs::Odometry::ConstPtr& odom){
  mypose_ = odom->pose.pose;
  //ROS_INFO("%f",odom->pose.pose.position.x);
}

void footprintCallback(const rm::PolygonStamped::ConstPtr& foot){
   if(!foot->polygon.points.empty()){
       foots[0] = foot->polygon.points[0];
       foots[1] = foot->polygon.points[1];
   }
}

void keyDownCallback(const keyboard::Key::ConstPtr& key){
    int k = key->code;
    if(k==97){
        keydown[0] = 1;
    }else if(k==119){
        keydown[1] = 1;
    }else if(k==100){
        keydown[2] = 1;
    }else if(k==115){
        keydown[3] = 1;
    }else if(k==114){
        if(keydown[4] == 0 && mode == HUMAN){
            mode = ROBOT;
            ROS_INFO("mode ROBOT");
        }else if(keydown[4] == 0 && mode==ROBOT){
            mode = HUMAN;
            ROS_INFO("mode HUMAN");
            rbque.pop();

        }

        rm::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        cmdvel_pub.publish(cmd);

        keydown[4] = 1;
    }
//    ROS_INFO("keydown : {%d}",k);

}
void keyUpCallback(const keyboard::Key::ConstPtr& key){
    int k = key->code;
    if(k==97){
        keydown[0] = 0;
    }else if(k==119){
        keydown[1] = 0;
    }else if(k==100){
        keydown[2] = 0;
    }else if(k==115){
        keydown[3] = 0;
    }else if(k==114){
        keydown[4] = 0;
    }
//    ROS_INFO("keyup : {%d}",k);

   
}
void cloudCallback(const sensor_msgs::LaserScan::ConstPtr& cloud){
    for(int i=0;i<40;i++){
        dis_f_ = MIN(dis_f_, cloud->ranges[i]);
    }
    for(int i=320;i<358;i++){
        dis_f_ = MIN(dis_f_, cloud->ranges[i]);
    }
     
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

bool isInBox(int tx, int ty, int ix, int iy, int box){
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
    if(x > map_.info.width || x < 0)
        return -1;
    if(y > map_.info.height || y < 0)
        return -1;
    return map_.data[y*map_.info.width + x];
}
int getVState(int x,int y){
   return visited[y*V_W + x];
}
void make_node(int mx, int my, int st){
  node_vc.push_back(std::make_pair(st,std::make_pair(mx,my)));
}
void send_goal(MoveBaseClient* ac, double mx, double my, double tx, double ty){
  move_base_msgs::MoveBaseGoal tempgoal;

  tempgoal.target_pose.header.frame_id = "map";
  tempgoal.target_pose.header.stamp = ros::Time::now();
  tempgoal.target_pose.pose.position.x = tx;
  tempgoal.target_pose.pose.position.y = ty;
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

      ROS_INFO("pub node");
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
  if(m == 100 || m== -1){
    return true;
  }
  return false;
}
void eraseNode(int mx, int my){
  for(int i=0;i<node_vc.size();i++){
    if(isInBox(mx,my,node_vc[i].second.first, node_vc[i].second.second, 4)){
      node_vc.erase(node_vc.begin()+i);
    }
  }
}
void go_front(int mx,int my){
  //publish(cmd_vel);
}
bool isSuccess(int x,int y){
  int r = 5;
  for(int dx=x-r;dx<x+r;dx++){
    for(int dy=y-r;dy<y+r;dy++){
      int st = getState(dx,dy);
      if(st == 100 || st == -1 )
        return false;
    }
  } 
  return true;
}
int footprintCost(double x, double y){
  unsigned int mx,my;
  worldToMap(x,y,mx,my);
  return getState(mx,my);
}

bool turnto(double* twist_z, rm::Quaternion &mr, double tr){
  //ROS_INFO("/%f %f/",mr,tr);


  tf::Quaternion q(mr.x, mr.y, mr.z, mr.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  ROS_INFO("turn to %f %f", yaw, tr);
 
  
  
  if(yaw > tr){
    if(yaw - tr < 0.05){
        return false;
    }
    *twist_z = (tr-yaw); 
 
  }else if(yaw < tr){
   if(tr - yaw < 0.05){
        return false;
    }
    *twist_z = (tr-yaw);
  }


 
 /*

  double w;
  if(mr > tr + 0.05 && mr < 1){
      w = (tr - mr)*2;
  }else if(mr < tr - 0.05 && mr > -1){
      w = -(tr - mr)*2;
  }else{
      return false;
  }
  *twist_z = w;
  */
  return true;
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
  ros::Subscriber footprint_sub = private_nh.subscribe<rm::PolygonStamped>("/move_base/local_costmap/footprint",100,&footprintCallback);
  ros::Subscriber cloud_sub  = private_nh.subscribe<sensor_msgs::LaserScan>("/scan", 100, &cloudCallback);
  ros::Subscriber remote_keydown = private_nh.subscribe<keyboard::Key>("/keyboard/keydown", 100, &keyDownCallback);
  ros::Subscriber remote_keyup = private_nh.subscribe<keyboard::Key>("/keyboard/keyup", 100, &keyUpCallback);


  bang_pub = private_nh.advertise<rm::PoseStamped>("bang", 100);
  vmap_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("vmap", 100); 
  node_pose_pub = private_nh.advertise<sensor_msgs::PointCloud>("node_pose",100);
  cmdvel_pub = private_nh.advertise<rm::Twist>("cmd_vel",100);
  potential_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("potential",100);


  ros::Rate loop_rate(10);

bool okgo = false;
// pair <int : 위방향 청소 1 아래방향 청소 2 // >
  
  while(ros::ok()){

    if(mode == HUMAN){
        rm::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        if(keydown[1] == 1){
            cmd.linear.x = 2;
        }else if(keydown[0] == 1){
            cmd.angular.z = 1;
        }else if(keydown[2] == 1){
            cmd.angular.z = -1;
        }else if(keydown[3] == 1){
            cmd.linear.x = -2;
        }

        cmdvel_pub.publish(cmd);
    }else{
     unsigned int mx, my;
    unsigned int v_mx, v_my;
    double wx = mypose_.position.x, wy=mypose_.position.y;
    worldToMap(wx,wy,mx,my);
    worldToMap_v(wx,wy,v_mx,v_my);
    
    // worldToMap(
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
        //ROS_INFO("my rotate : %f %f",mypose_.orientation.z,mypose_.orientation.w);
        make_node(mx, my+10, DOWN);
      }

          // 수행목표지점에 도착했을때
      if(rbque.empty() && !node_vc.empty()){
        // 목표 노드에 도착했으면 해당노드제거 후 수행명령큐에 명령삽입
                
          int tx, ty;
          tx = node_vc.back().second.first;
          ty = node_vc.back().second.second;
     
       
          double gtx,gty;
          mapToWorld(tx,ty,gtx,gty);  
          double pz = atan2((gty-wy),(gtx-wx));
  //        ROS_INFO("%f pz", atan2((gty-wy),(gtx-wx)));

          rm::Twist cmd;
          double w;
  //        ROS_INFO("[%f %f]",mypose_.orientation.z, pz);
            
//   
//                  send_goal(&ac,wx,wy, gtx, gty);
//                  rbque.push(GG);
//       continue;

//   
//          if(mypose_.orientation.z > pz + 0.05 && mypose_.orientation.z < 1){
//              cmd.angular.z = (pz - mypose_.orientation.z)*2;
//          }else if(mypose_.orientation.z < pz - 0.05 && mypose_.orientation.z > -1){
//              cmd.angular.z = (pz - mypose_.orientation.z)*2;
//          }
//          
          if(turnto(&w, mypose_.orientation, pz)){
              cmd.angular.z = w;
          }else{
    //          ROS_INFO("foot 1 : %f %f", foots[0].x, foots[0].y);
    //          ROS_INFO("foot 2 : %f %f", foots[1].x, foots[1].y);
             
              double diff_x = gtx - wx;
              double diff_y = gty - wy;
              double target_x[2];
              double target_y[2];   

              bool done = false;
             
              double scale = 1;//abs(diff_x);
              double dScale = 0.01;//diff_x/100;
              
//              if(footprintCost(foots[0].x, foots[0].y) == 0){
//                   okgo = true;
//              }
//
              while(!done)
              {
                if(scale < 0)
                {
                  okgo = true;
                  done = true;  
                }
                for(int i=0;i<2;i++){
                  target_x[i] = foots[i].x + scale * diff_x;
                  target_y[i] = foots[i].y + scale * diff_y;
                
                 if(footprintCost(target_x[i], target_y[i]) != 0){
                    done = true;
                 }
                }
                
                scale -=dScale;
              }
      //        ROS_INFO("seieii"); 

              if(!okgo){

//                  ROS_INFO("node send goal : %d",node_vc.size());
                
                 
                  send_goal(&ac,wx,wy, gtx, gty);
                  rbque.push(GG);
       
              }else{
                  okgo =false;
                  rbque.push(GSG);
              }
                  
              
          }

          cmdvel_pub.publish(cmd);


      }else{

          //ROS_INFO("node size : [%d]", node_vc.size()); 
          int st = rbque.front();
          if(st == FD || st == FU){
              // 현재위치에 해당되는 노드가 노드벡터안에 있을경우 해당노드삭제
              eraseNode(mx,my);
        
            // 벡터에 왼쪽부터 들어가므로 맨마지막에 들어가는 건 오른쪽임.
   //           ROS_INFO("%f, %f",mypose_.orientation.w, mypose_.orientation.z);

              unsigned int tmx, tmy, tvx, tvy;
              worldToMap(wx, wy-0.5,tmx,tmy);
              worldToMap_v(wx, wy-0.5,tvx,tvy);  
              // 왼쪽 탐색
               if(isSuccess(tmx, tmy) && getVState(tvx,tvy) == 0){
                
                    visitedUpdate(tvx,tvy,1);
                    candi_left.push_back(std::make_pair(tmx,tmy));
               }
             
              worldToMap(wx, wy+0.5,tmx,tmy);
              worldToMap_v(wx, wy+0.5,tvx,tvy);  
              // 오른쪽 탐색
               if(isSuccess(tmx, tmy) && getVState(tvx,tvy) == 0){
                    visitedUpdate(tvx,tvy,1);
                    candi_right.push_back(std::make_pair(tmx,tmy));
                }
               
              if(st == FD){
               
                rm::Twist cmd;

//                ROS_INFO("FD!");
                if(okgo){
//                  ROS_INFO("FD - okgo");
                    cmd.linear.x = 0.5;
                    cmd.linear.y = 0;
                    cmd.linear.z = 0;
             //       cmd.angular.z = (py_-wy)/2;
                    ROS_INFO("cha : %f", -py_+wy);
//                    for(int i=mx;i<mx+15;i++){
//                        if(getStatebool(i,my)){
//                          if(dis_f_ < 0.8 || dis_r_ < 0.5 || dis_l_ < 0.5){
                        
                      unsigned int t_x, t_y;
                      double t_wx, t_wy;
                  ROS_INFO("[%f %f] : my", wx,wy);
   
                  //    for(int i=0;i<3;i++){
                  //        t_wx = wx-0.7*cos(acos(mypose_.orientation.z)*2 - PI/10 + i*(PI/10));
                  //        t_wy = wy+0.7*sin(asin(mypose_.orientation.w)*2 - PI/10 + i*(PI/10));
                   
                   if(dis_f_ < DISF ){
//                          worldToMap(t_wx, t_wy, t_x,t_y);
 //                        ROS_INFO("[%f %f] : tx ty", t_wx,t_wy);


//                          if(getState(t_x,t_y) != 0){  
                             ROS_INFO("kiallalalla");
                             cmd.linear.x = 0;
                             cmd.linear.y = 0;
                             cmd.linear.z = 0;
                             cmd.angular.z = 0;
                             merge(st, mx, my);
                             rbque.pop();
                             okgo = false;                    
                            // ROS_INFO("node size : %d",rbque.size());
                           }
//                      }
                }else{
                    double w;
                    if(turnto(&w, mypose_.orientation, 0)){
                        cmd.angular.z = w;
                    }else{
                        okgo = true;
                    }
               }
                
                cmdvel_pub.publish(cmd);
              }else if(st == FU){
                
                rm::Twist cmd;
    
//                ROS_INFO("FU!");
                if(okgo){
//                    ROS_INFO("FU - okgo");
                    cmd.linear.x = 0.5;
                    cmd.linear.y = 0;
                    cmd.linear.z = 0;
                    cmd.angular.z = (py_-wy)/2;
                    ROS_INFO("cha : %f", py_-wy);
                        
                  unsigned int t_x, t_y;
                  double t_wx, t_wy;
                  
                    ROS_INFO("[%f %f] : my", wx,wy);
//                  for(int i=0;i<3;i++){
//                      t_wx = wx-0.4*cos(acos(mypose_.orientation.z)*2 - PI/10 + i*(PI/10));
//                      t_wy = wy+0.4*sin(asin(mypose_.orientation.w)*2 - PI/10 + i*(PI/10));
                       worldToMap(t_wx, t_wy, t_x,t_y);
                    

                     ROS_INFO("[%f %f] : tx ty",t_wx, t_wy);

 
 
//                      if(getState(t_x,t_y) != 0){  
                  if(dis_f_ < DISF){

                         ROS_INFO("kiallalalla");
                         cmd.linear.x = 0;
                         cmd.linear.y = 0;
                         cmd.linear.z = 0;
                         cmd.angular.z = 0;
                         merge(st, mx, my);
                         rbque.pop();
                         okgo = false;
                       //  ROS_INFO("node size : %d",rbque.size());                    
                       }
//                  }
                }else{
                   double w;
                    if(turnto(&w, mypose_.orientation, 3.14)){
                        cmd.angular.z = w;
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
          }else if(st == GG){
        
           int tx, ty;
            tx = node_vc.back().second.first;
            ty = node_vc.back().second.second;


              if(isInBox(tx, ty, mx, my,3)){
                ac.cancelAllGoals();
                ROS_INFO("pop back [%d]",node_vc.back().first);
                rbque.pop();
                rbque.push(node_vc.back().first);
                node_vc.pop_back();
              }

          }else if(st == GSG){
        
            int tx, ty;
            tx = node_vc.back().second.first;
            ty = node_vc.back().second.second;

ROS_INFO("GO SIMPLE");
        
             rm::Twist cmd;

             cmd.linear.x = 0.4;
             cmd.linear.y = 0;
             cmd.linear.z = 0;
             cmd.angular.z = 0;//(py_-wy)/2;
            
             unsigned int t_x, t_y;
             double t_wx, t_wy;
             t_wx = wx-0.5*cos(acos(mypose_.orientation.z)*2);
             t_wy = wy+0.5*sin(asin(mypose_.orientation.w)*2);
             worldToMap(t_wx, t_wy, t_x,t_y);


            //  if(getState(t_x,t_y) != 0 || isInBox(tx, ty, mx, my,3)){
             if(isInBox(tx,ty,mx,my,3)){
                ac.cancelAllGoals();

                cmd.linear.x = 0;

                ROS_INFO("pop back [%d]",node_vc.back().first);
                rbque.pop();
                rbque.push(node_vc.back().first);
                node_vc.pop_back();
             }
                    
              cmdvel_pub.publish(cmd);
          }
        
        }
    }
    }
    ros::spinOnce();
    loop_rate.sleep();

  }

return 0;
}
