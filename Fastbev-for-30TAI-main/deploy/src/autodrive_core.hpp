#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "yaml-cpp/yaml.h"

namespace autodrive {
constexpr double PI=3.14159265358979323846;
inline double clampd(double v,double a,double b){return std::max(a,std::min(b,v));}
inline double rad(double d){return d*PI/180.0;}
struct Point{double x=0,y=0,z=0;};
struct Ego{double x=0,y=0,z=0,yaw_deg=0,speed_mps=0;};
struct Radar{bool valid=false;double distance_m=std::numeric_limits<double>::quiet_NaN(),relative_velocity_mps=std::numeric_limits<double>::quiet_NaN();};
struct Detection{double x=0,y=0,z=0,dx=0,dy=0,dz=0,score=0;int class_id=-1;};
struct Input{Ego ego;Radar radar;std::string route_id;std::vector<Point> route;};
struct Control{double steer=0,throttle=0,brake=0;};
struct Candidate{double lateral_offset_m=0,reference_cost=0,smooth_cost=0,obstacle_cost=0,total_cost=0;std::vector<Point> points;};
struct Result{std::string fsm_state="NO_ROUTE";bool aeb_triggered=false;double d_safe_m=0,d_radar_m=std::numeric_limits<double>::quiet_NaN(),target_speed_kmh=0;Control control;};
struct Config{
 double cruise_speed_kmh=20,reaction_time_s=.15,safe_decel_mps2=6,safety_margin_m=2,stop_distance_m=11,stop_lane_half_width_m=2.1,detection_score_min=.25,follow_min_front_speed_kmh=10;
 bool follow_enabled=false;
 // Low-rate FOT/Pure-Pursuit settings (kept unchanged).
 double lookahead_base_m=2.2,lookahead_gain_s=.45,lookahead_min_m=2,lookahead_max_m=6,wheelbase_m=2.85,max_steer_angle_rad=.55;
 // High-rate Reference-Path-only tracking settings.
 double high_lookahead_base_m=3.5,high_lookahead_gain_s=.35,high_lookahead_min_m=3.5,high_lookahead_max_m=6.5;
 double high_progress_search_ahead_m=15.0,high_steer_lowpass_alpha=.22,high_max_steer_rate_per_s=1.20;
 double high_curve_speed_min_kmh=12.0,high_curve_speed_steer_start=.25,high_curve_speed_steer_full=.70;
 double pid_kp=.32,pid_ki=0,pid_kd=.05,obstacle_influence_m=4.5,w_reference=1,w_smooth=.25,w_bev=.2;
 std::vector<double> lateral_offsets_m{-0.8,0,0.8};
};
template<typename T>T y(const YAML::Node&n,const char*k,T d){if(!n||!n[k]||n[k].IsNull())return d;try{return n[k].as<T>();}catch(...){return d;}}
inline Config load_config(const YAML::Node&r){
 Config c;if(!r)return c;
 c.cruise_speed_kmh=y(r,"cruise_speed_kmh",c.cruise_speed_kmh);
 auto a=r["aeb"];
 c.reaction_time_s=y(a,"reaction_time_s",c.reaction_time_s);
 c.safe_decel_mps2=y(a,"safe_decel_mps2",c.safe_decel_mps2);
 c.safety_margin_m=y(a,"safety_margin_m",c.safety_margin_m);
 auto f=r["fsm"];
 c.stop_distance_m=y(f,"stop_distance_m",c.stop_distance_m);
 c.stop_lane_half_width_m=y(f,"stop_lane_half_width_m",c.stop_lane_half_width_m);
 c.detection_score_min=y(f,"detection_score_min",c.detection_score_min);
 c.follow_enabled=y(f,"follow_enabled",c.follow_enabled);
 c.follow_min_front_speed_kmh=y(f,"follow_min_front_speed_kmh",c.follow_min_front_speed_kmh);
 auto p=r["pure_pursuit"];
 c.lookahead_base_m=y(p,"lookahead_base_m",c.lookahead_base_m);
 c.lookahead_gain_s=y(p,"lookahead_gain_s",c.lookahead_gain_s);
 c.lookahead_min_m=y(p,"lookahead_min_m",c.lookahead_min_m);
 c.lookahead_max_m=y(p,"lookahead_max_m",c.lookahead_max_m);
 c.wheelbase_m=y(p,"wheelbase_m",c.wheelbase_m);
 c.max_steer_angle_rad=y(p,"max_steer_angle_rad",c.max_steer_angle_rad);
 auto h=r["high_rate_control"];
 c.high_lookahead_base_m=y(h,"lookahead_base_m",c.high_lookahead_base_m);
 c.high_lookahead_gain_s=y(h,"lookahead_gain_s",c.high_lookahead_gain_s);
 c.high_lookahead_min_m=y(h,"lookahead_min_m",c.high_lookahead_min_m);
 c.high_lookahead_max_m=y(h,"lookahead_max_m",c.high_lookahead_max_m);
 c.high_progress_search_ahead_m=y(h,"progress_search_ahead_m",c.high_progress_search_ahead_m);
 c.high_steer_lowpass_alpha=y(h,"steer_lowpass_alpha",c.high_steer_lowpass_alpha);
 c.high_max_steer_rate_per_s=y(h,"max_steer_rate_per_s",c.high_max_steer_rate_per_s);
 c.high_curve_speed_min_kmh=y(h,"curve_speed_min_kmh",c.high_curve_speed_min_kmh);
 c.high_curve_speed_steer_start=y(h,"curve_speed_steer_start",c.high_curve_speed_steer_start);
 c.high_curve_speed_steer_full=y(h,"curve_speed_steer_full",c.high_curve_speed_steer_full);
 auto pid=r["pid"];
 c.pid_kp=y(pid,"kp",c.pid_kp);c.pid_ki=y(pid,"ki",c.pid_ki);c.pid_kd=y(pid,"kd",c.pid_kd);
 auto fot=r["fot"];
 c.obstacle_influence_m=y(fot,"obstacle_influence_m",c.obstacle_influence_m);
 c.w_reference=y(fot,"w_reference",c.w_reference);
 c.w_smooth=y(fot,"w_smooth",c.w_smooth);
 c.w_bev=y(fot,"w_bev",c.w_bev);
 if(fot&&fot["lateral_offsets_m"]&&fot["lateral_offsets_m"].IsSequence()){
  c.lateral_offsets_m.clear();
  for(auto n:fot["lateral_offsets_m"])c.lateral_offsets_m.push_back(n.as<double>());
 }
 return c;
}
inline double num(const YAML::Node&n,const char*k,double d=0){if(!n||!n[k]||n[k].IsNull())return d;try{return n[k].as<double>();}catch(...){return d;}}
inline Input parse_input_json(const std::string&j){YAML::Node r=YAML::Load(j);Input i;auto e=r["ego"];i.ego.x=num(e,"x");i.ego.y=num(e,"y");i.ego.z=num(e,"z");i.ego.yaw_deg=num(e,"yaw_deg");i.ego.speed_mps=num(e,"speed_mps");auto rd=r["radar"];i.radar.valid=rd&&rd["valid"]&&!rd["valid"].IsNull()?rd["valid"].as<bool>():false;if(rd&&rd["nearest_distance_m"]&&!rd["nearest_distance_m"].IsNull())i.radar.distance_m=rd["nearest_distance_m"].as<double>();if(rd&&rd["relative_velocity_mps"]&&!rd["relative_velocity_mps"].IsNull())i.radar.relative_velocity_mps=rd["relative_velocity_mps"].as<double>();auto route=r["route"];if(route&&route["route_id"]&&!route["route_id"].IsNull())i.route_id=route["route_id"].as<std::string>();if(route&&route["points"]&&route["points"].IsSequence())for(auto p:route["points"]){Point q;q.x=num(p,"x");q.y=num(p,"y");q.z=num(p,"z");i.route.push_back(q);}return i;}
inline Point lidar_to_actor(const Detection&d){double ex=.00203327*d.x+.99970406*d.y+.02424172*d.z+.943713,ey=-.99998053*d.x+.00217566*d.y-.00584864*d.z,ez=-.00589965*d.x-.02422936*d.y+.99968902*d.z+1.84023;Point p;p.x=ex-1.4;p.y=-ey;p.z=ez+.25;return p;}
inline Point actor_to_world(Point l,const Ego&e){double a=rad(e.yaw_deg),c=std::cos(a),s=std::sin(a);Point p;p.x=e.x+c*l.x-s*l.y;p.y=e.y+s*l.x+c*l.y;p.z=e.z+l.z;return p;}

class Controller{
 Config c;
 // Longitudinal PID state belongs to this Controller instance. The high-rate and
 // low-rate services already construct different Controller objects.
 double integ=0,prev=0;
 bool have=false;
 std::chrono::steady_clock::time_point last=std::chrono::steady_clock::now();

 // High-rate-only tracking state. Low-rate step_with_fot() never reads/writes it.
 bool high_have_progress=false;
 double high_progress_s=0;
 size_t high_progress_segment=0;
 bool high_have_steer=false;
 double high_prev_steer=0;
 std::chrono::steady_clock::time_point high_last_steer=std::chrono::steady_clock::now();
 std::string high_route_id;

 struct Projection{
  size_t segment=0;
  double t=0;
  double s=0;
  double distance2=std::numeric_limits<double>::infinity();
 };

public:
 explicit Controller(const Config&x):c(x){}

 // Call on a new high-rate client/session so a restarted CARLA run begins from
 // the start of its route even if route_id is reused.
 void reset_high_rate_state(){
  high_have_progress=false;
  high_progress_s=0;
  high_progress_segment=0;
  high_have_steer=false;
  high_prev_steer=0;
  high_last_steer=std::chrono::steady_clock::now();
  high_route_id.clear();
 }

 // High-rate path-only controller: independent stabilized Pure Pursuit + AEB.
 // It intentionally does NOT invoke FOT or the low-rate FSM/obstacle logic.
 Result step_reference_path(const Input&i){
  Result r;
  if(i.route.size()<2){r.control.brake=.6;return r;}

  const double v=std::max(0.0,i.ego.speed_mps);
  r.d_safe_m=v*c.reaction_time_s+
      v*v/(2*std::max(.1,c.safe_decel_mps2))+c.safety_margin_m;
  r.d_radar_m=i.radar.distance_m;
  r.aeb_triggered=i.radar.valid&&std::isfinite(i.radar.distance_m)&&
      i.radar.distance_m<r.d_safe_m;
  r.fsm_state=r.aeb_triggered?"EMERGENCY_BRAKE":"CRUISE";

  const double steer=stable_reference_steer(i);
  r.control.steer=steer;

  if(r.aeb_triggered){
   r.target_speed_kmh=0;
  }else{
   r.target_speed_kmh=high_rate_target_speed(std::abs(steer));
  }

  speed(v,r.target_speed_kmh/3.6,r.control);
  if(r.aeb_triggered){
   r.control.throttle=0;
   r.control.brake=1;
  }
  return r;
 }

 // Low-rate FastBEV/FOT path is intentionally unchanged.
 Result step_with_fot(const Input&i,const std::vector<Detection>&det){
  return step(i,det,true,false);
 }

private:
 Result step(const Input&i,const std::vector<Detection>&det,bool enable_fot,bool force_cruise){
  Result r;if(i.route.size()<2){r.control.brake=.6;return r;}double v=std::max(0.0,i.ego.speed_mps);r.d_safe_m=v*c.reaction_time_s+v*v/(2*std::max(.1,c.safe_decel_mps2))+c.safety_margin_m;r.d_radar_m=i.radar.distance_m;r.aeb_triggered=i.radar.valid&&std::isfinite(i.radar.distance_m)&&i.radar.distance_m<r.d_safe_m;
  double front=std::numeric_limits<double>::infinity();std::vector<std::pair<Point,double>>obs;
  for(auto&d:det){if(d.score<c.detection_score_min)continue;Point l=lidar_to_actor(d);obs.push_back({actor_to_world(l,i.ego),.5*std::max(d.dx,d.dy)});if(d.class_id>=0&&d.class_id<=4&&l.x>0&&std::abs(l.y)<c.stop_lane_half_width_m)front=std::min(front,l.x);}
  if(r.aeb_triggered)r.fsm_state="EMERGENCY_BRAKE";else if(force_cruise)r.fsm_state="CRUISE";else if(front<c.stop_distance_m)r.fsm_state="STOP";else if(c.follow_enabled&&i.radar.valid&&std::isfinite(i.radar.relative_velocity_mps)&&3.6*(v+i.radar.relative_velocity_mps)>c.follow_min_front_speed_kmh)r.fsm_state="FOLLOW";else r.fsm_state="CRUISE";
  const std::vector<Point>* tracking_path=&i.route;std::vector<Candidate> planned;
  if(enable_fot){planned=candidates(i,obs);size_t bi=0;for(size_t n=1;n<planned.size();++n)if(planned[n].total_cost<planned[bi].total_cost)bi=n;if(!planned.empty())tracking_path=&planned[bi].points;}
  if(r.fsm_state=="STOP"||r.fsm_state=="EMERGENCY_BRAKE")r.target_speed_kmh=0;else if(r.fsm_state=="FOLLOW")r.target_speed_kmh=clampd(3.6*(v+i.radar.relative_velocity_mps),0,c.cruise_speed_kmh);else r.target_speed_kmh=c.cruise_speed_kmh;
  r.control.steer=pure(i.ego,*tracking_path);speed(v,r.target_speed_kmh/3.6,r.control);if(r.aeb_triggered){r.control.throttle=0;r.control.brake=1;}return r;
 }

 std::vector<Candidate> candidates(const Input&i,const std::vector<std::pair<Point,double>>&obs){
  std::vector<Candidate>out;for(double target:c.lateral_offsets_m){Candidate q;q.lateral_offset_m=target;double traveled=0;
   for(size_t n=0;n<i.route.size();++n){if(n)traveled+=std::hypot(i.route[n].x-i.route[n-1].x,i.route[n].y-i.route[n-1].y);size_t j=std::min(n+1,i.route.size()-1),k=n? n-1:0;double tx=i.route[j].x-i.route[k].x,ty=i.route[j].y-i.route[k].y,norm=std::max(1e-6,std::hypot(tx,ty));tx/=norm;ty/=norm;double t=clampd(traveled/8,0,1),sm=t*t*(3-2*t);Point p=i.route[n];p.x+=-ty*target*sm;p.y+=tx*target*sm;q.points.push_back(p);}
   q.reference_cost=std::abs(target);for(size_t n=2;n<q.points.size();++n){double a1=std::atan2(q.points[n-1].y-q.points[n-2].y,q.points[n-1].x-q.points[n-2].x),a2=std::atan2(q.points[n].y-q.points[n-1].y,q.points[n].x-q.points[n-1].x);q.smooth_cost+=std::abs(std::atan2(std::sin(a2-a1),std::cos(a2-a1)));}
   for(auto&p:q.points){for(auto&o:obs){double clear=std::hypot(p.x-o.first.x,p.y-o.first.y)-o.second;if(clear<c.obstacle_influence_m){double z=c.obstacle_influence_m-clear;q.obstacle_cost+=z*z;}}}if(!q.points.empty())q.obstacle_cost/=q.points.size();q.total_cost=c.w_reference*q.reference_cost+c.w_smooth*q.smooth_cost+c.w_bev*q.obstacle_cost;out.push_back(q);}return out;
 }

 // Original low-rate Pure Pursuit kept as-is.
 double pure(const Ego&e,const std::vector<Point>&path)const{
  if(path.empty())return 0;
  double look=clampd(c.lookahead_base_m+c.lookahead_gain_s*e.speed_mps,c.lookahead_min_m,c.lookahead_max_m);
  Point t=path.back();
  for(auto&p:path)if(std::hypot(p.x-e.x,p.y-e.y)>=look){t=p;break;}
  double yaw=rad(e.yaw_deg),dx=t.x-e.x,dy=t.y-e.y,lx=std::cos(yaw)*dx+std::sin(yaw)*dy,ly=-std::sin(yaw)*dx+std::cos(yaw)*dy,ld=std::max(.5,std::hypot(lx,ly)),delta=std::atan2(2*c.wheelbase_m*std::sin(std::atan2(ly,lx)),ld);
  return clampd(delta/std::max(.1,c.max_steer_angle_rad),-1,1);
 }

 static std::vector<double> cumulative_s(const std::vector<Point>&path){
  std::vector<double>s(path.size(),0.0);
  for(size_t n=1;n<path.size();++n)
   s[n]=s[n-1]+std::hypot(path[n].x-path[n-1].x,path[n].y-path[n-1].y);
  return s;
 }

 static Point point_at_s(const std::vector<Point>&path,const std::vector<double>&s,double target_s){
  if(path.empty())return {};
  if(target_s<=0)return path.front();
  if(target_s>=s.back())return path.back();
  auto it=std::lower_bound(s.begin(),s.end(),target_s);
  size_t j=static_cast<size_t>(it-s.begin());
  if(j==0)return path.front();
  const size_t k=j-1;
  const double ds=std::max(1e-9,s[j]-s[k]);
  const double t=clampd((target_s-s[k])/ds,0.0,1.0);
  Point p;
  p.x=path[k].x+t*(path[j].x-path[k].x);
  p.y=path[k].y+t*(path[j].y-path[k].y);
  p.z=path[k].z+t*(path[j].z-path[k].z);
  return p;
 }

 Projection project_forward(const Ego&e,const std::vector<Point>&path,const std::vector<double>&s){
  Projection best;
  const size_t last_seg=path.size()-2;
  size_t begin=0,end=last_seg;

  if(high_have_progress){
   begin=std::min(high_progress_segment,last_seg);
   end=begin;
   const double max_s=high_progress_s+std::max(2.0,c.high_progress_search_ahead_m);
   while(end<last_seg && s[end]<=max_s)++end;
  }

  for(size_t n=begin;n<=end;++n){
   const double ax=path[n].x,ay=path[n].y;
   const double bx=path[n+1].x,by=path[n+1].y;
   const double vx=bx-ax,vy=by-ay;
   const double len2=vx*vx+vy*vy;
   const double t=len2>1e-12?clampd(((e.x-ax)*vx+(e.y-ay)*vy)/len2,0.0,1.0):0.0;
   const double px=ax+t*vx,py=ay+t*vy;
   const double dx=e.x-px,dy=e.y-py;
   const double d2=dx*dx+dy*dy;
   const double seg_len=std::sqrt(std::max(0.0,len2));
   const double ps=s[n]+t*seg_len;
   if(d2<best.distance2){
    best.segment=n;best.t=t;best.s=ps;best.distance2=d2;
   }
  }
  return best;
 }

 double stable_reference_steer(const Input&i){
  if(i.route.size()<2)return 0;

  // Route changes reset progress. A new socket session is also reset explicitly
  // by run_high_rate_control_service().
  if(!high_route_id.empty() && !i.route_id.empty() && i.route_id!=high_route_id)
   reset_high_rate_state();
  if(!i.route_id.empty())high_route_id=i.route_id;

  const auto s=cumulative_s(i.route);
  Projection p=project_forward(i.ego,i.route,s);

  // Monotonic path progress is the key anti-jump constraint. Small lateral
  // motion can no longer make the target jump backward between path samples.
  if(high_have_progress && p.s<high_progress_s)p.s=high_progress_s;
  high_progress_s=clampd(p.s,0.0,s.back());
  auto it=std::upper_bound(s.begin(),s.end(),high_progress_s);
  high_progress_segment=it==s.begin()?0:
      std::min(static_cast<size_t>((it-s.begin())-1),i.route.size()-2);
  high_have_progress=true;

  const double look=clampd(
      c.high_lookahead_base_m+c.high_lookahead_gain_s*std::max(0.0,i.ego.speed_mps),
      c.high_lookahead_min_m,c.high_lookahead_max_m);
  const Point target=point_at_s(i.route,s,high_progress_s+look);

  const double yaw=rad(i.ego.yaw_deg);
  const double dx=target.x-i.ego.x,dy=target.y-i.ego.y;
  const double lx=std::cos(yaw)*dx+std::sin(yaw)*dy;
  const double ly=-std::sin(yaw)*dx+std::cos(yaw)*dy;
  const double ld=std::max(.5,std::hypot(lx,ly));
  const double alpha=std::atan2(ly,lx);
  const double delta=std::atan2(2*c.wheelbase_m*std::sin(alpha),ld);
  const double raw=clampd(delta/std::max(.1,c.max_steer_angle_rad),-1.0,1.0);

  // Low-pass + rate limit add damping and prevent frame-to-frame steering flips.
  const auto now=std::chrono::steady_clock::now();
  const double dt=high_have_steer?
      clampd(std::chrono::duration<double>(now-high_last_steer).count(),.02,.20):.05;
  high_last_steer=now;

  const double alpha_lp=clampd(c.high_steer_lowpass_alpha,.01,1.0);
  const double lp=high_have_steer?
      high_prev_steer+alpha_lp*(raw-high_prev_steer):
      alpha_lp*raw;
  const double max_step=std::max(.01,c.high_max_steer_rate_per_s)*dt;
  const double limited=clampd(lp,high_prev_steer-max_step,high_prev_steer+max_step);

  high_prev_steer=clampd(limited,-1.0,1.0);
  high_have_steer=true;
  return high_prev_steer;
 }

 double high_rate_target_speed(double abs_steer)const{
  const double vmax=std::max(0.0,c.cruise_speed_kmh);
  const double vmin=clampd(c.high_curve_speed_min_kmh,0.0,vmax);
  const double a=std::max(0.0,c.high_curve_speed_steer_start);
  const double b=std::max(a+1e-3,c.high_curve_speed_steer_full);
  const double t=clampd((abs_steer-a)/(b-a),0.0,1.0);
  return vmax+(vmin-vmax)*t;
 }

 void speed(double cur,double target,Control&o){
  auto now=std::chrono::steady_clock::now();
  double dt=clampd(std::chrono::duration<double>(now-last).count(),.01,.5);
  last=now;
  double err=target-cur;
  integ=clampd(integ+err*dt,-5,5);
  double der=have?(err-prev)/dt:0;
  have=true;prev=err;
  double u=c.pid_kp*err+c.pid_ki*integ+c.pid_kd*der;
  if(u>=0){o.throttle=clampd(u,0,.75);o.brake=0;}
  else{o.throttle=0;o.brake=clampd(-u,0,.8);}
 }
};
inline std::string result_json(const Result&r){std::ostringstream o;o.imbue(std::locale::classic());o<<std::setprecision(8);o<<"{\"fsm_state\":\""<<r.fsm_state<<"\",\"aeb_triggered\":"<<(r.aeb_triggered?"true":"false")<<",\"d_safe_m\":"<<r.d_safe_m<<",\"d_radar_m\":";if(std::isfinite(r.d_radar_m))o<<r.d_radar_m;else o<<"null";o<<",\"target_speed_kmh\":"<<r.target_speed_kmh<<",\"control\":{\"steer\":"<<r.control.steer<<",\"throttle\":"<<r.control.throttle<<",\"brake\":"<<r.control.brake<<"}}";return o.str();}
}
