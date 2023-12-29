#include <iostream>
#include <bits/stdc++.h>
using namespace std;
const double PI =3.1415926535897932384626433832795028841971;
double w=19;//需求量
double w1=3600;//利润
double w2=0.00015;//缺失材料数
double w3=-40;
double w4=-9;
double w5=0;
double w_d=100000;
double w_sta=0;
double w_a=0;
int first_7=-1;
struct work_table{
	int type,time_remain,mat_state,pro_state;
	double x,y;
	double profit;
	int flag[10];
	int need;
};
struct robot{
	int ID,type;
	double time,collide,lineSpeed_x,lineSpeed_y;
	double angleSpeed,direction,x,y;
	int table_id;
	double new_ngleSpeed;
	double r;
};
char mp[105][105]; 
int all_w[20];
int state[20];
void read_map(){
	for(int i=0;i<100;i++)
		for(int t=0;t<100;t++){
			cin>>mp[i][t];
			if(mp[i][t]>='1'&&mp[i][t]<='9'){
				int h=mp[i][t]-48;
				all_w[h]++;
			}
		}
	string a;
	cin>>a;
}
double dis(robot robots,work_table work_tables){
	return sqrt((robots.x-work_tables.x)*(robots.x-work_tables.x)+(robots.y-work_tables.y)*(robots.y-work_tables.y));
}
double dis_w(work_table work_table1,work_table work_table2){
	return sqrt((work_table1.x-work_table2.x)*(work_table1.x-work_table2.x)+(work_table1.x-work_table2.x)*(work_table1.y-work_table2.y));
}
double dis_r(robot robot1,robot robot2){
	return sqrt((robot1.x-robot2.x)*(robot1.x-robot2.x)+(robot1.y-robot2.y)*(robot1.y-robot2.y));
}
double cal_angle(work_table work_tables,robot robots){
	double dy=work_tables.y-robots.y,dx=work_tables.x-robots.x;
    double rotation_angle = atan2(dy, dx) - robots.direction;
    if (rotation_angle > PI) {
        rotation_angle -= 2 * PI;
    } else if (rotation_angle < -PI) {
        rotation_angle += 2 * PI;
    }
    return rotation_angle;
}
double control(robot &robots,work_table *work_tables,int k,int *remain_mat,int frameID,int ID,robot *robotAll){
	if(robots.type==0){
		int mi=0,flag=0;
		double sum=1e10;
		for(int i=0;i<k;i++){
			if((work_tables[i].flag[0]==0)&&remain_mat[work_tables[i].type]>0&&work_tables[i].pro_state>0){
				int all_r=0;
				all_r+=dis(robotAll[ID],work_tables[i])+dis(robotAll[(ID+1)%4],work_tables[i]);
				if(dis(robots,work_tables[i])+w1/work_tables[i].profit+remain_mat[work_tables[i].type]*w2+all_r*w_a<sum){
					mi=i,flag=1; 
					sum=dis(robots,work_tables[mi])+w1/work_tables[mi].profit+remain_mat[work_tables[i].type]*w2+all_r*w_a;
				}
			}
			
		}
		if(flag==0) mi=k+ID;
		work_tables[mi].flag[0]=1;
		if(flag!=0)remain_mat[work_tables[mi].type]--;
		robots.table_id=mi;
		robots.new_ngleSpeed=cal_angle(work_tables[mi],robots);
	}
	else if(robots.type==1){
		int mi=0,flag=0;
		double sum=1e10;
		for(int i=0;i<k;i++){
			if(work_tables[i].type==4||work_tables[i].type==5||work_tables[i].type==9){
				if(work_tables[i].flag[robots.type]==0&&((((1<<robots.type)&work_tables[i].mat_state))==0)){
					double w_n=0,w_4=0;
					if(remain_mat[work_tables[i].type]) w_n=w3;
					if(work_tables[i].type==4) w_4=w5;
					if(dis(robots,work_tables[i])+w*work_tables[i].need+w_n+remain_mat[work_tables[i].type]*w4+w_4+state[work_tables[i].type]*w_sta<sum){
						sum=dis(robots,work_tables[i])+w*work_tables[i].need+w_n+remain_mat[work_tables[i].type]*w4+w_4+state[work_tables[i].type]*w_sta,mi=i;
						flag=1;
					}
				}
			}
			
		}
		if(flag==0) robots.new_ngleSpeed=0;
		work_tables[mi].flag[robots.type]=1;
		robots.table_id=mi;
		robots.new_ngleSpeed=cal_angle(work_tables[mi],robots);
	}
	else if(robots.type==2){
		int mi=0,flag=0;
		double sum=1e10;
		for(int i=0;i<k;i++){
			if(work_tables[i].type==4||work_tables[i].type==6||work_tables[i].type==9){
				if(work_tables[i].flag[robots.type]==0&&((((1<<robots.type)&work_tables[i].mat_state))==0)){
					double w_n=0,w_4=0;
					if(remain_mat[work_tables[i].type]) w_n=w3;
					if(work_tables[i].type==4) w_4=w5;
					if(dis(robots,work_tables[i])+w*work_tables[i].need+remain_mat[work_tables[i].type]*w4+w_4+state[work_tables[i].type]*w_sta<sum){
						sum=dis(robots,work_tables[i])+w*work_tables[i].need+remain_mat[work_tables[i].type]*w4+w_4+state[work_tables[i].type]*w_sta,mi=i;
						flag=1;
					}
				} 
				
			}
		}
		if(flag==0) robots.new_ngleSpeed=0;
		work_tables[mi].flag[robots.type]=1;
		robots.table_id=mi;
		robots.new_ngleSpeed=cal_angle(work_tables[mi],robots);
	}
	else if(robots.type==3){
		int mi=0,flag=0;
		double sum=1e10;
		for(int i=0;i<k;i++){
			if(work_tables[i].type==6||work_tables[i].type==5||work_tables[i].type==9){	
				if(work_tables[i].flag[robots.type]==0&&((((1<<robots.type)&work_tables[i].mat_state))==0)){
					double w_n=0;
					if(remain_mat[work_tables[i].type]) w_n=w3;
					if(dis(robots,work_tables[i])+w*work_tables[i].need+w_n+remain_mat[work_tables[i].type]*w4+state[work_tables[i].type]*w_sta<sum){
						sum=dis(robots,work_tables[i])+w*work_tables[i].need+w_n+remain_mat[work_tables[i].type]*w4+state[work_tables[i].type]*w_sta,mi=i;
						flag=1;
					}
				} 
			}
		}
		if(flag==0) robots.new_ngleSpeed=0;
		work_tables[mi].flag[robots.type]=1;
		robots.table_id=mi;
		robots.new_ngleSpeed=cal_angle(work_tables[mi],robots);
	}
	else if(robots.type==4||robots.type==6||robots.type==5){
		int mi=0,flag=0;
		double sum=1e10;
		for(int i=0;i<k;i++){
			if(work_tables[i].type==7||work_tables[i].type==9){
				if(work_tables[i].flag[robots.type]==0&&((((1<<robots.type)&work_tables[i].mat_state))==0)){
					if(dis(robots,work_tables[i])+w*work_tables[i].need<sum){
						sum=dis(robots,work_tables[i])+w*work_tables[i].need,mi=i;
						flag=1;
					}
				} 
			}
		}
		if(flag==0) robots.new_ngleSpeed=0;
		work_tables[mi].flag[robots.type]=1;
		robots.table_id=mi;
		robots.new_ngleSpeed=cal_angle(work_tables[mi],robots);
	}
	else if(robots.type==7){
		int mi=0,flag=0;
		double sum=1e10;
		for(int i=0;i<k;i++){
			if(work_tables[i].type==8||work_tables[i].type==9){
				if(work_tables[i].flag[robots.type]==0&&((((1<<robots.type)&work_tables[i].mat_state))==0)){
					if(dis(robots,work_tables[i])<sum){
						sum=dis(robots,work_tables[i]),mi=i;
						flag=1;
					}
				} 
			}
		}
		if(flag==0) robots.new_ngleSpeed=0;
		work_tables[mi].flag[robots.type]=1;
		robots.table_id=mi;
		robots.new_ngleSpeed=cal_angle(work_tables[mi],robots);
	}
	return 0;
}
void check_remain_mat(work_table *work_tables,int *remain_mat,int k,robot *robots){
	for(int i=0;i<k;i++){
		if(work_tables[i].type==4){
			if(((1<<1)&work_tables[i].mat_state)==0) remain_mat[1]++;
			if(((1<<2)&work_tables[i].mat_state)==0) remain_mat[2]++;
		}
		if(work_tables[i].type==5){
			if(((1<<1)&work_tables[i].mat_state)==0) remain_mat[1]++;
			if(((1<<3)&work_tables[i].mat_state)==0) remain_mat[3]++;
		}
		if(work_tables[i].type==6){
			if(((1<<2)&work_tables[i].mat_state)==0) remain_mat[2]++;
			if(((1<<3)&work_tables[i].mat_state)==0) remain_mat[3]++;
		}
		if(work_tables[i].type==7){
			for(int t=4;t<=6;t++){
				if(((1<<t)&work_tables[i].mat_state)==0) remain_mat[t]++;
			}
		}
		if(work_tables[i].type==8){
			if(((1<<7)&work_tables[i].mat_state)==0) remain_mat[7]++;
		}
		if(work_tables[i].type==9){
			for(int t=1;t<=7;t++){
				if(((1<<t)&work_tables[i].mat_state)==0) remain_mat[t]++;
			}
		}
	}
	for(int i=0;i<=3;i++){
		remain_mat[robots[i].type]--;
	}
	for(int i=1;i<=9;i++)	state[i]=remain_mat[i];
}
int main() {
	for(int i=1;i<=10;i++){
		all_w[i]=0;
	}
    read_map();
	puts("OK");
    fflush(stdout);
    int frameID;
	if(all_w[7]==8){//1
		w=19.2,w1=4000,w2=0.00015,w3=0,w4=0,w_d=2.8,w_sta=-10000;
	}
	else if(all_w[7]==2){//2
		w=18,w1=4000,w2=0.00015,w3=-40,w4=0,w_sta=-15;
	}
	else if(all_w[7]==0){//3
		w=7,w1=4000,w2=0.00015,w3=0,w4=0,w_sta=-2,w_a=0.00095;
	}
	else if(all_w[7]==1){//4
		w=19,w1=4000,w2=0.00015,w3=-39,w4=-20,w5=-25.5;
	}
    while (scanf("%d", &frameID) != EOF) {
    	printf("%d\n", frameID);
        int money=0,k;
        cin>>money>>k;
        work_table work_tables[k+10];
        robot robots[5];
        int remain_mat[15];
        for(int i=0;i<10;i++) remain_mat[i]=0;
		//处理工作台信息
        for(int i=0;i<k;i++){
        	cin>>work_tables[i].type>>work_tables[i].x>>work_tables[i].y;
			cin>>work_tables[i].time_remain;
			cin>>work_tables[i].mat_state>>work_tables[i].pro_state;
			for(int t=0;t<10;t++) work_tables[i].flag[t]=0;
			if(work_tables[i].type==6||work_tables[i].type==5||work_tables[i].type==4){
				work_tables[i].need=2;
			}
			else if(work_tables[i].type==9){
				work_tables[i].need=100;
			}
			else if(work_tables[i].type==7){
				work_tables[i].need=3;
				if(first_7==-1) first_7=i;
			} 	
			int h=work_tables[i].mat_state;
			while(h){
				if(h&1) work_tables[i].need--;
				h>>=1;
			}
			if(work_tables[i].type==1) work_tables[i].profit=3000; 
			if(work_tables[i].type==2) work_tables[i].profit=3200; 
			if(work_tables[i].type==3) work_tables[i].profit=3400; 
			if(work_tables[i].type==4) work_tables[i].profit=7100; 
			if(work_tables[i].type==5) work_tables[i].profit=7800; 
			if(work_tables[i].type==6) work_tables[i].profit=8300; 
			if(work_tables[i].type==7) work_tables[i].profit=29000; 
		}
		work_tables[k].x=25,work_tables[k].y= 26,work_tables[k].type=0;
		work_tables[k+1].x=24,work_tables[k+1].y=25,work_tables[k+1].type=0;
		work_tables[k+2].x=26,work_tables[k+2].y= 25,work_tables[k+2].type=0;
		work_tables[k+3].x=25,work_tables[k+3].y= 24,work_tables[k+3].type=0;
		for(int i=0;i<=3;i++){
			cin>>robots[i].ID>>robots[i].type>>robots[i].time>>robots[i].collide;
			cin>>robots[i].angleSpeed>>robots[i].lineSpeed_x>>robots[i].lineSpeed_y>>robots[i].direction>>robots[i].x>>robots[i].y;
			if(robots[i].type!=0) robots[i].r=0.53;
			else robots[i].r=0.45;
		}
		string a;
		cin>>a;
		check_remain_mat(work_tables,remain_mat,k,robots);
		for(int i=0;i<k;i++){
			if(work_tables[i].time_remain>=0) state[work_tables[i].type]--;
			if(work_tables[i].pro_state) state[work_tables[i].type]--;
		}
		for(int i=0;i<4;i++){
			control(robots[i],work_tables,k,remain_mat,frameID,i,robots);
		}
        for(int i = 0; i< 4; i++){
			////////////////////////////
        	if(robots[i].type==0&&work_tables[robots[i].table_id].type>3){
				int check=0;
				for(int t=1;t<=9;t++){
					if(work_tables[robots[i].table_id].flag[t]) check=1;
				}
				if(check&&dis(robots[i],work_tables[robots[i].table_id])>=w_d){
					remain_mat[work_tables[robots[i].table_id].type]++;
					control(robots[i],work_tables,k,remain_mat,frameID,i,robots);
				}
			}
			//////////////////////////////////
        	double x=dis(robots[i],work_tables[robots[i].table_id]); 
        	double forward =6*x*0.2;
        	if(forward>6) forward=6; 
        	double angleSpeed=robots[i].new_ngleSpeed;
			for(int t=0;t<4;t++){
				if(t==i) continue;
				double r_r = atan2(robots[t].y-robots[i].y,robots[t].x-robots[i].x);
				double r_w=atan2(work_tables[robots[i].table_id].y-robots[i].y,work_tables[robots[i].table_id].x-robots[i].x);
				double angle=asin((robots[i].r+robots[t].r)/dis_r(robots[i],robots[t]));
				if(r_w>=r_r-angle&&r_w<=r_r+angle){
					if(r_w<=r_r) angleSpeed-=angle;
					else angleSpeed+=angle;
				}
			}
        	if(angleSpeed==0) forward=0;
        	else if(abs(angleSpeed)<=0.2) forward=6,angleSpeed=0;
        	if(angleSpeed>0) angleSpeed=PI*x*10;
			else if(angleSpeed<0)  angleSpeed=-PI*x*10;
			if(angleSpeed>PI) angleSpeed=PI;
			if(angleSpeed<-PI) angleSpeed=-PI;
			if(robots[i].y>=47&&robots[i].direction>0&&fabs(robots[i].direction-0.5*PI)<0.4) forward=2.5;
			if(robots[i].y<=3&&robots[i].direction<0&&fabs(robots[i].direction+0.5*PI)<0.4) forward=2.5;
			if(robots[i].x>=47&&robots[i].direction>-0.5*PI&&robots[i].direction<0.5*PI&&fabs(robots[i].direction)<0.4) forward=2.5;
			if(robots[i].x<=3&&robots[i].direction<0&&robots[i].direction<-0.5*PI&&robots[i].direction>0.5*PI){
				if(robots[i].direction>0&&fabs(robots[i].direction-PI)<0.4) forward=2.5;
				if(robots[i].direction<0&&fabs(robots[i].direction+PI)<0.4) forward=2.5;
			}
            printf("forward %d %lf\n", i, forward);
            printf("rotate %d %lf\n", i, angleSpeed);
            if(robots[i].ID!=-1&&robots[i].table_id==robots[i].ID){
            	if(robots[i].type==0&&frameID<=8700){
            		printf("buy %d\n", i);
				} 
            	else printf("sell %d\n", i);
			}
        }
        printf("OK\n");
        fflush(stdout);
    }
    return 0;
}
