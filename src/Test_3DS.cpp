#include"Energy.hpp"
#include <boost/random.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
//////////////////////////////////////////////////////////////
#include <boost/random.hpp>
#include <vector>
#include <random>
#include <numeric>
#include <functional>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <queue>
#include <assert.h>
#include <fstream>
#include <time.h>  
using namespace std;
using namespace cv;
bool entier(double n)
{
    int tmp = (int)n;
    if(tmp==n)
        return true;
    else

        return false;
    
    
}

bool Find(std::vector<int>V, int  val)
{
    bool F=false;
    for(int i=0; i<V.size(); i++)
    {
        if(V[i]==val)
            F= true;
    }

    return F;}

Eigen::Matrix3d X_rot(double angle)
{
    Eigen::Matrix3d rot;
    rot(0,0)=1;
    rot(0,1)=0;
    rot(0,2)=0;

    rot(1,0)=0;
    rot(1,1)=std::cos(angle);
    rot(1,2)=-std::sin(angle);
    rot(2,0)=0;
    rot(2,1)=std::sin(angle);
    rot(2,2)=std::cos(angle);
    // std::cout<<"R_X="<<std::endl;
    //std::cout<<rot<<std::endl;
    return rot;

}
///////////////////////////////////
Eigen::Matrix3d Y_rot(double angle)
{
    Eigen::Matrix3d rot;
    rot(0,0)=std::cos(angle);
    rot(0,1)=0;
    rot(0,2)=std::sin(angle);

    rot(1,0)=0;
    rot(1,1)=1;
    rot(1,2)=0;

    rot(2,0)=-std::sin(angle);
    rot(2,1)=0;
    rot(2,2)=std::cos(angle);
    //std::cout<<"R_Y="<<std::endl;
    //std::cout<<rot<<std::endl;
    return rot;

}

Eigen::Matrix3d Z_rot(double angle)
{
    Eigen::Matrix3d rot;
    rot(0,0)=std::cos(angle);
    rot(0,1)=-std::sin(angle);
    rot(0,2)=0;

    rot(1,0)=std::sin(angle);
    rot(1,1)=std::cos(angle);
    rot(1,2)=0;

    rot(2,0)=0;
    rot(2,1)=0;
    rot(2,2)=1;
    //std::cout<<"R_Z="<<std::endl;
    //std::cout<<rot<<std::endl;
    return rot;

}
Eigen::Matrix3d Matrix_Rot(double A,double B, double C)
{
    Eigen::Matrix3d M_X =X_rot(A);
    Eigen::Matrix3d M_Y =Y_rot(B);
    Eigen::Matrix3d M_Z =Z_rot(C);
    Eigen::Matrix3d M_rot=M_X*M_Y*M_Z;
    return M_rot;
}

struct data
{

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> data3;


    std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>>data2;
    std::vector<std::vector<int>>ID;

};

vector<Scalar> rgb = {Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255)};
int thicknessFromImage(const Mat &im){
    return max((im.cols + im.rows/2)/400, 2);
}


double Angle2D(Eigen::Vector2d v1, Eigen::Vector2d v2)
{
    
    
    float dot_p = v1.dot(v2);
    float angle = acos(fmax(fmin(dot_p,1.0f),-1.0f))/M_PI*180.0f;




    return angle;
}






///////////////////////////////////////////////////////////////// 



////////////////////////////////////////////////
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> read_Lidar_cloud(std::string filepath)
{std::ifstream fichier(filepath, ios::in);
    double ax,ay,az;
    int ar,av,ab,n;
    int i=0;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> dat;

    std::vector<std::vector<Eigen::Vector3d>> VEC;
    int k=0;
    std::vector<Eigen::Vector3d>V;
    if(fichier)  // si l'ouverture a fonctionné


    {
        std::string line;
        while(getline(fichier, line))

        {

            i++;

            std::istringstream buff(line);
            buff>>ax>>ay>>az>>ar>>av>>ab>>n;

            if(k==n)
            {Eigen::Vector3d p1=Eigen::Vector3d ((double)ax, (double)ay,(double)az);
                V.push_back(p1);


            }
            else
            {VEC.push_back(V);
                V.clear();
                Eigen::Vector3d p1=Eigen::Vector3d ((double)ax, (double)ay,(double)az);
                k++;
                V.push_back(p1);


            }}
        if (fichier.eof())
            VEC.push_back(V);}

    std::cout<<VEC.size()<<std::endl;

    for(int i=0; i<VEC.size(); i++)
    {
        std::vector<Eigen::Vector3d> V=VEC[i];

        int m=V.size();
        Eigen::Vector3d p1=V[0];
        Eigen::Vector3d p2=V[m-1];

        dat.push_back(std::make_pair(p1,p2));
    }
    return dat;
}
//////////////////////////////////////////////////////////////////////////////////////////


void saveResultAsOBJ(std::string filename ,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    std::ofstream file;
    file.open(filename.c_str());

    size_t lineID = 0;
    size_t pointID = 1;
    std::map<size_t,size_t> lines2points;

    for(size_t i=0; i<cloud->points.size(); i++)
    {
        if((i%2)==0)
        {Eigen::Vector3d P1 =  Eigen::Vector3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
            Eigen::Vector3d P2 =  Eigen::Vector3d(cloud->points[i+1].x,cloud->points[i+1].y,cloud->points[i+1].z);

            file << "v " << P1.x() << " " << P1.y() << " " << P1.z() << std::endl;
            file << "v " << P2.x() << " " << P2.y() << " " << P2.z() << std::endl;

            lines2points[lineID] = pointID;
            ++lineID;
            pointID+=2;

        }
        else
            continue;
    }

    std::map<size_t,size_t>::const_iterator it = lines2points.begin();
    for(; it!=lines2points.end(); ++it)
    {
        file << "l " << it->second << " " << it->second+1 << std::endl;
    }

    file.close();

}
vector<vector<int>> assoc_valid(std::pair<Cluster, Cluster> S1, std::pair<Cluster,Cluster>S2, bool& valid)
{

    Eigen::Vector3d v1,v2,v3,v4;

    double  mini=10;
    vector<vector<int>> V;
    vector<int>idx;
    idx.push_back(1);
    idx.push_back(-1);
    for(int a=0; a<idx.size(); a++)
    {

        v1=idx[a]*Mean_dir(S1.first);




        for(int b=0; b<idx.size(); b++)

        { v2=idx[b]*Mean_dir(S1.second);

            for(int c=0; c<idx.size(); c++)
            {
                v3=idx[c]*Mean_dir(S2.first);



                for(int d=0; d<idx.size(); d++)
                {
                    v4=idx[d]*Mean_dir(S2.second);

                    double val=std::abs(Angle(v1,v2)-Angle(v3,v4));
                    // if((Angle(v1,v2)<92)&&(Angle(v1,v2)>88)&&(Angle(v3,v4)<92)&&(Angle(v3,v4)>88))
                    if(val<4)
                    {vector<int>V1;


                        V1.push_back(idx[a]);
                        V1.push_back(idx[b]);
                        V1.push_back(idx[c]);
                        V1.push_back(idx[d]);
                        V.push_back(V1);

                    }
                }}}}
    if(V.size()>0)
        valid=true;

    return V;
}



int main (int argc, char** argv)
{
  cout << "Usage: " << argv[0] << " d_thr dist" << endl;
    if(argc<1) return 1;

    int m = 1;
    double d_thr=1.2;
    if (argc > m)
    d_thr = atof(argv[m++]);
    double  dist=2.;
    if (argc > m)
    dist = atof(argv[m++]);

    srand (time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud7(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> Data1=read_Lidar_cloud("../data/lidar_lines.txt");

    Eigen::Matrix4d M= Eigen::Matrix4d::Identity();

    std::vector<Line>ln1,ln2;
    std::vector<Cluster>cl1,cl2;
  double Mini=100000000000000000.0;
  Eigen::Matrix4d Tr= Eigen::Matrix4d::Identity();
  Eigen::Isometry3d IS1= Eigen::Isometry3d::Identity();
  Eigen::Isometry3d IS2 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d IS3 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d IS4 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d Qt = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d qt = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d R2= Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R=Matrix_Rot(0.43,0.61,0.66);//(25,35,38)

  Qt.linear()=R;
  M(0,0)=2;//1.25;
  M(1,1)=2;//1.25;
  M(2,2)=2;//1.25;
  M(0,3)=1.25;//0.79;//-1.76;
  M(1,3)=-2.36;//0.47;//2.5;
  M(2,3)=3.25;//-0.88;//3.8
    

    for(int i=0; i<Data1.size(); i++)
    {pcl::PointXYZ pt1=pcl::PointXYZ(Data1[i].first.x(),Data1[i].first.y(),Data1[i].first.z());
        pcl::PointXYZ pt2=pcl::PointXYZ(Data1[i].second.x(),Data1[i].second.y(),Data1[i].second.z());

        {cloud1->points.push_back(pt1);
            cloud1->points.push_back(pt2);
        } }
        
   pcl::copyPointCloud<pcl::PointXYZ,pcl::PointXYZ>(*cloud1, * cloud2); 
 // Add noise             
   boost::mt19937 rng1;
    rng1.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd1(0, 0.05);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor1(rng1, nd1);//Generate random/number
    //Add noise
    for (int i=0; i<cloud1->points.size(); i++)
    {
      cloud1->points[i].x =cloud1->points[i].x + static_cast<float> (var_nor1());
      cloud1->points[i].y = cloud1->points[i].y + static_cast<float> (var_nor1());
      cloud1->points[i].z = cloud1->points[i].z + static_cast<float> (var_nor1());
    }
    boost::mt19937 rng2;
    rng2.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd2(0, 0.05);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor2(rng2, nd2);//Generate random/number
    //Add noise
    for (int i=0; i<cloud2->points.size(); i++)
    {
      cloud2->points[i].x =cloud2->points[i].x + static_cast<float> (var_nor2());
      cloud2->points[i].y = cloud2->points[i].y + static_cast<float> (var_nor2());
      cloud2->points[i].z = cloud2->points[i].z + static_cast<float> (var_nor2());
    }
    
    
  //Apply transformation
  Eigen::Vector4f centroid0 ;
    pcl::compute3DCentroid (*cloud2, centroid0);
    pcl::PointXYZ P0=    pcl::PointXYZ(centroid0[0],centroid0[1],centroid0[2]);
       Eigen::Vector3d t0;
    t0(0)=P0.x;
    t0(1)=P0.y;
    t0(2)= P0.z;
    IS1.translation()=t0;
  for(int i=0; i<cloud2->points.size(); i++)
        {
        pcl::PointXYZ P;
        cloud2->points[i].x=cloud2->points[i].x-P0.x;
        cloud2->points[i].y=cloud2->points[i].y-P0.y;
         cloud2->points[i].z=cloud2->points[i].z-P0.z;

       }
    pcl::transformPointCloud (*cloud2, *cloud3, Qt.matrix()
                           );  
                           
                           
                    Eigen::Vector4f centroid1 ;                     
    pcl::compute3DCentroid (*cloud3, centroid1);
    pcl::PointXYZ P1=    pcl::PointXYZ(centroid1[0],centroid1[1],centroid1[2]);
       Eigen::Vector3d t1;
    t1(0)=P1.x;
    t1(1)=P1.y;
    t1(2)= P1.z;
    IS2.translation()=t1;
    for(int i=0; i<cloud3->points.size(); i++)
        {
        pcl::PointXYZ P;
        cloud3->points[i].x=cloud3->points[i].x-P1.x;
        cloud3->points[i].y=cloud3->points[i].y-P1.y;
         cloud3->points[i].z=cloud3->points[i].z-P1.z;

       }
    pcl::transformPointCloud (*cloud3, *cloud4, M);
     
  pcl::transformPointCloud(*cloud4, *cloud5, IS2.matrix()
                              );

    Eigen::Vector4f centroid3;
    
    pcl::compute3DCentroid (*cloud1, centroid3);
    pcl::PointXYZ P3=
            pcl::PointXYZ(centroid3[0],centroid3[1],centroid3[2]);
    //////////
    Eigen::Vector4f centroid2;
    pcl::compute3DCentroid (*cloud5, centroid2);
    pcl::PointXYZ P2=
            pcl::PointXYZ(centroid2[0],centroid2[1],centroid2[2]);
    Eigen::Vector3d t3;
    t3(0)=-P3.x;
    t3(1)=-P3.y;
    t3(2)= -P3.z;
    Eigen::Vector3d t2;
    t2(0)=-P2.x;
    t2(1)=-P2.y;
    t2(2)= -P2.z;
    IS3.translation()=t3;
    IS4.translation()=t2;
   
    pcl::transformPointCloud(*cloud5, *cloud5, IS4.matrix()
                              );

    pcl::transformPointCloud(*cloud1, *cloud1, IS3.matrix()
                              );





    saveResultAsOBJ("LINES1.obj",cloud1);
    
    saveResultAsOBJ("LINES2.obj",cloud5);
    



  //Delete lines

    int w=-1;
 for(int i=0; i<cloud1->points.size(); i++)
    {
        if((i%2)==0)
        {
        w++;
  if((i%8)!=0)
    {     
        Eigen::Vector3d P1 =  Eigen::Vector3d(cloud1->points[i].x,cloud1->points[i].y,cloud1->points[i].z);
            Eigen::Vector3d P2 =  Eigen::Vector3d(cloud1->points[i+1].x,cloud1->points[i+1].y,cloud1->points[i+1].z);
            //std::cout<<P1<<"        "<<P2<<std::endl;
            Line L2;

            L2.A=Eigen::Vector3d(P1.x(), P1.y(), P1.z());
            L2.B=Eigen::Vector3d(P2.x(), P2.y(), P2.z());;
            L2.d=direct_vec(L2.A,L2.B);
            pcl::PointXYZ p1=pcl::PointXYZ(L2.A.x(), L2.A.y(), L2.A.z());
            pcl::PointXYZ p2=pcl::PointXYZ(L2.B.x(), L2.B.y(), L2.B.z());
            L2.length=pcl::geometry::distance(p1,p2);
           L2.Id=w;
            std::cout<<L2.length<<std::endl;
            
            ln1.push_back(L2);


        }}
        else
            continue;
    }



    int w1=-1;
    for(int i=0; i<cloud5->points.size(); i++)
    {
        if((i%2)==0)
        {
        w1++;
        
if((i%6)!=0)
    {
 
        Eigen::Vector3d P1 =  Eigen::Vector3d(cloud5->points[i].x,cloud5->points[i].y,cloud5->points[i].z);
            Eigen::Vector3d P2 =  Eigen::Vector3d(cloud5->points[i+1].x,cloud5->points[i+1].y,cloud5->points[i+1].z);
            //std::cout<<P1<<"        "<<P2<<std::endl;
            Line L2;

            L2.A=Eigen::Vector3d(P1.x(), P1.y(), P1.z());
            L2.B=Eigen::Vector3d(P2.x(), P2.y(), P2.z());;
            L2.d=direct_vec(L2.A,L2.B);
            pcl::PointXYZ p1=pcl::PointXYZ(L2.A.x(), L2.A.y(), L2.A.z());
            pcl::PointXYZ p2=pcl::PointXYZ(L2.B.x(), L2.B.y(), L2.B.z());
            L2.length=pcl::geometry::distance(p1,p2);
            std::cout<<L2.length<<std::endl;
           L2.Id=w1;
            ln2.push_back(L2);


     
        }}
        else
            continue;
    }
    
    for(int i=0;i<ln1.size() - 1;i++)
    {
        for(int j=i+1;j<ln1.size();j++)
        {
            if(ln1[j].length> ln1[i].length)
            {
                Line temp=ln1[j];
                ln1[j]= ln1[i];
                ln1[i]=temp;
            }
        }
    }
    for(int i=0;i<ln2.size() - 1;i++)
    {
        for(int j=i+1;j<ln2.size();j++)
        {
            if(ln2[j].length> ln2[i].length)
            {
                Line temp=ln2[j];
                ln2[j]= ln2[i];
                ln2[i]=temp;
            }
        }
    }



    ////Clustering
    cl1=Cluster_generation(ln1);


    cl2=Cluster_generation(ln2);

    


    

    /////RANSAC
    int k=0;
    do{



        int a=rand() % cl1.size();

        int b=rand() % cl1.size();

        int c=rand() % cl2.size();

        int d=rand() % cl2.size();
        Cluster C1,C2,C3,C4;
        C1=cl1[a];
        C2=cl1[b];
        C3=cl2[c];
        C4=cl2[d];


        std::pair<Cluster, Cluster>S1=make_pair(C1,C2);
        std::pair<Cluster, Cluster>S2=make_pair(C3,C4);
        bool valid =false;
        vector<vector<int>> VEC1 =assoc_valid(S1,S2,valid);
        if(valid==true)
        {



            int Q=0,W=0;
            Eigen::Matrix3d R0= Eigen::Matrix3d::Identity();
            Eigen::Vector3d u0,u2,v0,v2;

            int e=rand() % C1.Lines.size();
            int n=rand() % C2.Lines.size();
            int g=rand() % C3.Lines.size();
            int m=rand() % C4.Lines.size();


            if((Line_To_Line_dist(C1.Lines[e],C2.Lines[n])>dist)&&(Line_To_Line_dist(C3.Lines[g],C4.Lines[m])>dist))
            {
                for(int i=0; i<VEC1.size(); i++)
                {     std::vector<std::pair<Line, Line>>h;
                    vector<int>V=VEC1[i];
                    h.push_back(std::make_pair(C1.Lines[e],C3.Lines[g]));
                    h.push_back(std::make_pair(C2.Lines[n],C4.Lines[m]));

                    u0=V[0]*Mean_dir(C1);
                    u2=V[2]*Mean_dir(C3);
                    v0=V[1]*Mean_dir(C2);
                    v2=V[3]*Mean_dir(C4);


                    R0=Rotation(u0,u2,v0,v2);
                    double N=angles(R0);
                    //if(N>0)
                    {





                        std::vector<Eigen::Vector3d> src,dst,dir;
                        for(int s=0; s<h.size(); s++)
                        {       Line L=h[s].first;
                            Line L2=tr_line(L,R0);
                            src.push_back(L2.A);
                            dst.push_back(h[s].second.A);
                            dir.push_back(h[s].second.d);

                            src.push_back(L2.B);
                            dst.push_back(h[s].second.A);
                            dir.push_back(h[s].second.d);

                        }

                        int q=-1;
                        Eigen::Matrix4d T= Eigen::Matrix4d::Identity();
                        T= scale_tr(src,dst,dir);

                        if(T(0,0)>0)


                        {k++;
                            double sum=Energy(ln1,ln2,T,R0,d_thr);//1.2


                            if(sum<Mini)
                            {std::cout<<k<<std::endl;

                                Mini=sum;
                                R2=R0;
                                Tr=T;

                                std::cout<<"Mini="<<Mini<<std::endl;
                                std::cout<<"Angle="<<angles(R2)<<std::endl;
                                std::cout<<Tr<<std::endl;



                            }}}}}}}
    while (k<50000);
    std::cout<<Tr<<std::endl;
    std::cout<<"Angle="<< angles(R2)<<std::endl;
    qt.linear()=R2;

    std::cout<<"Mini="<< Mini<<std::endl;
    return 0;
}
