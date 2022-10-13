#include"Energy.hpp"

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



data read_image_cloud(std::string filepath)
{std::ifstream fichier(filepath, ios::in);
    double N;
    int i=0;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> d1;
    std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>> d2;
    std::vector<std::vector<int>>id;
    data D;
    if(fichier)  // si l'ouverture a fonctionné

    {
        std::string line;
        while(getline(fichier, line))

        {i++;
            std::istringstream buff(line);
            buff>>N;
            if(N==1)
            {
                std::istringstream iss(line);

                vector<double> vect1;
                vector<double> vect;
                double q;
                while(iss>> q)
                {vect.push_back(q);}
                //std::cout<<vect.size()<<std::endl;

                std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> VV;

                std::vector<int> VEC;
                int g=-1;
                for(int p=0; p<vect.size(); p++)

                {

                    if((entier(vect[p]))&&(entier(vect[p+1]))&&(entier(vect[p+2])))
                    {g=p+1;

                    }
                    if(g!=-1)
                        break;

                }
                std::cout<<"g="<<g<<std::endl;

                for(int j=g; j<vect.size(); j++)

                {
                    vect1.push_back(vect[j]);
                    //  std::cout<<"vect="<<vect[j]<<std::endl;

                }

                for(int f=0;f<vect1.size(); f++)
                {
                    if((entier(vect1[f]))&&(entier(vect1[f+1])))
                    {
                        VEC.push_back(vect1[f]);
                    }}
                for(int f=0;f<vect1.size(); f++)
                {
                    if((entier(vect1[f]))&&(entier(vect1[f+1])))
                    {  int h=f+2;
                        Eigen::Vector2d P1=Eigen::Vector2d ((double)vect1[h], (double)vect1[h+1]);
                        Eigen::Vector2d P2=Eigen::Vector2d ((double)vect1[h+2], (double)vect1[h+3]);
                        VV.push_back(std::make_pair(P1,P2));
                    }}
                Eigen::Vector3d p1=Eigen::Vector3d ((double)vect[1], (double)vect[2],(double)vect[3]);
                Eigen::Vector3d p2=Eigen::Vector3d ((double)vect[4], (double)vect[5],(double)vect[6]);
                d1.push_back(std::make_pair(p1,p2));
                d2.push_back(VV);
                id.push_back(VEC);






            }}}
    D.data3=d1;
    D.data2=d2;
    D.ID=id;
    return D ;
}
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




pcl::PointCloud<pcl::PointXYZ>::Ptr data_to_cloud(Cluster  C)
{ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<C.Lines.size(); i++)

    {
        pcl::PointXYZ pt1=pcl::PointXYZ(C.Lines[i].A.x(),C.Lines[i].A.y(),C.Lines[i].A.z());
        pcl::PointXYZ pt2=pcl::PointXYZ(C.Lines[i].B.x(),C.Lines[i].B.y(),C.Lines[i].B.z());

        cloud->points.push_back(pt1);
        cloud->points.push_back(pt2);}
    return cloud;

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
{cout << "Usage: " << argv[0] << " file1.txt file2.txt    d_thr dist" << endl;
    if(argc<1) return 1;

    int m = 1;
    string file1="", file2="", file3="", file4="";
    if (argc > m)
        file1 = argv[m++];

    if (argc > m)
        file2 = argv[m++];
    

    double d_thr=0.54;
    if (argc > m)
        d_thr = atof(argv[m++]);
    double  dist=6.;
    if (argc > m)
        dist = atof(argv[m++]);
    srand (time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
    data Data=read_image_cloud(file1);
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> D1=Data.data3;
    std::vector<std::vector<int>>D3=Data.ID;
    std::vector<std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>> D2= Data.data2;;
    ///
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> Data1=read_Lidar_cloud(file2);

    Eigen::Matrix3d R2= Eigen::Matrix3d::Identity();


    Eigen::Matrix4d T= Eigen::Matrix4d::Identity();


    Eigen::Matrix4d Tr= Eigen::Matrix4d::Identity();
    double Mini=100000000;


    Eigen::Isometry3d qt = Eigen::Isometry3d::Identity();

    std::vector<Line> ln1,ln2;
    Eigen::Isometry3d IS1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d IS2 = Eigen::Isometry3d::Identity();


    
    for(int i=0; i<D1.size(); i++)
    {pcl::PointXYZ pt1=pcl::PointXYZ(D1[i].first.x(),D1[i].first.y(),D1[i].first.z());
        pcl::PointXYZ pt2=pcl::PointXYZ(D1[i].second.x(),D1[i].second.y(),D1[i].second.z());

        {cloud1->points.push_back(pt1);
            cloud1->points.push_back(pt2);}
    }
    
    for(int i=0; i<Data1.size(); i++)
    {pcl::PointXYZ pt1=pcl::PointXYZ(Data1[i].first.x(),Data1[i].first.y(),Data1[i].first.z());
        pcl::PointXYZ pt2=pcl::PointXYZ(Data1[i].second.x(),Data1[i].second.y(),Data1[i].second.z());

        {cloud2->points.push_back(pt1);
            cloud2->points.push_back(pt2);
        } }


    Eigen::Vector4f centroid1;
    
    pcl::compute3DCentroid (*cloud1, centroid1);
    pcl::PointXYZ P1=
            pcl::PointXYZ(centroid1[0],centroid1[1],centroid1[2]);
    //////////
    Eigen::Vector4f centroid2;
    pcl::compute3DCentroid (*cloud2, centroid2);
    pcl::PointXYZ P2=
            pcl::PointXYZ(centroid2[0],centroid2[1],centroid2[2]);
    Eigen::Vector3d t1;
    t1(0)=-P1.x;
    t1(1)=-P1.y;
    t1(2)= -P1.z;
    Eigen::Vector3d t2;
    t2(0)=-P2.x;
    t2(1)=-P2.y;
    t2(2)= -P2.z;
    IS1.translation()=t1;
    IS2.translation()=t2;
    pcl::transformPointCloud (*cloud2, *cloud2, IS2.matrix()
                              );

    pcl::transformPointCloud (*cloud1, *cloud1, IS1.matrix()
                              );




    



    saveResultAsOBJ("IMG_LINES.obj",cloud1);
    
    saveResultAsOBJ("LIDAR_LINES.obj",cloud2);



    int w=-1;
    for(int i=0; i<cloud1->points.size(); i++)
    {
        if((i%2)==0)
        {Eigen::Vector3d P1 =  Eigen::Vector3d(cloud1->points[i].x,cloud1->points[i].y,cloud1->points[i].z);
            Eigen::Vector3d P2 =  Eigen::Vector3d(cloud1->points[i+1].x,cloud1->points[i+1].y,cloud1->points[i+1].z);

            Line L3;
            w++;
            L3.A=Eigen::Vector3d(P1.x(), P1.y(), P1.z());
            L3.B=Eigen::Vector3d(P2.x(), P2.y(), P2.z());;
            L3.d=direct_vec(L3.A,L3.B);
            pcl::PointXYZ p1=pcl::PointXYZ(L3.A.x(), L3.A.y(), L3.A.z());
            pcl::PointXYZ p2=pcl::PointXYZ(L3.B.x(), L3.B.y(), L3.B.z());
            L3.length=pcl::geometry::distance(p1,p2);
            L3.Id=w;
            if(L3.length>0.01)
                ln1.push_back(L3);


        }
        else
            continue;
    }

    int B=-1;
    for(int i=0; i<cloud2->points.size(); i++)
    {
        if((i%2)==0)
        {Eigen::Vector3d P1 =  Eigen::Vector3d(cloud2->points[i].x,cloud2->points[i].y,cloud2->points[i].z);
            Eigen::Vector3d P2 =  Eigen::Vector3d(cloud2->points[i+1].x,cloud2->points[i+1].y,cloud2->points[i+1].z);
            //std::cout<<P1<<"        "<<P2<<std::endl;
            Line L2;
            B++;
            L2.A=Eigen::Vector3d(P1.x(), P1.y(), P1.z());
            L2.B=Eigen::Vector3d(P2.x(), P2.y(), P2.z());;
            L2.d=direct_vec(L2.A,L2.B);
            pcl::PointXYZ p1=pcl::PointXYZ(L2.A.x(), L2.A.y(), L2.A.z());
            pcl::PointXYZ p2=pcl::PointXYZ(L2.B.x(), L2.B.y(), L2.B.z());
            L2.length=pcl::geometry::distance(p1,p2);
            L2.Id=B;
            if(L2.length>0.01)
                ln2.push_back(L2);


        }
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


    std::vector<Cluster> cl1,cl2;
    ////Clustering
    cl1=Cluster_generation(ln1);


    cl2=Cluster_generation(ln2);

    /// selecting the vertical clusters

    Cluster V1,V2;


    ///LiDAR vertical cluster
    Eigen::Vector3d Z(0,0,1);
    double M1=1000;
    int g=-1;
    for(int i=0; i<cl2.size(); i++)
    {
        double val=1-std::abs(Mean_dir(cl2[i]).dot(Z));

        if(val<M1)
        {
            M1=val;
            g=i;
        }

    }
    V2=cl2[g];
    cl2.erase(cl2.begin()+g);



    //Image vertical cluster
    int O=-1;;
    Eigen::Vector2d Y(0,1);

    double MM=10000;

    for(int i=0; i<cl1.size(); i++)
    {
        Cluster C=cl1[i];

        double summ=0;
        for(int j=0; j<C.Lines.size(); j++)
        {
            Line L=C.Lines[j];

            int Id=L.Id;

            double signe=L.d.dot(Z);
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> K=D2[Id];
            std::vector<int>vec=D3[Id];
            Eigen::Vector2d E;
            double ang=0;
            int e=0;

            std::vector<int>dx;
            for(int r=0; r<vec.size(); r++)
            {
                if(Find(dx,vec[r])==false)
                {
                    dx.push_back(vec[r]);
                    int ic=vec[r];
                    int b=0;

                    double An=0;
                    for(int f=0; f<K.size(); f++)
                    {    if(vec[f]==ic)
                        {
                            b++;

                            double xx1=K[f].first.x();
                            double yy1=K[f].first.y();
                            double xx2=K[f].second.x();
                            double yy2=K[f].second.y();
                            Eigen::Vector2d pt1=Eigen::Vector2d(xx1,yy1);
                            Eigen::Vector2d pt2=Eigen::Vector2d(xx2,yy2);
                            Eigen::Vector2d v,VV,vv;
                            double an=0;
                            double w=std::sqrt(((xx2-xx1)*(xx2-xx1))+((yy2-yy1)*(yy2-yy1)));
                            v=pt2-pt1;
                            vv=v/v.norm();
                            if(vv.dot(Y)<0)

                                VV=(-1)*vv;

                            else


                                VV=vv;
                            an=Angle2D(Y,VV);
                            an*=w;
                            An+=an;

                            b+=w;
                        }}

                    An=An/b;

                    e++;
                    ang+=An;
                }}
            ang=ang/e;
            summ+=ang;
        }



        std::cout<<"size="<<C.Lines.size()<<std::endl;
        summ=summ/C.Lines.size();
        std::cout<<"angle cluster="<<summ<<std::endl;
        if(summ<MM)
        {
            MM=summ;
            O=i;
        }}

    std::cout<<"I="<<O<<std::endl;
    if(O!=-1)
    {
        V1=cl1[O];
        cl1.erase(cl1.begin()+O);
    }

    /////RANSAC
    int k=0;
    do{



        int a=rand() % cl1.size();

        int b=rand() % cl2.size();


        Cluster C1,C2,C3,C4;
        C1=V1;
        C2=cl1[a];
        C3=V2;
        C4=cl2[b];


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
                    if(N>0)
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
    while (k<500000);
    std::cout<<Tr<<std::endl;
    std::cout<<"Angle="<< angles(R2)<<std::endl;
    qt.linear()=R2;
    pcl::transformPointCloud (*cloud1, *cloud3, qt.matrix()
                              );
    saveResultAsOBJ("Rot_lines.obj",cloud3);

    pcl::transformPointCloud (*cloud3, *cloud4, Tr
                              );
    saveResultAsOBJ("TR_lines.obj",cloud4);
    std::cout<<"Mini="<< Mini<<std::endl;
    return 0;
}

