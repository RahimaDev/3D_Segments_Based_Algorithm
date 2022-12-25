#include "Transformation.hpp"
double Energy(std::vector<Line>ln1,std::vector<Line>ln2,Eigen::Matrix4d T,Eigen::Matrix3d R0,double th)

{
    int k=0;
    double sum=0;

    for(int l=0; l<ln1.size(); l++)
    { double h=(th*th);
        Line l0=tr_line(ln1[l],R0);
        
        Line l1=tr_line(l0,T);
        double S=0,S1=0;;




        for(int f=0; f<ln2.size(); f++)
        {


            Line l2=ln2[f];


            double over, d1;
            if(std::abs(l1.d.dot(l2.d))>0.98)
            {
                if(l1.d.dot(l2.d)<0)
                {
                    Line l3=inverse_l(l1);



                    double dist1,dist2, dist3,dist4,dist;
                    dist1=PointLineDistance(l3.A,l2);
                    dist2=PointLineDistance(l3.B,l2);
                    dist3=PointLineDistance(l2.A,l3);
                    dist4=PointLineDistance(l2.B,l3);
                    dist=(dist1+dist2+dist3+dist4)/4;

                    double d=h-(dist*dist);

                    d1=std::max(0.0,d);

                    if(d1>0)
                    {


                        over=  overlap( l3, l2);



                        S+=over*d1;


                    }}

                else
                {











                    double dist1,dist2, dist3,dist4,dist;
                    dist1=PointLineDistance(l1.A,l2);
                    dist2=PointLineDistance(l1.B,l2);
                    dist3=PointLineDistance(l2.A,l1);
                    dist4=PointLineDistance(l2.B,l1);
                    dist=(dist1+dist2+dist3+dist4)/4;


                    double d=h-(dist*dist);

                    d1=std::max(0.0,d);

                    if(d1>0)
                    {


                        over=  overlap( l1, l2);



                        S+=over*d1;;}}}}
        S1=h-S;
        sum+=S1;


    }
    return sum;
}



