#ifndef BA_REPROJECTION_FACTOR_H
#define BA_REPROJECTION_FACTOR_H

#include <Eigen/Core>
#include "Common.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//
namespace BA
{
    // Standard bundle adjustment cost function for variable
    // camera pose and calibration and point parameters.
    
    class ProjectionFactor
    {
    public:
        explicit ProjectionFactor(const Vec2 &point2D, const Mat33 &intrinsic) : observed_x_(point2D(0)),
                                                                                    observed_y_(point2D(1)),
                                                                                    intrinsic_(intrinsic) {}

        static ceres::CostFunction *Create(const Vec2 &point2D, const Mat33 &intrinsic)
        {
            return (new ceres::AutoDiffCostFunction<ProjectionFactor, 2, 4, 3, 3>(
                new ProjectionFactor(point2D, intrinsic)));
        }

        template <typename T>
        bool operator()(const T *const qvec,
                        const T *const tvec,
                        const T *const point3D,
                        T *residuals) const 
        {
            // Now you have to implement here
            
           //Pass point from world coordinates to camera coordinates
            T Point_in_camera[3];
            ceres::QuaternionRotatePoint(qvec,point3D, Point_in_camera);
            cout<<"Point_in_camera x"<<Point_in_camera[0]<<" y "<<Point_in_camera[1]<<" z "<<Point_in_camera[2]<<endl;
      
            //translation 
            Point_in_camera[0]+=tvec[0];
            Point_in_camera[1]+=tvec[1];
            Point_in_camera[2]+=tvec[2];    
            cout<<"translated point x "<<Point_in_camera[0]<<" y "<<Point_in_camera[1]<<" z "<<Point_in_camera[2]<<endl;
            
            //Normalized coordinates  (check if +1 or -1)
            T Pc[2];
            Pc[0] =Point_in_camera[0]/Point_in_camera[2];
            Pc[1] =Point_in_camera[1]/Point_in_camera[2];
            cout<<"normalized point "<<Pc[0]<<" other "<<Pc[1]<<endl;

            // Pixel coordinate using intrisic 
            T u[2];
            u[0]=T(intrinsic_(0,0))*Pc[0]+T(intrinsic_(0,2));
            u[1]=T(intrinsic_(1,1))*Pc[1]+T(intrinsic_(1,2));       
            cout<<"pixel point u "<<u[0]<<" v "<<u[1]<<endl;

            //Residual solution
            residuals[0]=u[0] - T(observed_x_);
            residuals[1]=u[1] - T(observed_y_);
            cout<<"residuals u "<<residuals[0]<<" and v "<<residuals[1]<<endl;

            return true;
        }


    private:
        const double observed_x_;
        const double observed_y_;
        const Mat33 intrinsic_;
    };
}
#endif