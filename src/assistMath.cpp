#include "assistMath.h"

void qToEuler(float& _roll,float& _pitch,float& _yaw,float q[])
{
    float_t w,x,y,z;
    //diffrent coordinates
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];
    //cout<<"q:"<<w<<"    "<<x<<"    "<<y<<"    "<<z<<endl;
    _roll  = atan2(2 * (w * x + y* z) , 1 - 2 * (x * x + y * y));
    if(2 * (w * y - z * x)>1)_pitch =asin(1.0) ;
    else if(2 * (w * y - z * x)<-1.0)_pitch = asin(-1.0);
    else _pitch = asin(2 * (w * y - z * x));
    _yaw   = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));
}

void eulerToQ(float_t roll,float_t pitch,float_t yaw,float q[])
{
    float fCosHRoll = cos(roll * .5f);
    float fSinHRoll = sin(roll * .5f);
    float fCosHPitch = cos(pitch * .5f);
    float fSinHPitch = sin(pitch * .5f);
    float fCosHYaw = cos(yaw * .5f);
    float fSinHYaw = sin(yaw * .5f);

    /// Cartesian coordinate System
    q[0] = fCosHRoll * fCosHPitch * fCosHYaw + fSinHRoll * fSinHPitch * fSinHYaw;
    q[1] = fSinHRoll * fCosHPitch * fCosHYaw - fCosHRoll * fSinHPitch * fSinHYaw;
    q[2]= fCosHRoll * fSinHPitch * fCosHYaw + fSinHRoll * fCosHPitch * fSinHYaw;
    q[3] = fCosHRoll * fCosHPitch * fSinHYaw - fSinHRoll * fSinHPitch * fCosHYaw;
}

void qmultiplyq(float q1[],float q2[],float q_result[])
{
    q_result[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
    q_result[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
    q_result[2]=q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
    q_result[3]=q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];
}

void  qToRotation(float q[],float R[])
{
    float_t w,x,y,z;
    // coordinates in NWU
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];

    R[0]=w*w+x*x-y*y-z*z;
    R[1]=2*(x*y-w*z);
    R[2]=2*(x*z+w*y);
    R[3]=2*(x*y+w*z);
    R[4]=w*w-x*x+y*y-z*z;
    R[5]=2*(y*z-w*x);
    R[6]=2*(x*z-w*y);
    R[7]=2*(y*z+w*x);
    R[8]=w*w-x*x-y*y+z*z;
}

void rotationToQ(float q[],cv::Mat R,int type)
{
    if(type==CV_32F){
        q[0]=sqrt(1+R.at<float>(0,0)+R.at<float>(1,1)+R.at<float>(2,2))/2;
        q[1]=(R.at<float>(2,1)-R.at<float>(1,2))/(4*q[0]);
        q[2]=(R.at<float>(0,2)-R.at<float>(2,0))/(4*q[0]);
        q[3]=(R.at<float>(1,0)-R.at<float>(0,1))/(4*q[0]);
    }
    else if(type==CV_64F){
        q[0]=sqrt(1+R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2))/2;
        q[1]=(R.at<double>(2,1)-R.at<double>(1,2))/(4*q[0]);
        q[2]=(R.at<double>(0,2)-R.at<double>(2,0))/(4*q[0]);
        q[3]=(R.at<double>(1,0)-R.at<double>(0,1))/(4*q[0]);
    }

}

void rotate_body_from_NWUworld(const float_t a_nwu[],float_t a_body[],float_t R[])
{
    for(int i=0;i<3;++i) a_body[i]=0;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            a_body[i]+=R[3*j+i]*a_nwu[j];
        }
    }
}

void rotate_NWUworld_from_body(const float_t a_body[],float_t a_nwu[],float_t R[])
{
    for(int i=0;i<3;++i) a_nwu[i]=0;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            a_nwu[i]+=R[3*i+j]*a_body[j];
        }
    }
}

//pointBODY = Rt * (pointWORLD - T)--convert from transform_NWUworld_from_body
void transform_body_from_NWUworld(float &bodyX, float &bodyY, float &bodyZ,
                                  float worldX, float worldY, float worldZ,
                                  float R[], float T[])
{
    float dx = worldX - T[0];
    float dy = worldY - T[1];
    float dz = worldZ - T[2];
    bodyX = R[0]*dx + R[3]*dy + R[6]*dz;
    bodyY = R[1]*dx + R[4]*dy + R[7]*dz;
    bodyZ = R[2]*dx + R[5]*dy + R[8]*dz;
}

//R * pointBODY + T = pointWORLD--test with q and T from vicon and rgbd camera
void transform_NWUworld_from_body(float bodyX, float bodyY, float bodyZ,
                                  float &worldX, float &worldY, float &worldZ,
                                  float R[], float T[])
{
    float dx = R[0]*bodyX + R[1]*bodyY + R[2]*bodyZ;
    float dy = R[3]*bodyX + R[4]*bodyY + R[5]*bodyZ;
    float dz = R[6]*bodyX + R[7]*bodyY + R[8]*bodyZ;
    worldX = dx + T[0];
    worldY = dy + T[1];
    worldZ = dz + T[2];
}

double getDistance(float_t dx,float_t dy,float_t dz)
{
    return sqrt(dx*dx+dy*dy+dz*dz);
}

double getDistance(float_t ds[3])
{
    return sqrt(ds[0]*ds[0]+ds[1]*ds[1]+ds[2]*ds[2]);
}

