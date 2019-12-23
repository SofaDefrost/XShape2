#include <cmath>
#include <algorithm>
#include <iostream>
class Vec3d
{
public:
    double x{0};
    double y{0};
    double z{0};
    Vec3d(){}
    Vec3d(double ax,double ay, double az){
        x=ax;
        y=ay;
        z=az;
    }

    double dot(const Vec3d b) const
    {
        return x*b.x + y*b.y + z*b.z;
    }

    Vec3d cross(const Vec3d b) const
    {
        return Vec3d(
                    y*b.z - z*b.y,
                    z*b.x - x*b.z,
                    x*b.y - y*b.x
                    );
    }

    Vec3d normalized() const {
        double d = length();
        return Vec3d(x/d, y/d, z/d);
    }

    void set(double ax, double ay, double az)
    {
        x = ax;
        y = ay;
        z = az;
    }

    Vec3d mul(const double a) const {
        return Vec3d(x*a, y*a, z*a);
    }

    Vec3d div(const double a) const {
        return Vec3d(x/a, y/a, z/a);
    }

    Vec3d operator+(const Vec3d& v)const{
        return Vec3d(x+v.x, y+v.y, z+v.z);
    }

    Vec3d operator-(const Vec3d& v)const{
        return Vec3d(x-v.x, y-v.y, z-v.z);
    }

    double length() const {
        return sqrt(x*x+y*y+z*z) ;
    }

    Vec3d vmax(const Vec3d& v)const {
        return Vec3d(std::max(x, v.x),std::max(y, v.y),std::max(z, v.z) );
    }

    Vec3d  mod(const Vec3d& v)const
    {
        Vec3d d = Vec3d( fmod(x, v.x),
                         fmod(y, v.y),
                         fmod(z, v.z) );
        return d;
    }


    Vec3d vmin(const Vec3d& v)const {
        return Vec3d(std::min(x, v.x),std::min(y, v.y),std::min(z, v.z) );
    }

    Vec3d vmax(const double& v)const {
        return Vec3d(std::max(x, v),std::max(y, v),std::max(z, v) );
    }

    Vec3d vmin(const double& v)const {
        return Vec3d( std::min(x, v), std::min(y, v), std::min(z, v) );
    }


    Vec3d abs()const {
        return Vec3d( fabs(x), fabs(y), fabs(z) );
    }
};
