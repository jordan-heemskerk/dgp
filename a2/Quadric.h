#pragma once
#include <OpenGP/types.h>
using namespace OpenGP;

/// This class stores a quadric as a symmetrix 4x4 matrix used by the error quadric mesh decimation algorithms.
class Quadric {
    typedef Vec3 Normal;
    typedef Vec3 Point;
public:
    /// Zero constructor
    Quadric() {}
    /// constructs the quadric from the point and normal specifying a plane
    Quadric(const Normal& n, const Point& p) {
        /// TASK: initialize the quadric given a normal and a point on a supporting plane
        float a = n(0);
        float b = n(1);
        float c = n(2);
        float d = -(n.dot(p));
        Q << a*a, a*b, a*c, a*d,
             a*b, b*b, b*c, b*d,
             a*c, b*c, c*c, c*d,
             a*d, b*d, c*d, d*d;
    }


    /// set all matric entries to zero
    void clear() {
        /// TASK: set the quadric to zero.
        Q = Eigen::Matrix4f::Zero();
    }


    /// add two quadrics
    Quadric operator+( const Quadric& _q ) const {
        /// TASK: implement quadric add
        Quadric toReturn(*this);
        toReturn += _q;
        return toReturn;
    }

    /// add given quadric to this quadric
    Quadric& operator+=( const Quadric& _q ) {
        /// TASK: implement quadric (self) add
        Q = Q + _q.Q;
        return *this;
    }

    // evaluate quadric Q at position p by computing (p^T * Q * p)
    double evaluate(const Point& p) const {
        /// TASK: evaluate the quadratic form at point p
        Eigen::Vector4f p_padded(p(0), p(1), p(2), 1);
        return p_padded.transpose() * Q * p_padded;
    }

private:
    /// TASK: how to store the 4x4 symmetrix matrix?
    /// hint: see Eigen::SelfAdjointView
    Eigen::Matrix4f Q;
};
