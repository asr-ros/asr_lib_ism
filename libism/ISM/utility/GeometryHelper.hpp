/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include "common_type/Pose.hpp"
#include "common_type/Quaternion.hpp"
#include "common_type/VoteSpecifier.hpp"
#include "common_type/Point.hpp"

namespace ISM {
  class GeometryHelper {
  public:

    //Methods are all inlined since heavily used in performance relevant code.

    /**Voting related methods**/

    /**
     * Calculates an absolute pose at refPoint based on a relative orientation refToSourceQuat towards source.
     *
     * @param source Absolute position in refPoint has been calculated be combining source and the relative position of refPoint towards source.
     * @param refPoint Absolute position for which we want to calculate the corresponding orientation.
     * @param refToSourceQuat Relative orientation of searched absolute pose relative to source.
     * @return 6D pose being looked for.
     */
    static PosePtr getReferencePose(const PosePtr& source, const PointPtr& refPoint, const QuaternionPtr& objectToRefPoseQuat) {
      PosePtr result = PosePtr(new Pose());
      getReferencePose(source, refPoint, objectToRefPoseQuat, result);
      return result;
    }

    //Faster variant that avoids allocation of local Pose in every call of this method.
    static void getReferencePose(const PosePtr& source, const PointPtr& refPoint, const QuaternionPtr& objectToRefPoseQuat, PosePtr& result) {
      result->point = refPoint;
      result->quat = eigenQuatToQuat(source->quat->eigen * objectToRefPoseQuat->eigen);
    }

    /**
     * Calculate an absolute position by combining refPose and a relative position vector composed of refToObjectQuat and radius.
     *
     * @param refPose Absolute position that is combined with relative pose.
     * @param refToObjectQuat Orientation of Eigen::Vector3d connecting refPose and position being searched for.
     * @param radius Length of Eigen::Vector3d connecting refPose and position being searched for.
     * @return 3D position being looked for.
     */
    static PointPtr getSourcePoint(const PosePtr& refPose, const QuaternionPtr& refToObjectQuat, double radius) {
      PointPtr result = PointPtr(new Point());
      getSourcePoint(refPose, refToObjectQuat, radius, result);
      return result;
    }

    //Faster variant that avoids allocation of local Pose in every call of this method.
    static void getSourcePoint(const PosePtr& refPose, const QuaternionPtr& refToObjectQuat, double radius, PointPtr& result) {
      Eigen::Vector3d relativeObjectVector = refToObjectQuat->eigen._transformVector(Eigen::Vector3d::UnitX());
      Eigen::Vector3d absoluteObjectVector = refPose->quat->eigen._transformVector(relativeObjectVector);
      result->eigen = refPose->point->eigen + (absoluteObjectVector * radius);
    }

    static PosePtr getSourcePose(const PosePtr& reference, const PointPtr& sourcePoint, const QuaternionPtr& refToObjectPoseQuat) {
      PosePtr result = PosePtr(new Pose());
      getSourcePose(reference, sourcePoint, refToObjectPoseQuat, result);
      return result;
    }

    //Faster variant that avoids allocation of local Pose in every call of this method.
    static void getSourcePose(const PosePtr& reference, const PointPtr& sourcePoint, const QuaternionPtr& refToObjectPoseQuat, PosePtr& result) {
      result->point = sourcePoint;
      result->quat->eigen = reference->quat->eigen * refToObjectPoseQuat->eigen;
    }

    static PointPtr applyQuatAndRadiusToPose(const PosePtr& pose, const QuaternionPtr& quat, double radius) {
      //apply transformation to viewport vector, then scale, than transform to object coordinates and add
      Eigen::Quaternion<double> rotation = pose->quat->eigen * quat->eigen;
      Eigen::Vector3d objectPoint = pose->point->eigen;

      Eigen::Vector3d resultVec = objectPoint + rotation._transformVector(Eigen::Vector3d::UnitX()) * radius;

      return vectorToPoint(resultVec);
    }

    static Quaternion selfQuat;
    static double constexpr epsylon = 1e-4;
    static bool isSelfVote(const VoteSpecifierPtr& vote)
    {
      if ( !(vote->radius > -epsylon && vote->radius < epsylon) ||
	   !quatEqual(selfQuat, *(vote->objectToRefPoseQuat)))
        {
	  return false;
        }
      return true;
    }

    /**Pose related methods**/

    static VoteSpecifierPtr createVoteSpecifier(const PosePtr& sourcePose, const PosePtr& refPose) {
      if ((refPose->point->eigen - sourcePose->point->eigen).norm() == 0) {
	//avoid special case
	sourcePose->point->eigen.x() = (sourcePose->point->eigen.x() + 0.0000001);
      };
      Eigen::Vector3d objToRefVector = refPose->point->eigen - sourcePose->point->eigen;
      Eigen::Vector3d refToObjVector = objToRefVector * -1.0;
      Eigen::Quaternion<double> p = sourcePose->quat->eigen;
      Eigen::Quaternion<double> r = refPose->quat->eigen;

      //rotate everything relative to object pose
      Eigen::Vector3d relativeRefPoint = p.inverse()._transformVector(objToRefVector);

      Eigen::Quaternion<double> otr = vectorRotationToEigenQuat(Eigen::Vector3d::UnitX(), relativeRefPoint);
      Eigen::Quaternion<double> otrp = (p.inverse()) * r;

      //rotate everything relative to ref pose
      Eigen::Vector3d relativeObjPoint = r.inverse()._transformVector(refToObjVector);
      Eigen::Quaternion<double> rto = vectorRotationToEigenQuat(Eigen::Vector3d::UnitX(), relativeObjPoint);
      Eigen::Quaternion<double> rtop = (r.inverse()) * p;

      return VoteSpecifierPtr(
			      new VoteSpecifier(
						eigenQuatToQuat(otr),
						eigenQuatToQuat(otrp),
						eigenQuatToQuat(rto),
						eigenQuatToQuat(rtop),
						objToRefVector.norm()
						)
			      );
    }

    static PosePtr getPoseFromVote(const PosePtr& source, const VoteSpecifierPtr& vote)
    {
      PointPtr referencePoint = applyQuatAndRadiusToPose(source, vote->objectToRefQuat, vote->radius);
      return getReferencePose(source, referencePoint, vote->objectToRefPoseQuat);
    }

    static bool poseEqual(const PosePtr& p1, const PosePtr& p2) {
      bool overall = (quatEqual(p1->quat, p2->quat) && pointEqual (p1->point, p2->point));
      return overall;
    }

    static Eigen::Vector3d getDirectionVector(const PosePtr& first, const PosePtr& second) {

      Eigen::Vector3d firstToSecond = second->point->eigen - first->point->eigen;
      Eigen::Quaternion<double> firstRotation = first->quat->eigen;
      return firstRotation.inverse()._transformVector(firstToSecond);

    }

    static bool sharedNeighborhoodEvaluation(const PosePtr& p1, const PosePtr& p2, double distanceNeighbourThreshold, double angleNeighbourThreshold)
    {
      double distance_angle = getAngleBetweenQuats(p1->quat, p2->quat);
      double distance_position = getDistanceBetweenPoints(p1->point, p2->point);

      return distanceNeighbourThreshold > distance_position
	&& angleNeighbourThreshold > fabs(distance_angle);
    }

    /**Point related methods**/

    static Eigen::Vector3d pointToVector(const PointPtr& p) {
      return p->eigen;
    }

    static PointPtr vectorToPoint(const Eigen::Vector3d& v) {
      return PointPtr(new Point(v[0], v[1], v[2]));
    }

    static double getDistanceBetweenPoints(const PointPtr& p1, const PointPtr& p2) {
      return (p1->eigen - p2->eigen).norm();

    }

    static double getSquaredDistanceBetweenPoints(const PointPtr &p1, const PointPtr &p2)
    {
      return (p1->eigen - p2->eigen).squaredNorm();
    }

    static bool pointEqual(const PointPtr& p1, const PointPtr& p2)
    {
      bool xAbs = fabs(p1->eigen.x() - p2->eigen.x()) < 0.00001;
      bool yAbs = fabs(p1->eigen.y() - p2->eigen.y()) < 0.00001;
      bool zAbs = fabs(p1->eigen.z() - p2->eigen.z()) < 0.00001;
      return (xAbs && yAbs && zAbs);
    }

    /**Quaternion related methods**/

    static Eigen::Quaternion<double> quatToEigenQuat(const QuaternionPtr& q) {
      return q->eigen;
    }

    static QuaternionPtr eigenQuatToQuat(const Eigen::Quaternion<double>& q) {
      return QuaternionPtr(new Quaternion(q.w(), q.x(), q.y(), q.z()));
    }

    static QuaternionPtr normalize(const QuaternionPtr& quat) {
      return eigenQuatToQuat(quat->eigen.normalized());
    }

    static double getAngleBetweenQuats(const QuaternionPtr& q1, const QuaternionPtr& q2) {

      Eigen::Vector3d v1 = getAxisFromQuat(q1);
      Eigen::Vector3d v2 = getAxisFromQuat(q2);
      return getAngleBetweenAxes(v1, v2);

    }
    static Eigen::Vector3d getAxisFromQuat(const QuaternionPtr& quat, const Eigen::Vector3d& viewport = Eigen::Vector3d::UnitX()) {
      Eigen::Quaternion<double> rotation = quat->eigen;
      return rotation._transformVector(viewport);
    }

    static double getAngleBetweenAxes(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {

      double cosOfAngle = v1.dot(v2) / (v1.norm() * v2.norm());
      if(cosOfAngle < -1.0) cosOfAngle = -1.0;
      if(cosOfAngle > 1.0) cosOfAngle = 1.0;

      return rad2deg(acos(cosOfAngle));

    }

    static bool quatEqual(const QuaternionPtr& q1, const QuaternionPtr& q2)
    {
      return quatEqual(*q1, *q2);
    }

    static bool quatEqual(const Quaternion& q1, const Quaternion& q2) {
      bool qxAbs = fabs(q1.eigen.x() - q2.eigen.x()) < 0.00001;
      bool qyAbs = fabs(q1.eigen.y() - q2.eigen.y()) < 0.00001;
      bool qzAbs = fabs(q1.eigen.z() - q2.eigen.z()) < 0.00001;
      bool qwAbs = fabs(q1.eigen.w() - q2.eigen.w()) < 0.00001;
      return (qxAbs && qyAbs && qzAbs && qwAbs);
    }

    static Eigen::Quaternion<double> vectorRotationToEigenQuat(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
      Eigen::Quaternion<double> q;
      q.setFromTwoVectors(v1, v2);
      return q;
    }

    static QuaternionPtr getQuatFromRPY(const QuaternionPtr& q, const double alpha, const double beta, const double gamma){
      Eigen::Quaternion<double> eq = q->eigen;
      Eigen::AngleAxisd rollAngle(deg2rad(alpha), Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(deg2rad(beta), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(deg2rad(gamma), Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond rot = yawAngle * pitchAngle * rollAngle;


      return eigenQuatToQuat(eq * rot);

    }

    static QuaternionPtr getAveragePose(const std::vector<QuaternionPtr>& poseQuats) {
      Eigen::Vector3d combined(0, 0, 0);

      for (const QuaternionPtr& quat : poseQuats) {
	combined += quat->eigen._transformVector(Eigen::Vector3d::UnitX());
      }

      Eigen::Quaternion<double> ret;
      ret.setFromTwoVectors(Eigen::Vector3d::UnitX(), combined);
      return eigenQuatToQuat(ret.normalized());
    }

    static bool getNextCombination(const std::vector<unsigned int>::iterator first, std::vector<unsigned int>::iterator k, const std::vector<unsigned int>::iterator last)
    {
      if ((first == last) || (first == k) || (last == k))
	return false;

      std::vector<unsigned int>::iterator it1 = first;
      std::vector<unsigned int>::iterator it2 = last;
      it1++;
      if (last == it1)
	return false;
      it1 = last;
      it1--;
      it1 = k;
      it2--;
      while (first != it1)
        {
	  if (*--it1 < *it2)
            {
	      std::vector<unsigned int>::iterator j = k;
	      while (!(*it1 < *j)) ++j;
	      std::iter_swap(it1,j);
	      it1++;
	      j++;
	      it2 = k;
	      std::rotate(it1,j,last);
	      while (last != j)
                {
		  j++;
		  it2++;
                }
	      std::rotate(k,it2,last);
	      return true;
            }
        }
      std::rotate(first,k,last);
      return false;
    }

    /**Various geometry related helpers**/

    static double rad2deg(double rad) {
      return rad * (180.0 / M_PI);
    }

    static double deg2rad(double deg) {
      return deg * (M_PI / 180.0);
    }

    static inline bool checkIfDoubleNaN(double& c)
    {
      /*
       * distance != distance for portable way check for nan
       * but be aware:
       * http://stackoverflow.com/questions/570669/checking-if-a-double-or-float-is-nan-in-c
       */
      bool isNan = c != c;
      if (isNan)
        {
	  std::cerr << " NaN ";
        }
      return isNan;
    }

  };

}
