#pragma once
#include "core.hh"


namespace cp {


//
// Type aliases
//

// chipmunk/chipmunk.h
using Array = cpArray;
using HashSet = cpHashSet;
using Body = cpBody;
using Shape = cpShape;
using CircleShape = cpCircleShape;
using SegmentShape = cpSegmentShape;
using PolyShape = cpPolyShape;
using Constraint = cpConstraint;
using PinJoint = cpPinJoint;
using SlideJoint = cpSlideJoint;
using PivotJoint = cpPivotJoint;
using GrooveJoint = cpGrooveJoint;
using DampedSpring = cpDampedSpring;
using DampedRotarySpring = cpDampedRotarySpring;
using RotaryLimitJoint = cpRotaryLimitJoint;
using RatchetJoint = cpRatchetJoint;
using GearJoint = cpGearJoint;
using SimpleMotorJoint = cpSimpleMotorJoint;
using CollisionHandler = cpCollisionHandler;
using ContactPointSet = cpContactPointSet;
using Arbiter = cpArbiter;
using Space = cpSpace;

// chipmunk/cpBB.h
using BB = cpBB;

// chipmunk/cpPolyline.h
// using Polyline = cpPolyline;
// using PolylineSet = cpPolylineSet;

// chipmunk/cpHastySpace.h
// using HastySpace = cpHastySpace;

// chipmunk/cpSpace.h
using SpaceDebugColor = cpSpaceDebugColor;
using SpaceDebugDrawOptions = cpSpaceDebugDrawOptions;

// chipmunk/cpShape.h
using PointQueryInfo = cpPointQueryInfo;
using SegmentQueryInfo = cpSegmentQueryInfo;
using ShapeFilter = cpShapeFilter;

// chipmunk/chipmunk_types.h
using Float = cpFloat;
using HashValue = cpHashValue;
using CollisionID = cpCollisionID;
using Bool = cpBool;
using DataPointer = cpDataPointer;
using CollisionType = cpCollisionType;
using Group = cpGroup;
using Bitmask = cpBitmask;
using Timestamp = cpTimestamp;
using Vect = cpVect;
using Transform = cpTransform;
using Mat2x2 = cpMat2x2;

// chipmunk/cpSpatialIndex.h
using SpatialIndexClass = cpSpatialIndexClass;
using SpatialIndex = cpSpatialIndex;
using SpaceHash = cpSpaceHash;
using BBTree = cpBBTree;
using Sweep1D = cpSweep1D;

// chipmunk/cpSimpleMotor.h
using SimpleMotor = cpSimpleMotor;

// chipmunk/chipmunk_structs.h
using ShapeClass = cpShapeClass;
using ConstraintClass = cpConstraintClass;
using ContactBufferHeader = cpContactBufferHeader;
using PostStepCallback = cpPostStepCallback;


//
// Constants
//

inline constexpr Transform TransformIdentity = { 1, 0, 0, 1, 0, 0 };
inline const auto SHAPE_FILTER_ALL = CP_SHAPE_FILTER_ALL;
inline const auto SHAPE_FILTER_NONE = CP_SHAPE_FILTER_NONE;


// `Vec2` <-> `cp::Vect` conversion
//

inline Vec2 convert(cp::Vect v) {
  return { v.x, v.y };
}
inline cp::Vect convert(Vec2 v) {
  return { v.x, v.y };
}


//
// Function aliases
//

// chipmunk/cpGrooveJoint.h
inline cpBool ConstraintIsGrooveJoint(const cpConstraint *constraint) {
  return cpConstraintIsGrooveJoint(constraint);
}
inline cpGrooveJoint *GrooveJointAlloc() {
  return cpGrooveJointAlloc();
}
inline cpGrooveJoint *GrooveJointInit(
    cpGrooveJoint *joint, cpBody *a, cpBody *b, Vec2 groove_a, Vec2 groove_b, Vec2 anchorB) {
  return cpGrooveJointInit(joint, a, b, convert(groove_a), convert(groove_b), convert(anchorB));
}
inline cpConstraint *GrooveJointNew(
    cpBody *a, cpBody *b, Vec2 groove_a, Vec2 groove_b, Vec2 anchorB) {
  return cpGrooveJointNew(a, b, convert(groove_a), convert(groove_b), convert(anchorB));
}
inline Vec2 GrooveJointGetGrooveA(const cpConstraint *constraint) {
  return convert(cpGrooveJointGetGrooveA(constraint));
}
inline void GrooveJointSetGrooveA(cpConstraint *constraint, Vec2 grooveA) {
  return cpGrooveJointSetGrooveA(constraint, convert(grooveA));
}
inline Vec2 GrooveJointGetGrooveB(const cpConstraint *constraint) {
  return convert(cpGrooveJointGetGrooveB(constraint));
}
inline void GrooveJointSetGrooveB(cpConstraint *constraint, Vec2 grooveB) {
  return cpGrooveJointSetGrooveB(constraint, convert(grooveB));
}
inline Vec2 GrooveJointGetAnchorB(const cpConstraint *constraint) {
  return convert(cpGrooveJointGetAnchorB(constraint));
}
inline void GrooveJointSetAnchorB(cpConstraint *constraint, Vec2 anchorB) {
  return cpGrooveJointSetAnchorB(constraint, convert(anchorB));
}

// chipmunk/chipmunk.h
inline cpFloat MomentForCircle(cpFloat m, cpFloat r1, cpFloat r2, Vec2 offset) {
  return cpMomentForCircle(m, r1, r2, convert(offset));
}
inline cpFloat AreaForCircle(cpFloat r1, cpFloat r2) {
  return cpAreaForCircle(r1, r2);
}
inline cpFloat MomentForSegment(cpFloat m, Vec2 a, Vec2 b, cpFloat radius) {
  return cpMomentForSegment(m, convert(a), convert(b), radius);
}
inline cpFloat AreaForSegment(Vec2 a, Vec2 b, cpFloat radius) {
  return cpAreaForSegment(convert(a), convert(b), radius);
}
inline cpFloat MomentForPoly(cpFloat m, int count, const Vec2 *verts, Vec2 offset, cpFloat radius) {
  return cpMomentForPoly(m, count, (const cpVect *)verts, convert(offset), radius);
}
inline cpFloat AreaForPoly(const int count, const Vec2 *verts, cpFloat radius) {
  return cpAreaForPoly(count, (const cpVect *)verts, radius);
}
inline Vec2 CentroidForPoly(const int count, const Vec2 *verts) {
  return convert(cpCentroidForPoly(count, (const cpVect *)verts));
}
inline cpFloat MomentForBox(cpFloat m, cpFloat width, cpFloat height) {
  return cpMomentForBox(m, width, height);
}
inline cpFloat MomentForBox2(cpFloat m, cpBB box) {
  return cpMomentForBox2(m, box);
}
inline int ConvexHull(int count, const Vec2 *verts, Vec2 *result, int *first, cpFloat tol) {
  return cpConvexHull(count, (const cpVect *)verts, (cpVect *)result, first, tol);
}

// chipmunk/cpSlideJoint.h
inline cpBool ConstraintIsSlideJoint(const cpConstraint *constraint) {
  return cpConstraintIsSlideJoint(constraint);
}
inline cpSlideJoint *SlideJointAlloc() {
  return cpSlideJointAlloc();
}
inline cpSlideJoint *SlideJointInit(cpSlideJoint *joint, cpBody *a, cpBody *b, Vec2 anchorA,
    Vec2 anchorB, cpFloat min, cpFloat max) {
  return cpSlideJointInit(joint, a, b, convert(anchorA), convert(anchorB), min, max);
}
inline cpConstraint *SlideJointNew(
    cpBody *a, cpBody *b, Vec2 anchorA, Vec2 anchorB, cpFloat min, cpFloat max) {
  return cpSlideJointNew(a, b, convert(anchorA), convert(anchorB), min, max);
}
inline Vec2 SlideJointGetAnchorA(const cpConstraint *constraint) {
  return convert(cpSlideJointGetAnchorA(constraint));
}
inline void SlideJointSetAnchorA(cpConstraint *constraint, Vec2 anchorA) {
  return cpSlideJointSetAnchorA(constraint, convert(anchorA));
}
inline Vec2 SlideJointGetAnchorB(const cpConstraint *constraint) {
  return convert(cpSlideJointGetAnchorB(constraint));
}
inline void SlideJointSetAnchorB(cpConstraint *constraint, Vec2 anchorB) {
  return cpSlideJointSetAnchorB(constraint, convert(anchorB));
}
inline cpFloat SlideJointGetMin(const cpConstraint *constraint) {
  return cpSlideJointGetMin(constraint);
}
inline void SlideJointSetMin(cpConstraint *constraint, cpFloat min) {
  return cpSlideJointSetMin(constraint, min);
}
inline cpFloat SlideJointGetMax(const cpConstraint *constraint) {
  return cpSlideJointGetMax(constraint);
}
inline void SlideJointSetMax(cpConstraint *constraint, cpFloat max) {
  return cpSlideJointSetMax(constraint, max);
}

// chipmunk/cpSpace.h
inline cpSpace *SpaceAlloc() {
  return cpSpaceAlloc();
}
inline cpSpace *SpaceInit(cpSpace *space) {
  return cpSpaceInit(space);
}
inline cpSpace *SpaceNew() {
  return cpSpaceNew();
}
inline void SpaceDestroy(cpSpace *space) {
  return cpSpaceDestroy(space);
}
inline void SpaceFree(cpSpace *space) {
  return cpSpaceFree(space);
}
inline int SpaceGetIterations(const cpSpace *space) {
  return cpSpaceGetIterations(space);
}
inline void SpaceSetIterations(cpSpace *space, int iterations) {
  return cpSpaceSetIterations(space, iterations);
}
inline Vec2 SpaceGetGravity(const cpSpace *space) {
  return convert(cpSpaceGetGravity(space));
}
inline void SpaceSetGravity(cpSpace *space, Vec2 gravity) {
  return cpSpaceSetGravity(space, convert(gravity));
}
inline cpFloat SpaceGetDamping(const cpSpace *space) {
  return cpSpaceGetDamping(space);
}
inline void SpaceSetDamping(cpSpace *space, cpFloat damping) {
  return cpSpaceSetDamping(space, damping);
}
inline cpFloat SpaceGetIdleSpeedThreshold(const cpSpace *space) {
  return cpSpaceGetIdleSpeedThreshold(space);
}
inline void SpaceSetIdleSpeedThreshold(cpSpace *space, cpFloat idleSpeedThreshold) {
  return cpSpaceSetIdleSpeedThreshold(space, idleSpeedThreshold);
}
inline cpFloat SpaceGetSleepTimeThreshold(const cpSpace *space) {
  return cpSpaceGetSleepTimeThreshold(space);
}
inline void SpaceSetSleepTimeThreshold(cpSpace *space, cpFloat sleepTimeThreshold) {
  return cpSpaceSetSleepTimeThreshold(space, sleepTimeThreshold);
}
inline cpFloat SpaceGetCollisionSlop(const cpSpace *space) {
  return cpSpaceGetCollisionSlop(space);
}
inline void SpaceSetCollisionSlop(cpSpace *space, cpFloat collisionSlop) {
  return cpSpaceSetCollisionSlop(space, collisionSlop);
}
inline cpFloat SpaceGetCollisionBias(const cpSpace *space) {
  return cpSpaceGetCollisionBias(space);
}
inline void SpaceSetCollisionBias(cpSpace *space, cpFloat collisionBias) {
  return cpSpaceSetCollisionBias(space, collisionBias);
}
inline cpTimestamp SpaceGetCollisionPersistence(const cpSpace *space) {
  return cpSpaceGetCollisionPersistence(space);
}
inline void SpaceSetCollisionPersistence(cpSpace *space, cpTimestamp collisionPersistence) {
  return cpSpaceSetCollisionPersistence(space, collisionPersistence);
}
inline cpDataPointer SpaceGetUserData(const cpSpace *space) {
  return cpSpaceGetUserData(space);
}
inline void SpaceSetUserData(cpSpace *space, cpDataPointer userData) {
  return cpSpaceSetUserData(space, userData);
}
inline cpBody *SpaceGetStaticBody(const cpSpace *space) {
  return cpSpaceGetStaticBody(space);
}
inline cpFloat SpaceGetCurrentTimeStep(const cpSpace *space) {
  return cpSpaceGetCurrentTimeStep(space);
}
inline cpBool SpaceIsLocked(cpSpace *space) {
  return cpSpaceIsLocked(space);
}
inline cpCollisionHandler *SpaceAddDefaultCollisionHandler(cpSpace *space) {
  return cpSpaceAddDefaultCollisionHandler(space);
}
inline cpCollisionHandler *SpaceAddCollisionHandler(
    cpSpace *space, cpCollisionType a, cpCollisionType b) {
  return cpSpaceAddCollisionHandler(space, a, b);
}
inline cpCollisionHandler *SpaceAddWildcardHandler(cpSpace *space, cpCollisionType type) {
  return cpSpaceAddWildcardHandler(space, type);
}
inline cpShape *SpaceAddShape(cpSpace *space, cpShape *shape) {
  return cpSpaceAddShape(space, shape);
}
inline cpBody *SpaceAddBody(cpSpace *space, cpBody *body) {
  return cpSpaceAddBody(space, body);
}
inline cpConstraint *SpaceAddConstraint(cpSpace *space, cpConstraint *constraint) {
  return cpSpaceAddConstraint(space, constraint);
}
inline void SpaceRemoveShape(cpSpace *space, cpShape *shape) {
  return cpSpaceRemoveShape(space, shape);
}
inline void SpaceRemoveBody(cpSpace *space, cpBody *body) {
  return cpSpaceRemoveBody(space, body);
}
inline void SpaceRemoveConstraint(cpSpace *space, cpConstraint *constraint) {
  return cpSpaceRemoveConstraint(space, constraint);
}
inline cpBool SpaceContainsShape(cpSpace *space, cpShape *shape) {
  return cpSpaceContainsShape(space, shape);
}
inline cpBool SpaceContainsBody(cpSpace *space, cpBody *body) {
  return cpSpaceContainsBody(space, body);
}
inline cpBool SpaceContainsConstraint(cpSpace *space, cpConstraint *constraint) {
  return cpSpaceContainsConstraint(space, constraint);
}
inline cpBool SpaceAddPostStepCallback(cpSpace *space, cpPostStepFunc func, void *key, void *data) {
  return cpSpaceAddPostStepCallback(space, func, key, data);
}
inline void SpacePointQuery(cpSpace *space, Vec2 point, cpFloat maxDistance, cpShapeFilter filter,
    cpSpacePointQueryFunc func, void *data) {
  return cpSpacePointQuery(space, convert(point), maxDistance, filter, func, data);
}
inline cpShape *SpacePointQueryNearest(
    cpSpace *space, Vec2 point, cpFloat maxDistance, cpShapeFilter filter, cpPointQueryInfo *out) {
  return cpSpacePointQueryNearest(space, convert(point), maxDistance, filter, out);
}
template<typename F>
inline void SpaceSegmentQuery(
    cpSpace *space, Vec2 start, Vec2 end, cpFloat radius, cpShapeFilter filter, F &&f) {
  return cpSpaceSegmentQuery(
      space, convert(start), convert(end), radius, filter,
      [](cpShape *shape, cpVect point, cpVect normal, float alpha, void *data) {
        auto &f = *static_cast<F *>(data);
        f(shape, convert(point), convert(normal), alpha);
      },
      &f);
}
inline cpShape *SpaceSegmentQueryFirst(cpSpace *space, Vec2 start, Vec2 end, cpFloat radius,
    cpShapeFilter filter, cpSegmentQueryInfo *out) {
  return cpSpaceSegmentQueryFirst(space, convert(start), convert(end), radius, filter, out);
}
inline void SpaceBBQuery(
    cpSpace *space, cpBB bb, cpShapeFilter filter, cpSpaceBBQueryFunc func, void *data) {
  return cpSpaceBBQuery(space, bb, filter, func, data);
}
inline cpBool SpaceShapeQuery(
    cpSpace *space, cpShape *shape, cpSpaceShapeQueryFunc func, void *data) {
  return cpSpaceShapeQuery(space, shape, func, data);
}
inline void SpaceEachBody(cpSpace *space, cpSpaceBodyIteratorFunc func, void *data) {
  return cpSpaceEachBody(space, func, data);
}
inline void SpaceEachShape(cpSpace *space, cpSpaceShapeIteratorFunc func, void *data) {
  return cpSpaceEachShape(space, func, data);
}
inline void SpaceEachConstraint(cpSpace *space, cpSpaceConstraintIteratorFunc func, void *data) {
  return cpSpaceEachConstraint(space, func, data);
}
inline void SpaceReindexStatic(cpSpace *space) {
  return cpSpaceReindexStatic(space);
}
inline void SpaceReindexShape(cpSpace *space, cpShape *shape) {
  return cpSpaceReindexShape(space, shape);
}
inline void SpaceReindexShapesForBody(cpSpace *space, cpBody *body) {
  return cpSpaceReindexShapesForBody(space, body);
}
inline void SpaceUseSpatialHash(cpSpace *space, cpFloat dim, int count) {
  return cpSpaceUseSpatialHash(space, dim, count);
}
inline void SpaceStep(cpSpace *space, cpFloat dt) {
  return cpSpaceStep(space, dt);
}
inline void SpaceDebugDraw(cpSpace *space, cpSpaceDebugDrawOptions *options) {
  return cpSpaceDebugDraw(space, options);
}

// chipmunk/cpRatchetJoint.h
inline cpBool ConstraintIsRatchetJoint(const cpConstraint *constraint) {
  return cpConstraintIsRatchetJoint(constraint);
}
inline cpRatchetJoint *RatchetJointAlloc() {
  return cpRatchetJointAlloc();
}
inline cpRatchetJoint *RatchetJointInit(
    cpRatchetJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet) {
  return cpRatchetJointInit(joint, a, b, phase, ratchet);
}
inline cpConstraint *RatchetJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet) {
  return cpRatchetJointNew(a, b, phase, ratchet);
}
inline cpFloat RatchetJointGetAngle(const cpConstraint *constraint) {
  return cpRatchetJointGetAngle(constraint);
}
inline void RatchetJointSetAngle(cpConstraint *constraint, cpFloat angle) {
  return cpRatchetJointSetAngle(constraint, angle);
}
inline cpFloat RatchetJointGetPhase(const cpConstraint *constraint) {
  return cpRatchetJointGetPhase(constraint);
}
inline void RatchetJointSetPhase(cpConstraint *constraint, cpFloat phase) {
  return cpRatchetJointSetPhase(constraint, phase);
}
inline cpFloat RatchetJointGetRatchet(const cpConstraint *constraint) {
  return cpRatchetJointGetRatchet(constraint);
}
inline void RatchetJointSetRatchet(cpConstraint *constraint, cpFloat ratchet) {
  return cpRatchetJointSetRatchet(constraint, ratchet);
}

// chipmunk/cpDampedRotarySpring.h
inline cpBool ConstraintIsDampedRotarySpring(const cpConstraint *constraint) {
  return cpConstraintIsDampedRotarySpring(constraint);
}
inline cpDampedRotarySpring *DampedRotarySpringAlloc() {
  return cpDampedRotarySpringAlloc();
}
inline cpDampedRotarySpring *DampedRotarySpringInit(cpDampedRotarySpring *joint, cpBody *a,
    cpBody *b, cpFloat restAngle, cpFloat stiffness, cpFloat damping) {
  return cpDampedRotarySpringInit(joint, a, b, restAngle, stiffness, damping);
}
inline cpConstraint *DampedRotarySpringNew(
    cpBody *a, cpBody *b, cpFloat restAngle, cpFloat stiffness, cpFloat damping) {
  return cpDampedRotarySpringNew(a, b, restAngle, stiffness, damping);
}
inline cpFloat DampedRotarySpringGetRestAngle(const cpConstraint *constraint) {
  return cpDampedRotarySpringGetRestAngle(constraint);
}
inline void DampedRotarySpringSetRestAngle(cpConstraint *constraint, cpFloat restAngle) {
  return cpDampedRotarySpringSetRestAngle(constraint, restAngle);
}
inline cpFloat DampedRotarySpringGetStiffness(const cpConstraint *constraint) {
  return cpDampedRotarySpringGetStiffness(constraint);
}
inline void DampedRotarySpringSetStiffness(cpConstraint *constraint, cpFloat stiffness) {
  return cpDampedRotarySpringSetStiffness(constraint, stiffness);
}
inline cpFloat DampedRotarySpringGetDamping(const cpConstraint *constraint) {
  return cpDampedRotarySpringGetDamping(constraint);
}
inline void DampedRotarySpringSetDamping(cpConstraint *constraint, cpFloat damping) {
  return cpDampedRotarySpringSetDamping(constraint, damping);
}
inline cpDampedRotarySpringTorqueFunc DampedRotarySpringGetSpringTorqueFunc(
    const cpConstraint *constraint) {
  return cpDampedRotarySpringGetSpringTorqueFunc(constraint);
}
inline void DampedRotarySpringSetSpringTorqueFunc(
    cpConstraint *constraint, cpDampedRotarySpringTorqueFunc springTorqueFunc) {
  return cpDampedRotarySpringSetSpringTorqueFunc(constraint, springTorqueFunc);
}

// chipmunk/cpPivotJoint.h
inline cpBool ConstraintIsPivotJoint(const cpConstraint *constraint) {
  return cpConstraintIsPivotJoint(constraint);
}
inline cpPivotJoint *PivotJointAlloc() {
  return cpPivotJointAlloc();
}
inline cpPivotJoint *PivotJointInit(
    cpPivotJoint *joint, cpBody *a, cpBody *b, Vec2 anchorA, Vec2 anchorB) {
  return cpPivotJointInit(joint, a, b, convert(anchorA), convert(anchorB));
}
inline cpConstraint *PivotJointNew(cpBody *a, cpBody *b, Vec2 pivot) {
  return cpPivotJointNew(a, b, convert(pivot));
}
inline cpConstraint *PivotJointNew2(cpBody *a, cpBody *b, Vec2 anchorA, Vec2 anchorB) {
  return cpPivotJointNew2(a, b, convert(anchorA), convert(anchorB));
}
inline Vec2 PivotJointGetAnchorA(const cpConstraint *constraint) {
  return convert(cpPivotJointGetAnchorA(constraint));
}
inline void PivotJointSetAnchorA(cpConstraint *constraint, Vec2 anchorA) {
  return cpPivotJointSetAnchorA(constraint, convert(anchorA));
}
inline Vec2 PivotJointGetAnchorB(const cpConstraint *constraint) {
  return convert(cpPivotJointGetAnchorB(constraint));
}
inline void PivotJointSetAnchorB(cpConstraint *constraint, Vec2 anchorB) {
  return cpPivotJointSetAnchorB(constraint, convert(anchorB));
}

// chipmunk/cpHastySpace.h
// cpSpace *HastySpaceNew() {
//   return cpHastySpaceNew();
// }
// void HastySpaceFree(cpSpace *space) {
//   return cpHastySpaceFree(space);
// }
// void HastySpaceSetThreads(cpSpace *space, unsigned long threads) {
//   return cpHastySpaceSetThreads(space, threads);
// }
// unsigned long HastySpaceGetThreads(cpSpace *space) {
//   return cpHastySpaceGetThreads(space);
// }
// void HastySpaceStep(cpSpace *space, cpFloat dt) {
//   return cpHastySpaceStep(space, dt);
// }

// chipmunk/cpMarch.h
// void MarchSoft(cpBB bb, unsigned long x_samples, unsigned long y_samples, cpFloat threshold,
//    cpMarchSegmentFunc segment, void *segment_data, cpMarchSampleFunc sample, void *sample_data) {
//   return cpMarchSoft(
//      bb, x_samples, y_samples, threshold, segment, segment_data, sample, sample_data);
// }
// void MarchHard(cpBB bb, unsigned long x_samples, unsigned long y_samples, cpFloat threshold,
//    cpMarchSegmentFunc segment, void *segment_data, cpMarchSampleFunc sample, void *sample_data) {
//   return cpMarchHard(
//      bb, x_samples, y_samples, threshold, segment, segment_data, sample, sample_data);
// }

// chipmunk/cpBody.h
inline cpBody *BodyAlloc() {
  return cpBodyAlloc();
}
inline cpBody *BodyInit(cpBody *body, cpFloat mass, cpFloat moment) {
  return cpBodyInit(body, mass, moment);
}
inline cpBody *BodyNew(cpFloat mass, cpFloat moment) {
  return cpBodyNew(mass, moment);
}
inline cpBody *BodyNewKinematic() {
  return cpBodyNewKinematic();
}
inline cpBody *BodyNewStatic() {
  return cpBodyNewStatic();
}
inline void BodyDestroy(cpBody *body) {
  return cpBodyDestroy(body);
}
inline void BodyFree(cpBody *body) {
  return cpBodyFree(body);
}
inline void BodyActivate(cpBody *body) {
  return cpBodyActivate(body);
}
inline void BodyActivateStatic(cpBody *body, cpShape *filter) {
  return cpBodyActivateStatic(body, filter);
}
inline void BodySleep(cpBody *body) {
  return cpBodySleep(body);
}
inline void BodySleepWithGroup(cpBody *body, cpBody *group) {
  return cpBodySleepWithGroup(body, group);
}
inline cpBool BodyIsSleeping(const cpBody *body) {
  return cpBodyIsSleeping(body);
}
inline cpBodyType BodyGetType(cpBody *body) {
  return cpBodyGetType(body);
}
inline void BodySetType(cpBody *body, cpBodyType type) {
  return cpBodySetType(body, type);
}
inline cpSpace *BodyGetSpace(const cpBody *body) {
  return cpBodyGetSpace(body);
}
inline cpFloat BodyGetMass(const cpBody *body) {
  return cpBodyGetMass(body);
}
inline void BodySetMass(cpBody *body, cpFloat m) {
  return cpBodySetMass(body, m);
}
inline cpFloat BodyGetMoment(const cpBody *body) {
  return cpBodyGetMoment(body);
}
inline void BodySetMoment(cpBody *body, cpFloat i) {
  return cpBodySetMoment(body, i);
}
inline Vec2 BodyGetPosition(const cpBody *body) {
  return convert(cpBodyGetPosition(body));
}
inline void BodySetPosition(cpBody *body, Vec2 pos) {
  return cpBodySetPosition(body, convert(pos));
}
inline Vec2 BodyGetCenterOfGravity(const cpBody *body) {
  return convert(cpBodyGetCenterOfGravity(body));
}
inline void BodySetCenterOfGravity(cpBody *body, Vec2 cog) {
  return cpBodySetCenterOfGravity(body, convert(cog));
}
inline Vec2 BodyGetVelocity(const cpBody *body) {
  return convert(cpBodyGetVelocity(body));
}
inline void BodySetVelocity(cpBody *body, Vec2 velocity) {
  return cpBodySetVelocity(body, convert(velocity));
}
inline Vec2 BodyGetForce(const cpBody *body) {
  return convert(cpBodyGetForce(body));
}
inline void BodySetForce(cpBody *body, Vec2 force) {
  return cpBodySetForce(body, convert(force));
}
inline cpFloat BodyGetAngle(const cpBody *body) {
  return cpBodyGetAngle(body);
}
inline void BodySetAngle(cpBody *body, cpFloat a) {
  return cpBodySetAngle(body, a);
}
inline cpFloat BodyGetAngularVelocity(const cpBody *body) {
  return cpBodyGetAngularVelocity(body);
}
inline void BodySetAngularVelocity(cpBody *body, cpFloat angularVelocity) {
  return cpBodySetAngularVelocity(body, angularVelocity);
}
inline cpFloat BodyGetTorque(const cpBody *body) {
  return cpBodyGetTorque(body);
}
inline void BodySetTorque(cpBody *body, cpFloat torque) {
  return cpBodySetTorque(body, torque);
}
inline Vec2 BodyGetRotation(const cpBody *body) {
  return convert(cpBodyGetRotation(body));
}
inline cpDataPointer BodyGetUserData(const cpBody *body) {
  return cpBodyGetUserData(body);
}
inline void BodySetUserData(cpBody *body, cpDataPointer userData) {
  return cpBodySetUserData(body, userData);
}
inline void BodySetVelocityUpdateFunc(cpBody *body, cpBodyVelocityFunc velocityFunc) {
  return cpBodySetVelocityUpdateFunc(body, velocityFunc);
}
inline void BodySetPositionUpdateFunc(cpBody *body, cpBodyPositionFunc positionFunc) {
  return cpBodySetPositionUpdateFunc(body, positionFunc);
}
inline void BodyUpdateVelocity(cpBody *body, Vec2 gravity, cpFloat damping, cpFloat dt) {
  return cpBodyUpdateVelocity(body, convert(gravity), damping, dt);
}
inline void BodyUpdatePosition(cpBody *body, cpFloat dt) {
  return cpBodyUpdatePosition(body, dt);
}
inline Vec2 BodyLocalToWorld(const cpBody *body, const Vec2 point) {
  return convert(cpBodyLocalToWorld(body, convert(point)));
}
inline Vec2 BodyWorldToLocal(const cpBody *body, const Vec2 point) {
  return convert(cpBodyWorldToLocal(body, convert(point)));
}
inline void BodyApplyForceAtWorldPoint(cpBody *body, Vec2 force, Vec2 point) {
  return cpBodyApplyForceAtWorldPoint(body, convert(force), convert(point));
}
inline void BodyApplyForceAtLocalPoint(cpBody *body, Vec2 force, Vec2 point) {
  return cpBodyApplyForceAtLocalPoint(body, convert(force), convert(point));
}
inline void BodyApplyImpulseAtWorldPoint(cpBody *body, Vec2 impulse, Vec2 point) {
  return cpBodyApplyImpulseAtWorldPoint(body, convert(impulse), convert(point));
}
inline void BodyApplyImpulseAtLocalPoint(cpBody *body, Vec2 impulse, Vec2 point) {
  return cpBodyApplyImpulseAtLocalPoint(body, convert(impulse), convert(point));
}
inline Vec2 BodyGetVelocityAtWorldPoint(const cpBody *body, Vec2 point) {
  return convert(cpBodyGetVelocityAtWorldPoint(body, convert(point)));
}
inline Vec2 BodyGetVelocityAtLocalPoint(const cpBody *body, Vec2 point) {
  return convert(cpBodyGetVelocityAtLocalPoint(body, convert(point)));
}
inline cpFloat BodyKineticEnergy(const cpBody *body) {
  return cpBodyKineticEnergy(body);
}
inline void BodyEachShape(cpBody *body, cpBodyShapeIteratorFunc func, void *data) {
  return cpBodyEachShape(body, func, data);
}
inline void BodyEachConstraint(cpBody *body, cpBodyConstraintIteratorFunc func, void *data) {
  return cpBodyEachConstraint(body, func, data);
}
inline void BodyEachArbiter(cpBody *body, cpBodyArbiterIteratorFunc func, void *data) {
  return cpBodyEachArbiter(body, func, data);
}

// chipmunk/cpRotaryLimitJoint.h
inline cpBool ConstraintIsRotaryLimitJoint(const cpConstraint *constraint) {
  return cpConstraintIsRotaryLimitJoint(constraint);
}
inline cpRotaryLimitJoint *RotaryLimitJointAlloc() {
  return cpRotaryLimitJointAlloc();
}
inline cpRotaryLimitJoint *RotaryLimitJointInit(
    cpRotaryLimitJoint *joint, cpBody *a, cpBody *b, cpFloat min, cpFloat max) {
  return cpRotaryLimitJointInit(joint, a, b, min, max);
}
inline cpConstraint *RotaryLimitJointNew(cpBody *a, cpBody *b, cpFloat min, cpFloat max) {
  return cpRotaryLimitJointNew(a, b, min, max);
}
inline cpFloat RotaryLimitJointGetMin(const cpConstraint *constraint) {
  return cpRotaryLimitJointGetMin(constraint);
}
inline void RotaryLimitJointSetMin(cpConstraint *constraint, cpFloat min) {
  return cpRotaryLimitJointSetMin(constraint, min);
}
inline cpFloat RotaryLimitJointGetMax(const cpConstraint *constraint) {
  return cpRotaryLimitJointGetMax(constraint);
}
inline void RotaryLimitJointSetMax(cpConstraint *constraint, cpFloat max) {
  return cpRotaryLimitJointSetMax(constraint, max);
}

// chipmunk/cpPolyline.h
// void PolylineFree(cpPolyline *line) {
//   return cpPolylineFree(line);
// }
// cpBool PolylineIsClosed(cpPolyline *line) {
//   return cpPolylineIsClosed(line);
// }
// cpPolyline *PolylineSimplifyCurves(cpPolyline *line, cpFloat tol) {
//   return cp*cpPolylineSimplifyCurves(line, tol);
// }
// cpPolyline *PolylineSimplifyVertexes(cpPolyline *line, cpFloat tol) {
//   return cp*cpPolylineSimplifyVertexes(line, tol);
// }
// cpPolyline *PolylineToConvexHull(cpPolyline *line, cpFloat tol) {
//   return cp*cpPolylineToConvexHull(line, tol);
// }
// cpPolylineSet *PolylineSetAlloc() {
//   return cpPolylineSetAlloc();
// }
// cpPolylineSet *PolylineSetInit(cpPolylineSet *set) {
//   return cp*cpPolylineSetInit(set);
// }
// cpPolylineSet *PolylineSetNew() {
//   return cpPolylineSetNew();
// }
// void PolylineSetDestroy(cpPolylineSet *set, cpBool freePolylines) {
//   return cpPolylineSetDestroy(set, freePolylines);
// }
// void PolylineSetFree(cpPolylineSet *set, cpBool freePolylines) {
//   return cpPolylineSetFree(set, freePolylines);
// }
// void PolylineSetCollectSegment(Vec2 v0, Vec2 v1, cpPolylineSet *lines) {
//   return cpPolylineSetCollectSegment(v0, v1, lines);
// }
// cpPolylineSet *PolylineConvexDecomposition(cpPolyline *line, cpFloat tol) {
//   return cp*cpPolylineConvexDecomposition(line, tol);
// }

// chipmunk/chipmunk_unsafe.h
inline void CircleShapeSetRadius(cpShape *shape, cpFloat radius) {
  return cpCircleShapeSetRadius(shape, radius);
}
inline void CircleShapeSetOffset(cpShape *shape, Vec2 offset) {
  return cpCircleShapeSetOffset(shape, convert(offset));
}
inline void SegmentShapeSetEndpoints(cpShape *shape, Vec2 a, Vec2 b) {
  return cpSegmentShapeSetEndpoints(shape, convert(a), convert(b));
}
inline void SegmentShapeSetRadius(cpShape *shape, cpFloat radius) {
  return cpSegmentShapeSetRadius(shape, radius);
}
inline void PolyShapeSetVerts(cpShape *shape, int count, Vec2 *verts, cpTransform transform) {
  return cpPolyShapeSetVerts(shape, count, (cpVect *)verts, transform);
}
inline void PolyShapeSetVertsRaw(cpShape *shape, int count, Vec2 *verts) {
  return cpPolyShapeSetVertsRaw(shape, count, (cpVect *)verts);
}
inline void PolyShapeSetRadius(cpShape *shape, cpFloat radius) {
  return cpPolyShapeSetRadius(shape, radius);
}

// chipmunk/cpConstraint.h
inline void ConstraintDestroy(cpConstraint *constraint) {
  return cpConstraintDestroy(constraint);
}
inline void ConstraintFree(cpConstraint *constraint) {
  return cpConstraintFree(constraint);
}
inline cpSpace *ConstraintGetSpace(const cpConstraint *constraint) {
  return cpConstraintGetSpace(constraint);
}
inline cpBody *ConstraintGetBodyA(const cpConstraint *constraint) {
  return cpConstraintGetBodyA(constraint);
}
inline cpBody *ConstraintGetBodyB(const cpConstraint *constraint) {
  return cpConstraintGetBodyB(constraint);
}
inline cpFloat ConstraintGetMaxForce(const cpConstraint *constraint) {
  return cpConstraintGetMaxForce(constraint);
}
inline void ConstraintSetMaxForce(cpConstraint *constraint, cpFloat maxForce) {
  return cpConstraintSetMaxForce(constraint, maxForce);
}
inline cpFloat ConstraintGetErrorBias(const cpConstraint *constraint) {
  return cpConstraintGetErrorBias(constraint);
}
inline void ConstraintSetErrorBias(cpConstraint *constraint, cpFloat errorBias) {
  return cpConstraintSetErrorBias(constraint, errorBias);
}
inline cpFloat ConstraintGetMaxBias(const cpConstraint *constraint) {
  return cpConstraintGetMaxBias(constraint);
}
inline void ConstraintSetMaxBias(cpConstraint *constraint, cpFloat maxBias) {
  return cpConstraintSetMaxBias(constraint, maxBias);
}
inline cpBool ConstraintGetCollideBodies(const cpConstraint *constraint) {
  return cpConstraintGetCollideBodies(constraint);
}
inline void ConstraintSetCollideBodies(cpConstraint *constraint, cpBool collideBodies) {
  return cpConstraintSetCollideBodies(constraint, collideBodies);
}
inline cpConstraintPreSolveFunc ConstraintGetPreSolveFunc(const cpConstraint *constraint) {
  return cpConstraintGetPreSolveFunc(constraint);
}
inline void ConstraintSetPreSolveFunc(
    cpConstraint *constraint, cpConstraintPreSolveFunc preSolveFunc) {
  return cpConstraintSetPreSolveFunc(constraint, preSolveFunc);
}
inline cpConstraintPostSolveFunc ConstraintGetPostSolveFunc(const cpConstraint *constraint) {
  return cpConstraintGetPostSolveFunc(constraint);
}
inline void ConstraintSetPostSolveFunc(
    cpConstraint *constraint, cpConstraintPostSolveFunc postSolveFunc) {
  return cpConstraintSetPostSolveFunc(constraint, postSolveFunc);
}
inline cpDataPointer ConstraintGetUserData(const cpConstraint *constraint) {
  return cpConstraintGetUserData(constraint);
}
inline void ConstraintSetUserData(cpConstraint *constraint, cpDataPointer userData) {
  return cpConstraintSetUserData(constraint, userData);
}
inline cpFloat ConstraintGetImpulse(cpConstraint *constraint) {
  return cpConstraintGetImpulse(constraint);
}

// chipmunk/cpPinJoint.h
inline cpBool ConstraintIsPinJoint(const cpConstraint *constraint) {
  return cpConstraintIsPinJoint(constraint);
}
inline cpPinJoint *PinJointAlloc() {
  return cpPinJointAlloc();
}
inline cpPinJoint *PinJointInit(
    cpPinJoint *joint, cpBody *a, cpBody *b, Vec2 anchorA, Vec2 anchorB) {
  return cpPinJointInit(joint, a, b, convert(anchorA), convert(anchorB));
}
inline cpConstraint *PinJointNew(cpBody *a, cpBody *b, Vec2 anchorA, Vec2 anchorB) {
  return cpPinJointNew(a, b, convert(anchorA), convert(anchorB));
}
inline Vec2 PinJointGetAnchorA(const cpConstraint *constraint) {
  return convert(cpPinJointGetAnchorA(constraint));
}
inline void PinJointSetAnchorA(cpConstraint *constraint, Vec2 anchorA) {
  return cpPinJointSetAnchorA(constraint, convert(anchorA));
}
inline Vec2 PinJointGetAnchorB(const cpConstraint *constraint) {
  return convert(cpPinJointGetAnchorB(constraint));
}
inline void PinJointSetAnchorB(cpConstraint *constraint, Vec2 anchorB) {
  return cpPinJointSetAnchorB(constraint, convert(anchorB));
}
inline cpFloat PinJointGetDist(const cpConstraint *constraint) {
  return cpPinJointGetDist(constraint);
}
inline void PinJointSetDist(cpConstraint *constraint, cpFloat dist) {
  return cpPinJointSetDist(constraint, dist);
}

// chipmunk/cpSimpleMotor.h
inline cpBool ConstraintIsSimpleMotor(const cpConstraint *constraint) {
  return cpConstraintIsSimpleMotor(constraint);
}
inline cpSimpleMotor *SimpleMotorAlloc() {
  return cpSimpleMotorAlloc();
}
inline cpSimpleMotor *SimpleMotorInit(cpSimpleMotor *joint, cpBody *a, cpBody *b, cpFloat rate) {
  return cpSimpleMotorInit(joint, a, b, rate);
}
inline cpConstraint *SimpleMotorNew(cpBody *a, cpBody *b, cpFloat rate) {
  return cpSimpleMotorNew(a, b, rate);
}
inline cpFloat SimpleMotorGetRate(const cpConstraint *constraint) {
  return cpSimpleMotorGetRate(constraint);
}
inline void SimpleMotorSetRate(cpConstraint *constraint, cpFloat rate) {
  return cpSimpleMotorSetRate(constraint, rate);
}

// chipmunk/cpGearJoint.h
inline cpBool ConstraintIsGearJoint(const cpConstraint *constraint) {
  return cpConstraintIsGearJoint(constraint);
}
inline cpGearJoint *GearJointAlloc() {
  return cpGearJointAlloc();
}
inline cpGearJoint *GearJointInit(
    cpGearJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio) {
  return cpGearJointInit(joint, a, b, phase, ratio);
}
inline cpConstraint *GearJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio) {
  return cpGearJointNew(a, b, phase, ratio);
}
inline cpFloat GearJointGetPhase(const cpConstraint *constraint) {
  return cpGearJointGetPhase(constraint);
}
inline void GearJointSetPhase(cpConstraint *constraint, cpFloat phase) {
  return cpGearJointSetPhase(constraint, phase);
}
inline cpFloat GearJointGetRatio(const cpConstraint *constraint) {
  return cpGearJointGetRatio(constraint);
}
inline void GearJointSetRatio(cpConstraint *constraint, cpFloat ratio) {
  return cpGearJointSetRatio(constraint, ratio);
}

// chipmunk/cpArbiter.h
inline cpFloat ArbiterGetRestitution(const cpArbiter *arb) {
  return cpArbiterGetRestitution(arb);
}
inline void ArbiterSetRestitution(cpArbiter *arb, cpFloat restitution) {
  return cpArbiterSetRestitution(arb, restitution);
}
inline cpFloat ArbiterGetFriction(const cpArbiter *arb) {
  return cpArbiterGetFriction(arb);
}
inline void ArbiterSetFriction(cpArbiter *arb, cpFloat friction) {
  return cpArbiterSetFriction(arb, friction);
}
inline Vec2 ArbiterGetSurfaceVelocity(cpArbiter *arb) {
  return convert(cpArbiterGetSurfaceVelocity(arb));
}
inline void ArbiterSetSurfaceVelocity(cpArbiter *arb, Vec2 vr) {
  return cpArbiterSetSurfaceVelocity(arb, convert(vr));
}
inline cpDataPointer ArbiterGetUserData(const cpArbiter *arb) {
  return cpArbiterGetUserData(arb);
}
inline void ArbiterSetUserData(cpArbiter *arb, cpDataPointer userData) {
  return cpArbiterSetUserData(arb, userData);
}
inline Vec2 ArbiterTotalImpulse(const cpArbiter *arb) {
  return convert(cpArbiterTotalImpulse(arb));
}
inline cpFloat ArbiterTotalKE(const cpArbiter *arb) {
  return cpArbiterTotalKE(arb);
}
inline cpBool ArbiterIgnore(cpArbiter *arb) {
  return cpArbiterIgnore(arb);
}
inline void ArbiterGetShapes(const cpArbiter *arb, cpShape **a, cpShape **b) {
  return cpArbiterGetShapes(arb, a, b);
}
inline void ArbiterGetBodies(const cpArbiter *arb, cpBody **a, cpBody **b) {
  return cpArbiterGetBodies(arb, a, b);
}
inline cpContactPointSet ArbiterGetContactPointSet(const cpArbiter *arb) {
  return cpArbiterGetContactPointSet(arb);
}
inline void ArbiterSetContactPointSet(cpArbiter *arb, cpContactPointSet *set) {
  return cpArbiterSetContactPointSet(arb, set);
}
inline cpBool ArbiterIsFirstContact(const cpArbiter *arb) {
  return cpArbiterIsFirstContact(arb);
}
inline cpBool ArbiterIsRemoval(const cpArbiter *arb) {
  return cpArbiterIsRemoval(arb);
}
inline int ArbiterGetCount(const cpArbiter *arb) {
  return cpArbiterGetCount(arb);
}
inline Vec2 ArbiterGetNormal(const cpArbiter *arb) {
  return convert(cpArbiterGetNormal(arb));
}
inline Vec2 ArbiterGetPointA(const cpArbiter *arb, int i) {
  return convert(cpArbiterGetPointA(arb, i));
}
inline Vec2 ArbiterGetPointB(const cpArbiter *arb, int i) {
  return convert(cpArbiterGetPointB(arb, i));
}
inline cpFloat ArbiterGetDepth(const cpArbiter *arb, int i) {
  return cpArbiterGetDepth(arb, i);
}
inline cpBool ArbiterCallWildcardBeginA(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardBeginA(arb, space);
}
inline cpBool ArbiterCallWildcardBeginB(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardBeginB(arb, space);
}
inline cpBool ArbiterCallWildcardPreSolveA(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardPreSolveA(arb, space);
}
inline cpBool ArbiterCallWildcardPreSolveB(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardPreSolveB(arb, space);
}
inline void ArbiterCallWildcardPostSolveA(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardPostSolveA(arb, space);
}
inline void ArbiterCallWildcardPostSolveB(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardPostSolveB(arb, space);
}
inline void ArbiterCallWildcardSeparateA(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardSeparateA(arb, space);
}
inline void ArbiterCallWildcardSeparateB(cpArbiter *arb, cpSpace *space) {
  return cpArbiterCallWildcardSeparateB(arb, space);
}

// chipmunk/cpShape.h
inline void ShapeDestroy(cpShape *shape) {
  return cpShapeDestroy(shape);
}
inline void ShapeFree(cpShape *shape) {
  return cpShapeFree(shape);
}
inline cpBB ShapeCacheBB(cpShape *shape) {
  return cpShapeCacheBB(shape);
}
inline cpBB ShapeUpdate(cpShape *shape, cpTransform transform) {
  return cpShapeUpdate(shape, transform);
}
inline cpFloat ShapePointQuery(const cpShape *shape, Vec2 p, cpPointQueryInfo *out) {
  return cpShapePointQuery(shape, convert(p), out);
}
inline cpBool ShapeSegmentQuery(
    const cpShape *shape, Vec2 a, Vec2 b, cpFloat radius, cpSegmentQueryInfo *info) {
  return cpShapeSegmentQuery(shape, convert(a), convert(b), radius, info);
}
inline cpContactPointSet ShapesCollide(const cpShape *a, const cpShape *b) {
  return cpShapesCollide(a, b);
}
inline cpSpace *ShapeGetSpace(const cpShape *shape) {
  return cpShapeGetSpace(shape);
}
inline cpBody *ShapeGetBody(const cpShape *shape) {
  return cpShapeGetBody(shape);
}
inline void ShapeSetBody(cpShape *shape, cpBody *body) {
  return cpShapeSetBody(shape, body);
}
inline cpFloat ShapeGetMass(cpShape *shape) {
  return cpShapeGetMass(shape);
}
inline void ShapeSetMass(cpShape *shape, cpFloat mass) {
  return cpShapeSetMass(shape, mass);
}
inline cpFloat ShapeGetDensity(cpShape *shape) {
  return cpShapeGetDensity(shape);
}
inline void ShapeSetDensity(cpShape *shape, cpFloat density) {
  return cpShapeSetDensity(shape, density);
}
inline cpFloat ShapeGetMoment(cpShape *shape) {
  return cpShapeGetMoment(shape);
}
inline cpFloat ShapeGetArea(cpShape *shape) {
  return cpShapeGetArea(shape);
}
inline Vec2 ShapeGetCenterOfGravity(cpShape *shape) {
  return convert(cpShapeGetCenterOfGravity(shape));
}
inline cpBB ShapeGetBB(const cpShape *shape) {
  return cpShapeGetBB(shape);
}
inline cpBool ShapeGetSensor(const cpShape *shape) {
  return cpShapeGetSensor(shape);
}
inline void ShapeSetSensor(cpShape *shape, cpBool sensor) {
  return cpShapeSetSensor(shape, sensor);
}
inline cpFloat ShapeGetElasticity(const cpShape *shape) {
  return cpShapeGetElasticity(shape);
}
inline void ShapeSetElasticity(cpShape *shape, cpFloat elasticity) {
  return cpShapeSetElasticity(shape, elasticity);
}
inline cpFloat ShapeGetFriction(const cpShape *shape) {
  return cpShapeGetFriction(shape);
}
inline void ShapeSetFriction(cpShape *shape, cpFloat friction) {
  return cpShapeSetFriction(shape, friction);
}
inline Vec2 ShapeGetSurfaceVelocity(const cpShape *shape) {
  return convert(cpShapeGetSurfaceVelocity(shape));
}
inline void ShapeSetSurfaceVelocity(cpShape *shape, Vec2 surfaceVelocity) {
  return cpShapeSetSurfaceVelocity(shape, convert(surfaceVelocity));
}
inline cpDataPointer ShapeGetUserData(const cpShape *shape) {
  return cpShapeGetUserData(shape);
}
inline void ShapeSetUserData(cpShape *shape, cpDataPointer userData) {
  return cpShapeSetUserData(shape, userData);
}
inline cpCollisionType ShapeGetCollisionType(const cpShape *shape) {
  return cpShapeGetCollisionType(shape);
}
inline void ShapeSetCollisionType(cpShape *shape, cpCollisionType collisionType) {
  return cpShapeSetCollisionType(shape, collisionType);
}
inline cpShapeFilter ShapeGetFilter(const cpShape *shape) {
  return cpShapeGetFilter(shape);
}
inline void ShapeSetFilter(cpShape *shape, cpShapeFilter filter) {
  return cpShapeSetFilter(shape, filter);
}
inline cpCircleShape *CircleShapeAlloc() {
  return cpCircleShapeAlloc();
}
inline cpCircleShape *CircleShapeInit(
    cpCircleShape *circle, cpBody *body, cpFloat radius, Vec2 offset) {
  return cpCircleShapeInit(circle, body, radius, convert(offset));
}
inline cpShape *CircleShapeNew(cpBody *body, cpFloat radius, Vec2 offset) {
  return cpCircleShapeNew(body, radius, convert(offset));
}
inline Vec2 CircleShapeGetOffset(const cpShape *shape) {
  return convert(cpCircleShapeGetOffset(shape));
}
inline cpFloat CircleShapeGetRadius(const cpShape *shape) {
  return cpCircleShapeGetRadius(shape);
}
inline cpSegmentShape *SegmentShapeAlloc() {
  return cpSegmentShapeAlloc();
}
inline cpSegmentShape *SegmentShapeInit(
    cpSegmentShape *seg, cpBody *body, Vec2 a, Vec2 b, cpFloat radius) {
  return cpSegmentShapeInit(seg, body, convert(a), convert(b), radius);
}
inline cpShape *SegmentShapeNew(cpBody *body, Vec2 a, Vec2 b, cpFloat radius) {
  return cpSegmentShapeNew(body, convert(a), convert(b), radius);
}
inline void SegmentShapeSetNeighbors(cpShape *shape, Vec2 prev, Vec2 next) {
  return cpSegmentShapeSetNeighbors(shape, convert(prev), convert(next));
}
inline Vec2 SegmentShapeGetA(const cpShape *shape) {
  return convert(cpSegmentShapeGetA(shape));
}
inline Vec2 SegmentShapeGetB(const cpShape *shape) {
  return convert(cpSegmentShapeGetB(shape));
}
inline Vec2 SegmentShapeGetNormal(const cpShape *shape) {
  return convert(cpSegmentShapeGetNormal(shape));
}
inline cpFloat SegmentShapeGetRadius(const cpShape *shape) {
  return cpSegmentShapeGetRadius(shape);
}

// chipmunk/cpDampedSpring.h
inline cpBool ConstraintIsDampedSpring(const cpConstraint *constraint) {
  return cpConstraintIsDampedSpring(constraint);
}
inline cpDampedSpring *DampedSpringAlloc() {
  return cpDampedSpringAlloc();
}
inline cpDampedSpring *DampedSpringInit(cpDampedSpring *joint, cpBody *a, cpBody *b, Vec2 anchorA,
    Vec2 anchorB, cpFloat restLength, cpFloat stiffness, cpFloat damping) {
  return cpDampedSpringInit(
      joint, a, b, convert(anchorA), convert(anchorB), restLength, stiffness, damping);
}
inline cpConstraint *DampedSpringNew(cpBody *a, cpBody *b, Vec2 anchorA, Vec2 anchorB,
    cpFloat restLength, cpFloat stiffness, cpFloat damping) {
  return cpDampedSpringNew(
      a, b, convert(anchorA), convert(anchorB), restLength, stiffness, damping);
}
inline Vec2 DampedSpringGetAnchorA(const cpConstraint *constraint) {
  return convert(cpDampedSpringGetAnchorA(constraint));
}
inline void DampedSpringSetAnchorA(cpConstraint *constraint, Vec2 anchorA) {
  return cpDampedSpringSetAnchorA(constraint, convert(anchorA));
}
inline Vec2 DampedSpringGetAnchorB(const cpConstraint *constraint) {
  return convert(cpDampedSpringGetAnchorB(constraint));
}
inline void DampedSpringSetAnchorB(cpConstraint *constraint, Vec2 anchorB) {
  return cpDampedSpringSetAnchorB(constraint, convert(anchorB));
}
inline cpFloat DampedSpringGetRestLength(const cpConstraint *constraint) {
  return cpDampedSpringGetRestLength(constraint);
}
inline void DampedSpringSetRestLength(cpConstraint *constraint, cpFloat restLength) {
  return cpDampedSpringSetRestLength(constraint, restLength);
}
inline cpFloat DampedSpringGetStiffness(const cpConstraint *constraint) {
  return cpDampedSpringGetStiffness(constraint);
}
inline void DampedSpringSetStiffness(cpConstraint *constraint, cpFloat stiffness) {
  return cpDampedSpringSetStiffness(constraint, stiffness);
}
inline cpFloat DampedSpringGetDamping(const cpConstraint *constraint) {
  return cpDampedSpringGetDamping(constraint);
}
inline void DampedSpringSetDamping(cpConstraint *constraint, cpFloat damping) {
  return cpDampedSpringSetDamping(constraint, damping);
}
inline cpDampedSpringForceFunc DampedSpringGetSpringForceFunc(const cpConstraint *constraint) {
  return cpDampedSpringGetSpringForceFunc(constraint);
}
inline void DampedSpringSetSpringForceFunc(
    cpConstraint *constraint, cpDampedSpringForceFunc springForceFunc) {
  return cpDampedSpringSetSpringForceFunc(constraint, springForceFunc);
}

// chipmunk/cpSpatialIndex.h
inline cpSpaceHash *SpaceHashAlloc() {
  return cpSpaceHashAlloc();
}
inline cpSpatialIndex *SpaceHashInit(cpSpaceHash *hash, cpFloat celldim, int numcells,
    cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex) {
  return cpSpaceHashInit(hash, celldim, numcells, bbfunc, staticIndex);
}
inline cpSpatialIndex *SpaceHashNew(
    cpFloat celldim, int cells, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex) {
  return cpSpaceHashNew(celldim, cells, bbfunc, staticIndex);
}
inline void SpaceHashResize(cpSpaceHash *hash, cpFloat celldim, int numcells) {
  return cpSpaceHashResize(hash, celldim, numcells);
}
inline cpBBTree *BBTreeAlloc() {
  return cpBBTreeAlloc();
}
inline cpSpatialIndex *BBTreeInit(
    cpBBTree *tree, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex) {
  return cpBBTreeInit(tree, bbfunc, staticIndex);
}
inline cpSpatialIndex *BBTreeNew(cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex) {
  return cpBBTreeNew(bbfunc, staticIndex);
}
inline void BBTreeOptimize(cpSpatialIndex *index) {
  return cpBBTreeOptimize(index);
}
inline void BBTreeSetVelocityFunc(cpSpatialIndex *index, cpBBTreeVelocityFunc func) {
  return cpBBTreeSetVelocityFunc(index, func);
}
inline cpSweep1D *Sweep1DAlloc() {
  return cpSweep1DAlloc();
}
inline cpSpatialIndex *Sweep1DInit(
    cpSweep1D *sweep, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex) {
  return cpSweep1DInit(sweep, bbfunc, staticIndex);
}
inline cpSpatialIndex *Sweep1DNew(cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex) {
  return cpSweep1DNew(bbfunc, staticIndex);
}
inline void SpatialIndexFree(cpSpatialIndex *index) {
  return cpSpatialIndexFree(index);
}
inline void SpatialIndexCollideStatic(cpSpatialIndex *dynamicIndex, cpSpatialIndex *staticIndex,
    cpSpatialIndexQueryFunc func, void *data) {
  return cpSpatialIndexCollideStatic(dynamicIndex, staticIndex, func, data);
}

// chipmunk/cpPolyShape.h
inline cpPolyShape *PolyShapeAlloc() {
  return cpPolyShapeAlloc();
}
inline cpPolyShape *PolyShapeInit(cpPolyShape *poly, cpBody *body, int count, const Vec2 *verts,
    cpTransform transform, cpFloat radius) {
  return cpPolyShapeInit(poly, body, count, (const cpVect *)verts, transform, radius);
}
inline cpPolyShape *PolyShapeInitRaw(
    cpPolyShape *poly, cpBody *body, int count, const Vec2 *verts, cpFloat radius) {
  return cpPolyShapeInitRaw(poly, body, count, (const cpVect *)verts, radius);
}
inline cpShape *PolyShapeNew(
    cpBody *body, int count, const Vec2 *verts, cpTransform transform, cpFloat radius) {
  return cpPolyShapeNew(body, count, (const cpVect *)verts, transform, radius);
}
inline cpShape *PolyShapeNewRaw(cpBody *body, int count, const Vec2 *verts, cpFloat radius) {
  return cpPolyShapeNewRaw(body, count, (const cpVect *)verts, radius);
}
inline cpPolyShape *BoxShapeInit(
    cpPolyShape *poly, cpBody *body, cpFloat width, cpFloat height, cpFloat radius) {
  return cpBoxShapeInit(poly, body, width, height, radius);
}
inline cpPolyShape *BoxShapeInit2(cpPolyShape *poly, cpBody *body, cpBB box, cpFloat radius) {
  return cpBoxShapeInit2(poly, body, box, radius);
}
inline cpShape *BoxShapeNew(cpBody *body, cpFloat width, cpFloat height, cpFloat radius) {
  return cpBoxShapeNew(body, width, height, radius);
}
inline cpShape *BoxShapeNew2(cpBody *body, cpBB box, cpFloat radius) {
  return cpBoxShapeNew2(body, box, radius);
}
inline int PolyShapeGetCount(const cpShape *shape) {
  return cpPolyShapeGetCount(shape);
}
inline Vec2 PolyShapeGetVert(const cpShape *shape, int index) {
  return convert(cpPolyShapeGetVert(shape, index));
}
inline cpFloat PolyShapeGetRadius(const cpShape *shape) {
  return cpPolyShapeGetRadius(shape);
}


//
// Init / deinit on program startup / exit -- allows creating resources at global scope
//

inline Space *space;
inline Body *background;

inline struct Init {
  Init() {
    space = SpaceNew();
    background = BodyNewStatic();
  }
  ~Init() {
    BodyFree(background);
    SpaceFree(space);
  }
} init;


//
// Add / remove utilities
//

inline Body *add(Body *body) {
  return SpaceAddBody(space, body);
}

inline Shape *add(Shape *body) {
  return SpaceAddShape(space, body);
}

inline Constraint *add(Constraint *body) {
  return SpaceAddConstraint(space, body);
}

inline void remove(Body *&body) {
  if (body) {
    cp::BodyEachConstraint(
        body,
        [](cp::Body *, cp::Constraint *constraint, void *) {
          if (cp::SpaceContainsConstraint(cp::space, constraint)) {
            cp::SpaceRemoveConstraint(cp::space, constraint);
          }
        },
        nullptr);
    cp::BodyEachShape(
        body,
        [](cp::Body *, cp::Shape *shape, void *) {
          if (cp::SpaceContainsShape(cp::space, shape)) {
            cp::SpaceRemoveShape(cp::space, shape);
          }
        },
        nullptr);
    if (cp::SpaceContainsBody(cp::space, body)) {
      cp::SpaceRemoveBody(cp::space, body);
    }
    cp::BodyFree(body);
    body = nullptr;
  }
}

inline void remove(Shape *&shape) {
  if (shape) {
    if (cp::SpaceContainsShape(cp::space, shape)) {
      cp::SpaceRemoveShape(cp::space, shape);
    }
    cp::ShapeFree(shape);
    shape = nullptr;
  }
}

inline void remove(Constraint *&constraint) {
  if (constraint) {
    if (cp::SpaceContainsConstraint(cp::space, constraint)) {
      cp::SpaceRemoveConstraint(cp::space, constraint);
    }
    cp::ConstraintFree(constraint);
    constraint = nullptr;
  }
}


}
