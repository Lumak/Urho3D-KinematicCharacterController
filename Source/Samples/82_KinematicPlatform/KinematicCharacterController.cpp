//
// Copyright (c) 2008-2019 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/Context.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsUtils.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/IO/Log.h>

#include <Bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <Bullet/BulletDynamics/Character/btKinematicCharacterController.h>
#include <cassert>

#include "KinematicCharacterController.h"

//=============================================================================
// NOTE! declare these function before #include <Urho3D/DebugNew> 
// otherwise you'll get a compile error for _DEBUG build
//=============================================================================
btPairCachingGhostObject* newPairCachingGhostObj()
{
    return new btPairCachingGhostObject();
}

btKinematicCharacterController* newKinematicCharCtrl(btPairCachingGhostObject *ghostCGO, 
                                                     btConvexShape *shape, float stepHeight, const btVector3 &upVec)
{
    return new btKinematicCharacterController(ghostCGO, shape, stepHeight, upVec);
}

#include <Urho3D/DebugNew.h>
//=============================================================================
//=============================================================================
const float STEP_HEIGHT = 0.4f;
const float JUMP_HEIGHT = 2.0f;
const float FALL_SPEED = 55.0f;
const float JUMP_SPEED = 9.0f;
const float MAX_SLOPE = 45.0f;
const float DEFAULT_DAMPING = 0.2f;
const Vector3 KINEMATIC_GRAVITY(0.0f, -14.0f, 0.0f);

//=============================================================================
//=============================================================================
KinematicCharacterController::KinematicCharacterController(Context* context)
    : Component(context)
    , colLayer_(1)
    , colMask_(0xffff)
    , gravity_(KINEMATIC_GRAVITY)
    , stepHeight_(STEP_HEIGHT)
    , maxJumpHeight_(JUMP_HEIGHT)
    , fallSpeed_(FALL_SPEED)
    , jumpSpeed_(JUMP_SPEED)
    , maxSlope_(MAX_SLOPE)
    , linearDamping_(DEFAULT_DAMPING)
    , angularDamping_(DEFAULT_DAMPING)
    , colShapeOffset_(Vector3::ZERO)
    , reapplyAttributes_(false)
{
    pairCachingGhostObject_.Reset( newPairCachingGhostObj() );
    pairCachingGhostObject_->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
}

KinematicCharacterController::~KinematicCharacterController()
{
    ReleaseKinematic();
}

void KinematicCharacterController::RegisterObject(Context* context)
{
    context->RegisterFactory<KinematicCharacterController>();

    URHO3D_ATTRIBUTE("Gravity", Vector3, gravity_, KINEMATIC_GRAVITY, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Collision Layer", int, colLayer_, 1, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Collision Mask", int, colMask_, 0xffff, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Linear Damping", float, linearDamping_, DEFAULT_DAMPING, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Angular Damping", float, angularDamping_, DEFAULT_DAMPING, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Step Height", float, stepHeight_, STEP_HEIGHT, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Max Jump Height", float, maxJumpHeight_, JUMP_HEIGHT, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Fall Speed", float, fallSpeed_, FALL_SPEED, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Jump Speed", float, jumpSpeed_, JUMP_SPEED, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Max Slope", float, maxSlope_, MAX_SLOPE, AM_DEFAULT);
}

void KinematicCharacterController::OnSetAttribute(const AttributeInfo& attr, const Variant& src)
{
    Serializable::OnSetAttribute(attr, src);

    reapplyAttributes_ = true;
}

void KinematicCharacterController::ApplyAttributes()
{
    if (reapplyAttributes_)
    {
        ApplySettings(true);
        reapplyAttributes_ = false;
    }
}

void KinematicCharacterController::ReleaseKinematic()
{
    if (kinematicController_.NotNull())
    {
        RemoveKinematicFromWorld();
    }

    kinematicController_.Reset();
    pairCachingGhostObject_.Reset();
}

void KinematicCharacterController::OnNodeSet(Node* node)
{
    if (node)
        node->AddListener(this);
}

void KinematicCharacterController::OnSceneSet(Scene* scene)
{
    if (scene)
    {
        if (scene == node_)
            URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

        physicsWorld_ = scene->GetComponent<PhysicsWorld>();

        if (physicsWorld_)
        {
            AddKinematicToWorld();
        }
    }
    else
    {
        RemoveKinematicFromWorld();
    }
}

void KinematicCharacterController::AddKinematicToWorld()
{
    if (physicsWorld_)
    {
        if (kinematicController_.Null())
        {
            CollisionShape *colShape = GetComponent<CollisionShape>();
            assert(colShape && "missing Collision Shape");

            pairCachingGhostObject_->setCollisionShape(colShape->GetCollisionShape());
            colShapeOffset_ = colShape->GetPosition();

            kinematicController_.Reset( newKinematicCharCtrl(pairCachingGhostObject_.Get(), 
                                                       dynamic_cast<btConvexShape*>(colShape->GetCollisionShape()),
                                                       stepHeight_, ToBtVector3(Vector3::UP)));
            // apply default settings
            ApplySettings();

            btDiscreteDynamicsWorld *phyicsWorld = physicsWorld_->GetWorld();
            phyicsWorld->addCollisionObject(pairCachingGhostObject_.Get(), colLayer_, colMask_);
            phyicsWorld->addAction(kinematicController_.Get());
        }
    }
}

void KinematicCharacterController::ApplySettings(bool reapply)
{
    kinematicController_->setGravity(ToBtVector3(gravity_));
    kinematicController_->setLinearDamping(linearDamping_);
    kinematicController_->setAngularDamping(angularDamping_);
    kinematicController_->setStepHeight(stepHeight_);
    kinematicController_->setMaxJumpHeight(maxJumpHeight_);
    kinematicController_->setMaxSlope(M_DEGTORAD * maxSlope_);
    kinematicController_->setJumpSpeed(jumpSpeed_);
    kinematicController_->setFallSpeed(fallSpeed_);

    if (reapply && pairCachingGhostObject_)
    {
        btDiscreteDynamicsWorld *phyicsWorld = physicsWorld_->GetWorld();
        phyicsWorld->removeCollisionObject(pairCachingGhostObject_.Get());
        phyicsWorld->addCollisionObject(pairCachingGhostObject_.Get(), colLayer_, colMask_);
    }

    SetTransform(node_->GetWorldPosition(), node_->GetWorldRotation());
}

void KinematicCharacterController::RemoveKinematicFromWorld()
{
    if (kinematicController_ && physicsWorld_)
    {
        btDiscreteDynamicsWorld *phyicsWorld = physicsWorld_->GetWorld();
        phyicsWorld->removeCollisionObject(pairCachingGhostObject_.Get());
        phyicsWorld->removeAction(kinematicController_.Get());
    }
}

void KinematicCharacterController::SetCollisionLayer(int layer)
{
    if (physicsWorld_)
    {
        if (layer != colLayer_)
        {
            colLayer_ = layer;
            btDiscreteDynamicsWorld *phyicsWorld = physicsWorld_->GetWorld();
            phyicsWorld->removeCollisionObject(pairCachingGhostObject_.Get());
            phyicsWorld->addCollisionObject(pairCachingGhostObject_.Get(), colLayer_, colMask_);
        }
    }
}

void KinematicCharacterController::SetCollisionMask(int mask)
{
    if (physicsWorld_)
    {
        if (mask != colMask_)
        {
            colMask_ = mask;
            btDiscreteDynamicsWorld *phyicsWorld = physicsWorld_->GetWorld();
            phyicsWorld->removeCollisionObject(pairCachingGhostObject_.Get());
            phyicsWorld->addCollisionObject(pairCachingGhostObject_.Get(), colLayer_, colMask_);
        }
    }
}

void KinematicCharacterController::SetCollisionLayerAndMask(int layer, int mask)
{
    if (physicsWorld_)
    {
        if (layer != colLayer_ || mask != colMask_)
        {
            colLayer_ = layer;
            colMask_ = mask;
            btDiscreteDynamicsWorld *phyicsWorld = physicsWorld_->GetWorld();
            phyicsWorld->removeCollisionObject(pairCachingGhostObject_.Get());
            phyicsWorld->addCollisionObject(pairCachingGhostObject_.Get(), colLayer_, colMask_);
        }
    }
}

const Vector3& KinematicCharacterController::GetPosition()
{
    btTransform t = pairCachingGhostObject_->getWorldTransform();
    position_ = ToVector3(t.getOrigin()) - colShapeOffset_;
    return position_;
}

const Quaternion& KinematicCharacterController::GetRotation()
{
    btTransform t = pairCachingGhostObject_->getWorldTransform();
    rotation_ = ToQuaternion(t.getRotation());
    return rotation_;
}

void KinematicCharacterController::SetTransform(const Vector3& position, const Quaternion& rotation)
{
    btTransform worldTrans;
    worldTrans.setIdentity();
    worldTrans.setRotation(ToBtQuaternion(rotation));
    worldTrans.setOrigin(ToBtVector3(position));
    pairCachingGhostObject_->setWorldTransform(worldTrans);
}

void KinematicCharacterController::GetTransform(Vector3& position, Quaternion& rotation)
{
    btTransform worldTrans = pairCachingGhostObject_->getWorldTransform();
    rotation = ToQuaternion(worldTrans.getRotation());
    position = ToVector3(worldTrans.getOrigin());
}

void KinematicCharacterController::SetLinearDamping(float linearDamping)
{
    if (linearDamping != linearDamping_)
    {
        linearDamping_ = linearDamping;
        kinematicController_->setLinearDamping(linearDamping_);
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetAngularDamping(float angularDamping)
{
    if (angularDamping != angularDamping_)
    {
        angularDamping_ = angularDamping;
        kinematicController_->setAngularDamping(angularDamping_);
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetGravity(const Vector3 &gravity)
{
    if (gravity != gravity_)
    {
        gravity_ = gravity;
        kinematicController_->setGravity(ToBtVector3(gravity_));
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetStepHeight(float stepHeight)
{
    if (stepHeight != stepHeight_)
    {
        stepHeight_ = stepHeight;
        kinematicController_->setStepHeight(stepHeight_);
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetMaxJumpHeight(float maxJumpHeight)
{
    if (maxJumpHeight != maxJumpHeight_)
    {
        maxJumpHeight_ =  maxJumpHeight;
        kinematicController_->setMaxJumpHeight(maxJumpHeight_);
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetFallSpeed(float fallSpeed)
{
    if (fallSpeed != fallSpeed_)
    {
        fallSpeed_ = fallSpeed;
        kinematicController_->setFallSpeed(fallSpeed_);
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetJumpSpeed(float jumpSpeed)
{
    if (jumpSpeed != jumpSpeed_)
    {
        jumpSpeed_ = jumpSpeed;
        kinematicController_->setJumpSpeed(jumpSpeed_);
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetMaxSlope(float maxSlope)
{
    if (maxSlope != maxSlope_)
    {
        maxSlope_ = maxSlope;
        kinematicController_->setMaxSlope(M_DEGTORAD * maxSlope_);
        MarkNetworkUpdate();
    }
}

void KinematicCharacterController::SetWalkDirection(const Vector3& walkDir)
{
    kinematicController_->setWalkDirection(ToBtVector3(walkDir));
}

bool KinematicCharacterController::OnGround() const
{
    return kinematicController_->onGround();
}

void KinematicCharacterController::Jump(const Vector3 &jump)
{
    kinematicController_->jump(ToBtVector3(jump));
}

bool KinematicCharacterController::CanJump() const
{
    return kinematicController_->canJump();
}

void KinematicCharacterController::ApplyImpulse(const Vector3 &impulse)
{
	kinematicController_->applyImpulse(ToBtVector3(impulse));
}

void KinematicCharacterController::SetAngularVelocity(const Vector3 &velocity)
{
	kinematicController_->setAngularVelocity(ToBtVector3(velocity));
}

const Vector3 KinematicCharacterController::GetAngularVelocity() const
{
    return ToVector3(kinematicController_->getAngularVelocity());
}

void KinematicCharacterController::SetLinearVelocity(const Vector3 &velocity)
{
	kinematicController_->setLinearVelocity(ToBtVector3(velocity));
}

const Vector3 KinematicCharacterController::GetLinearVelocity() const
{
    return ToVector3(kinematicController_->getLinearVelocity());
}

void KinematicCharacterController::Warp(const Vector3 &position)
{
    kinematicController_->warp(ToBtVector3(position));
}

void KinematicCharacterController::DrawDebugGeometry()
{
    kinematicController_->debugDraw(physicsWorld_);
}

