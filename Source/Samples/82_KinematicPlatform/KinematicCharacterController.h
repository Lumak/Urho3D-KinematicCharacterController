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

#pragma once

#include <Urho3D/Scene/Component.h>

namespace Urho3D
{
class PhysicsWorld;
class DebugRenderer;
}

using namespace Urho3D;

class btPairCachingGhostObject;
class btKinematicCharacterController;

//=============================================================================
//=============================================================================
class KinematicCharacterController : public Component
{
    URHO3D_OBJECT(KinematicCharacterController, Component);

public:
    KinematicCharacterController(Context* context);
    virtual ~KinematicCharacterController();

    static void RegisterObject(Context* context);
    virtual void OnSetAttribute(const AttributeInfo& attr, const Variant& src);
    virtual void ApplyAttributes();

    const Vector3& GetPosition();
    const Quaternion& GetRotation();
    void SetTransform(const Vector3& position, const Quaternion& rotation);
    void GetTransform(Vector3& position, Quaternion& rotation);

    void SetCollisionLayer(int layer);
    void SetCollisionMask(int mask);
    void SetCollisionLayerAndMask(int layer, int mask);

    void SetGravity(const Vector3 &gravity);
    const Vector3& GetGravity() const { return gravity_; }
    void SetLinearDamping(float linearDamping);
    float GetLinearDamping() const { return linearDamping_; }
    void SetAngularDamping(float angularDamping);
    float GetAngularDamping() const { return angularDamping_; }

    void SetStepHeight(float stepHeight);
    float GetStepHeight() const { return stepHeight_; }
    void SetMaxJumpHeight(float maxJumpHeight);
    float GetMaxJumpHeight() const { return maxJumpHeight_; }
    void SetFallSpeed(float fallSpeed);
    float GetFallSpeed() const { return fallSpeed_; }
    void SetJumpSpeed(float jumpSpeed);
    float GetJumpSpeed() const { return jumpSpeed_; }
    void SetMaxSlope(float maxSlope);
    float GetMaxSlope() const { return maxSlope_; }

    void SetWalkDirection(const Vector3& walkDir);
    bool OnGround() const;
    void Jump(const Vector3 &jump = Vector3::ZERO);
    /// ApplyImpulse is same as Jump
    void ApplyImpulse(const Vector3 &impulse);
    bool CanJump() const;

    void SetAngularVelocity(const Vector3 &velocity);
    const Vector3 GetAngularVelocity() const;
    void SetLinearVelocity(const Vector3 &velocity);
    const Vector3 GetLinearVelocity() const;
    void Warp(const Vector3 &position);
    virtual void DrawDebugGeometry();

protected:
    void ReleaseKinematic();
    void ApplySettings(bool reapply=false);
    virtual void OnNodeSet(Node* node);
    virtual void OnSceneSet(Scene* scene);
    void AddKinematicToWorld();
    void RemoveKinematicFromWorld();

protected:
    int colLayer_;
    int colMask_;
    float stepHeight_;
    float maxJumpHeight_;
    float jumpSpeed_;
    float fallSpeed_;
    float maxSlope_;
    float linearDamping_;
    float angularDamping_;
    Vector3 gravity_;

    WeakPtr<PhysicsWorld> physicsWorld_;
    UniquePtr<btPairCachingGhostObject> pairCachingGhostObject_;
    UniquePtr<btKinematicCharacterController> kinematicController_;

    Vector3 position_;
    Quaternion rotation_;
    Vector3 colShapeOffset_;
    bool reapplyAttributes_;

};
