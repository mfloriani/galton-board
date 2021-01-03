#pragma once

#include "Math\Vector3D.h"
#include "Math\Matrix4.h"

enum Camera_Movement 
{
    FORWARD,
    BACKWARD,
    UP,
    DOWN,
    LEFT,
    RIGHT
};

const float SPEED = 5.5f;

class Camera
{
public:
    // camera Attributes
    math::Vector3D Position;
    math::Vector3D Front;
    math::Vector3D Up;
    math::Vector3D Right;
    
    math::Matrix4 View;

    // constructor with vectors
    Camera( math::Vector3D position = math::Vector3D(0.0f, 0.0f, 0.0f) ) 
        : Position(position),
          Front(math::Vector3D(0.0f, 0.0f, 1.0f)),
          Up(math::Vector3D(0.0f, 1.0f, 0.0f)),
          Right(math::Vector3D(1.0f, 0.0f, 0.0f))
    {
    }
    // constructor with scalar values
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) 
        : Position(math::Vector3D(posX, posY, posZ)),
          Front(math::Vector3D(0.0f, 0.0f, 1.0f)), 
          Up(math::Vector3D(upX, upY, upZ)),
          Right(math::Vector3D(1.0f, 0.0f, 0.0f))
    {
    }

    void ProcessKeyboard(Camera_Movement direction)
    {
        float velocity = SPEED;
        if (direction == FORWARD)
            Position -= Front * velocity;
        if (direction == BACKWARD)
            Position += Front * velocity;
        if (direction == UP)
            Position += Up * velocity;
        if (direction == DOWN)
            Position -= Up * velocity;
        if (direction == LEFT)
            Position -= Right * velocity;
        if (direction == RIGHT)
            Position += Right * velocity;
    }

    math::Matrix4& UpdateView()
    {
        math::Vector3D R = Right;
        math::Vector3D U = Up;
        math::Vector3D L = Front;
        math::Vector3D P = Position;

        L = L.normalize();
        U = math::normalize( math::cross(L, R));
        R = math::cross(U, L);

        float x = -P.dot(R);
        float y = -P.dot(U);
        float z = -P.dot(L);

        Right = R;
        Up = U;
        Front = L;

        View.set(0, 0, Right.x);
        View.set(1, 0, Right.y);
        View.set(2, 0, Right.z);
        View.set(3, 0, x);
        
        View.set(0, 1, Up.x);
        View.set(1, 1, Up.y);
        View.set(2, 1, Up.z);
        View.set(3, 1, y);
       
        View.set(0, 2, Front.x);
        View.set(1, 2, Front.y);
        View.set(2, 2, Front.z);
        View.set(3, 2, z);
        
        View.set(0, 3, 0.0f);
        View.set(1, 3, 0.0f);
        View.set(2, 3, 0.0f);
        View.set(3, 3, 1.0f);

        return View;
    }

private:
    
};
