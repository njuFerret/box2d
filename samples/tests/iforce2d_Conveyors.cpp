/*
* Author: Chris Campbell - www.iforce2d.net
*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef IFORCE2D_CONVEYORS_H
#define IFORCE2D_CONVEYORS_H

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

#include "test.h"
//structure to store current surface velocity of a fixture
struct ConveyorSegment {
    float minAngle;
    float maxAngle;
    float surfaceVelocity;
};

class iforce2d_Conveyors : public Test
{
public:
    iforce2d_Conveyors()
    {
        //body definition
        b2BodyDef myBodyDef;
        myBodyDef.type = b2_dynamicBody;

        //box shape definition
        b2PolygonShape polygonShape;
        b2Vec2 boxVertices[4];
        boxVertices[0].Set(-2.4,-2);//make the moving surface side a little longer so we can tell which it is
        boxVertices[1].Set( 2.4,-2);
        boxVertices[2].Set( 2, 2);
        boxVertices[3].Set(-2, 2);
        polygonShape.Set(boxVertices, 4);

        //fixture definition
        b2FixtureDef myFixtureDef;
        myFixtureDef.density = 1;
        myFixtureDef.friction = 0.8f;//increase friction so relative velocity surfaces react quicker

        //circle shape definition
        b2CircleShape circleShape;
        circleShape.m_radius = 2;

        //create dynamic circle body
        myBodyDef.position.Set(-5, 10);
        myFixtureDef.shape = &circleShape;
        m_circleBody = m_world->CreateBody(&myBodyDef);
        b2Fixture* circleFixture = m_circleBody->CreateFixture(&myFixtureDef);

        //create dynamic box body
        myBodyDef.position.Set(5, 10);
        myFixtureDef.shape = &polygonShape;
        b2Body* boxBody = m_world->CreateBody(&myBodyDef);
        b2Fixture* polygonFixture = boxBody->CreateFixture(&myFixtureDef);

        //a static body
        myBodyDef.type = b2_staticBody;
        myBodyDef.position.Set(0, 0);
        b2Body* staticBody = m_world->CreateBody(&myBodyDef);

        //add four walls to the static body
        polygonShape.SetAsBox( 20, 1, b2Vec2(0, 0), 0);//ground
        staticBody->CreateFixture(&myFixtureDef);
        polygonShape.SetAsBox( 20, 1, b2Vec2(0, 40), 0);//ceiling
        staticBody->CreateFixture(&myFixtureDef);
        polygonShape.SetAsBox( 1, 20, b2Vec2(-20, 20), 0);//left wall
        staticBody->CreateFixture(&myFixtureDef);
        polygonShape.SetAsBox( 1, 20, b2Vec2(20, 20), 0);//right wall
        staticBody->CreateFixture(&myFixtureDef);

        //set moving surface segment in circle body
        m_circleConveyor.minAngle = -135 * DEGTORAD;
        m_circleConveyor.maxAngle =  -45 * DEGTORAD;
        m_circleConveyor.surfaceVelocity = 0;
        circleFixture->SetUserData(&m_circleConveyor);

        //set moving surface segment in box body
        m_polygonConveyor.minAngle = -135 * DEGTORAD;
        m_polygonConveyor.maxAngle =  -45 * DEGTORAD;
        m_polygonConveyor.surfaceVelocity = 0;
        polygonFixture->SetUserData(&m_polygonConveyor);

        //if the bodies sleep this whole plan will be foiled!
        m_circleBody->SetSleepingAllowed(false);
        boxBody->SetSleepingAllowed(false);

        //keep circle body as fixed rotation, as you might for a platform game character
        m_circleBody->SetFixedRotation(true);

        //some conveyor belts
        {
            int numPoints = 30;
            b2Vec2 chainVertices[numPoints];
            for (int i = 0; i < numPoints; i++) {
                float x = -19 + (30/(float)numPoints) * i;
                float y = 30 + 0.5f * sinf( 30 * x / (float)numPoints );
                chainVertices[i].Set(x,y);
            }
            b2ChainShape chainShape;
            chainShape.CreateChain(chainVertices, numPoints);
            b2Fixture* chainFixture = staticBody->CreateFixture(&chainShape, 0.0f);
            chainFixture->SetFriction(0.9f);

            m_chainConveyor1.minAngle = -180 * DEGTORAD;
            m_chainConveyor1.maxAngle =  180 * DEGTORAD;
            m_chainConveyor1.surfaceVelocity = 6;//a little too fast for the character bodies to walk along :)
            chainFixture->SetUserData(&m_chainConveyor1);

            for (int i = 0; i < numPoints; i++)
                chainVertices[i] += b2Vec2(10,-10);
            b2ChainShape chainShape2;
            chainShape2.CreateChain(chainVertices, numPoints);
            chainFixture = staticBody->CreateFixture(&chainShape2, 0.0f);
            chainFixture->SetFriction(0.9f);

            m_chainConveyor2.minAngle = -180 * DEGTORAD;
            m_chainConveyor2.maxAngle =  180 * DEGTORAD;
            m_chainConveyor2.surfaceVelocity = -4;//juuust slow enough for the character bodies to walk along :)
            chainFixture->SetUserData(&m_chainConveyor2);

            //some little boxes to liven things up
            {
                b2BodyDef rubbleBodyDef;
                rubbleBodyDef.type = b2_dynamicBody;
                b2PolygonShape rubbleShape;
                rubbleShape.SetAsBox(0.4f,0.5f);
                b2FixtureDef rubbleFixtureDef;
                rubbleFixtureDef.friction = 0.9f;
                rubbleFixtureDef.density = 3;
                rubbleFixtureDef.shape = &rubbleShape;
                for (int i = 0; i < 25; i++) {
                    rubbleBodyDef.position.Set( fmodf(i,5), i/6.0f );
                    rubbleBodyDef.position += b2Vec2(-18.5f, 32.5);
                    m_world->CreateBody(&rubbleBodyDef)->CreateFixture(&rubbleFixtureDef);
                }
            }
        }
    }

    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
    {
        b2Fixture* fixtureA = contact->GetFixtureA();
        b2Fixture* fixtureB = contact->GetFixtureB();

        //check each fixture to see if surface velocity should be applied
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);
        float surfaceVelocityModifier = 0;

        if ( ConveyorSegment* segment = (ConveyorSegment*)fixtureA->GetUserData() ) {
            b2Vec2 localNormal = fixtureA->GetBody()->GetLocalVector( worldManifold.normal);
            float angle = b2Atan2(localNormal.y, localNormal.x);
            if ( segment->minAngle < angle && angle < segment->maxAngle )
                surfaceVelocityModifier += segment->surfaceVelocity;
        }

        if ( ConveyorSegment* segment = (ConveyorSegment*)fixtureB->GetUserData() ) {
            b2Vec2 localNormal = fixtureB->GetBody()->GetLocalVector( -worldManifold.normal);
            float angle = b2Atan2(localNormal.y, localNormal.x);
            if ( segment->minAngle < angle && angle < segment->maxAngle )
                surfaceVelocityModifier += segment->surfaceVelocity;
        }

        contact->SetTangentSpeed( surfaceVelocityModifier );
    }

    void Keyboard(int key) override
    {
        switch (key) {
        case GLFW_KEY_Q: m_circleConveyor.surfaceVelocity = -5; break;
        case GLFW_KEY_E: m_circleConveyor.surfaceVelocity =  5; break;
        case GLFW_KEY_A: m_polygonConveyor.surfaceVelocity = -5; break;
        case GLFW_KEY_D: m_polygonConveyor.surfaceVelocity =  5; break;

        case GLFW_KEY_W: m_circleBody->ApplyLinearImpulse( b2Vec2( 0, 150 ), m_circleBody->GetWorldCenter() ,true); break;
        default: Test::Keyboard(key);
        }
    }

    void KeyboardUp(int key) override
    {
        switch (key) {
        case GLFW_KEY_Q: case GLFW_KEY_E: m_circleConveyor.surfaceVelocity = 0; break;
        case GLFW_KEY_A: case GLFW_KEY_D: m_polygonConveyor.surfaceVelocity = 0; break;
        default: Test::Keyboard(key);
        }
    }

    void Step(Settings& settings)
    {
        g_debugDraw.DrawString(5, m_textLine, "Use q/w/e to control circle, a/d to control polygon");
        m_textLine += 15;

        Test::Step(settings);
    }

    static Test* Create()
    {
        return new iforce2d_Conveyors;
    }

    ConveyorSegment m_circleConveyor;
    ConveyorSegment m_polygonConveyor;
    ConveyorSegment m_chainConveyor1, m_chainConveyor2;
    b2Body* m_circleBody;
};

static int testIndex = RegisterTest("iforce2d", "Conveyors", iforce2d_Conveyors::Create);

#endif