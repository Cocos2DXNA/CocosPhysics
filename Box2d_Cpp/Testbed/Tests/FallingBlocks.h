/*
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

#ifndef FALLINGBLOCKS_H
#define FALLINGBLOCKS_H

// This is a fun demo that shows off falling blocks
class FallingBlocks : public Test
{
public:
	FallingBlocks()
	{		
            // Define the ground box shape.
            float32 width = 50, height = 50;
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			bd.allowSleep = true;
			bd.position = b2Vec2(0.0,0.0);
			bd.type = b2_staticBody;
			ground = m_world->CreateBody(&bd);

            // Call the body factory which allocates memory for the ground body
            // from a pool and creates the ground box shape (also from a pool).
            // The body is also added to the world.

            // bottom
            b2EdgeShape groundBox;
            groundBox.Set(b2Vec2(-width/2,0), b2Vec2(width/2, 0));
            b2FixtureDef fd1;
            fd1.shape = &groundBox;
            ground->CreateFixture(&fd1);

            // top
            b2EdgeShape top;
            top.Set(b2Vec2(-width/2, height), b2Vec2(width/2, height));
            b2FixtureDef fd2;
            fd2.shape = &top;
            ground->CreateFixture(&fd2);

            // left
            b2EdgeShape left;
            left.Set(b2Vec2(-width/2, height), b2Vec2(-width/2,0));
            b2FixtureDef fd3;
            fd3.shape = &top;
            ground->CreateFixture(&fd3);


            // right
            b2EdgeShape right;
            right.Set(b2Vec2(width/2, height), b2Vec2(width/2,0));
            b2FixtureDef fd4;
            fd4.shape = &right;
            ground->CreateFixture(&fd4);
		}

		// Always make 150 bodies to fall

        for (int i = 0; i < 150; i++)
        {
            b2BodyDef def;
            def.position = b2Vec2(-width/2 + width * RandomFloat(0,1), height * RandomFloat(0,1));
            def.type = b2_dynamicBody;
            b2Body *body = m_world->CreateBody(&def);
            // Define another box shape for our dynamic body.
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(.5f, .5f); //These are mid points for our 1m box

            // Define the dynamic body fixture.
            b2FixtureDef fd;
            fd.shape = &dynamicBox;
            fd.density = 1.0f;
            fd.friction = 0.3f;
            body->CreateFixture(&fd);
        }
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'h':
			// Set the gravity to be heavier
			break;

		case 'l':
			// Set the gravity to be lighter
			break;
		}
	}

	void Step(Settings* settings)
	{/*
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += 15;
		m_debugDraw.DrawString(5, m_textLine, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_textLine += 15;

		settings->viewCenter.x = m_car->GetPosition().x;
		*/
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new FallingBlocks;
	}

};

#endif
