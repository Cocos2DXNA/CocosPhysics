using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;
using Box2D.Common;
using Box2D.Collision.Shapes;
using Box2D.Dynamics;

namespace box2dTest
{
    class Program
    {
        static void Main(string[] args)
        {
            var gravity = new b2Vec2(0.0f, -10.0f);
            b2World _world = new b2World(gravity);
            _world.SetAllowSleeping(true);
            _world.SetContinuousPhysics(true);

            // Call the body factory which allocates memory for the ground body
            // from a pool and creates the ground box shape (also from a pool).
            // The body is also added to the world.
            b2BodyDef def = b2BodyDef.Create();
            def.allowSleep = true;
            def.position = b2Vec2.Zero;
            def.type = b2BodyType.b2_staticBody;
            b2Body groundBody = _world.CreateBody(def);
            groundBody.SetActive(true);

            // Define the ground box shape.
            float width = 100f, height = 100f;
            // bottom
            b2EdgeShape groundBox = new b2EdgeShape();
            groundBox.Set(b2Vec2.Zero, new b2Vec2(width, 0));
            b2FixtureDef fd = b2FixtureDef.Create();
            fd.shape = groundBox;
            groundBody.CreateFixture(fd);

            // top
            groundBox = new b2EdgeShape();
            groundBox.Set(new b2Vec2(0, height), new b2Vec2(width, height));
            fd.shape = groundBox;
            groundBody.CreateFixture(fd);

            // left
            groundBox = new b2EdgeShape();
            groundBox.Set(new b2Vec2(0, height), b2Vec2.Zero);
            fd.shape = groundBox;
            groundBody.CreateFixture(fd);

            // right
            groundBox = new b2EdgeShape();
            groundBox.Set(new b2Vec2(width, height), new b2Vec2(width, 0));
            fd.shape = groundBox;
            groundBody.CreateFixture(fd);

            _world.Dump();

            Console.WriteLine("Enter the number of bodies you want to run?");
            string s = Console.ReadLine();
            Random ran = new Random();
            for (int i = 0; i < int.Parse(s); i++)
            {
                def = b2BodyDef.Create();
                def.position = new b2Vec2(width * (float)ran.NextDouble(), height * (float)ran.NextDouble());
                def.type = b2BodyType.b2_dynamicBody;
                b2Body body = _world.CreateBody(def);
                // Define another box shape for our dynamic body.
                var dynamicBox = new b2PolygonShape();
                dynamicBox.SetAsBox(.5f, .5f); //These are mid points for our 1m box

                // Define the dynamic body fixture.
                fd = b2FixtureDef.Create();
                fd.shape = dynamicBox;
                fd.density = 1f;
                fd.friction = 0.3f;
                b2Fixture fixture = body.CreateFixture(fd);
            }

            int iter = 0;
            for (float dt = 0f; dt < 26f; )
            {
                Update(_world, 1f / 30f);
                dt += 1f / 30f;
                iter++;
                if (iter == 30)
                {
                    Dump(_world);
                    iter = 0;
                }
            }

            Console.WriteLine("hit <enter> to exit");
            Console.ReadLine();

            Dump(_world);
        }

        public static void Update(b2World _world, float dt)
        {
            _world.Step(dt, 8, 1);

        }

        public static void Dump(b2World _world)
        {
            _world.Dump();
            b2Profile profile = _world.Profile;
            Console.WriteLine("]-----------[{0:F4}]-----------------------[", profile.step);
            Console.WriteLine("Solve Time = {0:F4}", profile.solve);
            Console.WriteLine("# bodies = {0}", profile.bodyCount);
            Console.WriteLine("# contacts = {0}", profile.contactCount);
            Console.WriteLine("# joints = {0}", profile.jointCount);
            Console.WriteLine("# toi iters = {0}", profile.toiSolverIterations);
            if (profile.step > 0f)
            {
                Console.WriteLine("Solve TOI Time = {0:F4} {1:F2}%", profile.solveTOI, profile.solveTOI / profile.step * 100f);
                Console.WriteLine("Solve TOI Advance Time = {0:F4} {1:F2}%", profile.solveTOIAdvance, profile.solveTOIAdvance / profile.step * 100f);
            }

            Console.WriteLine("BroadPhase Time = {0:F4}", profile.broadphase);
            Console.WriteLine("Collision Time = {0:F4}", profile.collide);
            Console.WriteLine("Solve Velocity Time = {0:F4}", profile.solveVelocity);
            Console.WriteLine("Solve Position Time = {0:F4}", profile.solvePosition);
            Console.WriteLine("Step Time = {0:F4}", profile.step);
        }
    }
}
