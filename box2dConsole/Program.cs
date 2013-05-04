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
            long span = 0L;
            float step = (float)(TimeSpan.FromTicks(333333).TotalMilliseconds)/1000f;
            Console.WriteLine("Cycle step = {0:F3} which is {1} fps", step, (int)(1f / step));
            int interval = 0;
            b2Profile m_maxProfile = new b2Profile();
            b2Profile m_totalProfile = new b2Profile();
            b2Profile aveProfile = new b2Profile();
            for (float dt = 0f; dt < 26f; )
            {
                long dtStart = DateTime.Now.Ticks;
                Update(_world, step);
                long duration = DateTime.Now.Ticks - dtStart;
                span += duration;
                dt += step;
                iter++;
                bool bdump = false;
                if (iter == 30)
                {
                    interval++;
                    bdump = true;
                    //Dump(_world);
                    TimeSpan ts = new TimeSpan(span);
                    float fs = (float)ts.TotalMilliseconds / (float)iter;
                    Console.WriteLine("{2}: iteration time is {0:F3} ms avg. and is {1:F3} cycles", fs, fs / step, interval);
                    iter = 0;
                    span = 0L;
                    int bodyCount = _world.BodyCount;
                    int contactCount = _world.ContactManager.ContactCount;
                    int jointCount = _world.JointCount;
                    Console.WriteLine("{3}:bodies/contacts/joints = {0}/{1}/{2}", bodyCount, contactCount, jointCount, interval);

                    int proxyCount = _world.GetProxyCount();
                    int treeheight = _world.GetTreeHeight();
                    int balance = _world.GetTreeBalance();
                    float quality = _world.GetTreeQuality();
                    Console.WriteLine("{4}:proxies/height/balance/quality = {0}/{1}/{2}/{3:F3}", proxyCount, height, balance, quality, interval);
                }

                b2Profile p = _world.Profile;
                // Track maximum profile times
                {
                    m_maxProfile.step = Math.Max(m_maxProfile.step, p.step);
                    m_maxProfile.collide = Math.Max(m_maxProfile.collide, p.collide);
                    m_maxProfile.solve = Math.Max(m_maxProfile.solve, p.solve);
                    m_maxProfile.solveInit = Math.Max(m_maxProfile.solveInit, p.solveInit);
                    m_maxProfile.solveVelocity = Math.Max(m_maxProfile.solveVelocity, p.solveVelocity);
                    m_maxProfile.solvePosition = Math.Max(m_maxProfile.solvePosition, p.solvePosition);
                    m_maxProfile.solveTOI = Math.Max(m_maxProfile.solveTOI, p.solveTOI);
                    m_maxProfile.broadphase = Math.Max(m_maxProfile.broadphase, p.broadphase);

                    m_totalProfile.step += p.step;
                    m_totalProfile.collide += p.collide;
                    m_totalProfile.solve += p.solve;
                    m_totalProfile.solveInit += p.solveInit;
                    m_totalProfile.solveVelocity += p.solveVelocity;
                    m_totalProfile.solvePosition += p.solvePosition;
                    m_totalProfile.solveTOI += p.solveTOI;
                    m_totalProfile.broadphase += p.broadphase;
                }
                if (interval > 0)
                {
                    float scale = 1.0f / (float)interval;
                    aveProfile.step = scale * m_totalProfile.step;
                    aveProfile.collide = scale * m_totalProfile.collide;
                    aveProfile.solve = scale * m_totalProfile.solve;
                    aveProfile.solveInit = scale * m_totalProfile.solveInit;
                    aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
                    aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
                    aveProfile.solveTOI = scale * m_totalProfile.solveTOI;
                    aveProfile.broadphase = scale * m_totalProfile.broadphase;
                }
                if (bdump)
                {
                    Console.WriteLine("{3}:step [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.step, aveProfile.step, m_maxProfile.step,interval);
                    Console.WriteLine("{3}:collide [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.collide, aveProfile.collide, m_maxProfile.collide, interval);
                    Console.WriteLine("{3}:solve [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.solve, aveProfile.solve, m_maxProfile.solve, interval);
                    Console.WriteLine("{3}:solve init [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit, interval);
                    Console.WriteLine("{3}:solve velocity [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity, interval);
                    Console.WriteLine("{3}:solve position [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition, interval);
                    Console.WriteLine("{3}:solveTOI [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI, interval);
                    Console.WriteLine("{3}:broad-phase [ave] (max) = {0:F2} [{1:F2}] ({2:F2})", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase, interval);
                }
            }

            Dump(_world);

            Console.WriteLine("hit <enter> to exit");
            Console.ReadLine();

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
