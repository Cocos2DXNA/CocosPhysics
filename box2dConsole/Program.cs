using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;
using Box2D.Common;
using Box2D.Collision.Shapes;
using Box2D.Dynamics;
using Box2D.Dynamics.Joints;
using Box2D.Dynamics.Contacts;
using Box2D.Collision;

namespace box2dconsole
{

    class Program
    {
        static b2Body m_bomb;
        static b2MouseJoint m_mouseJoint;
        static b2Vec2 m_bombSpawnPoint = b2Vec2.Zero;
        static bool m_bombSpawning;
        static b2Vec2 m_mouseWorld = b2Vec2.Zero;
        static b2World _world;
        static b2Body m_body;
        // Define the ground box shape.
        static float width = 100f, height = 100f;
        static float simulat_time = 5f; // time in seconds to run the sim

        static void Main(string[] args)
        {
            SetupWorld(false);
            AddPair();
            // FallingBoxesTest();
            // ApplyForce();
            // Run(new Action(ApplyForceKeyboardInput));
            Run(null);
        }

        static void ApplyForceKeyboardInput()
        {
            int dr = ran.Next(100);
            if (dr < 50)
            {
                // w
                b2Vec2 f = m_body.GetWorldVector(new b2Vec2(0.0f, -200.0f));
                b2Vec2 p = m_body.GetWorldPoint(new b2Vec2(0.0f, 2.0f));
                Console.WriteLine("ApplyForce: w f={0},{1}, p={2},{3}", f.x,f.y, p.x,p.y);
                m_body.ApplyForce(f, p);
            }
            else if (dr > 80 && dr < 90)
            {
                Console.WriteLine("ApplyForce: a");
                // a
                m_body.ApplyTorque(50.0f);
            }
            else if (dr > 90)
            {
                Console.WriteLine("ApplyForce: d");
                // d
                m_body.ApplyTorque(-50.0f);
            }
        }

        #region Listeners and Callbacks

        struct ContactPoint
        {
            public b2Fixture fixtureA;
            public b2Fixture fixtureB;
            public b2Vec2 normal;
            public b2Vec2 position;
            public b2PointState state;
        };
        
        class Destructo : b2DestructionListener
        {
            /// Called when any joint is about to be destroyed due
            /// to the destruction of one of its attached bodies.
            public override void SayGoodbye(b2Joint joint)
            {
                if (m_mouseJoint == joint)
                {
                    m_mouseJoint = null;
                }
                else
                {
                    // JointDestroyed(joint);
                }
                Console.WriteLine("ByeBye Joint: {0}", joint);
            }

            /// Called when any fixture is about to be destroyed due
            /// to the destruction of its parent body.
            public override void SayGoodbye(b2Fixture fixture)
            {
                Console.WriteLine("ByeBye Fixture: {0}", fixture);
            }
        }

        class QueryCallback : b2QueryCallback
        {
            public QueryCallback(b2Vec2 point)
            {
                m_point = point;
                m_fixture = null;
            }

            public override bool ReportFixture(b2Fixture fixture)
            {
                b2Body body = fixture.Body;
                if (body.BodyType == b2BodyType.b2_dynamicBody)
                {
                    bool inside = fixture.TestPoint(m_point);
                    if (inside)
                    {
                        m_fixture = fixture;

                        // We are done, terminate the query.
                        return false;
                    }
                }

                // Continue the query.
                return true;
            }

            b2Vec2 m_point;
            b2Fixture m_fixture;
        }

        class Contacto : b2ContactListener
        {
            const int k_maxContactPoints = 2048;
            private int m_pointCount = 0;
            private ContactPoint[] m_points = new ContactPoint[k_maxContactPoints];

            public override void PreSolve(Box2D.Dynamics.Contacts.b2Contact contact, ref Box2D.Collision.b2Manifold oldManifold)
            {
                b2Manifold manifold = contact.GetManifold();

                if (manifold.pointCount == 0)
                {
                    return;
                }

                b2Fixture fixtureA = contact.GetFixtureA();
                b2Fixture fixtureB = contact.GetFixtureB();

                b2PointState[] state1 = new b2PointState[b2Settings.b2_maxManifoldPoints];
                b2PointState[] state2 = new b2PointState[b2Settings.b2_maxManifoldPoints];
                b2Collision.b2GetPointStates(state1, state2, ref oldManifold, ref manifold);

                b2WorldManifold worldManifold = new b2WorldManifold();
                contact.GetWorldManifold(ref worldManifold);

                for (int i = 0; i < manifold.pointCount &&  m_pointCount < k_maxContactPoints; ++i)
                {
                    ContactPoint cp = m_points[m_pointCount];
                    cp.fixtureA = fixtureA;
                    cp.fixtureB = fixtureB;
                    cp.position = worldManifold.points[i];
                    cp.normal = worldManifold.normal;
                    cp.state = state2[i];
                    m_points[m_pointCount] = cp;
                    ++m_pointCount;
                }
            }

            public override void PostSolve(Box2D.Dynamics.Contacts.b2Contact contact, ref b2ContactImpulse impulse)
            {
            }
        }

        static Random ran = new Random();
        static float RandomFloat(float min, float max)
        {
            int m = ran.Next(100);
            float d = (float)m / 100.0f;
            return (min + (max - min) * d);
        }
        #endregion

        static void Run(Action iterCallback)
        {
            int iter = 0;
            long span = 0L;
            float step = (float)(TimeSpan.FromTicks(333333).TotalMilliseconds) / 1000f;
            Console.WriteLine("Cycle step = {0:F3} which is {1} fps", step, (int)(1f / step));
            int interval = 0;
#if PROFILING
            b2Profile m_maxProfile = new b2Profile();
            b2Profile m_totalProfile = new b2Profile();
            b2Profile aveProfile = new b2Profile();
#endif
            for (float dt = 0f; dt < simulat_time; )
            {
                long dtStart = DateTime.Now.Ticks;
                Update(_world, step);
                long duration = DateTime.Now.Ticks - dtStart;
                span += duration;
                dt += step;
                iter++;
                bool bdump = false;
                if (iterCallback != null)
                {
                    iterCallback();
                }
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
                    for (b2Body b = _world.BodyList; b != null; b = b.Next)
                    {
                        Console.WriteLine("Body: p={0:F3},{1:F3} v={2:F3},{3:F3}, w={4:F3}", b.Position.x, b.Position.y, b.LinearVelocity.x, b.LinearVelocity.y, b.AngularVelocity);
                    }
                }
#if PROFILING
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
#endif
            }
#if PROFILING
            Dump(_world);
#endif
            Console.WriteLine("hit <enter> to exit");
            Console.ReadLine();
        }

        static void SetupWorld(bool setupGround)
        {
            var gravity = new b2Vec2(0.0f, -10.0f);
            _world = new b2World(gravity);
            _world.SetAllowSleeping(true);
            _world.SetContinuousPhysics(true);
            _world.SetSubStepping(true);
            _world.SetWarmStarting(true);
            _world.SetDestructionListener(new Destructo());
            _world.SetContactListener(new Contacto());
            if (!setupGround)
            {
                return;
            }
            // Call the body factory which allocates memory for the ground body
            // from a pool and creates the ground box shape (also from a pool).
            // The body is also added to the world.
            b2BodyDef def = b2BodyDef.Create();
            def.allowSleep = true;
            def.position = b2Vec2.Zero;
            def.type = b2BodyType.b2_staticBody;
            b2Body groundBody = _world.CreateBody(def);
            groundBody.SetActive(true);

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
        }

        static void ApplyForce()
        {
            _world.Gravity = b2Vec2.Zero;

            float k_restitution = 0.4f;

            b2Body ground;
            {
                b2BodyDef bd = b2BodyDef.Create();
                bd.position.Set(0.0f, 20.0f);
                ground = _world.CreateBody(bd);

                b2EdgeShape shape = new b2EdgeShape();

                b2FixtureDef sd = b2FixtureDef.Create();
                sd.shape = shape;
                sd.density = 0.0f;
                sd.restitution = k_restitution;

                // Left vertical
                shape.Set(new b2Vec2(-20.0f, -20.0f), new b2Vec2(-20.0f, 20.0f));
                ground.CreateFixture(sd);

                // Right vertical
                shape.Set(new b2Vec2(20.0f, -20.0f), new b2Vec2(20.0f, 20.0f));
                ground.CreateFixture(sd);

                // Top horizontal
                shape.Set(new b2Vec2(-20.0f, 20.0f), new b2Vec2(20.0f, 20.0f));
                ground.CreateFixture(sd);

                // Bottom horizontal
                shape.Set(new b2Vec2(-20.0f, -20.0f), new b2Vec2(20.0f, -20.0f));
                ground.CreateFixture(sd);
            }

            {
                b2Transform xf1 = b2Transform.Zero;
                xf1.q.Set(0.3524f * b2Settings.b2_pi);
                xf1.p = xf1.q.GetXAxis();

                b2Vec2[] vertices = new b2Vec2[3];
                vertices[0] = b2Math.b2Mul(xf1, new b2Vec2(-1.0f, 0.0f));
                vertices[1] = b2Math.b2Mul(xf1, new b2Vec2(1.0f, 0.0f));
                vertices[2] = b2Math.b2Mul(xf1, new b2Vec2(0.0f, 0.5f));

                b2PolygonShape poly1 = new b2PolygonShape();
                poly1.Set(vertices, 3);

                b2FixtureDef sd1 = b2FixtureDef.Create();
                sd1.shape = poly1;
                sd1.density = 4.0f;

                b2Transform xf2 = b2Transform.Zero;
                xf2.q.Set(-0.3524f * b2Settings.b2_pi);
                xf2.p = -xf2.q.GetXAxis();

                vertices[0] = b2Math.b2Mul(xf2, new b2Vec2(-1.0f, 0.0f));
                vertices[1] = b2Math.b2Mul(xf2, new b2Vec2(1.0f, 0.0f));
                vertices[2] = b2Math.b2Mul(xf2, new b2Vec2(0.0f, 0.5f));

                b2PolygonShape poly2 = new b2PolygonShape();
                poly2.Set(vertices, 3);

                b2FixtureDef sd2 = b2FixtureDef.Create();
                sd2.shape = poly2;
                sd2.density = 2.0f;

                b2BodyDef bd = b2BodyDef.Create();
                bd.type = b2BodyType.b2_dynamicBody;
                bd.angularDamping = 5.0f;
                bd.linearDamping = 0.1f;

                bd.position = new b2Vec2(0.0f, 2.0f);
                bd.angle = b2Settings.b2_pi;
                bd.allowSleep = false;
                m_body = _world.CreateBody(bd);
                m_body.CreateFixture(sd1);
                m_body.CreateFixture(sd2);
            }

            {
                b2PolygonShape shape = new b2PolygonShape();
                shape.SetAsBox(0.5f, 0.5f);

                b2FixtureDef fd = b2FixtureDef.Create();
                fd.shape = shape;
                fd.density = 1.0f;
                fd.friction = 0.3f;

                for (int i = 0; i < 10; ++i)
                {
                    b2BodyDef bd = b2BodyDef.Create();
                    bd.type = b2BodyType.b2_dynamicBody;

                    bd.position = new b2Vec2(0.0f, 5.0f + 1.54f * i);
                    b2Body body = _world.CreateBody(bd);

                    body.CreateFixture(fd);

                    float gravity = 10.0f;
                    float I = body.Inertia;
                    float mass = body.Mass;

                    // For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
                    float radius = b2Math.b2Sqrt(2.0f * I / mass);

                    b2FrictionJointDef jd = new b2FrictionJointDef();
                    jd.localAnchorA.SetZero();
                    jd.localAnchorB.SetZero();
                    jd.BodyA = ground;
                    jd.BodyB = body;
                    jd.CollideConnected = true;
                    jd.maxForce = mass * gravity;
                    jd.maxTorque = mass * radius * gravity;

                    _world.CreateJoint(jd);
                }
            }
        }

        static void AddPair()
        {
            _world.Gravity = new b2Vec2(0.0f, 0.0f);

            {
                b2CircleShape shape = new b2CircleShape();
                shape.Position.SetZero();
                shape.Radius = 0.1f;

                float minX = -6.0f;
                float maxX = 0.0f;
                float minY = 4.0f;
                float maxY = 6.0f;

                for (int i = 0; i < 400; ++i)
                {
                    b2BodyDef bd = b2BodyDef.Create();
                    bd.type = b2BodyType.b2_dynamicBody;
                    bd.position = new b2Vec2(RandomFloat(minX, maxX), RandomFloat(minY, maxY));
                    b2Body body = _world.CreateBody(bd);
                    body.CreateFixture(shape, 0.01f);
                }
            }
            {
                b2PolygonShape shape = new b2PolygonShape();
                shape.SetAsBox(1.5f, 1.5f);
                b2BodyDef bd = b2BodyDef.Create();
                bd.type = b2BodyType.b2_dynamicBody;
                bd.position.Set(-40.0f, 5.0f);
                bd.bullet = true;
                b2Body body = _world.CreateBody(bd);
                body.CreateFixture(shape, 1.0f);
                body.LinearVelocity = new b2Vec2(150.0f, 0.0f);
            }
        }

        static void FallingBoxesTest()
        {
            b2BodyDef def;
            b2FixtureDef fd;
            Console.WriteLine("Enter the number of bodies you want to run?");
            string s = Console.ReadLine();
            Random ran = new Random();
            float y = height;
            int count = 0, max = int.Parse(s);
            while (count < max)
            {
                float xStart = (count / 30 % 2 == 1) ? 8f : 0f;
                for (int i = 0; i < 30 &&  count < max; i++, count++)
                {
                    def = b2BodyDef.Create();
                    float x = xStart + (float)i / 30.0f * ((float)width - 30.0f * 2.0f - xStart * 2.0f) + (float)i * 2.0f;
                    x += -width / 2.0f;
                    def.position = new b2Vec2(x, y + (float)(count / 30) + (float)(i + 1));
                    /*                    def.position = new b2Vec2(
                                              xStart + (float)i / 30f * (width - 30*2 - xStart*2f) + (float)i * 2f
                                            , y + (float)(count / 30) + (float)(i+1));
                     */
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
                y -= 10f;
            }


        }

        public static void Update(b2World _world, float dt)
        {
            _world.Step(dt, 8, 3);

        }

        public static void Dump(b2World _world)
        {
#if PROFILING
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
#endif
        }
    }
}
