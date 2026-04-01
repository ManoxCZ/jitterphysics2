/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Buoyancy;
using Jitter2.LinearMath;

namespace JitterDemo;

/// <summary>
/// Demonstrates buoyancy physics: dynamic bodies with <see cref="RigidBody.AffectedByBuoyancy"/>
/// float and sink correctly based on their mass relative to the displaced fluid.
/// </summary>
public class Demo31 : IDemo
{
    public string Name => "Buoyancy";

    public string Description =>
        "Bodies with AffectedByBuoyancy=true float in water. " +
        "Light bodies rise, heavy bodies sink. " +
        "Mass is varied so some float and some sink.";

    public void Build(Playground pg, World world)
    {
        pg.AddFloor();

        // ------------------------------------------------------------------
        // Register a flat water plane at Y = 3.
        // ------------------------------------------------------------------
        var waterPlane = new WaterPlane(waterLevel: 3.0f);
        world.Buoyancy.Add(waterPlane);

        // ------------------------------------------------------------------
        // Create a pool of static walls so bodies don't drift away.
        // ------------------------------------------------------------------
        const float poolHalf = 8.0f;
        const float wallHeight = 6.0f;
        const float wallThickness = 0.5f;

        void AddWall(JVector position, JVector size)
        {
            var wall = world.CreateRigidBody();
            wall.AddShape(new BoxShape(size));
            wall.Position = position;
            wall.MotionType = MotionType.Static;
        }

        // Four walls around the pool.
        AddWall(new JVector( poolHalf + wallThickness * 0.5f, wallHeight * 0.5f, 0),
                new JVector(wallThickness, wallHeight, poolHalf * 2 + wallThickness * 2));
        AddWall(new JVector(-poolHalf - wallThickness * 0.5f, wallHeight * 0.5f, 0),
                new JVector(wallThickness, wallHeight, poolHalf * 2 + wallThickness * 2));
        AddWall(new JVector(0, wallHeight * 0.5f,  poolHalf + wallThickness * 0.5f),
                new JVector(poolHalf * 2, wallHeight, wallThickness));
        AddWall(new JVector(0, wallHeight * 0.5f, -poolHalf - wallThickness * 0.5f),
                new JVector(poolHalf * 2, wallHeight, wallThickness));

        // ------------------------------------------------------------------
        // Drop various shapes with different masses into the water.
        //
        // Default water density: 1000 kg/m³.
        // A body floats when  mass < fluidDensity × bodyVolume.
        //
        // Shape volumes (approximate):
        //   BoxShape(1,1,1)  ≈ 1.0 m³   → floats at mass < 1000
        //   SphereShape(0.5) ≈ 0.52 m³  → floats at mass < 520
        //   CylinderShape(1,0.5) ≈ 0.79 m³ → floats at mass < 785
        // ------------------------------------------------------------------

        // Row of light boxes → all float.
        for (int i = -3; i <= 3; i++)
        {
            var body = world.CreateRigidBody();
            body.AddShape(new BoxShape(1.0f));
            body.Position = new JVector(i * 1.5f, 6.0f, -3.0f);
            body.SetMassInertia(5.0f);        // 5 kg in 1 m³ → well below 1000 → floats
            body.AffectedByBuoyancy = true;
        }

        // Row of heavy boxes → all sink.
        for (int i = -3; i <= 3; i++)
        {
            var body = world.CreateRigidBody();
            body.AddShape(new BoxShape(1.0f));
            body.Position = new JVector(i * 1.5f, 6.0f, 0.0f);
            body.SetMassInertia(1500.0f);     // 1500 kg in 1 m³ → above 1000 → sinks
            body.AffectedByBuoyancy = true;
        }

        // Mixed spheres: light ones float, heavy ones sink.
        for (int i = -3; i <= 3; i++)
        {
            float mass = i < 0 ? 3.0f : 800.0f; // left: float, right: sink
            var body = world.CreateRigidBody();
            body.AddShape(new SphereShape(0.5f));
            body.Position = new JVector(i * 1.5f, 6.5f, 3.0f);
            body.SetMassInertia(mass);
            body.AffectedByBuoyancy = true;
        }

        // A few cylinders for visual variety.
        for (int i = -2; i <= 2; i++)
        {
            var body = world.CreateRigidBody();
            body.AddShape(new CylinderShape(1.0f, 0.4f));
            body.Position = new JVector(i * 2.0f, 7.0f, 6.0f);
            body.SetMassInertia(10.0f);       // very light → floats high
            body.AffectedByBuoyancy = true;
        }

        world.SubstepCount = 2;
    }
}
