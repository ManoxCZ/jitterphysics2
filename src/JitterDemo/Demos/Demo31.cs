/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Buoyancy;
using Jitter2.LinearMath;
using JitterDemo.Renderer;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo;

/// <summary>
/// Demonstrates the <see cref="Boat"/> class: a buoyancy-driven hull steered
/// with engine thrust and a rudder. The boat floats on a flat <see cref="WaterPlane"/>
/// and can be driven around the pool with the arrow keys.
/// </summary>
public class Demo31 : IDemo, IDrawUpdate, ICleanDemo
{
    public string Name => "Boat";

    public string Description =>
        "A buoyancy-driven boat on a flat water plane. " +
        "The hull uses AffectedByBuoyancy for realistic floating physics.";

    public string Controls => "Arrow Keys - Throttle and steer";

    private Boat boat = null!;
    private World world = null!;

    public void Build(Playground pg, World world)
    {
        this.world = world;
        pg.AddFloor();

        // Flat water surface at Y = 3.
        world.Buoyancy.Add(new WaterPlane(waterLevel: 3.0f));

        // Pool walls keep the boat and debris in view.
        const float poolHalf = 20.0f;
        const float wallHeight = 7.0f;
        const float wallThick = 0.5f;

        void AddWall(JVector position, JVector size)
        {
            var wall = world.CreateRigidBody();
            wall.AddShape(new BoxShape(size));
            wall.Position = position;
            wall.MotionType = MotionType.Static;
        }

        AddWall(new JVector(poolHalf + wallThick * 0.5f, wallHeight * 0.5f, 0),
                new JVector(wallThick, wallHeight, poolHalf * 2 + wallThick * 2));
        AddWall(new JVector(-poolHalf - wallThick * 0.5f, wallHeight * 0.5f, 0),
                new JVector(wallThick, wallHeight, poolHalf * 2 + wallThick * 2));
        AddWall(new JVector(0, wallHeight * 0.5f, poolHalf + wallThick * 0.5f),
                new JVector(poolHalf * 2, wallHeight, wallThick));
        AddWall(new JVector(0, wallHeight * 0.5f, -poolHalf - wallThick * 0.5f),
                new JVector(poolHalf * 2, wallHeight, wallThick));

        //// Floating buoys scattered around the pool for the player to navigate.
        //for (int i = -2; i <= 2; i++)
        //{
        //    var buoy = world.CreateRigidBody();
        //    buoy.AddShape(new SphereShape(0.5f));
        //    buoy.Position = new JVector(i * 5.0f, 5.0f, 8.0f);
        //    buoy.SetMassInertia(50.0f);   // light → floats well above waterline
        //    buoy.AffectedByBuoyancy = true;

        //    var crate = world.CreateRigidBody();
        //    crate.AddShape(new BoxShape(1.0f));
        //    crate.Position = new JVector(i * 5.0f + 2.0f, 5.0f, -8.0f);
        //    crate.SetMassInertia(5.0f);   // very light crate → floats high
        //    crate.AffectedByBuoyancy = true;
        //}

        // The player-controlled boat.
        boat = new Boat(world);
        boat.Body.Position = new JVector(0, 3.1f, -5.0f);
        boat.Body.DeactivationTime = TimeSpan.MaxValue;

        world.SubstepCount = 2;
    }

    public void DrawUpdate()
    {
        DebugRenderer dr = RenderWindow.Instance.DebugRenderer;

        foreach (var surface in world.Buoyancy.WaterSurfaces)
        {
            WaterSurfaceDebugDraw.Draw(surface, dr);
        }

        var kb = Keyboard.Instance;

        float throttle = 0f;
        float steer = 0f;

        if (kb.IsKeyDown(Keyboard.Key.Up))
        {
            throttle = 1.0f;
        }
        else if (kb.IsKeyDown(Keyboard.Key.Down))
        {
            throttle = -0.5f;
        }

        if (kb.IsKeyDown(Keyboard.Key.Left))
        {
            steer = 1.0f;
        }
        else if (kb.IsKeyDown(Keyboard.Key.Right))
        {
            steer = -1.0f;
        }

        boat.SetInput(throttle, steer);
        boat.Step(1.0f / 100.0f);
    }

    public void CleanUp()
    {
        boat.Destroy();
    }
}
