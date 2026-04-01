/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace JitterDemo;

/// <summary>
/// A simple boat driven by engine thrust and rudder steering, floating on
/// water via the buoyancy system. Modelled after the <see cref="RayCastCar"/>
/// pattern: create an instance, call <see cref="SetInput"/> every frame, then
/// call <see cref="Step"/> with the physics timestep.
/// </summary>
/// <remarks>
/// Stabilising forces (heave damping, roll/pitch damping) are applied via
/// <see cref="World.PreSubStep"/> so they are recomputed from current velocity
/// before every substep — exactly how <c>BuoyancySystem</c> works. Without
/// this, forces set once per render frame are stale for subsequent substeps
/// and the hull oscillates without settling.
/// Call <see cref="Destroy"/> when the boat is no longer needed to unregister
/// the callback.
/// </remarks>
public class Boat
{
    private readonly World world;

    /// <summary>Gets the rigid body that represents the boat hull.</summary>
    public RigidBody Body { get; }

    /// <summary>Maximum engine thrust force in Newtons.</summary>
    public float ThrustForce { get; set; } = 20000.0f;

    /// <summary>Maximum rudder side-force in Newtons.</summary>
    public float RudderForce { get; set; } = 10000.0f;

    /// <summary>Rate at which the throttle ramps toward the desired value (units per second).</summary>
    public float AccelerationRate { get; set; } = 3.0f;

    /// <summary>Rate at which steering ramps toward the desired value (units per second).</summary>
    public float SteerRate { get; set; } = 5.0f;

    /// <summary>
    /// Damping coefficient for vertical (heave) velocity in N/(m/s).
    /// Applied every substep to resist bobbing without killing horizontal motion.
    /// </summary>
    public float HeaveDamping { get; set; } = 40000.0f;

    /// <summary>
    /// Damping coefficient for roll and pitch (X/Z angular velocity) in N·m/(rad/s).
    /// Yaw (Y) is deliberately left free so steering remains responsive.
    /// </summary>
    public float RollPitchDamping { get; set; } = 60000.0f;

    // Stern offset in body-local space (+Z = back of boat).
    private static readonly JVector SternLocal = new(0f, 0f, 3.0f);

    private float destThrottle;
    private float destSteering;
    private float throttle;
    private float steering;

    /// <summary>
    /// Creates a new <see cref="Boat"/> in <paramref name="world"/>.
    /// Move <see cref="Body"/>.Position after construction.
    /// </summary>
    public Boat(World world)
    {
        this.world = world;

        Body = world.CreateRigidBody();

        // Hull: wide flat box (3 m wide × 0.6 m tall × 6 m long).
        // Hull volume ≈ 10.8 m³; at ρ=1000 kg/m³ and 4 500 kg mass it
        // floats with ~42 % submerged, leaving a healthy freeboard.
        var hull = new TransformedShape(new BoxShape(3.0f, 0.6f, 6.0f), JVector.Zero);

        // Wheelhouse sits on top toward the bow (−Z side).
        var cabin = new TransformedShape(new BoxShape(1.5f, 0.7f, 2.5f), new JVector(0f, 0.65f, -0.5f));

        Body.AddShape(hull);
        Body.AddShape(cabin);

        Body.SetMassInertia(4500.0f);
        Body.AffectedByBuoyancy = true;

        // Near-default Jitter damping — the buoyancy system and our PreSubStep
        // callback handle all meaningful drag; extra body damping would fight them.
        Body.Damping = (0.002f, 0.005f);

        world.PreSubStep += ApplyStabilizingForces;
    }

    /// <summary>
    /// Unregisters the internal <see cref="World.PreSubStep"/> callback.
    /// Call this when switching away from the demo (e.g. in <c>ICleanDemo.CleanUp</c>).
    /// </summary>
    public void Destroy()
    {
        world.PreSubStep -= ApplyStabilizingForces;
    }

    /// <summary>
    /// Sets the desired input. Both values are clamped to [−1, 1].
    /// </summary>
    /// <param name="throttle">+1 = full forward, −1 = full reverse.</param>
    /// <param name="steer">+1 = turn port (left), −1 = turn starboard (right).</param>
    public void SetInput(float throttle, float steer)
    {
        destThrottle = Math.Clamp(throttle, -1.0f, 1.0f);
        destSteering = Math.Clamp(steer,    -1.0f, 1.0f);
    }

    /// <summary>
    /// Applies engine thrust and rudder forces for one frame.
    /// Call this from <see cref="IDrawUpdate.DrawUpdate"/> after reading input.
    /// </summary>
    /// <param name="dt">Physics timestep in seconds (typically 1/100).</param>
    public void Step(float dt)
    {
        throttle += Math.Clamp(destThrottle - throttle, -dt * AccelerationRate, dt * AccelerationRate);
        steering += Math.Clamp(destSteering - steering, -dt * SteerRate,        dt * SteerRate);

        JVector forward = JVector.Transform(-JVector.UnitZ, Body.Orientation);
        JVector right   = JVector.Transform( JVector.UnitX, Body.Orientation);

        JVector stern = Body.Position + JVector.Transform(SternLocal, Body.Orientation);

        // Engine thrust from the stern pushes the bow forward.
        Body.AddForce(forward * (ThrustForce * throttle), stern);

        // Rudder: side force at stern yaws the hull.
        // A 0.3 floor makes it responsive from standstill.
        float forwardSpeed = JVector.Dot(Body.Velocity, forward);
        float rudderScale  = Math.Clamp(MathF.Abs(forwardSpeed) * 0.1f + 0.3f, 0f, 1f);
        Body.AddForce(right * (RudderForce * steering * rudderScale), stern);
    }

    // Called by World.PreSubStep before every substep with up-to-date velocities.
    private void ApplyStabilizingForces(float substepDt)
    {
        if (Body.MotionType != MotionType.Dynamic || !Body.IsActive) return;

        // Oppose vertical velocity to damp heave (bobbing).
        Body.AddForce(new JVector(0f, -Body.Velocity.Y * HeaveDamping, 0f), wakeup: false);

        // Oppose roll and pitch angular velocity (leave yaw / Y free for steering).
        JVector av = Body.AngularVelocity;
        Body.Torque += new JVector(-av.X * RollPitchDamping, 0f, -av.Z * RollPitchDamping);
    }
}
