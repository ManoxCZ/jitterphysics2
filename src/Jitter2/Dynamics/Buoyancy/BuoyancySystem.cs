/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;

namespace Jitter2.Dynamics.Buoyancy;

/// <summary>
/// Manages a collection of <see cref="IWaterSurface"/> objects and applies buoyancy and
/// drag forces to all dynamic rigid bodies that have
/// <see cref="RigidBody.AffectedByBuoyancy"/> set to <see langword="true"/>.
/// </summary>
/// <remarks>
/// <para>
/// An instance of this class is automatically created and owned by <see cref="World"/>.
/// Access it via <see cref="World.Buoyancy"/>. Forces are applied via the
/// <see cref="World.PreSubStep"/> callback so they are correctly integrated in every
/// substep when <see cref="World.SubstepCount"/> is greater than one.
/// </para>
/// <example>
/// <code>
/// // Register a flat water surface at Y = 0.
/// world.Buoyancy.Add(new WaterPlane(waterLevel: 0));
///
/// // Make a body float.
/// body.AffectedByBuoyancy = true;
/// </code>
/// </example>
/// </remarks>
public sealed class BuoyancySystem
{
    private readonly World _world;

    /// <summary>
    /// The list of water surfaces that contribute buoyancy forces.
    /// Add or remove <see cref="IWaterSurface"/> instances at any time.
    /// </summary>
    public List<IWaterSurface> WaterSurfaces { get; } = new();

    /// <summary>
    /// Creates a new <see cref="BuoyancySystem"/> and registers it with the world's
    /// <see cref="World.PreSubStep"/> event.
    /// </summary>
    internal BuoyancySystem(World world)
    {
        _world = world;
        world.PreSubStep += Apply;
    }

    /// <summary>
    /// Adds a water surface to this system.
    /// </summary>
    public void Add(IWaterSurface surface) => WaterSurfaces.Add(surface);

    /// <summary>
    /// Removes a water surface from this system.
    /// </summary>
    public bool Remove(IWaterSurface surface) => WaterSurfaces.Remove(surface);

    private void Apply(Real substepDt)
    {
        if (WaterSurfaces.Count == 0) return;

        foreach (var body in _world.RigidBodies)
        {
            if (!body.AffectedByBuoyancy) continue;
            if (body.MotionType != MotionType.Dynamic) continue;
            if (!body.IsActive) continue;

            foreach (var surface in WaterSurfaces)
            {
                BuoyancyHelper.ApplyBuoyancy(body, surface, _world.Gravity, substepDt);
            }
        }
    }
}
