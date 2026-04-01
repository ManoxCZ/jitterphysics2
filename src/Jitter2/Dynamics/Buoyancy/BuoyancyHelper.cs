/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Buoyancy;

/// <summary>
/// Provides static methods for computing submerged volume and center of buoyancy for
/// rigid body shapes, and for applying the resulting buoyancy and drag forces.
/// </summary>
/// <remarks>
/// <para>
/// Volume is computed analytically by tessellating each shape into triangles (cached per shape
/// instance), transforming them to world space, and clipping against the water surface using the
/// divergence theorem. A reference point on the water plane is used so that the water-plane cap
/// triangles contribute zero volume, making explicit cap closure unnecessary.
/// </para>
/// <para>
/// The algorithm is adapted from the method described in
/// "Water interaction model for boats in video games" (Klan, 2009).
/// </para>
/// </remarks>
public static class BuoyancyHelper
{
    // Cache tessellated triangles per shape (local space, computed once per shape instance).
    private static readonly ConditionalWeakTable<RigidBodyShape, JTriangle[]> TessCache = new();

    /// <summary>
    /// Default number of icosahedron subdivisions used when tessellating a shape for buoyancy.
    /// Higher values give more accurate volume estimates at the cost of per-frame computation.
    /// </summary>
    public static int TessellationSubdivisions { get; set; } = 2;

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Applies buoyancy and drag forces from <paramref name="surface"/> to all shapes
    /// on <paramref name="body"/>. Call this once per body per substep from
    /// <see cref="BuoyancySystem"/>.
    /// </summary>
    /// <param name="body">The body to apply forces to. Must be Dynamic.</param>
    /// <param name="surface">The water surface providing fluid properties.</param>
    /// <param name="gravity">World gravity vector (e.g. (0, -9.81, 0)).</param>
    /// <param name="substepDt">Duration of the current substep in seconds.</param>
    public static void ApplyBuoyancy(RigidBody body, IWaterSurface surface,
        JVector gravity, Real substepDt)
    {
        if (body.MotionType != MotionType.Dynamic) return;

        ref var data = ref body.Data;
        JVector bodyPos = data.Position;
        JQuaternion orientation = data.Orientation;

        // Fast broadphase: skip if body center is far above the water surface.
        // We'll still process if the bounding box bottom might be submerged.
        Real waterAtCenter = surface.GetHeight(bodyPos.X, bodyPos.Z);

        // Body AABB minimum Y from WorldBoundingBox of its shapes.
        Real minY = Real.MaxValue;
        Real maxY = Real.MinValue;
        foreach (var shape in body.InternalShapes)
        {
            if (shape.WorldBoundingBox.Min.Y < minY) minY = shape.WorldBoundingBox.Min.Y;
            if (shape.WorldBoundingBox.Max.Y > maxY) maxY = shape.WorldBoundingBox.Max.Y;
        }

        // Nothing submerged.
        if (minY >= waterAtCenter) return;

        // Check if the water surface covers the body's position.
        if (!surface.Contains(bodyPos)) return;

        Real totalSubmergedVolume = (Real)0.0;
        JVector totalCenterOfBuoyancy = JVector.Zero;

        foreach (var shape in body.InternalShapes)
        {
            ComputeSubmergedVolume(shape, bodyPos, orientation, surface,
                out Real vol, out JVector cob);

            if (vol > (Real)0.0)
            {
                totalSubmergedVolume += vol;
                totalCenterOfBuoyancy += vol * cob;
            }
        }

        if (totalSubmergedVolume <= (Real)0.0) return;

        // Keep body awake while submerged.
        body.SetActivationState(true);

        JVector centerOfBuoyancy = totalCenterOfBuoyancy * ((Real)1.0 / totalSubmergedVolume);

        // Archimedes: F = ρ_fluid × |g| × V_submerged, directed opposite to gravity.
        JVector buoyancyForce = -gravity * (surface.FluidDensity * totalSubmergedVolume);
        body.AddForce(buoyancyForce, centerOfBuoyancy, wakeup: false);

        // Estimate submerged fraction for drag scaling.
        Real bodyVolume = EstimateBodyVolume(body);
        Real submergedFraction = bodyVolume > (Real)0.0
            ? MathR.Min(totalSubmergedVolume / bodyVolume, (Real)1.0)
            : (Real)1.0;

        // Linear drag opposing velocity at center of buoyancy.
        JVector velAtCoB = data.Velocity +
            JVector.Cross(data.AngularVelocity, centerOfBuoyancy - bodyPos);
        JVector linearDragForce = -velAtCoB *
            (surface.FluidDensity * surface.LinearDrag * totalSubmergedVolume);
        body.AddForce(linearDragForce, centerOfBuoyancy, wakeup: false);

        // Angular drag opposing angular velocity.
        JVector angularDragTorque = -data.AngularVelocity *
            (surface.FluidDensity * surface.AngularDrag * totalSubmergedVolume);
        body.Torque += angularDragTorque;
    }

    // -------------------------------------------------------------------------
    // Volume computation
    // -------------------------------------------------------------------------

    /// <summary>
    /// Computes the submerged volume and center of buoyancy for a single shape.
    /// </summary>
    public static void ComputeSubmergedVolume(RigidBodyShape shape,
        in JVector bodyPosition, in JQuaternion bodyOrientation,
        IWaterSurface surface,
        out Real submergedVolume, out JVector centerOfBuoyancy)
    {
        JTriangle[] triangles = GetOrBuildTessellation(shape);

        Real vol = (Real)0.0;
        JVector cob = JVector.Zero;

        // Reference point: on the water plane directly above the body.
        // Placing the reference on the water plane makes the closing cap
        // triangles degenerate (zero volume), so we can skip building them.
        Real wh = surface.GetHeight(bodyPosition.X, bodyPosition.Z);
        JVector reference = new JVector(bodyPosition.X, wh, bodyPosition.Z);

        foreach (ref readonly JTriangle tri in triangles.AsSpan())
        {
            // Transform triangle from body-local to world space.
            JVector v0 = bodyPosition + JVector.Transform(tri.V0, bodyOrientation);
            JVector v1 = bodyPosition + JVector.Transform(tri.V1, bodyOrientation);
            JVector v2 = bodyPosition + JVector.Transform(tri.V2, bodyOrientation);

            // Per-vertex depth below water surface (positive = submerged).
            Real d0 = wh - v0.Y;
            Real d1 = wh - v1.Y;
            Real d2 = wh - v2.Y;

            bool b0 = d0 > (Real)0.0;
            bool b1 = d1 > (Real)0.0;
            bool b2 = d2 > (Real)0.0;
            int count = (b0 ? 1 : 0) + (b1 ? 1 : 0) + (b2 ? 1 : 0);

            if (count == 0) continue;

            if (count == 3)
            {
                AccumulateTetrahedron(ref vol, ref cob, reference, v0, v1, v2);
            }
            else if (count == 1)
            {
                // One vertex submerged: clip to one triangle.
                JVector sub, ab1, ab2;
                Real ds, da1, da2;

                if      (b0) { sub = v0; ab1 = v1; ab2 = v2; ds = d0; da1 = d1; da2 = d2; }
                else if (b1) { sub = v1; ab1 = v0; ab2 = v2; ds = d1; da1 = d0; da2 = d2; }
                else         { sub = v2; ab1 = v0; ab2 = v1; ds = d2; da1 = d0; da2 = d1; }

                // Intersection points on water plane.
                JVector p1 = Lerp(sub, ab1, ds, da1);
                JVector p2 = Lerp(sub, ab2, ds, da2);

                AccumulateTetrahedron(ref vol, ref cob, reference, sub, p1, p2);
            }
            else // count == 2
            {
                // Two vertices submerged: clip to two triangles.
                JVector above, s1, s2;
                Real da, ds1, ds2;

                if      (!b0) { above = v0; s1 = v1; s2 = v2; da = d0; ds1 = d1; ds2 = d2; }
                else if (!b1) { above = v1; s1 = v0; s2 = v2; da = d1; ds1 = d0; ds2 = d2; }
                else          { above = v2; s1 = v0; s2 = v1; da = d2; ds1 = d0; ds2 = d1; }

                JVector p1 = Lerp(s1, above, ds1, da);
                JVector p2 = Lerp(s2, above, ds2, da);

                // Split quad (s1, s2, p2, p1) into two triangles preserving winding.
                AccumulateTetrahedron(ref vol, ref cob, reference, s1, s2, p2);
                AccumulateTetrahedron(ref vol, ref cob, reference, s1, p2, p1);
            }
        }

        // Take absolute value: sign depends on mesh winding vs. reference placement.
        submergedVolume = MathR.Abs(vol);
        centerOfBuoyancy = submergedVolume > (Real)1e-10f
            ? cob * ((Real)1.0 / vol)
            : bodyPosition;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /// <summary>
    /// Returns the interpolated point on edge (a → b) at the water plane.
    /// <paramref name="da"/> is depth of a (positive = below), <paramref name="db"/> is depth of b.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static JVector Lerp(in JVector a, in JVector b, Real da, Real db)
    {
        // da > 0, db < 0  →  t = da / (da - db)
        Real t = da / (da - db);
        return a + t * (b - a);
    }

    /// <summary>
    /// Accumulates the signed volume and weighted centroid of the tetrahedron
    /// formed by <paramref name="reference"/> and triangle (a, b, c).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AccumulateTetrahedron(ref Real vol, ref JVector cob,
        in JVector reference,
        in JVector a, in JVector b, in JVector c)
    {
        JVector ar = a - reference;
        JVector br = b - reference;
        JVector cr = c - reference;

        // Signed volume of tetrahedron from reference to triangle.
        Real dV = (Real)(1.0 / 6.0) * JVector.Dot(ar, JVector.Cross(br, cr));

        vol += dV;
        // Centroid of tetrahedron (reference + a + b + c) / 4, weighted by dV.
        cob += dV * ((reference + a + b + c) * (Real)0.25f);
    }

    // -------------------------------------------------------------------------
    // Tessellation cache
    // -------------------------------------------------------------------------

    private static JTriangle[] GetOrBuildTessellation(RigidBodyShape shape)
    {
        return TessCache.GetValue(shape, static s =>
        {
            var list = new List<JTriangle>();
            Collision.Shapes.ShapeHelper.Tessellate(s, list, TessellationSubdivisions);
            return list.ToArray();
        });
    }

    // -------------------------------------------------------------------------
    // Body volume estimate (for drag fraction)
    // -------------------------------------------------------------------------

    // Cache estimated total volume per shape to avoid recomputing.
    private static readonly ConditionalWeakTable<RigidBodyShape, StrongBox<Real>> VolumeCache = new();

    private static Real EstimateBodyVolume(RigidBody body)
    {
        Real total = (Real)0.0;
        foreach (var shape in body.InternalShapes)
        {
            var box = VolumeCache.GetValue(shape, static s =>
            {
                Collision.Shapes.ShapeHelper.CalculateMassInertia(s, out _, out _, out Real m);
                return new StrongBox<Real>(MathR.Abs(m));
            });
            total += box.Value;
        }
        return total;
    }
}
