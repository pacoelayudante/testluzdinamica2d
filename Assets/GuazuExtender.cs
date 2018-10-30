using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class GuazuExtender {

    public static bool LineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, ref Vector2 result)
    {
        float bx = p2.x - p1.x;
        float by = p2.y - p1.y;
        float dx = p4.x - p3.x;
        float dy = p4.y - p3.y;
        float bDotDPerp = bx * dy - by * dx;
        if (bDotDPerp == 0)
        {
            return false;
        }
        float cx = p3.x - p1.x;
        float cy = p3.y - p1.y;
        float t = (cx * dy - cy * dx) / bDotDPerp;

        //result = new Vector2(p1.x + t * bx, p1.y + t * by);
        result.Set(p1.x + t * bx, p1.y + t * by);
        return true;
    }

    public static bool PuntoEnTriangulo(Vector2 triA, Vector2 triB, Vector2 triC, Vector2 pt)
    {
        // Compute vectors        
        var v0 = triC - triA;
        var v1 = triB - triA;
        var v2 = pt - triA;

        // Compute dot products
        var dot00 = Vector2.Dot(v0, v0);
        var dot01 = Vector2.Dot(v0, v1);
        var dot02 = Vector2.Dot(v0, v2);
        var dot11 = Vector2.Dot(v1, v1);
        var dot12 = Vector2.Dot(v1, v2);

        // Compute barycentric coordinates
        var invDenom = 1f / (dot00 * dot11 - dot01 * dot01);
        var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        // Check if point is in triangle
        return (u >= 0) && (v >= 0) && (u + v < 1);
    }

    public static float QueLadoDeLaLinea(Vector2 lineaA, Vector2 lineaB, Vector2 punto)
    {
        return (lineaB.x - lineaA.x) * (punto.y - lineaA.y) - (lineaB.y - lineaA.y) * (punto.x - lineaA.x);
    }
    public static bool PuntoEnConoInfinito(Vector2 centroCono, Vector2 conoContraReloj, Vector2 conoConReloj, Vector2 punto)
    {
        return QueLadoDeLaLinea(centroCono, conoConReloj, punto) < 0f && QueLadoDeLaLinea(centroCono, conoContraReloj, punto) > 0f;
    }
}
