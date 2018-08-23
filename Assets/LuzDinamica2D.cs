using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LuzDinamica2D : MonoBehaviour {
    class Circulo
    {
        public Vector2 pos;
        public float radio;
        public Vector2 OffsetRayo(Vector2 desde,float signo)
        {
            if (radio == 0f) return Vector2.zero;
            float d = (pos - desde).magnitude;
            float h = Mathf.Sqrt(d * d - radio * radio) * radio / d;
            float d_ = Mathf.Sqrt( radio * radio - h * h);
            return Vector2.Perpendicular(desde - pos).normalized * h * signo + (desde - pos).normalized*d_;
        }
    }
    class PuntoClaveLuz
    {
        float angulo;
        float distancia;
        float distancia2;
        Vector2 punto;
        Vector2 rayoDesdeOrigen;
        PuntoClaveLuz mellizo;
        public PuntoClaveLuz(Circulo referencia, Vector2 desde)
        {
            punto = referencia.pos;
            rayoDesdeOrigen = punto - desde;
            distancia = rayoDesdeOrigen.magnitude;
            distancia2 = distancia * distancia;
            if (referencia.radio == 0f)
            {
                angulo = Mathf.Atan2(rayoDesdeOrigen.y, rayoDesdeOrigen.x);
            }
            else
            {
                float r2 = referencia.radio * referencia.radio;
                float h = Mathf.Sqrt(distancia2 - r2) * referencia.radio / distancia;
                float d_ = Mathf.Sqrt(r2 - h * h);

                Vector2 offsetPerp = Vector2.Perpendicular(rayoDesdeOrigen).normalized * h;
                Vector2 offsetParal = (rayoDesdeOrigen).normalized * d_;

                mellizo = new PuntoClaveLuz(this);

                mellizo.punto = punto - offsetPerp + offsetParal;
                mellizo.rayoDesdeOrigen = rayoDesdeOrigen - offsetPerp + offsetParal;
                mellizo.angulo = Mathf.Atan2(mellizo.rayoDesdeOrigen.y, mellizo.rayoDesdeOrigen.x);

                punto += offsetPerp + offsetParal;
                rayoDesdeOrigen += offsetPerp + offsetParal;
                angulo = Mathf.Atan2(rayoDesdeOrigen.y, rayoDesdeOrigen.x);
                
                mellizo.distancia = distancia = rayoDesdeOrigen.magnitude;
                mellizo.distancia2 = distancia2 = distancia * distancia;
            }
        }
        public PuntoClaveLuz(PuntoClaveLuz mellizo)
        {
            this.mellizo = mellizo;
        }
    }

    public float radio,grosorRayo=.001f;
    List<Vector2> destinosDeRayos = new List<Vector2>();

    static void GenerarCirculos(List<Circulo> vertices)
    {
        GenerarCirculosBox( vertices);
        GenerarCirculosCircle( vertices);
        GenerarCirculosCapsule( vertices);
        GenerarCirculosEdge( vertices);
        GenerarCirculosPolygon( vertices);
        GenerarCirculosComposite( vertices);
    }
    static void GenerarCirculosBox(List<Circulo> vertices)
    {
        foreach (var coll in FindObjectsOfType<BoxCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius });
        }
    }
    static void GenerarCirculosCircle(List<Circulo> vertices)
    {
        foreach (var coll in FindObjectsOfType<CircleCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset), radio = coll.transform.TransformVector(Vector3.right).magnitude * coll.radius });
        }
    }
    static void GenerarCirculosCapsule(List<Circulo> vertices)
    {
        foreach (var coll in FindObjectsOfType<CapsuleCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            Vector2 tamDeforme = coll.transform.TransformVector(coll.size);
            if ((coll.direction == CapsuleDirection2D.Horizontal && tamDeforme.x <= tamDeforme.y)
                || (coll.direction == CapsuleDirection2D.Vertical && tamDeforme.y <= tamDeforme.x))
            {
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset), radio = Mathf.Max(tamDeforme.x , tamDeforme.y) });
            }
            else
            {
                float radio = Mathf.Min(tamDeforme.x, tamDeforme.y);
                Vector3 offset = Vector3.zero;
                if (coll.direction == CapsuleDirection2D.Horizontal) offset = coll.transform.right * (tamDeforme.x - radio) * .5f;
                else offset = coll.transform.up * (tamDeforme.y - radio) * .5f;
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset) + offset, radio = radio });
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset) - offset, radio = radio });
            }
        }
    }
    static void GenerarCirculosEdge(List<Circulo> vertices)
    {
        foreach (var coll in FindObjectsOfType<EdgeCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            foreach (var p in coll.points)
            {
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(p + coll.offset), radio = coll.edgeRadius });
            }
        }
    }
    static void GenerarCirculosPolygon(List<Circulo> vertices)
    {
        foreach (var coll in FindObjectsOfType<PolygonCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            foreach (var p in coll.points)
            {
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(p + coll.offset), radio = 0f });
            }
        }
    }
    static void GenerarCirculosComposite (List<Circulo> vertices)
    {
        foreach (var coll in FindObjectsOfType<CompositeCollider2D>())
        {
            if (!coll.enabled) continue;
            float radio = coll.geometryType == CompositeCollider2D.GeometryType.Outlines ? coll.edgeRadius : 0f;
            for (int i = 0; i < coll.pathCount; i++)
            {
                Vector2[] path = new Vector2[coll.GetPathPointCount(i)];
                coll.GetPath(i, path);
                foreach (var p in path)
                {
                    vertices.Add(new Circulo() { pos = coll.transform.TransformDirection(p) +coll.transform.position, radio = radio });
                }
            }
        }
    }



#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        List<Circulo> circs = new List<Circulo>();
        GenerarCirculos(circs);
        destinosDeRayos.Clear();
        UnityEditor.Handles.color = Gizmos.color = Color.gray;
        UnityEditor.Handles.DrawWireDisc(transform.position, Vector3.forward, radio);
        UnityEditor.Handles.color = Gizmos.color = Color.magenta;
        float radio2 = radio * radio;
        foreach (var c in circs)
        {

            if (c.radio == 0f)
            {
                Gizmos.DrawLine(c.pos - Vector2.up * .25f, c.pos + Vector2.up * .25f);
                Gizmos.DrawLine(c.pos - Vector2.right * .25f, c.pos + Vector2.right * .25f);
                if ((c.pos - (Vector2)transform.position).sqrMagnitude <= radio2) destinosDeRayos.Add(c.pos);
            }
            else
            {
                UnityEditor.Handles.DrawWireDisc(c.pos, Vector3.forward, c.radio);
                if ((c.pos - (Vector2)transform.position).sqrMagnitude <= radio2)
                {
                    destinosDeRayos.Add(c.pos + c.OffsetRayo(transform.position, 1f));
                    destinosDeRayos.Add(c.pos + c.OffsetRayo(transform.position, -1f));
                }
            }
        }
        if (destinosDeRayos.Count > 0)
        {
            destinosDeRayos.Sort((Vector2 a, Vector2 b) => { return Mathf.Atan2(a.y - transform.position.y, a.x - transform.position.x).CompareTo(Mathf.Atan2(b.y - transform.position.y, b.x - transform.position.x)); });

            destinosDeRayos.Add(destinosDeRayos[0]);
            Vector2 previo = destinosDeRayos[0];
            Vector2 previoHit = previo;

            RaycastHit2D[] hits = new RaycastHit2D[3];
            int cantHits = 0;

            foreach (var r in destinosDeRayos)
            {
                UnityEditor.Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.green, .2f);
                Gizmos.DrawLine(r, previo);
                Gizmos.DrawRay( transform.position, r - (Vector2)transform.position);
                previo = r;
                if(grosorRayo>0f)cantHits = Physics2D.CircleCast(transform.position, grosorRayo, r - (Vector2)transform.position, new ContactFilter2D(), hits, /*Vector2.Distance(r, transform.position)*/radio);
                else cantHits = Physics2D.Raycast(transform.position, r - (Vector2)transform.position, new ContactFilter2D(), hits, /*Vector2.Distance(r, transform.position)*/radio);
                UnityEditor.Handles.color = Gizmos.color = Color.green;
                if (cantHits > 0)
                {
                    Gizmos.DrawLine(previoHit, hits[0].point);
                    previoHit = hits[0].point;
                    UnityEditor.Handles.color = Gizmos.color = Color.yellow;
                    Gizmos.DrawRay(hits[0].point, hits[0].normal*.5f);
                    UnityEditor.Handles.color = Gizmos.color = Color.blue;
                    if (cantHits > 1)
                    {
                        Gizmos.DrawLine(previoHit, hits[1].point);
                        UnityEditor.Handles.Label(hits[1].point,"\n"+hits[0].distance+"_"+hits[1].distance);

                        if (hits[0].distance > hits[1].distance)
                        {
                            UnityEditor.Handles.color = Gizmos.color = Color.blue;
                            UnityEditor.Handles.DrawWireDisc(hits[0].point, Vector3.forward, .5f);
                        }

                    }
                    else
                    {
                        Gizmos.DrawLine(previoHit, (Vector2)transform.position + (r - (Vector2)transform.position).normalized * radio);
                        UnityEditor.Handles.Label(r, "\n" + hits[0].distance + "_" + radio);
                    }
                }
                else
                {
                    //Gizmos.DrawLine(previoHit, (Vector2)transform.position+(r - (Vector2)transform.position).normalized*radio );
                    //previoHit = (Vector2)transform.position + (r - (Vector2)transform.position).normalized * radio;
                    Gizmos.DrawLine(previoHit, r);
                    previoHit = r;
                    UnityEditor.Handles.color = Gizmos.color = Color.cyan;
                    Gizmos.DrawLine(previoHit, (Vector2)transform.position + (r - (Vector2)transform.position).normalized * radio);
                }
            }
        }
    }
#endif
}
