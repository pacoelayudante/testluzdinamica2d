using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LuzDinamica2D : MonoBehaviour {
    static ContactFilter2D filtroVacio= new ContactFilter2D();
    static int cantHits = 0;
    static RaycastHit2D[] hits = new RaycastHit2D[1];
    static Dictionary<Collider,List<Circulo>> colliders=new Dictionary<Collider, List<Circulo>> ();

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
    class VectorDeLuz:System.IComparable<VectorDeLuz>
    {
        public bool rayoInterrumpido;
       public float angulo;
        public float distancia;
        public float distancia2;
        public Vector2 punto,origen,puntoHit;
        public Vector2 rayoDesdeOrigen;
        public VectorDeLuz horizonteAlfa,horizonteBeta;

        int cantHits = 0;
        public RaycastHit2D[] hits = new RaycastHit2D[2];

        public VectorDeLuz(Circulo referencia, Vector2 desde, float radio=0f)
        {
            origen = desde;
            punto = referencia.pos;
            rayoDesdeOrigen = punto - origen;
            distancia = rayoDesdeOrigen.magnitude;
            distancia2 = distancia * distancia;
            angulo = Mathf.Atan2(rayoDesdeOrigen.y, rayoDesdeOrigen.x);
            /*if (referencia.radio == 0f)
            {
            }
            else*/
            if(referencia.radio>0f)
            {
                float r2 = referencia.radio * referencia.radio;
                float h = Mathf.Sqrt(distancia2 - r2) * referencia.radio / distancia;
                float d_ = Mathf.Sqrt(r2 - h * h);

                Vector2 offsetPerp = Vector2.Perpendicular(rayoDesdeOrigen).normalized * h;
                Vector2 offsetParal = (rayoDesdeOrigen).normalized * d_;

                horizonteAlfa = new VectorDeLuz(this);
                horizonteBeta = new VectorDeLuz(this);
                horizonteBeta.origen = horizonteAlfa.origen = origen;

                horizonteAlfa.punto = punto - offsetPerp - offsetParal;
                horizonteAlfa.rayoDesdeOrigen = rayoDesdeOrigen - offsetPerp - offsetParal;
                horizonteBeta.punto = punto + offsetPerp - offsetParal;
                horizonteBeta.rayoDesdeOrigen = rayoDesdeOrigen + offsetPerp - offsetParal;

                horizonteAlfa.angulo = Mathf.Atan2(horizonteAlfa.rayoDesdeOrigen.y, horizonteAlfa.rayoDesdeOrigen.x);
                horizonteBeta.angulo = Mathf.Atan2(horizonteBeta.rayoDesdeOrigen.y, horizonteBeta.rayoDesdeOrigen.x);

                horizonteAlfa.distancia = horizonteBeta.distancia = horizonteBeta.rayoDesdeOrigen.magnitude;
                horizonteAlfa.distancia2 = horizonteBeta.distancia2 = horizonteBeta.distancia * horizonteBeta.distancia;

                horizonteAlfa.CalcularRayo(radio);
                horizonteBeta.CalcularRayo(radio);
            }
            CalcularRayo(radio);
        }
        public VectorDeLuz(VectorDeLuz basico)
        {
        }

        public void CalcularRayo(float radio)
        {
            cantHits = Physics2D.Raycast(origen, rayoDesdeOrigen, filtroVacio, hits, radio);
            if (cantHits > 0)
            {
                if(hits[0].distance < distancia*.999f)
                {
                    rayoInterrumpido = true;
                }
                else
                {
                    if (hits[0].distance*.999f < distancia)
                    {//choco con vertice clave
                        if (cantHits > 1)
                        {//choco con algo mas ademas de vertice clave
                            /*hits[0].point = hits[1].point;
                            hits[0].distance = hits[1].distance;
                            hits[0].normal = hits[1].normal;*/
                            hits[0] = hits[1];
                        }
                        else
                        {//no choco con nada mas que vertice clave
                            hits[0].point = origen + rayoDesdeOrigen.normalized * radio;
                            hits[0].distance = radio;
                            hits[0].normal = -rayoDesdeOrigen.normalized;
                        }
                    }
                }
            }
            else
            {//No choco nada? inventale el vertice final
                hits[0].point = origen + rayoDesdeOrigen.normalized*radio;
                hits[0].distance = radio;
                hits[0].normal = rayoDesdeOrigen.normalized;
                cantHits = 1;
            }
            puntoHit = hits[0].point;
        }

        public void Gizmo()
        {
            if (cantHits > 0) Gizmos.DrawLine(puntoHit, origen);
            else
            {
                Gizmos.color = Color.black;
                Gizmos.DrawRay(origen, rayoDesdeOrigen);
            }
        }

        public int CompareTo(VectorDeLuz otro)
        {
            return angulo.CompareTo(otro.angulo);
        }
    }

    public float radio,grosorRayo=.001f;
    List<VectorDeLuz> destinosDeRayos = new List<VectorDeLuz>();

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
            if ((c.pos - (Vector2)transform.position).sqrMagnitude <= radio2)
            {
                destinosDeRayos.Add(new VectorDeLuz(c, transform.position,radio));
                if(c.radio>0f)//if (destinosDeRayos[destinosDeRayos.Count - 1].horizonteAlfa != null)
                {
                    destinosDeRayos.Add(destinosDeRayos[destinosDeRayos.Count - 1].horizonteAlfa);
                    destinosDeRayos.Add(destinosDeRayos[destinosDeRayos.Count - 2].horizonteBeta);
                }
            }

            if (c.radio == 0f)
            {
                Gizmos.DrawLine(c.pos - Vector2.up * .1f, c.pos + Vector2.up * .1f);
                Gizmos.DrawLine(c.pos - Vector2.right * .1f, c.pos + Vector2.right * .1f);
                //if ((c.pos - (Vector2)transform.position).sqrMagnitude <= radio2) destinosDeRayos.Add(c.pos);
            }
            else
            {
                UnityEditor.Handles.DrawWireDisc(c.pos, Vector3.forward, c.radio);
                /*if ((c.pos - (Vector2)transform.position).sqrMagnitude <= radio2)
                {
                    destinosDeRayos.Add(c.pos + c.OffsetRayo(transform.position, 1f));
                    destinosDeRayos.Add(c.pos + c.OffsetRayo(transform.position, -1f));
                }*/
            }
        }
        if (destinosDeRayos.Count > 0)
        {
            foreach(var rayo in destinosDeRayos)
            {
                Gizmos.color =Color.Lerp(Color.clear,rayo.rayoInterrumpido?Color.red: Color.blue,.3f);
                rayo.Gizmo();
            }

            destinosDeRayos.Sort();
            destinosDeRayos.Add(destinosDeRayos[0]);
            for (int i = 0; i < destinosDeRayos.Count - 1; i++)
            {
                VectorDeLuz a = destinosDeRayos[i];
                VectorDeLuz b = destinosDeRayos[i + 1];
                bool hover = PuntoEnTriangulo(a.punto, b.punto, transform.position, UnityEditor.HandleUtility.GUIPointToWorldRay(UnityEngine.Event.current.mousePosition).GetPoint(1f));
                //if (hover)
                {
                    if (hover)
                    {
                        Gizmos.color = a.rayoInterrumpido ? Color.red : Color.blue;
                        a.Gizmo();
                        Gizmos.color = b.rayoInterrumpido ? Color.red : Color.blue;
                        b.Gizmo();

                        /*    Gizmos.color = Color.yellow;
                        Gizmos.DrawRay(a.puntoHit,Vector2.Perpendicular( a.hits[0].normal) * 999f);
                        Gizmos.DrawRay(a.puntoHit, Vector2.Perpendicular(-a.hits[0].normal) * 999f);
                        Gizmos.DrawRay(b.puntoHit, Vector2.Perpendicular(b.hits[0].normal) * 999f);
                        Gizmos.DrawRay(b.puntoHit, Vector2.Perpendicular(-b.hits[0].normal) * 999f);*/
                    }
                    UnityEditor.Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.white, .7f);
                    if (a.rayoInterrumpido && b.rayoInterrumpido)
                    {
                          UnityEditor.Handles.DrawAAConvexPolygon(a.rayoInterrumpido? a.puntoHit:a.punto, b.rayoInterrumpido? b.puntoHit:b.punto, transform.position);
                    }
                    else
                    {
                        Vector2 medioCerca = Vector2.Lerp(a.rayoInterrumpido ? a.puntoHit : a.punto, b.rayoInterrumpido ? b.puntoHit : b.punto, .5f);
                        Vector2 medioLejos = Vector2.Lerp(!a.rayoInterrumpido ? a.puntoHit : a.punto, !b.rayoInterrumpido ? b.puntoHit : b.punto, .5f);
                        Vector2 rayoCercaLejos = medioLejos - medioCerca;
                        //Vector2 perp = Vector2.Perpendicular(a.punto- b.punto).normalized;

                        cantHits = Physics2D.Raycast(transform.position, Vector2.Lerp(a.rayoDesdeOrigen, b.rayoDesdeOrigen, .5f), filtroVacio, hits, radio);
                        //cantHits = Physics2D.Raycast(medio-perp*.1f, perp, filtroVacio, hits, .2f);
                        //cantHits = Physics2D.Raycast(medioCerca, rayoCercaLejos, filtroVacio, hits, rayoCercaLejos.magnitude);
                        if (cantHits > 0)
                        {
                            UnityEditor.Handles.DrawAAConvexPolygon(a.rayoInterrumpido ? a.puntoHit : a.punto, b.rayoInterrumpido ? b.puntoHit : b.punto, transform.position);
                        }
                        else UnityEditor.Handles.DrawAAConvexPolygon(a.puntoHit, b.puntoHit, transform.position);

                        if (false){
                            if (cantHits > 0)
                            {
                                //hits[0].distance += (a.distancia+b.distancia)/2f;
                                if (hits[0].distance < 0.001f) UnityEditor.Handles.DrawAAConvexPolygon(a.rayoInterrumpido ? a.puntoHit : a.punto, b.rayoInterrumpido ? b.puntoHit : b.punto, transform.position);
                                else if (hits[0].distance > rayoCercaLejos.magnitude * .999f) UnityEditor.Handles.DrawAAConvexPolygon(a.puntoHit, b.puntoHit, transform.position);
                                else
                                {
                                    if (a.distancia > b.distancia) UnityEditor.Handles.DrawAAConvexPolygon(a.rayoInterrumpido ? a.puntoHit : a.punto, b.puntoHit, transform.position);
                                    else UnityEditor.Handles.DrawAAConvexPolygon(a.puntoHit, b.rayoInterrumpido ? b.puntoHit : b.punto, transform.position);
                                }
                                /*UnityEditor.Handles.DrawAAConvexPolygon(hits[0].distance>a.distancia? a.puntoHit:a.punto, hits[0].distance > b.distancia ? b.puntoHit : b.punto, transform.position);*/
                                if (hover)
                                {
                                    UnityEditor.Handles.Label(hits[0].point, hits[0].distance.ToString("0.00\n") + (rayoCercaLejos.magnitude - hits[0].distance).ToString());
                                    UnityEditor.Handles.Label(a.punto, a.distancia.ToString("0.00"));
                                    UnityEditor.Handles.Label(b.punto, b.distancia.ToString("0.00"));
                                }
                            }
                            else UnityEditor.Handles.DrawAAConvexPolygon(a.puntoHit, b.puntoHit, transform.position);
                        }

                        Gizmos.color =Color.Lerp(Color.clear, Color.green,hover?1f:.2f);
                        //Gizmos.DrawRay(medio - perp * .1f, perp*.2f);
                        //Gizmos.DrawRay(medioCerca, rayoCercaLejos);
                        Gizmos.DrawRay(transform.position, Vector2.Lerp(a.rayoDesdeOrigen, b.rayoDesdeOrigen, .5f).normalized*radio);
                    }
                }
            }

                /*
                RaycastHit2D[] hits = new RaycastHit2D[2],hits2=new RaycastHit2D[2];
                int cantHits = 0,cantHits2=0;
                //destinosDeRayos.Sort((Vector2 a, Vector2 b) => { return Mathf.Atan2(a.y - transform.position.y, a.x - transform.position.x).CompareTo(Mathf.Atan2(b.y - transform.position.y, b.x - transform.position.x)); });
                destinosDeRayos.Sort();
                destinosDeRayos.Add(destinosDeRayos[0]);

                for (int i = 0; i < destinosDeRayos.Count-1; i++)
                {
                    VectorDeLuz a = destinosDeRayos[i];
                    VectorDeLuz b = destinosDeRayos[i+1];
                    if (PuntoEnTriangulo(a.punto, b.punto, transform.position, UnityEditor.HandleUtility.GUIPointToWorldRay(UnityEngine.Event.current.mousePosition).GetPoint(1f)))
                    {

                        UnityEditor.Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.red, .7f);
                        cantHits = Physics2D.Raycast(transform.position, a.rayoDesdeOrigen, new ContactFilter2D(), hits, radio);
                        cantHits2 = Physics2D.Raycast(transform.position, b.rayoDesdeOrigen, new ContactFilter2D(), hits2, radio);
                        if (cantHits > 0)
                        {
                            if (hits[0].distance > a.distancia*.999f)
                            {// es limite o aun mas alla
                                if (hits[0].distance > a.distancia)
                                {
                                    hits[1].point = hits[0].point;
                                    hits[1].distance = hits[0].distance;
                                }
                                else
                                {
                                    if (cantHits < 2)
                                    {
                                        hits[1].point = (Vector2)transform.position + a.rayoDesdeOrigen.normalized * radio;
                                        hits[1].distance = radio;
                                    }
                                }
                                hits[0].point = a.punto;
                                hits[0].distance = a.distancia;
                                cantHits = 2;
                            }
                            else
                            {//esta mas adentro
                                UnityEditor.Handles.DrawWireDisc(hits[0].point, Vector3.forward, .1f);
                                cantHits = 1;
                            }
                        }
                        else
                        {
                            hits[0].point = a.punto;
                            hits[0].distance = a.distancia;
                            hits[1].point = (Vector2)transform.position + a.rayoDesdeOrigen.normalized * radio;
                            hits[1].distance = radio;
                            cantHits = 2;
                        }
                        if (cantHits2 > 0)
                        {
                            if (hits2[0].distance > b.distancia * .999f)
                            {
                                if (hits2[0].distance > b.distancia)
                                {
                                    hits2[1].point = hits2[0].point;
                                    hits2[1].distance = hits2[0].distance;
                                }
                                else
                                {
                                    if (cantHits2 < 2)
                                    {
                                        hits2[1].point = (Vector2)transform.position + b.rayoDesdeOrigen.normalized * radio;
                                        hits2[1].distance = radio;
                                    }
                                }
                                hits2[0].point = b.punto;
                                hits2[0].distance = b.distancia;
                                cantHits2 = 2;
                            }
                            else
                            {
                                UnityEditor.Handles.DrawWireDisc(hits2[0].point, Vector3.forward, .1f);
                                cantHits2 = 1;
                            }
                        }
                        else
                        {
                            hits2[0].point = b.punto;
                            hits2[0].distance = b.distancia;
                            hits2[1].point = (Vector2)transform.position + b.rayoDesdeOrigen.normalized * radio;
                            hits2[1].distance = radio;
                            cantHits2 = 2;
                        }

                        if (cantHits > 1 && cantHits2 > 1)
                        {
                            UnityEditor.Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.blue, .3f);
                            UnityEditor.Handles.DrawAAConvexPolygon(hits[1].point, hits2[1].point, transform.position);
                        }
                        else
                        {
                            UnityEditor.Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.red, .3f);
                            UnityEditor.Handles.DrawAAConvexPolygon(hits[0].point, hits2[0].point, transform.position);
                        }
                        break;
                    }
                }

                    VectorDeLuz previo = destinosDeRayos[0];
                Vector2 previoHit = previo.punto;

                foreach (var r in destinosDeRayos)
                {
                    UnityEditor.Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.green, .2f);
                    Gizmos.DrawLine(r.punto, previo.punto);
                    Gizmos.DrawRay( transform.position, r.rayoDesdeOrigen);
                    previo = r;
                    if(grosorRayo>0f)cantHits = Physics2D.CircleCast(transform.position, grosorRayo, r.rayoDesdeOrigen, new ContactFilter2D(), hits, radio);
                    else cantHits = Physics2D.Raycast(transform.position, r.rayoDesdeOrigen, new ContactFilter2D(), hits, radio);
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
                            //UnityEditor.Handles.Label(hits[1].point,"\n"+hits[0].distance+"_"+hits[1].distance);

                            if (hits[0].distance > hits[1].distance)
                            {
                                UnityEditor.Handles.color = Gizmos.color = Color.blue;
                                UnityEditor.Handles.DrawWireDisc(hits[0].point, Vector3.forward, .5f);
                            }

                        }
                        else
                        {
                            Gizmos.DrawLine(previoHit, (Vector2)transform.position + (r.rayoDesdeOrigen).normalized * radio);
                           // UnityEditor.Handles.Label(r.punto, "\n" + hits[0].distance + "_" + radio);
                        }
                    }
                    else
                    {
                        //Gizmos.DrawLine(previoHit, (Vector2)transform.position+(r - (Vector2)transform.position).normalized*radio );
                        //previoHit = (Vector2)transform.position + (r - (Vector2)transform.position).normalized * radio;
                        Gizmos.DrawLine(previoHit, r.punto);
                        previoHit = r.punto;
                        UnityEditor.Handles.color = Gizmos.color = Color.cyan;
                        Gizmos.DrawLine(previoHit, (Vector2)transform.position + (r.rayoDesdeOrigen).normalized * radio);
                    }
                }*/
            }
    }

    bool PuntoEnTriangulo(Vector2 triA, Vector2 triB, Vector2 triC, Vector2 p)
    {
        // Compute vectors        
        var v0 = triC - triA;
        var v1 = triB - triA;
        var v2 = p - triA;

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
#endif
}
