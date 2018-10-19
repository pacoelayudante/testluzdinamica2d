using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class LuzDinamica2D : MonoBehaviour {
    static ContactFilter2D filtroVacio= new ContactFilter2D();
    static int cantHits = 0;
    static RaycastHit2D[] hits = new RaycastHit2D[1];
    static Dictionary<Collider2D, List<Circulo>> colliders = new Dictionary<Collider2D, List<Circulo>>();

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
        public bool rayoInterrumpido,toqueVerticeClave;
       public float angulo;
        public float distancia;
        public float distancia2;
        public Vector2 punto,origen,puntoHit;
        public Vector2 rayoDesdeOrigen;
        public VectorDeLuz horizonteAlfa,horizonteBeta;

        int cantHits = 0;
        public RaycastHit2D[] hits = new RaycastHit2D[2];
        public Vector2 PuntoCercano
        { get { return rayoInterrumpido ? puntoHit : punto; } }
        public float DistanciaCercana
        { get { return rayoInterrumpido ? hits[0].distance : distancia; } }

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
                        toqueVerticeClave = true;
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
                if (distancia > radio)
                {//esta muy lejos, inventale un choque pero con limite luz
                    rayoInterrumpido = true;
                    distancia = radio;
                    distancia2 = distancia * distancia;
                    hits[0].point = origen + rayoDesdeOrigen.normalized * radio;
                }
                else
                {
                    hits[0].point = origen + rayoDesdeOrigen.normalized * radio;
                }
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
    class HazDeLuz
    {
        public enum Forma
        {
            Largo,Corto,CortoALargo,LargoACorto
        }
        Forma forma = Forma.Corto;
        Vector3 origen;
        VectorDeLuz conReloj, contraReloj;
        public HazDeLuz(VectorDeLuz conReloj, VectorDeLuz contraReloj)
        {
            this.conReloj = conReloj;
            this.contraReloj = contraReloj;
            origen = conReloj.origen;
        }
        public bool Hover(Vector3 punto)
        {
            return PuntoEnTriangulo(conReloj.puntoHit, contraReloj.puntoHit, origen, punto);
        }

        public void Gizmo(float radio)
        {
            bool hover = Hover(HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f));
            //if (hover)
            {
                if (hover)
                {
                    Gizmos.color = conReloj.rayoInterrumpido ? Color.red : Color.blue;
                    if (conReloj.toqueVerticeClave) Gizmos.color += Color.green;
                    conReloj.Gizmo();
                    Gizmos.color = contraReloj.rayoInterrumpido ? Color.red : Color.blue;
                    if (contraReloj.toqueVerticeClave) Gizmos.color += Color.green;
                    contraReloj.Gizmo();

                    /*    Gizmos.color = Color.yellow;
                    Gizmos.DrawRay(a.puntoHit,Vector2.Perpendicular( a.hits[0].normal) * 999f);
                    Gizmos.DrawRay(a.puntoHit, Vector2.Perpendicular(-a.hits[0].normal) * 999f);
                    Gizmos.DrawRay(b.puntoHit, Vector2.Perpendicular(b.hits[0].normal) * 999f);
                    Gizmos.DrawRay(b.puntoHit, Vector2.Perpendicular(-b.hits[0].normal) * 999f);*/
                }
                Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.white, .7f);
                if (conReloj.rayoInterrumpido && contraReloj.rayoInterrumpido)
                {
                    Handles.DrawAAConvexPolygon(conReloj.PuntoCercano, contraReloj.PuntoCercano, origen);
                }
                else
                {
                    Vector2 medioCerca = Vector2.Lerp(conReloj.PuntoCercano, contraReloj.PuntoCercano, .5f);
                    Vector2 medioLejos = Vector2.Lerp(!conReloj.rayoInterrumpido ? conReloj.puntoHit : conReloj.punto, !contraReloj.rayoInterrumpido ? contraReloj.puntoHit : contraReloj.punto, .5f);
                    Vector2 rayoCercaLejos = medioLejos - medioCerca;
                    //Vector2 perp = Vector2.Perpendicular(a.punto- b.punto).normalized;

                    //cantHits = Physics2D.Raycast(transform.position, Vector2.Lerp(a.rayoDesdeOrigen, b.rayoDesdeOrigen, .5f), filtroVacio, hits, radio);
                    cantHits = Physics2D.Raycast(origen, medioCerca - (Vector2)origen, filtroVacio, hits, radio);
                    //cantHits = Physics2D.Raycast(medio-perp*.1f, perp, filtroVacio, hits, .2f);
                    //cantHits = Physics2D.Raycast(medioCerca, rayoCercaLejos, filtroVacio, hits, rayoCercaLejos.magnitude);
                    if (hover)
                    {
                        Handles.DrawSolidDisc(medioCerca, Vector3.forward, .01f);
                    }
                    if (cantHits > 0)
                    {
                        if (.99f * hits[0].distance < (conReloj.DistanciaCercana + contraReloj.DistanciaCercana) * .5f)
                        {
                            Handles.DrawAAConvexPolygon(conReloj.PuntoCercano, contraReloj.PuntoCercano, origen);
                        }
                        else
                        {
                            if (conReloj.DistanciaCercana > contraReloj.DistanciaCercana) Handles.DrawAAConvexPolygon(conReloj.PuntoCercano, contraReloj.puntoHit, origen);
                            else Handles.DrawAAConvexPolygon(conReloj.puntoHit, contraReloj.PuntoCercano, origen);
                        }
                        if (hover)
                        {
                            Handles.DrawSolidRectangleWithOutline(new Rect(hits[0].point - Vector2.one * .01f, Vector2.one * .02f), Color.clear, Color.red);
                            GUI.contentColor = Color.red;
                            Handles.Label(medioCerca, (.99f * hits[0].distance).ToString("0.000 <") + ((conReloj.DistanciaCercana + contraReloj.DistanciaCercana) * .5f).ToString("\n0.000"));
                            Handles.Label(conReloj.PuntoCercano, conReloj.DistanciaCercana.ToString("\n0.00"));
                            Handles.Label(contraReloj.PuntoCercano, contraReloj.DistanciaCercana.ToString("\n0.00"));
                            GUI.contentColor = Color.white;
                        }
                    }
                    else Handles.DrawAAConvexPolygon(conReloj.puntoHit, contraReloj.puntoHit, origen);
                    
                    Gizmos.color = Color.Lerp(Color.clear, Color.green, hover ? 1f : .2f);

                    Gizmos.DrawLine(origen, medioCerca);
                }
            }
        }

            bool PuntoEnTriangulo(Vector2 triA, Vector2 triB, Vector2 triC, Vector2 p) {
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
    }

    public float radio,grosorRayo=.001f;
    List<VectorDeLuz> destinosDeRayos = new List<VectorDeLuz>();
    List<HazDeLuz> hacesDeLuz = new List<HazDeLuz>();

    static void GenerarCirculos()
    {
        colliders.Clear();
        GenerarCirculosBox( );
        GenerarCirculosCircle();
        GenerarCirculosCapsule();
        GenerarCirculosEdge();
        GenerarCirculosPolygon();
        GenerarCirculosComposite();
    }
    static void GenerarCirculosBox()
    {
        foreach (var coll in FindObjectsOfType<BoxCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            List<Circulo> vertices = new List<Circulo>();
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius });
            colliders.Add(coll, vertices);
        }
    }
    static void GenerarCirculosCircle()
    {
        foreach (var coll in FindObjectsOfType<CircleCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            List<Circulo> vertices = new List<Circulo>();
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset), radio = coll.transform.TransformVector(Vector3.right).magnitude * coll.radius });
            colliders.Add(coll, vertices);
        }
    }
    static void GenerarCirculosCapsule()
    {
        foreach (var coll in FindObjectsOfType<CapsuleCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            List<Circulo> vertices = new List<Circulo>();
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
            colliders.Add(coll, vertices);
        }
    }
    static void GenerarCirculosEdge()
    {
        foreach (var coll in FindObjectsOfType<EdgeCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            List<Circulo> vertices = new List<Circulo>();
            foreach (var p in coll.points)
            {
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(p + coll.offset), radio = coll.edgeRadius });
            }
            colliders.Add(coll, vertices);
        }
    }
    static void GenerarCirculosPolygon()
    {
        foreach (var coll in FindObjectsOfType<PolygonCollider2D>())
        {
            if (!coll.enabled) continue;
            if (coll.usedByComposite) continue;
            List<Circulo> vertices = new List<Circulo>();
            foreach (var p in coll.points)
            {
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(p + coll.offset), radio = 0f });
            }
            colliders.Add(coll, vertices);
        }
    }
    static void GenerarCirculosComposite ()
    {
        foreach (var coll in FindObjectsOfType<CompositeCollider2D>())
        {
            if (!coll.enabled) continue;
            List<Circulo> vertices = new List<Circulo>();
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
            colliders.Add(coll, vertices);
        }
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        GenerarCirculos();
        hacesDeLuz.Clear();
        destinosDeRayos.Clear();
        UnityEditor.Handles.color = Gizmos.color = Color.gray;
        UnityEditor.Handles.DrawWireDisc(transform.position, Vector3.forward, radio);
        UnityEditor.Handles.color = Gizmos.color = Color.magenta;
        float radio2 = radio * radio;
        foreach (var col in Physics2D.OverlapCircleAll(transform.position, radio))
        {
            foreach (var c in colliders[col])
            {
                //if ((c.pos - (Vector2)transform.position).sqrMagnitude <= radio2)
                {
                    destinosDeRayos.Add(new VectorDeLuz(c, transform.position, radio));
                    if (c.radio > 0f)//if (destinosDeRayos[destinosDeRayos.Count - 1].horizonteAlfa != null)
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
        }
        if (destinosDeRayos.Count > 0)
        {
            foreach (var rayo in destinosDeRayos)
            {
                Gizmos.color = Color.Lerp(Color.clear, rayo.rayoInterrumpido ? Color.red : Color.blue, .3f);
                rayo.Gizmo();
            }

            destinosDeRayos.Sort();
            destinosDeRayos.Add(destinosDeRayos[0]);
            for (int i = 0; i < destinosDeRayos.Count - 1; i++)
            {
                hacesDeLuz.Add(new HazDeLuz(destinosDeRayos[i], destinosDeRayos[i + 1]));
                hacesDeLuz[i].Gizmo(radio);
            }
        }
    }

#endif
}
