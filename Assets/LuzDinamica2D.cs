using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class LuzDinamica2D : MonoBehaviour
{
    static ContactFilter2D filtroVacio = new ContactFilter2D();
    static int cantHits = 0;
    static RaycastHit2D[] hits = new RaycastHit2D[1];
    static Collider2D[] colls = new Collider2D[1];
    static Dictionary<Collider2D, List<Circulo>> colliders = new Dictionary<Collider2D, List<Circulo>>();

    class Circulo
    {
        public Vector2 pos;
        public float radio;
        public Collider2D padre;
        public Vector2 OffsetRayo(Vector2 desde, float signo)
        {
            if (radio == 0f) return Vector2.zero;
            float d = (pos - desde).magnitude;
            float h = Mathf.Sqrt(d * d - radio * radio) * radio / d;
            float d_ = Mathf.Sqrt(radio * radio - h * h);
            return Vector2.Perpendicular(desde - pos).normalized * h * signo + (desde - pos).normalized * d_;
        }
    }
    class VectorDeLuz : System.IComparable<VectorDeLuz>
    {
        public bool rayoInterrumpido, toqueVerticeClave, esHorizonte;
        public float angulo;
        public float distancia;
        public float distancia2;
        public Vector2 punto, origen, puntoHit;
        public Vector2 puntoRelativoAOrigen;
        public Ray2D rayo;
        public VectorDeLuz horizonteAlfa, horizonteBeta;
        public Collider2D padre;

        int cantHits = 0;
        public RaycastHit2D[] hits = new RaycastHit2D[2];
        public Vector2 PuntoCercano
        { get { return rayoInterrumpido ? puntoHit : punto; } }
        public float DistanciaCercana
        { get { return rayoInterrumpido ? hits[0].distance : distancia; } }
        public float distanciaLejana;

        public VectorDeLuz(Circulo referencia, Vector2 desde, float radio = 0f)
        {
            origen = desde;
            punto = referencia.pos;
            padre = referencia.padre;
            puntoRelativoAOrigen = punto - origen;
            distancia = puntoRelativoAOrigen.magnitude;
            distancia2 = distancia * distancia;
            angulo = Mathf.Atan2(puntoRelativoAOrigen.y, puntoRelativoAOrigen.x);

            if (referencia.radio > 0f)
            {
                float r2 = referencia.radio * referencia.radio;
                float h = Mathf.Sqrt(distancia2 - r2) * referencia.radio / distancia;
                float d_ = Mathf.Sqrt(r2 - h * h);

                Vector2 offsetPerp = Vector2.Perpendicular(puntoRelativoAOrigen).normalized * h;
                Vector2 offsetParal = (puntoRelativoAOrigen).normalized * d_;

                horizonteAlfa = new VectorDeLuz(this);
                horizonteBeta = new VectorDeLuz(this);
                horizonteBeta.origen = horizonteAlfa.origen = origen;

                horizonteAlfa.punto = punto - offsetPerp - offsetParal;
                horizonteAlfa.puntoRelativoAOrigen = puntoRelativoAOrigen - offsetPerp - offsetParal;
                horizonteBeta.punto = punto + offsetPerp - offsetParal;
                horizonteBeta.puntoRelativoAOrigen = puntoRelativoAOrigen + offsetPerp - offsetParal;

                horizonteAlfa.angulo = Mathf.Atan2(horizonteAlfa.puntoRelativoAOrigen.y, horizonteAlfa.puntoRelativoAOrigen.x);
                horizonteBeta.angulo = Mathf.Atan2(horizonteBeta.puntoRelativoAOrigen.y, horizonteBeta.puntoRelativoAOrigen.x);

                horizonteAlfa.distancia = horizonteBeta.distancia = horizonteBeta.puntoRelativoAOrigen.magnitude;
                horizonteAlfa.distancia2 = horizonteBeta.distancia2 = horizonteBeta.distancia * horizonteBeta.distancia;

                horizonteAlfa.CalcularRayo(radio);
                horizonteBeta.CalcularRayo(radio);
            }
            CalcularRayo(radio);
        }
        public VectorDeLuz(VectorDeLuz basico)
        {
            esHorizonte=true;
            padre = basico.padre;
        }

        public void CalcularRayo(float radio)
        {
            rayo = new Ray2D(origen, puntoRelativoAOrigen.normalized);
            cantHits = Physics2D.Raycast(origen, puntoRelativoAOrigen, filtroVacio, hits, radio);
            if (cantHits > 0)
            {
                if (hits[0].distance < distancia * .999f)
                {
                    rayoInterrumpido = true;
                }
                else
                {
                    if (hits[0].distance * .999f < distancia && hits[0].collider == padre)
                    {//choco con vertice clave

                        toqueVerticeClave = true;
                        if (!esHorizonte && padre.OverlapPoint(rayo.GetPoint(hits[0].distance / .999f)))
                        {//vertice clave igual es como un coso grueso y en realidad interrumpe
                            rayoInterrumpido = true;
                            cantHits = 1;
                        }
                        else
                        {
                            if (cantHits > 1)
                            {//choco con algo mas ademas de vertice clave
                             /*hits[0].point = hits[1].point;
                             hits[0].distance = hits[1].distance;
                             hits[0].normal = hits[1].normal;*/
                                hits[0] = hits[1];
                            }
                            else
                            {//no choco con nada mas que vertice clave
                                hits[0].point = origen + rayo.direction * radio;
                                hits[0].distance = radio;
                                hits[0].normal = -rayo.direction;
                            }
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
                    hits[0].point = origen + rayo.direction * radio;
                }
                else
                {
                    hits[0].point = origen + rayo.direction * radio;
                }
                hits[0].distance = radio;
                hits[0].normal = rayo.direction;
                cantHits = 1;
            }
            puntoHit = hits[0].point;
            distanciaLejana = hits[0].distance;
        }

        public void Gizmo()
        {
            if (cantHits > 0) Gizmos.DrawLine(puntoHit, origen);
            else
            {
                Gizmos.color = Color.black;
                Gizmos.DrawRay(origen, puntoRelativoAOrigen);
            }
        }

        public int CompareTo(VectorDeLuz otro)
        {
            return angulo.CompareTo(otro.angulo);
        }
    }
    class HazDeLuz
    {
        bool conRelojCerca, contraRelojCerca, conInterseccion;
        float radioFrente, radioCentro;
        Vector3 origen;
        Vector2 frente, centro;
        VectorDeLuz conReloj, contraReloj;
        Ray2D rayoInterno;
        RaycastHit2D[] hitsRayoInterno = new RaycastHit2D[1];
        int cantHits = 0;
        Collider2D[] colls=new Collider2D[1];
        public HazDeLuz(VectorDeLuz conReloj, VectorDeLuz contraReloj)
        {
            this.conReloj = conReloj;
            this.contraReloj = contraReloj;
            origen = conReloj.origen;
            EstablecerForma();
        }

        void EstablecerForma()
        {
            contraRelojCerca = contraReloj.rayoInterrumpido;
            conRelojCerca = conReloj.rayoInterrumpido;
            if (conReloj.rayoInterrumpido && contraReloj.rayoInterrumpido)
            {
                if (conReloj.hits[0].normal != contraReloj.hits[0].normal && !conReloj.toqueVerticeClave && !contraReloj.toqueVerticeClave)
                {
                    conInterseccion = true;
                    bool hover= Hover(HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f));
                    if (hover)
                    {
                        Handles.color = Color.green;
                        Handles.DrawLine(contraReloj.PuntoCercano, contraReloj.PuntoCercano + Vector2.Perpendicular(contraReloj.hits[0].normal));
                        Handles.DrawLine(conReloj.PuntoCercano, conReloj.PuntoCercano + Vector2.Perpendicular(conReloj.hits[0].normal));
                    }
                    if (!LineIntersection(contraReloj.PuntoCercano,contraReloj.PuntoCercano+ Vector2.Perpendicular( contraReloj.hits[0].normal), conReloj.PuntoCercano, Vector2.Perpendicular(conReloj.hits[0].normal)+conReloj.PuntoCercano, ref centro)) centro = origen;
                }
            }
            else
            //if (!conReloj.rayoInterrumpido || !contraReloj.rayoInterrumpido)
            {
                centro = (conReloj.PuntoCercano + contraReloj.PuntoCercano) / 2f;
                cantHits = Physics2D.OverlapCircle(centro, .001f, filtroVacio, colls);
                if (cantHits > 0)
                {
                    contraRelojCerca = conRelojCerca = true;
                }
                else if (conReloj.rayoInterrumpido == contraReloj.rayoInterrumpido)
                {//niguno interrumpido
                    if (LineIntersection(contraReloj.PuntoCercano,conReloj.puntoHit,contraReloj.puntoHit,conReloj.PuntoCercano,ref centro))
                    {
                        frente = (centro + contraReloj.PuntoCercano) / 2f;
                        cantHits = Physics2D.OverlapCircle(frente, .001f, filtroVacio, colls);
                        if (cantHits > 0)
                        {
                            contraRelojCerca = true;
                        }
                        frente = (centro + conReloj.PuntoCercano) / 2f;
                        cantHits = Physics2D.OverlapCircle(frente, .001f, filtroVacio, colls);
                        if (cantHits > 0)
                        {
                            conRelojCerca = true;
                        }
                    }
                }
                /*
                radioFrente = ((conReloj.PuntoCercano - (Vector2)origen*2f + contraReloj.PuntoCercano) / 2f).magnitude;//(conReloj.DistanciaCercana + contraReloj.DistanciaCercana) / 2f;
                if (conReloj.rayoInterrumpido != contraReloj.rayoInterrumpido)
                {
                    if (conReloj.rayoInterrumpido) radioCentro = (conReloj.DistanciaCercana + contraReloj.distanciaLejana) / 2f;
                    else radioCentro = (conReloj.distanciaLejana + contraReloj.DistanciaCercana) / 2f;
                }
                else radioCentro = conReloj.distanciaLejana;
                   // radioFrente = Mathf.Min(conReloj.DistanciaCercana, contraReloj.DistanciaCercana);

               // radioCentro = Mathf.Max(conReloj.distanciaLejana, contraReloj.distanciaLejana);
                rayoInterno = new Ray2D( origen, conReloj.puntoRelativoAOrigen+contraReloj.puntoRelativoAOrigen);
                if (Hover(HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f)))
                {
                    GUI.color = Color.red + Color.green / 2f;
                    Handles.Label(hitsRayoInterno[0].point, hitsRayoInterno[0].distance.ToString());
                    Handles.Label(contraReloj.PuntoCercano, contraReloj.DistanciaCercana + "\n" + contraReloj.distanciaLejana);
                    Handles.Label(conReloj.PuntoCercano, conReloj.DistanciaCercana + "\n" + conReloj.distanciaLejana);
                    GUI.color = Color.white;
                }
                if (Physics2D.Raycast(rayoInterno.origin, rayoInterno.direction, filtroVacio, hitsRayoInterno,radioCentro/.999f) > 0)
                {
                    if (Hover(HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f)))
                    {
                        float tam = .25f * HandleUtility.GetHandleSize(origen);
                        var p = hitsRayoInterno[0].point;
                        Handles.color = Color.gray + Color.red;
                        Handles.DrawLine(p - new Vector2(-1, -1) * tam, p - new Vector2(1, 1) * tam);
                        Handles.DrawLine(p - new Vector2(1, -1) * tam, p - new Vector2(-1, 1) * tam);
                        Handles.DrawLine(p - new Vector2(-1, -1) * tam, p - new Vector2(1, 1) * tam);
                        Handles.DrawLine(p - new Vector2(1, -1) * tam, p - new Vector2(-1, 1) * tam);
                        p =  (conReloj.PuntoCercano+contraReloj.PuntoCercano) / 2f;
                        Handles.color = Color.gray + Color.blue;
                        Handles.DrawLine(p - new Vector2(-1, -1) * tam, p - new Vector2(1, 1) * tam);
                        Handles.DrawLine(p - new Vector2(1, -1) * tam, p - new Vector2(-1, 1) * tam);
                        Handles.DrawLine(p - new Vector2(-1, -1) * tam, p - new Vector2(1, 1) * tam);
                        Handles.DrawLine(p - new Vector2(1, -1) * tam, p - new Vector2(-1, 1) * tam);
                    }
                    if (hitsRayoInterno[0].distance > radioFrente * .999f)
                    {
                        contraRelojCerca = conRelojCerca = true;
                    }
                    else
                    {

                    }
                    if (Hover(HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f)))
                    {
                        GUI.color = Color.red + Color.green / 2f;
                        Handles.Label(hitsRayoInterno[0].point, hitsRayoInterno[0].distance+"\nF>"+radioFrente+"\nC>"+radioCentro);
                        GUI.color = Color.white;
                    }
                }*/
            }
        }

        public bool Hover(Vector3 punto)
        {
            return PuntoEnTriangulo(conReloj.puntoHit, contraReloj.puntoHit, origen, punto);
        }

        public void Gizmo(float radio)
        {
            bool hover = Hover(HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f));
            if (hover)
            {
                Gizmos.color = conReloj.rayoInterrumpido ? Color.red : Color.blue;
                if (conReloj.toqueVerticeClave) Gizmos.color += Color.green;
                conReloj.Gizmo();
                Gizmos.color = contraReloj.rayoInterrumpido ? Color.red : Color.blue;
                if (contraReloj.toqueVerticeClave) Gizmos.color += Color.green;
                contraReloj.Gizmo();
            }
            Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.white, .4f);
            /*if (forma == Forma.Corto)//if (conReloj.rayoInterrumpido && contraReloj.rayoInterrumpido)
            {
                Handles.DrawAAConvexPolygon(contraReloj.PuntoCercano, conReloj.PuntoCercano, origen);
            }
            else if (forma == Forma.Largo)
            {
                Handles.DrawAAConvexPolygon(contraReloj.puntoHit, conReloj.puntoHit, origen);
            }
            else if (forma == Forma.LargoACorto)
            {
                Handles.DrawAAConvexPolygon(contraReloj.PuntoCercano, conReloj.puntoHit, origen);
            }
            else if (forma == Forma.CortoALargo)
            {
                Handles.DrawAAConvexPolygon(contraReloj.puntoHit, conReloj.PuntoCercano, origen);
            }*/
            if (!conReloj.rayoInterrumpido && !contraReloj.rayoInterrumpido && conRelojCerca && contraRelojCerca || conInterseccion)
            {
                Handles.DrawAAConvexPolygon(contraReloj.PuntoCercano ,centro,conReloj.PuntoCercano , origen);
            }
            else Handles.DrawAAConvexPolygon(contraRelojCerca?contraReloj.PuntoCercano: contraReloj.puntoHit, conRelojCerca? conReloj.PuntoCercano:conReloj.puntoHit, origen);
            if (!conReloj.rayoInterrumpido || !contraReloj.rayoInterrumpido)
            {
                //Vector2 medioCerca = Vector2.Lerp(conReloj.PuntoCercano, contraReloj.PuntoCercano, .5f);
                //Vector2 medioLejos = Vector2.Lerp(!conReloj.rayoInterrumpido ? conReloj.puntoHit : conReloj.punto, !contraReloj.rayoInterrumpido ? contraReloj.puntoHit : contraReloj.punto, .5f);
                //Vector2 rayoCercaLejos = medioLejos - medioCerca;

                //cantHits = Physics2D.Raycast(origen, medioCerca - (Vector2)origen, filtroVacio, hits, radio);

                if (hover)
                {
                    Handles.color = Color.green;
                    Handles.DrawLine(conReloj.PuntoCercano, contraReloj.puntoHit);
                    Handles.DrawLine(contraReloj.PuntoCercano, conReloj.puntoHit);
                    Handles.color = Color.yellow;
                    //Handles.DrawWireArc(origen, Vector3.forward, conReloj.rayo.direction, Vector3.Angle(conReloj.rayo.direction, contraReloj.rayo.direction) +5f, radioFrente);
                    //Handles.DrawWireArc(origen, Vector3.forward, contraReloj.rayo.direction, -Vector3.Angle(conReloj.rayo.direction, contraReloj.rayo.direction) -5f, radioCentro);

                    float tam = .25f*HandleUtility.GetHandleSize(origen);
                    Handles.DrawLine(conReloj.PuntoCercano - new Vector2(-1, -1) * tam, conReloj.PuntoCercano - new Vector2(1, 1) * tam);
                    Handles.DrawLine(conReloj.PuntoCercano - new Vector2(1, -1) * tam, conReloj.PuntoCercano - new Vector2(-1, 1) * tam);
                    Handles.DrawLine(contraReloj.PuntoCercano - new Vector2(-1, -1) * tam, contraReloj.PuntoCercano - new Vector2(1, 1) * tam);
                    Handles.DrawLine(contraReloj.PuntoCercano - new Vector2(1, -1) * tam, contraReloj.PuntoCercano - new Vector2(-1, 1) * tam);
                    Handles.DrawLine(conReloj.puntoHit - new Vector2(-1, 0) * tam, conReloj.puntoHit - new Vector2(1, 0) * tam);
                    Handles.DrawLine(conReloj.puntoHit - new Vector2(0, -1) * tam, conReloj.puntoHit - new Vector2(0, 1) * tam);
                    Handles.DrawLine(contraReloj.puntoHit - new Vector2(-1, 0) * tam, contraReloj.puntoHit - new Vector2(1, 0) * tam);
                    Handles.DrawLine(contraReloj.puntoHit - new Vector2(0, -1) * tam, contraReloj.puntoHit - new Vector2(0, 1) * tam);

                    Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.white, .7f);
                }
                /*if (cantHits > 0)
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
                else Handles.DrawAAConvexPolygon(conReloj.puntoHit, contraReloj.puntoHit, origen);*/

                Gizmos.color = Color.Lerp(Color.clear, Color.green, hover ? 1f : .2f);

                //Gizmos.DrawLine(origen, medioCerca);
                //Gizmos.DrawLine(medioCerca, medioLejos);
                Gizmos.DrawLine(origen, rayoInterno.GetPoint(radioCentro / .999f));
                Gizmos.DrawRay(origen, (conReloj.rayo.direction+contraReloj.rayo.direction).normalized*(radioCentro / .999f));
            }

        }

        static bool LineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, ref Vector2 result)
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

            result = new Vector2(p1.x + t * bx, p1.y + t * by);
            return true;
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
    }

    public float radio, grosorRayo = .001f;
    List<VectorDeLuz> destinosDeRayos = new List<VectorDeLuz>();
    List<HazDeLuz> hacesDeLuz = new List<HazDeLuz>();

    static void GenerarCirculos()
    {
        colliders.Clear();
        GenerarCirculosBox();
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
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
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
            vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset), radio = coll.transform.TransformVector(Vector3.right).magnitude * coll.radius, padre = coll });
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
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset), radio = Mathf.Max(tamDeforme.x, tamDeforme.y), padre = coll });
            }
            else
            {
                float radio = Mathf.Min(tamDeforme.x, tamDeforme.y);
                Vector3 offset = Vector3.zero;
                if (coll.direction == CapsuleDirection2D.Horizontal) offset = coll.transform.right * (tamDeforme.x - radio) * .5f;
                else offset = coll.transform.up * (tamDeforme.y - radio) * .5f;
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset) + offset, radio = radio, padre = coll });
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(coll.offset) - offset, radio = radio, padre = coll });
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
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(p + coll.offset), radio = coll.edgeRadius, padre = coll });
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
                vertices.Add(new Circulo() { pos = coll.transform.TransformPoint(p + coll.offset), radio = 0f, padre = coll });
            }
            colliders.Add(coll, vertices);
        }
    }
    static void GenerarCirculosComposite()
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
                    vertices.Add(new Circulo() { pos = coll.transform.TransformDirection(p) + coll.transform.position, radio = radio, padre = coll });
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
        Handles.color = Gizmos.color = Color.gray;
        Handles.DrawWireDisc(transform.position, Vector3.forward, radio);
        Handles.color = Gizmos.color = Color.magenta;
        float radio2 = radio * radio;
        foreach (var col in Physics2D.OverlapCircleAll(transform.position, radio))
        {
            foreach (var c in colliders[col])
            {
                destinosDeRayos.Add(new VectorDeLuz(c, transform.position, radio));
                if (c.radio > 0f)
                {
                    destinosDeRayos.Add(destinosDeRayos[destinosDeRayos.Count - 1].horizonteAlfa);
                    destinosDeRayos.Add(destinosDeRayos[destinosDeRayos.Count - 2].horizonteBeta);
                }

                if (c.radio == 0f)
                {
                    Gizmos.DrawLine(c.pos - Vector2.up * .1f, c.pos + Vector2.up * .1f);
                    Gizmos.DrawLine(c.pos - Vector2.right * .1f, c.pos + Vector2.right * .1f);
                }
                else
                {
                    Handles.DrawWireDisc(c.pos, Vector3.forward, c.radio);
                }
            }
        }
        if (destinosDeRayos.Count > 0)
        {
            /*foreach (var rayo in destinosDeRayos)
            {
                Gizmos.color = Color.Lerp(Color.clear, rayo.rayoInterrumpido ? Color.red : Color.blue, .3f);
                rayo.Gizmo();
            }*/

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
