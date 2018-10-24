using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

[RequireComponent(typeof(CircleCollider2D))]
public class Luz2D : MonoBehaviour
{
    static ContactFilter2D filtroVacio=new ContactFilter2D();
    static float epsilonMenor = .001f;
    static float epsilonMayor = .999f;
    Dictionary<Collider2D, CirculoVertice[]> colliders = new Dictionary<Collider2D, CirculoVertice[]>();
    List<VectorDeLuz> destinosDeRayos = new List<VectorDeLuz>();
    List<HazDeLuz> hacesDeLuz = new List<HazDeLuz>();

    public ContactFilter2D filtro;
    public float radio = 5f;

    //List<Collider2D> colliders = new List<Collider2D>();
    Collider2D[] collidersTocados = new Collider2D[500];
    int cantColliders = 0;

    new CircleCollider2D collider;
    CircleCollider2D Collider
    {
        get
        {
            if (!collider)
            {
                collider = GetComponent<CircleCollider2D>();
            }
            return collider;
        }
    }
    private void OnValidate()
    {
#if UNITY_EDITOR
        Undo.RecordObject(Collider, "validando collider luz");
#endif
        Collider.isTrigger = true;
        Collider.radius = radio;
        Collider.offset = Vector2.zero;
    }

    class CirculoVertice
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
#if UNITY_EDITOR
        public void OnDrawGizmos()
        {
            if (radio == 0f)
            {
                Gizmos.DrawWireCube(pos, HandleUtility.GetHandleSize(pos) * .1f*Vector2.one);
            }
            else
            {
                Handles.DrawWireDisc(pos, Vector3.forward, radio);
            }
        }
#endif
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        Handles.color = Gizmos.color = Color.black;
        Handles.DrawWireDisc(transform.position, Vector3.forward, radio);
        Gizmos.DrawSphere(transform.position, HandleUtility.GetHandleSize(transform.position) * .05f);
        ColectarColliders();
        ColectarCirculos();
        ConstruirHacesDeLuz();
        Handles.color = Gizmos.color = Color.magenta*.75f;
        foreach (var circs in colliders.Values)
        {
            foreach(var circulo in circs)
            {
                circulo.OnDrawGizmos();
            }
        }
        foreach(var haz in hacesDeLuz)
        {
            haz.Gizmo(radio);
        }
    }
#endif

    class VectorDeLuz : System.IComparable<VectorDeLuz>
    {
        public bool rayoInterrumpido, toqueVerticeClave, esHorizonte;
        public float angulo;
        public float distancia;
        public float distanciaSq;
        public Vector2 puntoDestino, origen, puntoHit;
        public Vector2 puntoDestinoRelativo;
        public Ray2D rayo;
        public VectorDeLuz horizonteAlfa, horizonteBeta;
        public Collider2D colliderPadre;

        int cantHits = 0;
        public RaycastHit2D[] hits = new RaycastHit2D[2];
        public Vector2 PuntoCercano
        { get { return rayoInterrumpido ? puntoHit : puntoDestino; } }
        public Vector2 PuntoLejano
        { get { return rayoInterrumpido ? puntoDestino : puntoHit; } }
        public float DistanciaCercana
        { get { return rayoInterrumpido ? hits[0].distance : distancia; } }

        public VectorDeLuz(CirculoVertice referencia, Vector2 desde, float radio = 0f)
        {
            origen = desde;
            puntoDestino = referencia.pos;
            colliderPadre = referencia.padre;
            puntoDestinoRelativo = puntoDestino - origen;
            distancia = puntoDestinoRelativo.magnitude - referencia.radio;
            distanciaSq = distancia * distancia;
            angulo = Mathf.Atan2(puntoDestinoRelativo.y, puntoDestinoRelativo.x);
            rayo = new Ray2D(origen, puntoDestinoRelativo.normalized);

            if (referencia.radio > 0f)
            {
                float r2 = referencia.radio * referencia.radio;
                float h = Mathf.Sqrt(distanciaSq - r2) * referencia.radio / distancia;
                float d_ = Mathf.Sqrt(r2 - h * h);

                Vector2 offsetPerp = Vector2.Perpendicular(puntoDestinoRelativo).normalized * h;
                Vector2 offsetParal = (puntoDestinoRelativo).normalized * d_;

                horizonteAlfa = new VectorDeLuz(this, -offsetPerp, -offsetParal);
                horizonteBeta = new VectorDeLuz(this, offsetPerp, -offsetParal);

                puntoDestino = rayo.GetPoint(distancia);
                puntoDestinoRelativo = puntoDestino - origen;

                horizonteAlfa.CalcularRayo(radio);
                horizonteBeta.CalcularRayo(radio);
            }
            CalcularRayo(radio);
        }
        public VectorDeLuz(VectorDeLuz original, Vector2 offsetPerp, Vector2 offsetParal)
        {
            esHorizonte = true;
            origen = original.origen;
            colliderPadre = original.colliderPadre;
            puntoDestino = original.puntoDestino + offsetPerp + offsetParal;
            puntoDestinoRelativo = original.puntoDestinoRelativo + offsetPerp + offsetParal;
            rayo = new Ray2D(origen, puntoDestinoRelativo.normalized);
            angulo = Mathf.Atan2(puntoDestinoRelativo.y, puntoDestinoRelativo.x);
            distancia = puntoDestinoRelativo.magnitude;
            distanciaSq = distancia * distancia;
        }

        public void CalcularRayo(float radio)
        {
            cantHits = Physics2D.Raycast(rayo.origin, /*puntoDestinoRelativo*/rayo.direction, filtroVacio, hits, radio);      
            if (cantHits > 0)
            {
                if (hits[0].distance < distancia * epsilonMayor)
                {//un chanqui, para casos limite (si lo hacemos sin epsilon, a veces tira interrumpido por si mismo)
                    rayoInterrumpido = true;
                }
                else
                {
                    if (hits[0].distance * epsilonMayor < distancia && hits[0].collider == colliderPadre)
                    {//choco con vertice clave (lo mismo, el chanqui) en otro caso choco con lo siguiente
                        toqueVerticeClave = true;
                        if (!esHorizonte && colliderPadre.OverlapPoint(rayo.GetPoint(hits[0].distance / epsilonMayor)))
                        {//vertice clave igual es como un coso grueso y en realidad interrumpe (osea estamos dentro)
                            rayoInterrumpido = true;
                            cantHits = 1;
                        }
                        else
                        {
                            if (cantHits > 1)
                            {//choco con algo mas ademas de vertice clave
                                hits[0] = hits[1];
                            }
                            else
                            {//no choco con nada mas que vertice clave
                                hits[0].point = rayo.GetPoint(radio);
                                hits[0].distance = radio;
                                hits[0].normal = -rayo.direction;
                            }
                        }
                    }/*
                    else
                    {
                        hits[0].point = rayo.GetPoint(radio);
                        hits[0].distance = radio;
                        hits[0].normal = -rayo.direction;
                    }*/
                }
            }
            else
            {//No choco nada? inventale el vertice final
                if (distancia > radio)
                {//esta muy lejos, inventale un choque pero con limite luz
                    rayoInterrumpido = true;
                }
                hits[0].point = rayo.GetPoint(radio);
                hits[0].distance = radio;
                hits[0].normal = -rayo.direction;
                cantHits = 1;
            }
            puntoHit = hits[0].point;
        }
        public void Gizmo()
        {
            Gizmos.DrawLine(PuntoLejano, origen);
        }
        public int CompareTo(VectorDeLuz otro)
        {
            return angulo.CompareTo(otro.angulo);
        }
    }

    class HazDeLuz
    {
        bool hover;

        Vector3 origen;
        VectorDeLuz conReloj, contraReloj;

        Vector2 frente, puntoInterseccion;

        Ray2D rayoInterno;
        int cantHits = 0;
        RaycastHit2D[] hitsRayoInterno = new RaycastHit2D[1];

        bool conRelojCerca, contraRelojCerca, conInterseccion;
        float radioFrente, radioCentro;
        public HazDeLuz(VectorDeLuz conReloj, VectorDeLuz contraReloj)
        {
            this.conReloj = conReloj;
            this.contraReloj = contraReloj;
            origen = conReloj.origen;

            hover = GuazuExtender.PuntoEnTriangulo(conReloj.puntoDestino, contraReloj.puntoDestino, origen,
                HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f));

            EstablecerForma();
        }

        void EstablecerForma()
        {
            contraRelojCerca = contraReloj.rayoInterrumpido;
            conRelojCerca = conReloj.rayoInterrumpido;

            if (conReloj.rayoInterrumpido && contraReloj.rayoInterrumpido)
            {//ambos chocaron con algo antes de alcanzar final
                if (conReloj.hits[0].normal != contraReloj.hits[0].normal && !conReloj.toqueVerticeClave && !contraReloj.toqueVerticeClave)
                {//Nos fijamos si NO son paralelos, porque en ese caso buscamos donde se cruzan para hacer bien la forma. Y si chocamos con algun vertice clave, entonces estamos jugados porque nos sabemos la 'normal' asique no chequeamos de esta manera
                    conInterseccion = true;
                    //la interseccion es el punto donde se cruzan las tangentes (perpendicular a las normales) de los Hits
                    if (!GuazuExtender.LineIntersection(
                        contraReloj.PuntoCercano,
                        contraReloj.PuntoCercano+Vector2.Perpendicular(contraReloj.hits[0].normal),
                        conReloj.PuntoCercano,
                        conReloj.PuntoCercano+Vector2.Perpendicular(conReloj.hits[0].normal),
                        ref puntoInterseccion))
                        puntoInterseccion = (conReloj.PuntoCercano + contraReloj.PuntoCercano) / 2f;
                    //Y sino entonces es el punto medio entre ambos hits y ya

                    if (hover)
                    {
                        Handles.color = (Color.magenta + Color.red) * .5f;
                        Handles.DrawDottedLine(contraReloj.PuntoCercano - Vector2.Perpendicular(contraReloj.hits[0].normal), contraReloj.PuntoCercano + Vector2.Perpendicular(contraReloj.hits[0].normal), HandleUtility.GetHandleSize(contraReloj.PuntoCercano) * 5f);
                        Handles.DrawDottedLine(conReloj.PuntoCercano - Vector2.Perpendicular(conReloj.hits[0].normal), conReloj.PuntoCercano + Vector2.Perpendicular(conReloj.hits[0].normal), HandleUtility.GetHandleSize(conReloj.PuntoCercano) * 5f);
                        Handles.DrawWireDisc(puntoInterseccion, Vector3.forward, HandleUtility.GetHandleSize(puntoInterseccion) * .1f);
                    }

                }
            }
            else
            //if (!conReloj.rayoInterrumpido || !contraReloj.rayoInterrumpido)
            {
                puntoInterseccion = (conReloj.PuntoCercano + contraReloj.PuntoCercano) / 2f;
                Collider2D[] overlappedColliders = new Collider2D[1];
                cantHits = Physics2D.OverlapCircle(puntoInterseccion, epsilonMenor, filtroVacio, overlappedColliders);
                if (cantHits > 0)//aca basicamente estamos comprobando si el eje esta adelante o no, rayos rozantes, de un mismo eje
                {
                    contraRelojCerca = conRelojCerca = true;
                }
                else if (conReloj.rayoInterrumpido == contraReloj.rayoInterrumpido)
                {//aca ya sabemos que no hay un eje pasando por el frente del trapecio loco
                    //nos fijamos el caso donde ambos rayos son razantes y siguen (trapecio loco)
                    if (GuazuExtender.LineIntersection(contraReloj.PuntoCercano, conReloj.puntoHit, contraReloj.puntoHit, conReloj.PuntoCercano, ref puntoInterseccion))
                    {//buscamos interseccion en el centro del trapecio
                        //pero chequeamos por cada eje si el haz es corto o largo
                        cantHits = Physics2D.OverlapCircle((puntoInterseccion + contraReloj.PuntoCercano) / 2f, epsilonMenor, filtroVacio, overlappedColliders);
                        if (cantHits > 0)
                        {
                            contraRelojCerca = true;
                        }
                        cantHits = Physics2D.OverlapCircle((puntoInterseccion + conReloj.PuntoCercano) / 2f, .001f, filtroVacio, overlappedColliders);
                        if (cantHits > 0)
                        {
                            conRelojCerca = true;
                        }
                        //si ambos tienen el haz corto, entonces hay intereseccion loca
                        conInterseccion = (conRelojCerca && contraRelojCerca);
                    }
                    else
                    {//si no es entonces el trapecio es mas bien deforme?
                        Debug.LogError("GuazuExtender.LineIntersection(contraReloj.PuntoCercano, conReloj.puntoHit, contraReloj.puntoHit, conReloj.PuntoCercano, ref puntoInterseccion) == false");
                    }
                }
                else
                {//en este caso hay un rayo que roza y sigue pero el otro rayo es interrumpido
                    //no hay trapecio loco, sino triangulo, (necesitaremos trapecio loco)

                }
            }
        }

        public void Gizmo(float radio)
        {
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

            if (conInterseccion)
            {
                Handles.DrawAAConvexPolygon(contraRelojCerca ? contraReloj.PuntoCercano : contraReloj.PuntoLejano, puntoInterseccion, conRelojCerca ? conReloj.PuntoCercano : conReloj.PuntoLejano, origen);
            }
            else Handles.DrawAAConvexPolygon(contraRelojCerca ? contraReloj.PuntoCercano : contraReloj.PuntoLejano, conRelojCerca ? conReloj.PuntoCercano : conReloj.PuntoLejano, origen);

            if(hover){
                Handles.color = Color.yellow;
                float tam = .25f * HandleUtility.GetHandleSize(origen);
                Handles.DrawLine(conReloj.PuntoCercano - new Vector2(-1, -1) * tam, conReloj.PuntoCercano - new Vector2(1, 1) * tam);
                Handles.DrawLine(conReloj.PuntoCercano - new Vector2(1, -1) * tam, conReloj.PuntoCercano - new Vector2(-1, 1) * tam);
                Handles.DrawLine(contraReloj.PuntoCercano - new Vector2(-1, -1) * tam, contraReloj.PuntoCercano - new Vector2(1, 1) * tam);
                Handles.DrawLine(contraReloj.PuntoCercano - new Vector2(1, -1) * tam, contraReloj.PuntoCercano - new Vector2(-1, 1) * tam);
                Handles.DrawLine(conReloj.PuntoLejano - new Vector2(-1, 0) * tam, conReloj.PuntoLejano - new Vector2(1, 0) * tam);
                Handles.DrawLine(conReloj.PuntoLejano - new Vector2(0, -1) * tam, conReloj.PuntoLejano - new Vector2(0, 1) * tam);
                Handles.DrawLine(contraReloj.PuntoLejano - new Vector2(-1, 0) * tam, contraReloj.PuntoLejano - new Vector2(1, 0) * tam);
                Handles.DrawLine(contraReloj.PuntoLejano - new Vector2(0, -1) * tam, contraReloj.PuntoLejano - new Vector2(0, 1) * tam);

                Handles.color = Color.green;
                Handles.DrawLine(conReloj.PuntoCercano, contraReloj.PuntoLejano);
                Handles.DrawLine(contraReloj.PuntoCercano, conReloj.PuntoLejano);
            }
        }

        
    }

    void ConstruirHacesDeLuz()
    {
        hacesDeLuz.Clear();
        destinosDeRayos.Clear();
        foreach (var circulos in colliders.Values)
        {
            foreach (var circulo in circulos)
            {
                destinosDeRayos.Add(new VectorDeLuz(circulo, transform.position, radio));
                if (circulo.radio > 0f)
                {
                    destinosDeRayos.Add(destinosDeRayos[destinosDeRayos.Count - 1].horizonteAlfa);
                    destinosDeRayos.Add(destinosDeRayos[destinosDeRayos.Count - 2].horizonteBeta);
                }
            }
        }
        if (destinosDeRayos.Count > 0)
        {
            destinosDeRayos.Sort();
            destinosDeRayos.Add(destinosDeRayos[0]);
            for (int i = 0; i < destinosDeRayos.Count - 1; i++)
            {
                hacesDeLuz.Add(new HazDeLuz(destinosDeRayos[i], destinosDeRayos[i + 1]));
            }
        }
    }

    void ColectarColliders()
    {
        cantColliders = Collider.OverlapCollider(filtro, collidersTocados);
        if (cantColliders == collidersTocados.Length) Debug.Log("(cantColliders == collidersTocados.Length)");
    }
    void ColectarCirculos()
    {
        colliders.Clear();
        for (int i=0; i<cantColliders; i++)
        {
            var coll = collidersTocados[i];
            if (coll.GetType() == typeof(BoxCollider2D)) GenerarCirculosBox((BoxCollider2D)coll);
            else if (coll.GetType() == typeof(CircleCollider2D)) GenerarCirculosCircle((CircleCollider2D)coll);
            else if (coll.GetType() == typeof(CapsuleCollider2D)) GenerarCirculosCapsule((CapsuleCollider2D)coll);
            else if (coll.GetType() == typeof(EdgeCollider2D)) GenerarCirculosEdge((EdgeCollider2D)coll);
            else if (coll.GetType() == typeof(PolygonCollider2D)) GenerarCirculosPolygon((PolygonCollider2D)coll);
            else if (coll.GetType() == typeof(CompositeCollider2D)) GenerarCirculosComposite((CompositeCollider2D)coll);
        }
    }
    void GenerarCirculosBox(BoxCollider2D coll)
    {
        if (!coll.enabled) return;
        if (coll.usedByComposite) return;
        CirculoVertice[] vertices =new CirculoVertice[4];
        vertices[0]=(new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
        vertices[1] = (new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
        vertices[2] = (new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
        vertices[3] = (new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f), radio = coll.edgeRadius, padre = coll });
        colliders.Add(coll, vertices);
    }
    void GenerarCirculosCircle(CircleCollider2D coll)
    {
        if (!coll.enabled) return;
        if (coll.usedByComposite) return;
        CirculoVertice[] vertices = new CirculoVertice[1];
        vertices[0]=(new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset), radio = coll.transform.TransformVector(Vector3.right).magnitude * coll.radius, padre = coll });
        colliders.Add(coll, vertices);
    }
    void GenerarCirculosCapsule(CapsuleCollider2D coll)
    {
        if (!coll.enabled) return;
        if (coll.usedByComposite) return;
        List<CirculoVertice> vertices = new List<CirculoVertice>();
        Vector2 tamDeforme = coll.transform.TransformVector(coll.size);
        if ((coll.direction == CapsuleDirection2D.Horizontal && tamDeforme.x <= tamDeforme.y)
            || (coll.direction == CapsuleDirection2D.Vertical && tamDeforme.y <= tamDeforme.x))
        {
            vertices.Add(new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset), radio = Mathf.Max(tamDeforme.x, tamDeforme.y), padre = coll });
        }
        else
        {
            float radio = Mathf.Min(tamDeforme.x, tamDeforme.y);
            Vector3 offset = Vector3.zero;
            if (coll.direction == CapsuleDirection2D.Horizontal) offset = coll.transform.right * (tamDeforme.x - radio) * .5f;
            else offset = coll.transform.up * (tamDeforme.y - radio) * .5f;
            vertices.Add(new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset) + offset, radio = radio, padre = coll });
            vertices.Add(new CirculoVertice() { pos = coll.transform.TransformPoint(coll.offset) - offset, radio = radio, padre = coll });
        }
        colliders.Add(coll, vertices.ToArray());
    }
    void GenerarCirculosEdge(EdgeCollider2D coll)
    {
        if (!coll.enabled) return;
        if (coll.usedByComposite) return;
        CirculoVertice[] vertices = new CirculoVertice[coll.pointCount];
        int count = 0;
        foreach (var p in coll.points)
        {
            vertices[count++]=(new CirculoVertice() { pos = coll.transform.TransformPoint(p + coll.offset), radio = coll.edgeRadius, padre = coll });
        }
        colliders.Add(coll, vertices);
    }
    void GenerarCirculosPolygon(PolygonCollider2D coll)
    {
        if (!coll.enabled) return;
        if (coll.usedByComposite) return;
        CirculoVertice[] vertices = new CirculoVertice[coll.GetTotalPointCount()];
        int count = 0;
        foreach (var p in coll.points)
        {
            vertices[count++]=(new CirculoVertice() { pos = coll.transform.TransformPoint(p + coll.offset), radio = 0f, padre = coll });
        }
        colliders.Add(coll, vertices);
    }
    void GenerarCirculosComposite(CompositeCollider2D coll)
    {
        if (!coll.enabled) return;
        List<CirculoVertice> vertices = new List<CirculoVertice>();
        float radio = coll.geometryType == CompositeCollider2D.GeometryType.Outlines ? coll.edgeRadius : 0f;
        for (int i = 0; i < coll.pathCount; i++)
        {
            Vector2[] path = new Vector2[coll.GetPathPointCount(i)];
            coll.GetPath(i, path);
            foreach (var p in path)
            {
                vertices.Add(new CirculoVertice() { pos = coll.transform.TransformDirection(p) + coll.transform.position, radio = radio, padre = coll });
            }
        }
        colliders.Add(coll, vertices.ToArray());
    }
}
