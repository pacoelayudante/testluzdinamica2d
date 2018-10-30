using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

[RequireComponent(typeof(CircleCollider2D))]
//[RequireComponent(typeof(MeshFilter))]
//[RequireComponent(typeof(MeshRenderer))]
[ExecuteInEditMode]
public class Luz2D : MonoBehaviour
{
    static float epsilonMenor = .001f;
    static float epsilonMayor = .999f;
    Dictionary<Collider2D, CirculoVertice[]> colliders = new Dictionary<Collider2D, CirculoVertice[]>();
    List<VectorDeLuz> destinosDeRayos = new List<VectorDeLuz>();
    List<HazDeLuz> hacesDeLuz = new List<HazDeLuz>();

    public ContactFilter2D filtro;
    [SerializeField]
    float radio = 5f;
    [Range(0f,180f)]
    [SerializeField]
    float aperturaDeAngulo = 180f;
    public Color color = Color.white;
    [Range(1f,120f)]
    [SerializeField]
    float maximaAperturaHaz = 45f;
    public Material material;
     
    Collider2D[] collidersTocados = new Collider2D[500];
    int cantColliders = 0;

    new CircleCollider2D collider;
    CircleCollider2D Collider
    { get { return collider ? collider : collider = GetComponent<CircleCollider2D>(); } }
    Mesh mesh;

    private void Update()
    {
        ConstruirHacesDeLuz();
        GenerarMalla();
        if (mesh)
        {
            Graphics.DrawMesh(mesh, Matrix4x4.identity, material, gameObject.layer);
        }
    }

    private void OnTriggerEnter2D(Collider2D coll)
    {
        if (colliders.ContainsKey(coll)) return;
        if (coll.GetType() == typeof(BoxCollider2D)) GenerarCirculosBox((BoxCollider2D)coll);
        else if (coll.GetType() == typeof(CircleCollider2D)) GenerarCirculosCircle((CircleCollider2D)coll);
        else if (coll.GetType() == typeof(CapsuleCollider2D)) GenerarCirculosCapsule((CapsuleCollider2D)coll);
        else if (coll.GetType() == typeof(EdgeCollider2D)) GenerarCirculosEdge((EdgeCollider2D)coll);
        else if (coll.GetType() == typeof(PolygonCollider2D)) GenerarCirculosPolygon((PolygonCollider2D)coll);
        else if (coll.GetType() == typeof(CompositeCollider2D)) GenerarCirculosComposite((CompositeCollider2D)coll);
    }
    private void OnTriggerExit2D(Collider2D collision)
    {
        colliders.Remove(collision);
    }

    private void OnValidate()
    {
#if UNITY_EDITOR
        Undo.RecordObject(Collider, "validando collider luz");
#endif
        Collider.isTrigger = true;
        Collider.radius = radio;
        Collider.offset = Vector2.zero;
        if (maximaAperturaHaz < 0f) maximaAperturaHaz *= -1;
        if (aperturaDeAngulo < 0f) aperturaDeAngulo = 0f;
    }

    class CirculoVertice
    {
        public Vector2 Pos
        {
            get
            {
                return actualizador(puntoReferencia, padre);
            }
        }
        public Vector2 puntoReferencia;
        public float radio;
        public Collider2D padre;
        public System.Func<Vector2,Collider2D, Vector2> actualizador;
        public CirculoVertice()
        {
            if(actualizador==null)actualizador = ActualizadorDefault;
        }
        Vector2 ActualizadorDefault(Vector2 p, Collider2D col)
        {
            return Vector2.zero;
        }
        /*public Vector2 OffsetRayo(Vector2 desde, float signo)
        {
            if (radio == 0f) return Vector2.zero;
            float d = (pos - desde).magnitude;
            float h = Mathf.Sqrt(d * d - radio * radio) * radio / d;
            float d_ = Mathf.Sqrt(radio * radio - h * h);
            return Vector2.Perpendicular(desde - pos).normalized * h * signo + (desde - pos).normalized * d_;
        }*/
#if UNITY_EDITOR
        public void OnDrawGizmos()
        {
            if (radio == 0f)
            {
                Gizmos.DrawWireCube(Pos, HandleUtility.GetHandleSize(Pos) * .1f*Vector2.one);
            }
            else
            {
                Handles.DrawWireDisc(Pos, Vector3.forward, radio);
            }
        }
#endif
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        Handles.color = Gizmos.color = Color.black;
        Handles.DrawWireArc(transform.position, Vector3.forward, transform.TransformDirection(Mathf.Cos(-aperturaDeAngulo * Mathf.Deg2Rad), Mathf.Sin(-aperturaDeAngulo * Mathf.Deg2Rad),0f),aperturaDeAngulo*2f, radio);
        if (aperturaDeAngulo < 180f)
        {
            Gizmos.DrawRay(transform.position, transform.TransformDirection(Mathf.Cos(-aperturaDeAngulo * Mathf.Deg2Rad), Mathf.Sin(-aperturaDeAngulo * Mathf.Deg2Rad), 0f) * radio);
            Gizmos.DrawRay(transform.position, transform.TransformDirection(Mathf.Cos(aperturaDeAngulo * Mathf.Deg2Rad), Mathf.Sin(aperturaDeAngulo * Mathf.Deg2Rad), 0f) * radio);
        }
        Gizmos.DrawSphere(transform.position, HandleUtility.GetHandleSize(transform.position) * .05f);
        if (!EditorApplication.isPlaying && !EditorApplication.isPlayingOrWillChangePlaymode && Selection.Contains(gameObject))
        {
            ColectarColliders();
            ColectarCirculos();
            ConstruirHacesDeLuz();
        }
        Handles.color = Gizmos.color = Color.magenta*.75f;
        foreach (var circs in colliders.Values)
        {
            foreach(var circulo in circs)
            {
                circulo.OnDrawGizmos();
            }
        }
            foreach (var haz in hacesDeLuz)
            {
                haz.Gizmo(mesh==null || !enabled);
            }
    }
#endif

    class VectorDeLuz : System.IComparable<VectorDeLuz>
    {
        public bool enRangoAngular = true;
        public bool rayoInterrumpido, toqueVerticeClave, esHorizonte;
        public float angulo;
        public float anguloRelativo;
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

        public VectorDeLuz(CirculoVertice referencia, Vector2 desde, ContactFilter2D filtro, float radio = 0f, float umbralAngulo = 180f, Vector2 referenciaAngulo = new Vector2())
        {
            Actualizar(referencia, desde, filtro, radio, umbralAngulo, referenciaAngulo);
        }
        public void Actualizar(CirculoVertice referencia, Vector2 desde, ContactFilter2D filtro, float radio = 0f, float umbralAngulo = 180f, Vector2 referenciaAngulo = new Vector2()) {
            origen = desde;
            puntoDestino = referencia.Pos;
            colliderPadre = referencia.padre;
            puntoDestinoRelativo = puntoDestino - origen;
            distancia = puntoDestinoRelativo.magnitude;
            distanciaSq = distancia * distancia;
            anguloRelativo = angulo = Mathf.Atan2(-puntoDestinoRelativo.y, puntoDestinoRelativo.x);
            rayo = new Ray2D(origen, puntoDestinoRelativo.normalized);

            if (referencia.radio > 0f)
            {
                float r2 = referencia.radio * referencia.radio;
                float h = Mathf.Sqrt(distanciaSq - r2) * referencia.radio / distancia;
                float d_ = Mathf.Sqrt(r2 - h * h);

                Vector2 offsetPerp = Vector2.Perpendicular(puntoDestinoRelativo).normalized * h;
                Vector2 offsetParal = (puntoDestinoRelativo).normalized * d_;

                horizonteAlfa = new VectorDeLuz(this, -offsetPerp, -offsetParal,umbralAngulo,referenciaAngulo);
                horizonteBeta = new VectorDeLuz(this, offsetPerp, -offsetParal, umbralAngulo, referenciaAngulo);

                horizonteAlfa.CalcularRayo(filtro, radio);
                horizonteBeta.CalcularRayo(filtro, radio);
            }
            if (umbralAngulo < 180f)
            {
                anguloRelativo = Vector2.SignedAngle(puntoDestinoRelativo, referenciaAngulo);
                enRangoAngular = anguloRelativo < umbralAngulo && anguloRelativo > -umbralAngulo;
                anguloRelativo *= Mathf.Deg2Rad;
            }
            else enRangoAngular = true;
            CalcularRayo(filtro,radio);
        }
        public VectorDeLuz(VectorDeLuz original, Vector2 offsetPerp, Vector2 offsetParal, float umbralAngulo = 180f, Vector2 referenciaAngulo = new Vector2())
        {
            Actualizar(original, offsetPerp, offsetParal, umbralAngulo, referenciaAngulo);
        }
        public void Actualizar(VectorDeLuz original, Vector2 offsetPerp, Vector2 offsetParal, float umbralAngulo = 180f, Vector2 referenciaAngulo = new Vector2()) {
            esHorizonte = true;
            origen = original.origen;
            colliderPadre = original.colliderPadre;
            puntoDestino = original.puntoDestino + offsetPerp + offsetParal;
            puntoDestinoRelativo = original.puntoDestinoRelativo + offsetPerp + offsetParal;
            rayo = new Ray2D(origen, puntoDestinoRelativo.normalized);
            anguloRelativo = angulo = Mathf.Atan2(-puntoDestinoRelativo.y, puntoDestinoRelativo.x);
            distancia = puntoDestinoRelativo.magnitude;
            distanciaSq = distancia * distancia;
            if (umbralAngulo < 180f)
            {
                anguloRelativo = Vector2.SignedAngle(puntoDestinoRelativo, referenciaAngulo);
                enRangoAngular = anguloRelativo < umbralAngulo && anguloRelativo > -umbralAngulo;
                anguloRelativo *= Mathf.Deg2Rad;
            }
            else enRangoAngular = true;
        }
        public VectorDeLuz(float anguloRadianes, Vector2 desde, ContactFilter2D filtro, float radio = 0f, float umbralAngulo = 180f, Vector2 referenciaAngulo = new Vector2())
        {
            Actualizar(anguloRadianes, desde, filtro, radio, umbralAngulo , referenciaAngulo );
        }
        public void Actualizar(float anguloRadianes, Vector2 desde, ContactFilter2D filtro, float radio = 0f, float umbralAngulo = 180f, Vector2 referenciaAngulo = new Vector2()) {
            anguloRelativo = this.angulo = anguloRadianes;
            origen = desde;
            rayo = new Ray2D(origen, new Vector2(Mathf.Cos(anguloRadianes),-Mathf.Sin(anguloRadianes)));
            distancia = radio*1.1f;
            puntoDestino = rayo.GetPoint(distancia);
            puntoDestinoRelativo = rayo.direction * distancia;
            distanciaSq = distancia * distancia;
            if (umbralAngulo < 180f)
            {
                anguloRelativo = Vector2.SignedAngle(puntoDestinoRelativo, referenciaAngulo)* Mathf.Deg2Rad;
            }
            else enRangoAngular = true;
            CalcularRayo(filtro, radio);
        }

        public void CalcularRayo(ContactFilter2D filtro, float radio=0)
        {
            if (!enRangoAngular) return;
            cantHits = Physics2D.Raycast(rayo.origin, rayo.direction, filtro, hits, radio);      
            if (cantHits > 0)
            {
                if (hits[0].distance < distancia * epsilonMayor)
                {//un chanqui, para casos limite (si lo hacemos sin epsilon, a veces tira interrumpido por si mismo)
                    rayoInterrumpido = true;
                }
                else
                {
                    if (hits[0].distance * epsilonMayor < distancia && hits[0].collider == colliderPadre && colliderPadre)
                    {//choco con vertice clave (lo mismo, el chanqui) en otro caso choco con lo siguiente
                        //si no tiene collider padre es porque es rayo generado para resolucion angular
                        toqueVerticeClave = true;
                        if (!esHorizonte && colliderPadre.OverlapPoint(rayo.GetPoint(hits[0].distance / epsilonMayor)))
                        {//vertice clave igual es como un coso grueso y en realidad interrumpe (osea estamos dentro)
                            rayoInterrumpido = true;
                            cantHits = 1;
                            //por razones, como se la estamos dando de lleno a un vertice vamos a chamuyar la normal
                            hits[0].normal = -rayo.direction;
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
                    }
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
            Gizmos.DrawLine(puntoDestino, origen);
        }
        public int CompareTo(VectorDeLuz otro)
        {
            return anguloRelativo.CompareTo(otro.anguloRelativo);
        }
    }

    class HazDeLuz
    {
        public Vector3 ConReloj
        { get { return conRelojCerca ? conReloj.PuntoCercano : conReloj.PuntoLejano; } }
        public Vector3 ContraReloj
        { get { return contraRelojCerca ? contraReloj.PuntoCercano : contraReloj.PuntoLejano; } }
        public bool TieneInterseccion { get { return conInterseccion; } }
        public Vector3 Interseccion { get { return puntoInterseccion; } }

        bool hover;

        Vector3 origen;
        VectorDeLuz conReloj, contraReloj;

        Vector2 frente, puntoInterseccion;

        int cantHits = 0;
        float radio;
        RaycastHit2D[] hitsRayoInterno = new RaycastHit2D[1];

        bool conRelojCerca, contraRelojCerca, conInterseccion;
        float radioFrente, radioCentro;
        public HazDeLuz(ContactFilter2D filtro, VectorDeLuz conReloj, VectorDeLuz contraReloj, float radio = 0f)
        {
            this.radio = radio;
            this.conReloj = conReloj;
            this.contraReloj = contraReloj;
            origen = conReloj.origen;

           //hover = GuazuExtender.PuntoEnConoInfinito(origen, contraReloj.puntoDestino, conReloj.puntoDestino, HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f));//GuazuExtender.PuntoEnTriangulo(conReloj.puntoDestino, contraReloj.puntoDestino, origen, HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f));

            EstablecerForma(filtro);
        }

        void EstablecerForma(ContactFilter2D filtro)
        {
            contraRelojCerca = contraReloj.rayoInterrumpido;
            conRelojCerca = conReloj.rayoInterrumpido;

            if (conReloj.rayoInterrumpido && contraReloj.rayoInterrumpido)
            {//ambos chocaron con algo antes de alcanzar final
                if (conReloj.hits[0].normal != contraReloj.hits[0].normal || conReloj.toqueVerticeClave || contraReloj.toqueVerticeClave)
                {//Nos fijamos si NO son paralelos, porque en ese caso buscamos donde se cruzan para hacer bien la forma. Y si chocamos con algun vertice clave la normal la chamuyamos
                    conInterseccion = true;
                    var contraPerp = Vector2.Perpendicular(contraReloj.hits[0].normal);
                    var conPerp = Vector2.Perpendicular(conReloj.hits[0].normal);
                    if (contraReloj.toqueVerticeClave) contraPerp = (conReloj.PuntoLejano - contraReloj.PuntoCercano);
                    if (conReloj.toqueVerticeClave) conPerp = (contraReloj.PuntoLejano - conReloj.PuntoCercano);
                    //la interseccion es el punto donde se cruzan las tangentes (perpendicular a las normales) de los Hits
                    if (GuazuExtender.LineIntersection(contraReloj.PuntoCercano, contraReloj.PuntoCercano + contraPerp, conReloj.PuntoCercano, conReloj.PuntoCercano+conPerp, ref puntoInterseccion))
                    {//y ahora chequeamos que el punto este dentro del haz, porque... eso significa que la superficie es circular o algo asi y esto no sirve
                        if (!GuazuExtender.PuntoEnConoInfinito(origen, contraReloj.rayo.GetPoint(1), conReloj.rayo.GetPoint(1), puntoInterseccion))
                        {
                            conInterseccion = false;
                        }
                    }//Y sino entonces es el punto medio entre ambos hits y ya
                    else conInterseccion = false;                    

                    if (hover)
                    {
                        Handles.color = (Color.magenta + Color.red) * .5f;
                        Handles.DrawDottedLine(contraReloj.PuntoCercano - contraPerp.normalized, contraReloj.PuntoCercano + contraPerp.normalized, HandleUtility.GetHandleSize(contraReloj.PuntoCercano) * 5f);
                        Handles.DrawDottedLine(conReloj.PuntoCercano - conPerp.normalized, conReloj.PuntoCercano + conPerp.normalized, HandleUtility.GetHandleSize(conReloj.PuntoCercano) * 5f);
                    }

                }
            }
            else
            {
                puntoInterseccion = (conReloj.PuntoCercano + contraReloj.PuntoCercano) / 2f;
                Collider2D[] overlappedColliders = new Collider2D[1];
                cantHits = Physics2D.OverlapCircle(puntoInterseccion, epsilonMenor, filtro, overlappedColliders);
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
                        cantHits = Physics2D.OverlapCircle((puntoInterseccion + contraReloj.PuntoCercano) / 2f, epsilonMenor, filtro, overlappedColliders);
                        if (cantHits > 0)
                        {
                            contraRelojCerca = true;
                        }
                        cantHits = Physics2D.OverlapCircle((puntoInterseccion + conReloj.PuntoCercano) / 2f, .001f, filtro, overlappedColliders);
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
                {//en este caso hay un rayo que roza y sigue pero el otro rayo es interrumpido, ademas ya sabemos que no chocaron por delante, 
                    if (contraReloj.hits[0].normal != conReloj.hits[0].normal || (contraReloj.toqueVerticeClave && contraReloj.rayoInterrumpido) || (conReloj.toqueVerticeClave && conReloj.rayoInterrumpido))
                    {//si las normales son diferentes, entonces
                        //hacemos lo mismo que si fuesen dos rayos interrumpidos, pero el que roza tomamos el choque de mas alla del vertice clave (que igual esta seteado como hits[0]) asique lo mismo pero en vez de usar punto cercano es hits[0] segurin
                        conInterseccion = true;
                        bool chequearCono = true;
                        var contraPerp = Vector2.Perpendicular(contraReloj.hits[0].normal);
                        var conPerp = Vector2.Perpendicular(conReloj.hits[0].normal);
                        if (contraReloj.toqueVerticeClave && contraReloj.rayoInterrumpido)
                        {
                            contraPerp = contraReloj.rayo.direction;
                            chequearCono = false;
                        }
                        if (conReloj.toqueVerticeClave && conReloj.rayoInterrumpido)
                        {
                            conPerp = conReloj.rayo.direction;
                            chequearCono = false;
                        }
                        if (GuazuExtender.LineIntersection(contraReloj.hits[0].point, contraReloj.hits[0].point + contraPerp, conReloj.hits[0].point, conReloj.hits[0].point + conPerp, ref puntoInterseccion))
                        {
                            if (chequearCono)
                            {//no se chequea el cono cuando pasa lo que mas arriba dice porque no, cosas
                                if (!GuazuExtender.PuntoEnConoInfinito(origen, contraReloj.rayo.GetPoint(1), conReloj.rayo.GetPoint(1), puntoInterseccion))
                                {
                                    conInterseccion = false;
                                }
                            }
                        }
                        else conInterseccion = false;
                        if (hover)
                        {
                            Handles.color = (Color.magenta + Color.red) * .5f;
                            Handles.DrawDottedLine(contraReloj.hits[0].point - contraPerp.normalized, contraReloj.hits[0].point + contraPerp.normalized, HandleUtility.GetHandleSize(contraReloj.hits[0].point) * 5f);
                            Handles.DrawDottedLine(conReloj.hits[0].point - conPerp.normalized, conReloj.hits[0].point + conPerp.normalized, HandleUtility.GetHandleSize(conReloj.hits[0].point) * 5f);
                        }
                    }
                }
            }

            if (conInterseccion)
            {//vamos a forzar que la intereseccion se de dentro del radio del cosoooo
                if (Vector2.Distance(origen, puntoInterseccion) > radio) puntoInterseccion = Vector2.MoveTowards(origen, puntoInterseccion, radio);
            }
        }

        public void Gizmo(bool dibujarDibujo = true)
        {
            /*if(Event.current!=null)hover = GuazuExtender.PuntoEnTriangulo(conReloj.puntoDestino, contraReloj.puntoDestino, origen, HandleUtility.GUIPointToWorldRay(Event.current.mousePosition).GetPoint(1f));*/
            if (hover)
            {
                Gizmos.color = conReloj.rayoInterrumpido ? Color.red : Color.blue;
                if (conReloj.toqueVerticeClave) Gizmos.color += Color.green;
                conReloj.Gizmo();
                Gizmos.color = contraReloj.rayoInterrumpido ? Color.red : Color.blue;
                if (contraReloj.toqueVerticeClave) Gizmos.color += Color.green;
                contraReloj.Gizmo();

                Handles.color = (Color.magenta + Color.red) * .5f*(conInterseccion?1f:.5f);
                Handles.DrawWireDisc(puntoInterseccion, Vector3.forward, HandleUtility.GetHandleSize(puntoInterseccion) * .1f);
            }
            Handles.color = Gizmos.color = Color.Lerp(Color.clear, Color.white, .4f);

            if (dibujarDibujo)
            {
                if (conInterseccion)
                {
                    Handles.DrawAAConvexPolygon(contraRelojCerca ? contraReloj.PuntoCercano : contraReloj.PuntoLejano, puntoInterseccion, conRelojCerca ? conReloj.PuntoCercano : conReloj.PuntoLejano, origen);
                }
                else Handles.DrawAAConvexPolygon(contraRelojCerca ? contraReloj.PuntoCercano : contraReloj.PuntoLejano, conRelojCerca ? conReloj.PuntoCercano : conReloj.PuntoLejano, origen);
            }

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
        var referenciaAngular = 0f;
        if (aperturaDeAngulo < 180f) referenciaAngular = -Vector2.SignedAngle(Vector2.right,transform.right);
        foreach (var circulos in colliders.Values)
        {
            foreach (var circulo in circulos)
            {
                var vectorNuevo = new VectorDeLuz(circulo, transform.position, filtro, radio, aperturaDeAngulo, transform.right);
                if(vectorNuevo.enRangoAngular)destinosDeRayos.Add(vectorNuevo);
                if (circulo.radio > 0f)
                {
                    if (vectorNuevo.horizonteAlfa.enRangoAngular) destinosDeRayos.Add(vectorNuevo.horizonteAlfa);
                    if (vectorNuevo.horizonteBeta.enRangoAngular) destinosDeRayos.Add(vectorNuevo.horizonteBeta);
                }
            }
        }
        //if (destinosDeRayos.Count > 0)
        {
            destinosDeRayos.Sort();
            if (aperturaDeAngulo >= 180f)
            {
                if(destinosDeRayos.Count == 0) 
                {
                    referenciaAngular = -Vector2.SignedAngle(Vector2.right, transform.right)*Mathf.Deg2Rad;
                    destinosDeRayos.Add(new VectorDeLuz(referenciaAngular, transform.position, filtro, radio, aperturaDeAngulo, transform.right));
                    destinosDeRayos.Add(new VectorDeLuz(referenciaAngular+Mathf.PI*2f/3f, transform.position, filtro, radio, aperturaDeAngulo, transform.right));
                    destinosDeRayos.Add(new VectorDeLuz(referenciaAngular+Mathf.PI * 4f / 3f, transform.position, filtro, radio, aperturaDeAngulo, transform.right));
                }
                destinosDeRayos.Add(destinosDeRayos[0]);
            }
            else
            {
                destinosDeRayos.Add(new VectorDeLuz((referenciaAngular + aperturaDeAngulo) * Mathf.Deg2Rad, transform.position, filtro, radio, aperturaDeAngulo, transform.right));
                destinosDeRayos.Insert(0, new VectorDeLuz((referenciaAngular - aperturaDeAngulo) * Mathf.Deg2Rad, transform.position, filtro, radio, aperturaDeAngulo, transform.right));
                if (destinosDeRayos.Count == 2) destinosDeRayos.Insert(1, new VectorDeLuz(referenciaAngular* Mathf.Deg2Rad, transform.position, filtro, radio, aperturaDeAngulo, transform.right));
            }
            if (maximaAperturaHaz > 0f)
            {
                float diferenciaAngular = 0f;
                var offsetAngular = (Vector2.Angle(transform.right,Vector2.right)%(maximaAperturaHaz)-maximaAperturaHaz/2f) * Mathf.Deg2Rad;
                var maximaAperturaHazRadianes = maximaAperturaHaz * Mathf.Deg2Rad;
                for (int i = 0; i < destinosDeRayos.Count - 1; i++)
                {//aca chequeamos que los rayos de luz no tengan mucha diferencia angular entre si (para que el resultado no sea tan cuadrado en caso de pocos colliders en alguna region y un re fix cuando no hay al menos dos en cuadrantes opuestos
                    diferenciaAngular = destinosDeRayos[i + 1].anguloRelativo - destinosDeRayos[i].anguloRelativo;
                    if (diferenciaAngular > maximaAperturaHazRadianes)
                    {
                        var rayosExtras = Mathf.FloorToInt(diferenciaAngular / maximaAperturaHazRadianes);
                        var distEntreRayos = diferenciaAngular / (rayosExtras+1);
                        int indexOffset = 0;
                        for (int r = 0; r < rayosExtras; r++)
                        {
                            indexOffset++;
                            destinosDeRayos.Insert(i+indexOffset, new VectorDeLuz(destinosDeRayos[i].angulo + distEntreRayos * indexOffset+ offsetAngular, transform.position, filtro, radio));
                        }
                        if (i == 0) destinosDeRayos[0].anguloRelativo += Mathf.PI * 2f;
                        i += indexOffset;
                    }
                    else if (i == 0) destinosDeRayos[0].anguloRelativo += Mathf.PI * 2f;
                }
            }

            for (int i = 0; i < destinosDeRayos.Count - 1; i++)
            {
                hacesDeLuz.Add(new HazDeLuz(filtro,destinosDeRayos[i], destinosDeRayos[i + 1], radio));
            }
        }
    }
    
    void GenerarMalla()
    {
        if (hacesDeLuz.Count == 0) return;
        if (!mesh)
        {
            mesh = new Mesh();
        mesh.MarkDynamic();
        }

        List<Vector3> vertices = new List<Vector3>();
        List<Color> colores = new List<Color>();
        List<int> tris = new List<int>();
        vertices.Add( transform.position );
        colores.Add(color);
        foreach (var haz in hacesDeLuz)
        {
            vertices.Add(haz.ContraReloj);
            colores.Add(color);
            if (haz.TieneInterseccion)
            {
                vertices.Add(haz.Interseccion);
                colores.Add(color);
                tris.Add(vertices.Count - 1);
                tris.Add(0);
                tris.Add(vertices.Count - 2);
            }
            vertices.Add(haz.ConReloj);
            colores.Add(color);
            tris.Add(vertices.Count - 1);
            tris.Add(0);
            tris.Add(vertices.Count - 2);
        }

        mesh.Clear();
        mesh.SetVertices(vertices);
        mesh.SetTriangles(tris,0);
        mesh.SetColors(colores);
        mesh.RecalculateBounds();
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
        vertices[0]=(new CirculoVertice() { actualizador=ActualizadorTransPointConOffset, puntoReferencia= coll.size/2f/*pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f)*/, radio = coll.edgeRadius, padre = coll });
        vertices[1] = (new CirculoVertice() {
            actualizador = ActualizadorTransPointConOffset,
            puntoReferencia = new Vector2(-coll.size.x,coll.size.y)/2f/*pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y + coll.size.y / 2f, 0f)*/, radio = coll.edgeRadius, padre = coll });
        vertices[2] = (new CirculoVertice() {
            actualizador = ActualizadorTransPointConOffset,
            puntoReferencia = new Vector2(coll.size.x, -coll.size.y) / 2f/*pos = coll.transform.TransformPoint(coll.offset.x + coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f)*/, radio = coll.edgeRadius, padre = coll });
        vertices[3] = (new CirculoVertice() {
            actualizador = ActualizadorTransPointConOffset,
            puntoReferencia = -coll.size / 2f/*pos = coll.transform.TransformPoint(coll.offset.x - coll.size.x / 2f, coll.offset.y - coll.size.y / 2f, 0f)*/, radio = coll.edgeRadius, padre = coll });
        colliders.Add(coll, vertices);
    }
    void GenerarCirculosCircle(CircleCollider2D coll)
    {
        if (!coll.enabled) return;
        if (coll.usedByComposite) return;
        CirculoVertice[] vertices = new CirculoVertice[1];
        vertices[0]=(new CirculoVertice() { actualizador = ActualizadorTransPointConOffset, puntoReferencia = Vector2.zero/*pos = coll.transform.TransformPoint(coll.offset)*/, radio = coll.transform.TransformVector(Vector3.right).magnitude * coll.radius, padre = coll });
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
            vertices.Add(new CirculoVertice() { actualizador=ActualizadorTransPointConOffset, puntoReferencia=Vector2.zero/*pos = coll.transform.TransformPoint(coll.offset)*/, radio = Mathf.Max(tamDeforme.x, tamDeforme.y), padre = coll });
        }
        else
        {
            /*float radio = Mathf.Min(tamDeforme.x, tamDeforme.y);
            Vector3 offset = Vector3.zero;
            if (coll.direction == CapsuleDirection2D.Horizontal) offset = coll.transform.right * (tamDeforme.x - radio) * .5f;
            else offset = coll.transform.up * (tamDeforme.y - radio) * .5f;*/
            vertices.Add(new CirculoVertice() { actualizador= ActualizadorCapsule, puntoReferencia = Vector2.one/*pos = coll.transform.TransformPoint(coll.offset) + offset*/, radio = radio, padre = coll });
            vertices.Add(new CirculoVertice() { actualizador = ActualizadorCapsule, puntoReferencia = -Vector2.one/*pos = coll.transform.TransformPoint(coll.offset) - offset*/, radio = radio, padre = coll });
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
            vertices[count++]=(new CirculoVertice() { actualizador=ActualizadorTransPointConOffset, puntoReferencia=p/*pos = coll.transform.TransformPoint(p + coll.offset)*/, radio = coll.edgeRadius, padre = coll });
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
            vertices[count++]=(new CirculoVertice() { actualizador=ActualizadorTransPointConOffset, puntoReferencia=p/*pos = coll.transform.TransformPoint(p + coll.offset)*/, radio = 0f, padre = coll });
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
                vertices.Add(new CirculoVertice() { actualizador=ActualizadorTransDirectionMasCollPos, puntoReferencia=p/*pos = coll.transform.TransformDirection(p) + coll.transform.position*/, radio = radio, padre = coll });
            }
        }
        colliders.Add(coll, vertices.ToArray());
    }

    static Vector2 ActualizadorCapsule(Vector2 punto, Collider2D collider)
    {
        var capsule = collider as CapsuleCollider2D;
        Vector2 tamDeforme = capsule.transform.TransformVector(capsule.size);
        float radio = Mathf.Min(tamDeforme.x, tamDeforme.y);
        Vector2 offset = Vector2.zero;
        if (capsule.direction == CapsuleDirection2D.Horizontal) offset = capsule.transform.right * (tamDeforme.x - radio) * .5f;

        return (Vector2)collider.transform.TransformPoint(collider.offset)+ offset* punto.x;
    }
    static Vector2 ActualizadorTransPointConOffset(Vector2 punto, Collider2D collider)
    {
        return collider.transform.TransformPoint(punto + collider.offset);
    }
    static Vector2 ActualizadorTransDirectionMasCollPos(Vector2 punto, Collider2D collider)
    {
        return collider.transform.TransformDirection(punto) + collider.transform.position;
    }
}
