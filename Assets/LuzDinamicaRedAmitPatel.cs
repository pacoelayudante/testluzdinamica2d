//Based on the algorithm exposed by Amit Patel (Red Blob Games)
// @ https://www.redblobgames.com/articles/visibility/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngineInternal;

public class LuzDinamicaRedAmitPatel : MonoBehaviour {
    public static Dictionary<Collider2D, SeguimientoCollider2D> collidersRelevantes = new Dictionary<Collider2D, SeguimientoCollider2D>();

    public static void Interesar(LuzDinamicaRedAmitPatel luz, Collider2D collider)
    {
        if (!collidersRelevantes.ContainsKey(collider))
        {
            collidersRelevantes.Add(collider, new SeguimientoCollider2D(luz, collider));
        }
    }
    public static void PerderInteres(LuzDinamicaRedAmitPatel luz, Collider2D collider)
    {
        if (!collidersRelevantes.ContainsKey(collider)) Debug.LogError(string.Format("PerderInteres({0}, {1}) -> !collidersRelevantes.ContainsKey({1})",luz,collider));
        if (!collidersRelevantes[collider].PerderInteres(luz))
        {
            collidersRelevantes[collider].Destruir();
            collidersRelevantes.Remove(collider);
        }
    }

    public class SeguimientoCollider2D
    {
        Collider2D colliderOriginal;
        ProtoForma2D protoForma;
        List<LuzDinamicaRedAmitPatel> lucesInteresadas = new List<LuzDinamicaRedAmitPatel>();
        List<ProtoSegmento2D> protoSegmento = new List<ProtoSegmento2D>();
        public bool HayInteres
        {
            get
            {
                for (int i = lucesInteresadas.Count - 1; i >= 0; i--)
                {
                    if (lucesInteresadas[i]) return true;
                    else lucesInteresadas.RemoveAt(i);
                }
                return false;
            }
        }
        public SeguimientoCollider2D(LuzDinamicaRedAmitPatel luz, Collider2D colliderOriginal)
        {
            lucesInteresadas.Add(luz);
            this.colliderOriginal = colliderOriginal;
            if (colliderOriginal.GetType() == typeof(BoxCollider2D)) protoForma = new ProtoFormaBox2D(colliderOriginal as BoxCollider2D);
        }
        public bool PerderInteres(LuzDinamicaRedAmitPatel luz)
        {
            lucesInteresadas.Remove(luz);
            return HayInteres;
        }

        public void Destruir()
        {

        }
    }

    public class ProtoSegmento2D
    {
        ProtoVertice2D inicio, fin;
    }
    public class ProtoVertice2D
    {
        public Transform transform;
        public Vector2 posLocal,posGlobal;
        public float radio;
        public ProtoVertice2D(Transform transform, float x, float y, float radio = 0f)
        {
            this.transform = transform;
            Actualizar(x, y);
            this.radio = radio;
        }
        public void Actualizar(float x, float y)
        {
            posLocal.Set(x, y);
            posGlobal = transform.TransformPoint(posLocal);
        }
    }

    public abstract class ProtoForma2D { }
    public class ProtoFormaBox2D:ProtoForma2D
    {
        BoxCollider2D collider;
        ProtoVertice2D abaizq, arrizq, arrder, abader;
        public ProtoFormaBox2D(BoxCollider2D collider)
        {
            this.collider = collider;
            abaizq = new ProtoVertice2D(collider.transform, collider.offset.x - collider.size.x, collider.offset.y - collider.size.y);
            arrizq = new ProtoVertice2D(collider.transform, collider.offset.x - collider.size.x, collider.offset.y + collider.size.y);
            arrder = new ProtoVertice2D(collider.transform, collider.offset.x + collider.size.x, collider.offset.y + collider.size.y);
            abader = new ProtoVertice2D(collider.transform, collider.offset.x + collider.size.x, collider.offset.y - collider.size.y);
        }
        public void Actualizar()
        {
            abaizq.Actualizar(collider.offset.x - collider.size.x, collider.offset.y - collider.size.y);
            abaizq.Actualizar(collider.offset.x - collider.size.x, collider.offset.y - collider.size.y);
            abaizq.Actualizar(collider.offset.x - collider.size.x, collider.offset.y - collider.size.y);
            abaizq.Actualizar(collider.offset.x - collider.size.x, collider.offset.y - collider.size.y);
        }
    }
}
