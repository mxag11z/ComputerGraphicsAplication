//	Program developed by
//	
//	Informatika Fakultatea
//	Euskal Herriko Unibertsitatea
//	http://www.ehu.eus/if
//
// to compile it: gcc dibujar-triangulos-y-objetos.c -lGL -lGLU -lglut
//
// 
//
/*Matriz de proyeccion variable global
 Model View variable global */

#include <GL/glut.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cargar-triangulo.h"
#include <stdbool.h>

typedef struct mlist
    {
    double m[16];
    struct mlist *hptr;
    } mlist;
    
    /*Objeto completo*/
typedef struct triobj
    {

    hiruki *triptr;
    //Numero de triangulos
    int num_triangles;
    mlist *mptr;
    struct triobj *hptr;
    } triobj;


// información de textura

extern int load_ppm(char *file, unsigned char **bufferptr, int *dimxptr, int * dimyptr);
unsigned char *bufferra;
int dimx,dimy,dimentsioa;
int flag_projection = 0;
int flag_backface = 0;

// Matriz de proyeccion en perspectiva parametros

double projectionMatrix[16]; // Matriz de proyección

double l = -1.0, r = 1.0;  
double b = -1.0, t = 1.0;   
double n = 1.0, f = 1000;   

//Matriz de proyeccion en paralelo parametros
double lpl = -1.0, rpl = 1.0;   
double bpl = -1.0, tpl = 1.0;   
double npl = -1.0, fpl = -1000.0;  

int modo_textura = 1;

int indexx;
hiruki *triangulosptr;
triobj *foptr;
triobj *sel_ptr;
int denak;
int lineak;
int objektuak;
int kamera;
char aldaketa;
int ald_lokala;
/*Cameras*/

triobj *camera_list = NULL;  // Lista de cámaras
triobj *active_camera = NULL;  // Cámara activa
char fitxiz[100];
double modelView[16];


// Función para crear la matriz de proyección en perspectiva
void set_projection_matrix() {



    if(flag_projection == 0){
    // Matriz de proyección en paralelo
    projectionMatrix[0] = (2.0)/(rpl - lpl);
    projectionMatrix[1] = 0.0;
    projectionMatrix[2] = 0.0;
    projectionMatrix[3] = 0.0;

    projectionMatrix[4] = 0.0;
    projectionMatrix[5] = (2.0)/(tpl - bpl);
    projectionMatrix[6] = 0.0;
    projectionMatrix[7] = 0.0;

    projectionMatrix[8] = 0.0;
    projectionMatrix[9] = 0.0;
    projectionMatrix[10] = 2/(npl-fpl);
    projectionMatrix[11] = 0.0;

    projectionMatrix[12] = -(rpl+lpl)/(rpl-lpl);
    projectionMatrix[13] = -(tpl+bpl)/(tpl-bpl);
    projectionMatrix[14] = -(fpl+npl)/(npl - fpl);
    projectionMatrix[15] = 1.0;
    }
    else{   // Matriz de proyección en perspectiva 

    projectionMatrix[0] = (2.0 * n)/(r - l);
    projectionMatrix[1] = 0.0;
    projectionMatrix[2] = 0.0;
    projectionMatrix[3] = 0.0;

    projectionMatrix[4] = 0.0;
    projectionMatrix[5] = (2.0 * n)/(t - b);
    projectionMatrix[6] = 0.0;
    projectionMatrix[7] = 0.0;

    projectionMatrix[8] = (r + l)/(r - l);
    projectionMatrix[9] = (t + b)/(t - b);
    projectionMatrix[10] = -(f + n)/(n-f);
    projectionMatrix[11] = -1.0;

    projectionMatrix[12] = 0.0;
    projectionMatrix[13] = 0.0;
    projectionMatrix[14] = -(2.0 * f * n)/(n-f);
    projectionMatrix[15] = 0.0;
    
    }
}





/*Se agrega la matriz al inicio de las matrices, sistema local*/
void objektuari_aldaketa_sartu_ezk(double m[16])
{
    //Reservamos la nueva matriz
    mlist *new_matrix = (mlist *)malloc(sizeof(mlist));
    memcpy(new_matrix->m, m, sizeof(double) * 16);
    new_matrix->hptr = sel_ptr->mptr;
    sel_ptr->mptr = new_matrix;
}


/*Se agrega la matriz al final de la lista de matrices, sistema global*/
void objektuari_aldaketa_sartu_esk(double m[16])
{
    mlist *new_matrix = (mlist *)malloc(sizeof(mlist));
    memcpy(new_matrix->m, m, sizeof(double) * 16);
    new_matrix->hptr = 0;

    if (sel_ptr->mptr == 0)
        sel_ptr->mptr = new_matrix;
    else
    {
        mlist *current = sel_ptr->mptr;
        while (current->hptr != 0)
            current = current->hptr;
        current->hptr = new_matrix;
    }
}


// Función para multiplicar matrices 4x4
void multiplicar_matrices(double *resultado, double *m1, double *m2) {
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            double sum = 0;
            for(int k = 0; k < 4; k++) {
                sum += m1[i + k*4] * m2[k + j*4];
            }
            resultado[i + j*4] = sum;
        }
    }
}

// Obtener matriz de cambio de sistema de referencia de la cámara
void obtener_CSR_partiendo_de_M(GLdouble* M, GLdouble* MCSR) {
    // Transpone la parte de rotación 
    MCSR[0] = M[0];  MCSR[4] = M[1];  MCSR[8] = M[2];   MCSR[12] = 0;
    MCSR[1] = M[4];  MCSR[5] = M[5];  MCSR[9] = M[6];   MCSR[13] = 0;
    MCSR[2] = M[8];  MCSR[6] = M[9];  MCSR[10] = M[10]; MCSR[14] = 0;
    MCSR[3] = 0;     MCSR[7] = 0;     MCSR[11] = 0;     MCSR[15] = 1;
    
    // Calcula los productos punto negativos
    double Ex = M[12], Ey = M[13], Ez = M[14];
    
    MCSR[12] = -(Ex * MCSR[0] + Ey * MCSR[1] + Ez * MCSR[2]);
    MCSR[13] = -(Ex * MCSR[4] + Ey * MCSR[5] + Ez * MCSR[6]);
    MCSR[14] = -(Ex * MCSR[8] + Ey * MCSR[9] + Ez * MCSR[10]);
}





// TODO
// debe devolver el pointer correspondiente a las coordenadas u y v
unsigned char* color_textura(float u, float v) {

    if (u < 0.0) {
        u = 0.0; 
    } else if (u > 1.0) {
        u = 1.0; 
    }
    
    if (v < 0.0) {
        v = 0.0; 
    } else if (v > 1.0) {
        v = 1.0; 
    }

    int buffer_size = dimx * dimy * 3;
    int x = (int)(u * (dimx - 1));
    int y = (int)((1 - v) * (dimy - 1));
    int index = (y * dimx + x) * 3; // Cada pixel tiene 3 bytes (RGB)
  
    return bufferra + index;
}


void dibujar_linea_z(float linea, float c1x, float c1z, float c1u, float c1v, float c2x, float c2z, float c2u, float c2v) {
    float xkoord, zkoord;
    float cx, cz, cu, cv;
    float u, v;
    int i, puntukop, puntukopz;
    unsigned char r, g, b;
    unsigned char *colorv;

    puntukop = (int)ceil(((c2x-c1x)*(float)dimentsioa)/2.0);
   
    if (puntukop > 0) {
        cx = (c2x - c1x) / (float)puntukop;
        cz = (c2z - c1z) / (float)puntukop;
        cu = (c2u - c1u) /(float)puntukop;
        cv = (c2v - c1v) / (float)puntukop;
    }
    else {
        cx = 0;
        cz = 0;
        cu = 0;
        cv = 0;
    }

    // Establecer el color antes de comenzar a dibujar
    if (!modo_textura) {
        glColor3f(0.0, 0.0, 0.0);  // Color negro
    }
    
    glBegin(GL_POINTS);
    
    for (i = 0, xkoord = c1x, zkoord = c1z, u = c1u, v = c1v; 
         i < puntukop; 
         i++, xkoord += cx, zkoord += cz, u += cu, v += cv) {
        
        if (modo_textura) {
            colorv = color_textura(u, v); 
            r = colorv[0];
            g = colorv[1];
            b = colorv[2];    
            glColor3ub(r, g, b);
        }
        glVertex3f(xkoord, linea, zkoord);
    }

    glEnd();
}



void print_matrizea(char *str)
{
int i;

printf("%s\n",str);
for (i = 0;i<4;i++)
   printf("%lf, %lf, %lf, %lf\n",sel_ptr->mptr->m[i*4],sel_ptr->mptr->m[i*4+1],sel_ptr->mptr->m[i*4+2],
                                 sel_ptr->mptr->m[i*4+3]);
}


// TODO (transform)
// supone que la cuarta coordenada es un 1

void mxp(punto *pptr, double m[16], punto p) {
    double x = p.x, y = p.y, z = p.z;
    
    // Calcular las coordenadas transformadas
    double xp = m[0] * x + m[4] * y + m[8] * z + m[12];
    double yp = m[1] * x + m[5] * y + m[9] * z + m[13];
    double zp = m[2] * x + m[6] * y + m[10] * z + m[14];
    double w = m[3] * x + m[7] * y + m[11] * z + m[15];

    // División perspectiva
    pptr->x = xp / w;
    pptr->y = yp / w;
    pptr->z = zp / w;
    pptr->u = p.u;
    pptr->v = p.v;
}

void calcular_modelView() {
    double temp[16];
    double mundo[16];
    
    
    for(int i = 0; i < 16; i++) {
        mundo[i] = (i % 5 == 0) ? 1.0 : 0.0;
    }
    
    // Obtener Mcamara⁻¹ y multiplicarla por Mmundo
    obtener_CSR_partiendo_de_M(active_camera->mptr->m, temp);
    multiplicar_matrices(modelView, temp, mundo);
}

void aplicar_transformaciones_objeto(punto *pptr, triobj *obj, punto p) {
    punto temp = p;
    punto resultado, resultado_proj;

    // Aplicar transformaciones del objeto
    mlist *current = obj->mptr;
    while (current != 0) {
        mxp(&resultado, current->m, temp);
        temp = resultado;
        current = current->hptr;
    }
    *pptr = temp;
}




void ordena(punto *p1ptr, punto *p2ptr, punto *p3ptr, punto **psptr, punto **pmptr,punto **piptr){

  if(p1ptr->y>p2ptr->y){
    *psptr = p1ptr;
    *piptr = p2ptr;
  }else{
    *psptr = p2ptr;
    *piptr = p1ptr;
  }
  if(p3ptr->y>(*psptr)->y){
    *pmptr = *psptr;
    *psptr = p3ptr;
  }else if(p3ptr->y<(*piptr)->y){
    *pmptr = *piptr;
    *piptr = p3ptr;
  }else *pmptr = p3ptr;


}

// Inicializar la cámara
void crear_nueva_camara(double x, double y, double z) {
    double Mcamara[16];
    // Matriz de la cámara con la posición especificada
    Mcamara[0] = 1;  Mcamara[4] = 0;  Mcamara[8] = 0;  Mcamara[12] = x;
    Mcamara[1] = 0;  Mcamara[5] = 1;  Mcamara[9] = 0;  Mcamara[13] = y;
    Mcamara[2] = 0;  Mcamara[6] = 0;  Mcamara[10] = 1; Mcamara[14] = z;
    Mcamara[3] = 0;  Mcamara[7] = 0;  Mcamara[11] = 0; Mcamara[15] = 1;

    // Crear y cargar la cámara como un objeto
    triobj *camera = (triobj *)malloc(sizeof(triobj));

    int retval = cargar_triangulos("camera.txt", &(camera->num_triangles), &(camera->triptr));
    
    if (retval != 1) {
        printf("Error al cargar camera.txt\n");
        free(camera);
        return;
    }

    // Inicializar la matriz de transformación de la cámara
    camera->mptr = (mlist *)malloc(sizeof(mlist));
    memcpy(camera->mptr->m, Mcamara, sizeof(double) * 16);
    camera->mptr->hptr = 0;
    
    // Si es la primera cámara (no hay lista)
    if (camera_list == NULL) {
        camera_list = camera;
        active_camera = camera;
        camera->hptr = NULL;
    } else {
        // Añadir a la lista de cámaras existente
        camera->hptr = camera_list;
        camera_list = camera;
    }

    printf("Cámara creada en posición (%.2f, %.2f, %.2f)\n", x, y, z);
}



void calcular_laterales(punto p1, punto p2, float* leftx, float* rightx, float* leftz, float* rightz, float* leftu, float* rightu, float* leftv, float* rightv)
{
    if (p1.x < p2.x) {
        *leftx = p1.x;
        *rightx = p2.x;
        *leftz = p1.z;
        *rightz = p2.z;
        *leftu = p1.u;
        *rightu = p2.u;
        *leftv = p1.v;
        *rightv = p2.v;
    } else {
        *leftx = p2.x;
        *rightx = p1.x;
        *leftz = p2.z;
        *rightz = p1.z;
        *leftu = p2.u;
        *rightu = p1.u;
        *leftv = p2.v;
        *rightv = p1.v;
    }
}

//BackFace
bool es_cara_trasera(punto p1, punto p2, punto p3) {
    // Calcular vectores del triángulo
    double v1x = p2.x - p1.x;
    double v1y = p2.y - p1.y;
    double v1z = p2.z - p1.z;
    
    double v2x = p3.x - p1.x;
    double v2y = p3.y - p1.y;
    double v2z = p3.z - p1.z;
    
    
    double nx = v1y*v2z - v1z*v2y;  
    double ny = v1z*v2x - v1x*v2z;  
    double nz = v1x*v2y - v1y*v2x;  
    
    double vx = -p1.x;
    double vy = -p1.y;
    double vz = -p1.z;
    
    double prod_escalar = vx*nx + vy*ny + vz*nz;
    
    if (prod_escalar >= 0) {
        return true;  // Es cara trasera
    }
    return false;  // Es cara frontal
}

void dibujar_triangulo(triobj *optr, int ti) {
    hiruki *tptr;
    float x1, y1, z1, u1, v1, x2, y2, z2, u2, v2, x3, y3, z3, u3, v3;
    float c1x, c1z, c1u, c1v, c2x, c2z, c2u, c2v;
    float y;
    float lerrotartea, cambio1, cambio1z, cambio1u, cambio1v, cambio2, cambio2z, cambio2u, cambio2v;
    float leftx, rightx, leftz, rightz, leftu, rightu, leftv, rightv;
    punto csi, csm, cmi, pinf;
    int lerrokop, i;
    punto p1, p2, p3;
    /*Inicializacion de punteros*/
    punto *p1ptr = &p1;
    punto *p2ptr = &p2;
    punto *p3ptr = &p3;
    punto *psptr;
    punto *pmptr;
    punto *piptr;

    if (ti >= optr->num_triangles) return;

    tptr = optr->triptr + ti;

    
    aplicar_transformaciones_objeto(&p1, optr, tptr->p1);
    aplicar_transformaciones_objeto(&p2, optr, tptr->p2);
    aplicar_transformaciones_objeto(&p3, optr, tptr->p3);

    mxp(&p1,modelView,p1);
    mxp(&p2,modelView,p2);
    mxp(&p3,modelView,p3);
    
    /*Si un punto se encuentra por detras de la camara no dibujar esa parte*/
     
    if (p1.z >= 0 || p2.z >= 0 || p3.z >= 0) {
        return;  
    }
    //Calculo de cara trasera


    if(flag_backface == 0){
    if (!es_cara_trasera(p1, p2, p3)) {
        return;  // No dibujar cara trasera
    }}


    mxp(&p1, projectionMatrix, p1);
    mxp(&p2, projectionMatrix, p2);
    mxp(&p3, projectionMatrix, p3);
   
    if (lineak == 1) {
        glBegin(GL_POLYGON);
        glVertex3d(p1.x, p1.y, p1.z);
        glVertex3d(p2.x, p2.y, p2.z);
        glVertex3d(p3.x, p3.y, p3.z);
        glEnd();
        return;
    }

    float cy = 2.0 / (float) dimentsioa; // Distancia entre las líneas horizontales
    /*Ordenando los puntos*/
    ordena(p1ptr, p2ptr, p3ptr, &psptr, &pmptr, &piptr);

    /*Calculo del numero de pasos*/
    int num_steps_s_m = (int)ceil(((psptr->y - pmptr->y) * (float)dimentsioa) / 2.0);
    int num_steps_s_i = (int)ceil(((psptr->y - piptr->y) * (float)dimentsioa) / 2.0);
    int num_steps_m_i = (int)ceil(((pmptr->y - piptr->y) * (float)dimentsioa) / 2.0);

    /*cambios en coordenadas x,z,u,v*/
    float cxsm = (pmptr->x - psptr->x) / (float)num_steps_s_m;
    float cxsi = (piptr->x - psptr->x) / (float)num_steps_s_i;
    float czsm = (pmptr->z - psptr->z) / (float)num_steps_s_m;
    float czsi = (piptr->z - psptr->z) / (float)num_steps_s_i;
    float cusm = (pmptr->u - psptr->u) / (float)num_steps_s_m;
    float cusi = (piptr->u - psptr->u) / (float)num_steps_s_i;
    float cvsm = (pmptr->v - psptr->v) / (float)num_steps_s_m;
    float cvsi = (piptr->v - psptr->v) / (float)num_steps_s_i;
    float cxmi = (piptr->x - pmptr->x) / (float)num_steps_m_i;
    float czmi = (piptr->z - pmptr->z) / (float)num_steps_m_i;
    float cumi = (piptr->u - pmptr->u) / (float)num_steps_m_i;
    float cvmi = (piptr->v - pmptr->v) / (float)num_steps_m_i;

    /*Debugging*/
    
  /*  printf("Punto superior: (%f, %f, %f, %f, %f)\n", psptr->x, psptr->y, psptr->z, psptr->u, psptr->v);
    printf("Punto medio: (%f, %f, %f, %f, %f)\n", pmptr->x, pmptr->y, pmptr->z, pmptr->u, pmptr->v);
    printf("Punto inferior: (%f, %f, %f, %f, %f)\n", piptr->x, piptr->y, piptr->z, piptr->u, piptr->v);*/

    /*Inicializacion de csm,csi,cmi*/
    csm = *psptr;
    csi = *psptr;
    cmi = *pmptr;
    pinf = *piptr;

    /*caso especial*/
    if (num_steps_s_m < 1) {
        calcular_laterales(csm, cmi, &leftx, &rightx, &leftz, &rightz, &leftu, &rightu, &leftv, &rightv);
        dibujar_linea_z(csm.y, leftx, leftz, leftu, leftv, rightx, rightz, rightu, rightv);
    } /*superior a mediano*/
    else {
        for (i = 0; i <= num_steps_s_m; i++) {
            csm.u = (csm.u < 0.0f) ? 0.0f : (csm.u > 1.0f) ? 1.0f : csm.u;
            csm.v = (csm.v < 0.0f) ? 0.0f : (csm.v > 1.0f) ? 1.0f : csm.v;
            csi.u = (csi.u < 0.0f) ? 0.0f : (csi.u > 1.0f) ? 1.0f : csi.u;
            csi.v = (csi.v < 0.0f) ? 0.0f : (csi.v > 1.0f) ? 1.0f : csi.v;
            calcular_laterales(csm, csi, &leftx, &rightx, &leftz, &rightz, &leftu, &rightu, &leftv, &rightv);
            dibujar_linea_z(csm.y, leftx, leftz, leftu, leftv, rightx, rightz, rightu, rightv);

            //printf("Corte superior - medio en v: %.8f, Corte superior inferior en v: %.8f\n", csm.v, csi.v);
            csm.y -= cy;
            csm.x += cxsm;
            csi.x += cxsi;
            csm.z += czsm;
            csi.z += czsi;
            csm.u += cusm;
            csi.u += cusi;
            csm.v += cvsm;
            csi.v += cvsi;
        }
    }
    /*Caso especial*/
    if (num_steps_m_i < 1) {
        calcular_laterales(cmi, pinf, &leftx, &rightx, &leftz, &rightz, &leftu, &rightu, &leftv, &rightv);
        dibujar_linea_z(cmi.y, leftx, leftz, leftu, leftv, rightx, rightz, rightu, rightv);
    } /*Mediano a inferior*/
    else {
        for (i = 0; i <= num_steps_m_i; i++) {
            cmi.u = (cmi.u < 0.0f) ? 0.0f : (cmi.u > 1.0f) ? 1.0f : cmi.u;
            cmi.v = (cmi.v < 0.0f) ? 0.0f : (cmi.v > 1.0f) ? 1.0f : cmi.v;
            csi.u = (csi.u < 0.0f) ? 0.0f : (csi.u > 1.0f) ? 1.0f : csi.u;
            csi.v = (csi.v < 0.0f) ? 0.0f : (csi.v > 1.0f) ? 1.0f : csi.v;
            calcular_laterales(cmi, csi, &leftx, &rightx, &leftz, &rightz, &leftu, &rightu, &leftv, &rightv);
            dibujar_linea_z(cmi.y, leftx, leftz, leftu, leftv, rightx, rightz, rightu, rightv);

            cmi.y -= cy;
            cmi.x += cxmi;
            csi.x += cxsi;
            cmi.z += czmi;
            csi.z += czsi;
            cmi.u += cumi;
            csi.u += cusi;
            cmi.v += cvmi;
            csi.v += cvsi;
        }
    }
}


static void marraztu(void) {
    float u,v;
    int i,j;
    triobj *auxptr;


    if (foptr == 0) return;

    if (objektuak == 1) glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
    else {
        if (denak == 0) glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
    }

    triangulosptr = sel_ptr->triptr;
    
    if (objektuak == 1) {
        if (denak == 1) {
            // Dibujar objetos normales
            printf("Dibujando objetos normales\n");
            for (auxptr = foptr; auxptr != 0; auxptr = auxptr->hptr) {
                printf("Dibujando objeto: %p\n", (void*)auxptr);
                /*Recorre los triangulos */
                for (i = 0; i < auxptr->num_triangles; i++) {
                    dibujar_triangulo(auxptr, i);
                }
            }
            
            // Dibujar cámaras (excepto la activa si estamos en modo cámara)
            /*if (camera_list != NULL) {
                printf("Dibujando cámaras\n");
                triobj *cam = camera_list;
                while (cam != NULL) {
                    if (!kamera || cam != active_camera) {
                        printf("Dibujando cámara: %p\n", (void*)cam);
                        for (i = 0; i < cam->num_triangles; i++) {
                            dibujar_triangulo(cam, i);
                        }
                    }
                    cam = cam->hptr;
                }
            }*/
        } else {
            // Dibujar solo el objeto seleccionado
            printf("Dibujando solo objeto seleccionado: %p\n", (void*)sel_ptr);
            for (i = 0; i < sel_ptr->num_triangles; i++) {
                dibujar_triangulo(sel_ptr, i);
            }
        }
    } else {
        // Dibujar un solo triángulo
        printf("Dibujando triángulo individual del objeto: %p\n", (void*)sel_ptr);
        dibujar_triangulo(sel_ptr, indexx);
    }

    glFlush();
}



void read_from_file(char *fitx)
{
int i,retval;
triobj *optr;

    printf("%s fitxategitik datuak hartzera\n",fitx);
    optr = (triobj *)malloc(sizeof(triobj));
    retval = cargar_triangulos(fitx, &(optr->num_triangles), &(optr->triptr));
    // TODO (transform...)
    // int cargar_triangulos_color(char *fitxiz, int *hkopptr, hiruki **hptrptr,unsigned char **rgbptr);
    // retval = cargar_triangulos_color(...)
    if (retval !=1) 
         {
         printf("%s fitxategitik datuak hartzerakoan arazoak izan ditut\n",fitxiz);
         free(optr);
         }
       else
         {
         triangulosptr = optr->triptr;
         printf("Matriz del objeto...\n");
         /**/
         optr->mptr = (mlist *)malloc(sizeof(mlist));
         /*Identidad*/
         for (i=0; i<16; i++) optr->mptr->m[i] =0;
         optr->mptr->m[0] = 1.0;
         optr->mptr->m[5] = 1.0;
         optr->mptr->m[10] = 1.0;
         optr->mptr->m[15] = 1.0;
         optr->mptr->hptr = 0;

         
        
    
        optr->hptr = foptr;
        foptr = optr;
        sel_ptr = optr;
    
  
         }
     printf("datuak irakurrita\n");
}

void x_aldaketa(int dir)
{
    double m[16];
    if (aldaketa == 'r') {
        // Rotación en el eje X
        double angle = (dir == 1) ? 20.0 : -20.0;
        double rad = angle * M_PI / 180.0;
        m[0] = 1; m[4] = 0; m[8] = 0; m[12] = 0;
        m[1] = 0; m[5] = cos(rad); m[9] = -sin(rad); m[13] = 0;
        m[2] = 0; m[6] = sin(rad); m[10] = cos(rad); m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    } else if (aldaketa == 't') {
        // Traslación en el eje X
        double distance = (dir == 1) ? 0.1 : -0.1;
        m[0] = 1; m[4] = 0; m[8] = 0; m[12] = distance;
        m[1] = 0; m[5] = 1; m[9] = 0; m[13] = 0;
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    }
    if (ald_lokala == 1)
        objektuari_aldaketa_sartu_ezk(m);
    else
        objektuari_aldaketa_sartu_esk(m);
}

void y_aldaketa(int dir)
{
    double m[16];
    if (aldaketa == 'r') {
        // Rotación en el eje Y
        double angle = (dir == 1) ? 20.0 : -20.0;
        double rad = angle * M_PI / 180.0;
        m[0] = cos(rad); m[4] = 0; m[8] = sin(rad); m[12] = 0;
        m[1] = 0; m[5] = 1; m[9] = 0; m[13] = 0;
        m[2] = -sin(rad); m[6] = 0; m[10] = cos(rad); m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    } else if (aldaketa == 't') {
        // Traslación en el eje Y
        double distance = (dir == 1) ? 0.1: -0.1;
        m[0] = 1; m[4] = 0; m[8] = 0; m[12] = 0;
        m[1] = 0; m[5] = 1; m[9] = 0; m[13] = distance;
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    }
    if (ald_lokala == 1)
        objektuari_aldaketa_sartu_ezk(m);
    else
        objektuari_aldaketa_sartu_esk(m);
}

void z_aldaketa(int dir)
{
    double m[16];
    if (aldaketa == 'r') {
        // Rotación en el eje Z
        double angle = (dir == 1) ? 20.0 : -20.0;
        double rad = angle * M_PI / 180.0;
        m[0] = cos(rad); m[4] = -sin(rad); m[8] = 0; m[12] = 0;
        m[1] = sin(rad); m[5] = cos(rad); m[9] = 0; m[13] = 0;
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    } else if (aldaketa == 't') {
        // Traslación en el eje Z
        double distance = (dir == 1) ? 0.1 : -0.1;
        m[0] = 1; m[4] = 0; m[8] = 0; m[12] = 0;
        m[1] = 0; m[5] = 1; m[9] = 0; m[13] = 0;
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = distance;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    }
    if (ald_lokala == 1)
        objektuari_aldaketa_sartu_ezk(m);
    else
        objektuari_aldaketa_sartu_esk(m);
}


void delete_objeto() {
   
        // Código original para borrar objetos normales
        if (sel_ptr != 0) {
            if (foptr == sel_ptr && sel_ptr->hptr == 0) {
                printf("Solo queda un objeto\n");
                return;
            }
            
            triobj *prev = 0;
            triobj *current = foptr;
            
            while (current != 0 && current != sel_ptr) {
                prev = current;
                current = current->hptr;
            }
            
            if (current == sel_ptr) {
                if (prev == 0)
                    foptr = sel_ptr->hptr;
                else
                    prev->hptr = sel_ptr->hptr;
                
                mlist *matrix = sel_ptr->mptr;
                while (matrix != 0) {
                    mlist *temp = matrix;
                    matrix = matrix->hptr;
                    free(temp);
                }
                
                free(sel_ptr->triptr);
                free(sel_ptr);
                
                sel_ptr = foptr;
                indexx = 0;
            }
        }
    }


void escalar(float factor) {
    double m[16];
    // Matriz de escalamiento uniforme
    m[0] = factor; m[4] = 0;      m[8] = 0;      m[12] = 0;
    m[1] = 0;      m[5] = factor; m[9] = 0;      m[13] = 0;
    m[2] = 0;      m[6] = 0;      m[10] = factor; m[14] = 0;
    m[3] = 0;      m[7] = 0;      m[11] = 0;     m[15] = 1;

    if (ald_lokala == 1)
        objektuari_aldaketa_sartu_ezk(m);
    else
        objektuari_aldaketa_sartu_esk(m);
}

void undo()
{
    if (sel_ptr != 0 && sel_ptr->mptr != 0)
    {
        if (sel_ptr->mptr->hptr == 0)
        {
            printf("Es la ultima transformación.\n");
            return;
        }
        
        mlist *temp = sel_ptr->mptr;
        sel_ptr->mptr = sel_ptr->mptr->hptr;
        free(temp);
    }
    else
    {
        printf("No hay objeto seleccionado.\n");
    }
}
void camara_adelante() {
    if (!kamera) return;  // Solo si estamos en modo cámara
    
    double m[16];
    // Mover hacia adelante en Z local (dirección de vista)
    double distance = -0.1;  // Negativo porque queremos ir hacia donde miramos (-Z)
    
    m[0] = 1; m[4] = 0; m[8] = 0; m[12] = 0;
    m[1] = 0; m[5] = 1; m[9] = 0; m[13] = 0;
    m[2] = 0; m[6] = 0; m[10] = 1; m[14] = distance;
    m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    
    objektuari_aldaketa_sartu_ezk(m);
    calcular_modelView();
}

void camara_rotar_x(int direccion) {
    if (!kamera) return;
    
    double m[16];
    double angle = (direccion == 1) ? 5.0 : -5.0;  // Rotación más suave
    double rad = angle * M_PI / 180.0;
    
    m[0] = 1; m[4] = 0; m[8] = 0; m[12] = 0;
    m[1] = 0; m[5] = cos(rad); m[9] = -sin(rad); m[13] = 0;
    m[2] = 0; m[6] = sin(rad); m[10] = cos(rad); m[14] = 0;
    m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    
    objektuari_aldaketa_sartu_ezk(m);
    calcular_modelView();
}

void camara_rotar_z(int direccion) {
    if (!kamera) return;
    
    double m[16];
    double angle = (direccion == 1) ? 5.0 : -5.0;
    double rad = angle * M_PI / 180.0;
    
    m[0] = cos(rad); m[4] = -sin(rad); m[8] = 0; m[12] = 0;
    m[1] = sin(rad); m[5] = cos(rad); m[9] = 0; m[13] = 0;
    m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
    m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
    
    objektuari_aldaketa_sartu_ezk(m);
    calcular_modelView();
}

void cambiar_camara() {
    if (!camera_list) return;
    
    if (active_camera == NULL) {
        active_camera = camera_list;
    } else {
        active_camera = active_camera->hptr;
        if (active_camera == NULL) {
            active_camera = camera_list;
        }
    }
    
    if (kamera) {
        sel_ptr = active_camera;
    }
    
    calcular_modelView();
    glutPostRedisplay();
}





// This function will be called whenever the user pushes one key
static void teklatua (unsigned char key, int x, int y)
{
int retval;
int i;
FILE *obj_file;

switch(key)
	{
	case 13: 
	        if (foptr != 0)  
	                         // si no hay objeto que no haga nada
	            {
	            indexx ++;  
		                // pero si es el último? hay que controlarlo!
		    if (indexx == sel_ptr->num_triangles) 
		        {
		        indexx = 0;
		        if ((denak == 1) && (objektuak == 0))
		            {
		            glClear( GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT );
		            glFlush();
		            }
		        }
		    }
		break;
	case 'd':
        if (denak == 1) {
        denak = 0;
        } else {
            denak = 1;
        }
    break;
	case 'o':
		if (objektuak == 1) objektuak = 0;
		    else objektuak = 1;
		break;
	case 'c':
    if (kamera == 0) {
        if (active_camera != NULL) {  // Verificar que existe una cámara activa
            kamera = 1;
            sel_ptr = active_camera;
            printf("Modo cámara activado\n");
        } else {
            printf("No hay cámara activa\n");
            return;
        }
    } else {
        kamera = 0;
        sel_ptr = foptr;
        printf("Modo objeto activado\n");
    }
    calcular_modelView();
    break;
	case 'C':
        if(flag_camera==0){
            printf("Lo que ve la CAMARA");
            flag_camera = 1;
        }else{
            printf("Lo que ve el objeto");
            flag_camera = 0;
        }
	case 'm':  // o la tecla que prefieras
    modo_textura = !modo_textura;
    printf("Modo %s\n", modo_textura ? "textura" : "color negro");
    break;
	case 'l':
		if (lineak == 1) lineak = 0;
		    else lineak = 1;
		break;
	case 't':
	        aldaketa = 't';
		break;
	case 'r':
		aldaketa = 'r';
		break;
	case 'g':
		if (ald_lokala == 1){ 
            printf("EN SISTEMA GLOBAL\n");
            ald_lokala = 0;
        }
		    else {
                printf("EN SISTEMA LOCAL\n");
                ald_lokala = 1;}
		break;
    case 'x':
            x_aldaketa(1);
            break;
    case 'y':
            y_aldaketa(1);
            break;
    case 'z':
            z_aldaketa(1);
            break;
    case 'X':
            x_aldaketa(0);
            break;
    case 'Y':
            y_aldaketa(0);
            break;
    case 'Z':
            z_aldaketa(0);
            break;
    case 'u':
            undo();
            break;
    //case 'C':  // Mayúscula para cambiar cámara
    
   // break;

case 's':  // Modificar para manejar borrado de cámaras
    
        delete_objeto();
       
    
    break;
   case '+':
        escalar(1.1); // Aumentar tamaño 
        break;
case '-':
        escalar(0.9); // Reducir tamaño 
        break;
	case 'f':
	        /*Ask for file*/
	        printf("idatzi fitxategi izena\n"); 
	        scanf("%s", &(fitxiz[0]));
	        read_from_file(fitxiz);
	        indexx = 0;
            break;
        case 9: /* <TAB> */
    if (foptr != 0) {
    
        
        // Guardar el siguiente objeto
        triobj *next = sel_ptr->hptr;
        if (next == 0) {
            next = foptr;
        }
        
        // Asignar el siguiente objeto
        sel_ptr = next;
        indexx = 0;
    
    }
    break;

case 'C':  // Cambiar cámara
    cambiar_camara();
    break;
case 'w':  // Mover cámara hacia adelante
    camara_adelante();
    break;
case 'a':  // Rotar cámara en Z (izquierda)
    camara_rotar_z(1);
    break;
case 'q':  // Rotar cámara en X (arriba)
    camara_rotar_x(1);
    break;
case 'e':  // Rotar cámara en X (abajo)
    camara_rotar_x(0);
    break;
	case 27:  // <ESC>
		exit( 0 );
		break;

case 'b':
    if(flag_backface == 0){
        printf("DIBUJANDO CARAS TRASERAS\n");
        flag_backface = 1;
    }else{
        printf("BACKFACE CULLING ACTIVADO\n");
        flag_backface = 0;
    }
    break;

case 'p':
    if(flag_projection == 0){
        set_projection_matrix();
        printf("MODO PARALELO\n");
        flag_projection=1;}
    else{
        set_projection_matrix();
        printf("MODO PERSPECTIVA\n");
        flag_projection = 0;
        }
        break;

default:
		printf("%d %c\n", key, key );
	}

glutPostRedisplay();
}


void viewportberria (int zabal, int garai)
{
if (zabal < garai)  dimentsioa = zabal;
    else  dimentsioa = garai;
glViewport(0,0,dimentsioa,dimentsioa);
printf("linea kopuru berria = %d\n",dimentsioa);
}
       
int main(int argc, char** argv) {
    int retval;

    printf(" Triangeluak: barneko puntuak eta testura\n Triángulos con puntos internos y textura \n");
    printf("Press <ESC> to finish\n");
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_RGB|GLUT_DEPTH);
    dimentsioa = 500;
    glutInitWindowSize(dimentsioa, dimentsioa);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("KBG/GO praktika");

    glutDisplayFunc(marraztu);
    glutKeyboardFunc(teklatua);
    glutReshapeFunc(viewportberria);

    /* Cargar textura */
    retval = load_ppm("testura.ppm", &bufferra, &dimx, &dimy);
    if (retval != 1) {
        printf("Ez dago testuraren fitxategia (testura.ppm)\n");
        exit(-1);
    }

    /* Inicializaciones */
    glClearColor(0.0f, 0.0f, 0.7f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_DEPTH_TEST);

    /* Variables globales */
    denak = 0;
    lineak = 0;
    objektuak = 0;
    kamera = 0;
    foptr = 0;
    sel_ptr = 0;
    aldaketa = 'r';
    ald_lokala = 1;


   
    
    

     /* Cargar objeto inicial */
    if (argc > 1) {
        read_from_file(argv[1]);
    } else {
        read_from_file("z-1+1.txt");
    }


    // Inicializar la cámara
    crear_nueva_camara(0.0,0.0,2.0);
    // crear_nueva_camara(0.0,0.0,0.0);

    set_projection_matrix();
    calcular_modelView();

    glutMainLoop();
    return 0;
}
