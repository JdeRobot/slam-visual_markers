#include "drawworld.h"
#include <iostream>
#include <glut.h>

#define REFRESH_TIME 100 //ms
#define MAXWORLD 30.

DrawWorld::DrawWorld(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml>& builder) : Gtk::DrawingArea(cobject), Gtk::GL::Widget<DrawWorld>()
{
    Glib::RefPtr<Gdk::GL::Config> glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB | Gdk::GL::MODE_DEPTH | Gdk::GL::MODE_DOUBLE);
    if (!glconfig)
    {
        std::cerr << "*** Cannot find the double-buffered visual.\n" << "*** Trying single-buffered visual.\n";

        // Try single-buffered visual
        glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB | Gdk::GL::MODE_DEPTH);
        if (!glconfig) {
            std::cerr << "*** Cannot find any OpenGL-capable visual.\n";
            std::exit(1);
        }
    }

    /*Set OpenGL-capability to the widget.*/
    this->unrealize();
    if (!this->set_gl_capability(glconfig) || !this->is_gl_capable())
    {
        std::cerr << "No Gl capability\n";
        std::exit(1);
    }
    this->realize();

    /*Add events*/
    this->add_events(Gdk::BUTTON1_MOTION_MASK |
            Gdk::BUTTON2_MOTION_MASK |
            Gdk::BUTTON3_MOTION_MASK |
            Gdk::BUTTON_PRESS_MASK |
            Gdk::BUTTON_RELEASE_MASK |
            Gdk::VISIBILITY_NOTIFY_MASK);

    this->signal_motion_notify_event().connect(sigc::mem_fun(this, &DrawWorld::on_motion_notify));
    this->signal_button_press_event().connect(sigc::mem_fun(this, &DrawWorld::on_button_press));
    this->signal_scroll_event().connect(sigc::mem_fun(this, &DrawWorld::on_drawarea_scroll));

    /*Call to expose_event*/
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &DrawWorld::on_timeout), REFRESH_TIME);

    /*Init Glut*/
    int val_init = 0;
    glutInit(&val_init, NULL);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    /*GL Camera Position and FOA*/
    this->glcam_pos.X = -100.;
    this->glcam_pos.Y = -100.;
    this->glcam_pos.Z = 70.;
    this->glcam_foa.X = 0.;
    this->glcam_foa.Y = 0.;
    this->glcam_foa.Z = 0.;

    this->radius = 20.0;
    this->lati = 0.2;
    this->longi = -1.0;
    this->old_x = 0.0;
    this->old_y = 0.0;
}

DrawWorld::~DrawWorld()
{

}

void
DrawWorld::SetGraphicAlgorithmer(IGraphicAlgorithmer* alg)
{
    m_GraphicAlgorithmer = alg;
}

bool
DrawWorld::on_expose_event(GdkEventExpose* event)
{
    Gtk::Allocation allocation = get_allocation();
    GLfloat width, height;

    Glib::RefPtr<Gdk::GL::Window> glwindow = get_gl_window();
    glwindow->gl_begin(get_gl_context());

    glDrawBuffer(GL_BACK);
    glClearColor(0.6f, 0.8f, 1.0f, 0.0f);

    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    width = allocation.get_width();
    height = allocation.get_height();
    //printf("ON EXPOSE EVENT W %d H %d\n", (int)width, (int)height);

    this->InitOGL(width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    /*Angulo	ratio		znear, zfar*/
    gluPerspective(50.0, width / height, 1.0, 50000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /*pos cam		pto central	vector up*/
    gluLookAt(this->glcam_pos.X, this->glcam_pos.Y, this->glcam_pos.Z,
            this->glcam_foa.X, this->glcam_foa.Y, this->glcam_foa.Z,
            0., 0., 1.);

    /*Draw world*/
    this->DrawScene();

    /*Swap buffers*/
    if (glwindow->is_double_buffered())
    {
        glwindow->swap_buffers();
    }
    else
    {
        glFlush();
    }

    glwindow->gl_end();

    return true;
}

bool
DrawWorld::on_motion_notify(GdkEventMotion* event)
{
    float desp = 0.01;
    float x = event->x;
    float y = event->y;

    /* if left mouse button is toggled */
    if (event->state & GDK_BUTTON1_MASK) { // aquí además comprobamos punto de pulsación
        if ((x - old_x) > 0.0) longi -= desp;
        else if ((x - old_x) < 0.0) longi += desp;

        if ((y - old_y) > 0.0) lati += desp;
        else if ((y - old_y) < 0.0) lati -= desp;

        this->glcam_pos.X = radius * cosf(lati) * cosf(longi) + this->glcam_foa.X;
        this->glcam_pos.Y = radius * cosf(lati) * sinf(longi) + this->glcam_foa.Y;
        this->glcam_pos.Z = radius * sinf(lati) + this->glcam_foa.Z;
    }

    /* if right mouse button is toggled */
    if (event->state & GDK_BUTTON3_MASK) {
        if ((x - old_x) > 0.0) longi -= desp;
        else if ((x - old_x) < 0.0) longi += desp;

        if ((y - old_y) > 0.0) lati += desp;
        else if ((y - old_y) < 0.0) lati -= desp;

        this->glcam_foa.X = -radius * cosf(lati) * cosf(longi) + this->glcam_pos.X;
        this->glcam_foa.Y = -radius * cosf(lati) * sinf(longi) + this->glcam_pos.Y;
        this->glcam_foa.Z = -radius * sinf(lati) + this->glcam_pos.Z;
    }

    old_x = x;
    old_y = y;
}


bool
DrawWorld::on_button_press(GdkEventButton* event)
{
    float x = event->x;
    float y = event->y;
    TPinHoleCamera myActualCamera;
    HPoint2D myActualPoint2D, boton;
    HPoint3D cameraPos3D, myActualPoint3D, intersectionPoint;

    if (event->button == 2) { // con el botón del centro vamos a coger el destino
        myActualCamera.position.X = this->glcam_pos.X;
        myActualCamera.position.Y = this->glcam_pos.Y;
        myActualCamera.position.Z = this->glcam_pos.Z;
        myActualCamera.foa.X = this->glcam_foa.X;
        myActualCamera.foa.Y = this->glcam_foa.Y;
        myActualCamera.foa.Z = this->glcam_foa.Z;
        myActualCamera.roll = 0.;
        myActualCamera.skew = 0.;
        myActualCamera.rows = 450;
        myActualCamera.columns = 994;
        myActualCamera.v0 = 497.; // 994/2
        myActualCamera.u0 = 225.; // 450/2
        myActualCamera.fdistx = 483.;
        myActualCamera.fdisty = 483.;
        update_camera_matrix(&myActualCamera);

        // Modificamos la asignación de valores del pixel para el backproject,
        // sistema de referencia óptico
        myActualPoint2D.x = 450. - 1. - event->y;
        myActualPoint2D.y = event->x;
        myActualPoint2D.h = 1.;

        // Proyectamos tal punto en 3D sobre el Plano Imagen de nuestra cámara virtual
        backproject(&myActualPoint3D, myActualPoint2D, myActualCamera);

        // Coordenadas en 3D de la posicion de la cámara
        cameraPos3D.X = myActualCamera.position.X;
        cameraPos3D.Y = myActualCamera.position.Y;
        cameraPos3D.Z = myActualCamera.position.Z;
        cameraPos3D.H = 1;

        linePlaneIntersection(myActualPoint3D, cameraPos3D, &intersectionPoint);
        //this->destino.x = intersectionPoint.X*SCALE;
        //this->destino.y = intersectionPoint.Y*SCALE;
    }
}

bool
DrawWorld::on_drawarea_scroll(GdkEventScroll * event)
{
    float vx, vy, vz;

    vx = (this->glcam_foa.X - this->glcam_pos.X) / radius;
    vy = (this->glcam_foa.Y - this->glcam_pos.Y) / radius;
    vz = (this->glcam_foa.Z - this->glcam_pos.Z) / radius;

    if (event->direction == GDK_SCROLL_UP) {
        this->glcam_foa.X = this->glcam_foa.X + vx;
        this->glcam_foa.Y = this->glcam_foa.Y + vy;
        this->glcam_foa.Z = this->glcam_foa.Z + vz;

        this->glcam_pos.X = this->glcam_pos.X + vx;
        this->glcam_pos.Y = this->glcam_pos.Y + vy;
        this->glcam_pos.Z = this->glcam_pos.Z + vz;
    }

    if (event->direction == GDK_SCROLL_DOWN) {
        this->glcam_foa.X = this->glcam_foa.X - vx;
        this->glcam_foa.Y = this->glcam_foa.Y - vy;
        this->glcam_foa.Z = this->glcam_foa.Z - vz;

        this->glcam_pos.X = this->glcam_pos.X - vx;
        this->glcam_pos.Y = this->glcam_pos.Y - vy;
        this->glcam_pos.Z = this->glcam_pos.Z - vz;
    }
}

bool
DrawWorld::on_timeout()
{
    /*Force our program to redraw*/
    Glib::RefPtr<Gdk::Window> win = get_window();
    if (win)
    {
        Gdk::Rectangle r(0, 0, get_allocation().get_width(), get_allocation().get_height());
        win->invalidate_rect(r, false);
    }
    return true;
}

void
DrawWorld::InitOGL(int w, int h)
{
    GLfloat ambient[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat diffuse[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat position[] = {0.0, 3.0, 3.0, 0.0};
    GLfloat lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
    GLfloat local_view[] = {0.0};

    glViewport(0, 0, (GLint) w, (GLint) h);
    glDrawBuffer(GL_BACK);
    glClearColor(0.6f, 0.8f, 1.0f, 0.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* With this, the pioneer appears correctly, but the cubes don't */
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);
    glEnable(GL_LIGHT0);
    // glEnable (GL_LIGHTING);
    glEnable(GL_POINT_SMOOTH);

    glEnable(GL_TEXTURE_2D); // Enable Texture Mapping
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST); // Enables Depth Testing
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH); // Enables Smooth Color Shading
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void
DrawWorld::DrawScene()
{
    int i, c, row, j, k;
    //Tvoxel start, end;
    float r, lati, longi, dx, dy, dz;
    float matColors[4];
    float Xp_sensor, Yp_sensor;
    float dpan = 0.5, dtilt = 0.5;

    // Absolute Frame of Reference
    // floor
    glColor3f(0.6, 0.6, 0.6);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    for (i = 0; i < ((int) MAXWORLD + 1); i++) {
        v3f(-(int) MAXWORLD * 10 / 2. + (float) i * 10, -(int) MAXWORLD * 10 / 2., 0.);
        v3f(-(int) MAXWORLD * 10 / 2. + (float) i * 10, (int) MAXWORLD * 10 / 2., 0.);
        v3f(-(int) MAXWORLD * 10 / 2., -(int) MAXWORLD * 10 / 2. + (float) i * 10, 0.);
        v3f((int) MAXWORLD * 10 / 2., -(int) MAXWORLD * 10 / 2. + (float) i * 10, 0.);
    }
    glEnd();

    // absolute axis
//    glLineWidth(3.0f);
//    glColor3f(0.7, 0., 0.);
//    glBegin(GL_LINES);
//    v3f(0.0, 0.0, 0.0);
//    v3f(10.0, 0.0, 0.0);
//    glEnd();
//    glColor3f(0., 0.7, 0.);
//    glBegin(GL_LINES);
//    v3f(0.0, 0.0, 0.0);
//    v3f(0.0, 10.0, 0.0);
//    glEnd();
//    glColor3f(0., 0., 0.7);
//    glBegin(GL_LINES);
//    v3f(0.0, 0.0, 0.0);
//    v3f(0.0, 0.0, 10.0);
//    glEnd();
//    glLineWidth(1.0f);

    //this->gui->callGraphicsOnApi();
    m_GraphicAlgorithmer->RunGraphicAlgorithm();

//    // Robot Frame of Reference
//    mypioneer.posx = this->robotx / 100.;
//    mypioneer.posy = this->roboty / 100.;
//    mypioneer.posz = 0.;
//    mypioneer.foax = this->robotx / 100.;
//    mypioneer.foay = this->roboty / 100.;
//    mypioneer.foaz = 10.;
//    mypioneer.roll = this->robottheta;

//    glTranslatef(mypioneer.posx, mypioneer.posy, mypioneer.posz);
//    dx = (mypioneer.foax - mypioneer.posx);
//    dy = (mypioneer.foay - mypioneer.posy);
//    dz = (mypioneer.foaz - mypioneer.posz);
//    longi = (float) atan2(dy, dx)*360. / (2. * PI);
//    glRotatef(longi, 0., 0., 1.);
//    r = sqrt(dx * dx + dy * dy + dz * dz);
//    if (r < 0.00001) lati = 0.;
//    else lati = acos(dz / r)*360. / (2. * PI);
//    glRotatef(lati, 0., 1., 0.);
//    glRotatef(mypioneer.roll, 0., 0., 1.);

    // X axis
//    glColor3f(1., 0., 0.);
//    glBegin(GL_LINES);
//    v3f(0.0, 0.0, 0.0);
//    v3f(5.0, 0.0, 0.0);
//    glEnd();

//    // Y axis
//    glColor3f(0., 1., 0.);
//    glBegin(GL_LINES);
//    v3f(0.0, 0.0, 0.0);
//    v3f(0.0, 5.0, 0.0);
//    glEnd();

//    // Z axis
//    glColor3f(0., 0., 1.);
//    glBegin(GL_LINES);
//    v3f(0.0, 0.0, 0.0);
//    v3f(0.0, 0.0, 5.0);
//    glEnd();

//    // robot body
//    glEnable(GL_LIGHTING);
//    glPushMatrix();
//    glTranslatef(1., 0., 0.);

//    // the body it is not centered. With this translation we center it
//    loadModel(); // CARGAMOS EL MODELO DEL PIONEER
//    glPopMatrix();
//    glDisable(GL_LIGHTING);

//    // lasers
//    glLineWidth(1.0f);
//    glEnable(GL_LIGHTING); // luces... entramos en parte de dibujado del pioneer con texturas
//    matColors[0] = 1.0;
//    matColors[1] = 0.0;
//    matColors[2] = 0.0;
//    matColors[3] = 0.5;
//    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, matColors);

//    start.x = laser_coord[0]*10.;
//    start.y = laser_coord[1];
//    for (k = 0; k<this->numLasers; k++) {
//        Xp_sensor = this->distanceData[k] * cos(((float) k - 90.) * DEGTORAD);
//        Yp_sensor = this->distanceData[k] * sin(((float) k - 90.) * DEGTORAD);

//        // Coordenadas del punto detectado por el US con respecto al sistema del sensor, eje x+ normal al sensor
//        end.x = laser_coord[0]*10. + Xp_sensor * laser_coord[3] - Yp_sensor * laser_coord[4];
//        end.y = laser_coord[1] + Yp_sensor * laser_coord[3] + Xp_sensor * laser_coord[4];

//        glBegin(GL_POLYGON);
//        glVertex3f(laser_coord[0]*10. / 100., laser_coord[1] / 100., 3.2);
//        glVertex3f(start.x / 100., start.y / 100., 3.2);
//        glVertex3f(end.x / 100., end.y / 100., 3.2);
//        glEnd();

//        start.x = end.x;
//        start.y = end.y;
//    }
//    glDisable(GL_LIGHTING);
}

void DrawWorld::linePlaneIntersection(HPoint3D A, HPoint3D B, HPoint3D *intersectionPoint) {
    HPoint3D v; // Line director vector: it the same to take A or B as origin or destination extrem...
    float t;

    v.X = (B.X - A.X);
    v.Y = (B.Y - A.Y);
    v.Z = (B.Z - A.Z);

    // We'll calculate the ground intersection (Z = 0) on our robot system. Parametric equations:
    // intersectionPoint->Z = A.Z + t*v.Z => t = (-A.Z / v.Z)
    t = (-A.Z) / (v.Z);
    intersectionPoint->X = A.X + (t * v.X);
    intersectionPoint->Y = A.Y + (t * v.Y);
    intersectionPoint->Z = A.Z + (t * v.Z);
}

void DrawWorld::getDestino() {
    //printf("hola\n");

    //point->x = destino.x;
    //point->y = destino.y;
    //*point=destino;
    //printf ("myPoint = [%f, %f]\n", destino.x, destino.y);
}
