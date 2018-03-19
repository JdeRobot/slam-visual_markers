#ifndef DRAWWORLD_H
#define DRAWWORLD_H

#include "IGraphichAlgorithmer.h"
#include <gtkmm.h>
#include <gtkglmm.h>
#include <libglademm.h>
#include <progeo/progeo.h>

#define v3f glVertex3f

class DrawWorld : public Gtk::DrawingArea, public Gtk::GL::Widget<DrawWorld>
{
private:
    IGraphicAlgorithmer* m_GraphicAlgorithmer;

public:
    DrawWorld(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml>& builder);
    virtual ~DrawWorld();

    void SetGraphicAlgorithmer(IGraphicAlgorithmer* alg);

    void getDestino();
    //CvPoint2D32f destino;

protected:
    virtual bool on_expose_event(GdkEventExpose* event);
    virtual bool on_motion_notify(GdkEventMotion* event);
    virtual bool on_button_press(GdkEventButton* event);
    virtual bool on_drawarea_scroll(GdkEventScroll * event);
    bool on_timeout();

    void InitOGL(int w, int h);
    void DrawScene();

    HPoint3D glcam_pos;
    HPoint3D glcam_foa;
    HPoint3D cam_pos;

    //float scale;
    float radius;
    float lati;
    float longi;
    float old_x;
    float old_y;

private:
    void linePlaneIntersection(HPoint3D A, HPoint3D B, HPoint3D *intersectionPoint);
};

#endif // DRAWWORLD_H
