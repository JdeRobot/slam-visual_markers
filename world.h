#ifndef WORLD_H
#define WORLD_H

#include "drawworld.h"
#include "IGraphichAlgorithmer.h"
#include <gtkmm.h>
#include <libglademm.h>

class World
{
private:
    Gtk::Main m_GtkMain;
    Glib::RefPtr<Gnome::Glade::Xml> m_RefXml;
    Gtk::Window* m_WorldWindow;
    DrawWorld* m_DrawWorld;
    IGraphicAlgorithmer* m_GraphicAlgorithmer;

public:
    World(IGraphicAlgorithmer* alg);
    virtual ~World();

    void show();
    void hide();
};

#endif // WORLD_H
