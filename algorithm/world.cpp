#include "world.h"
#include <iostream>
#include <gtkglmm.h>
#include <libgnomecanvasmm.h>

World::World(IGraphicAlgorithmer* alg) : m_GtkMain(0, 0)
{
    m_GraphicAlgorithmer = alg;

    if (!Gtk::GL::init_check(NULL, NULL))
    {
        std::cerr << "Couldn't initialize GL\n";
        std::exit(1);
    }

    Gnome::Canvas::init();

    m_RefXml = Gnome::Glade::Xml::create("./ardrone_slam.glade");
    m_RefXml->get_widget("world_window", m_WorldWindow);
    m_RefXml->get_widget_derived("world", m_DrawWorld);
    m_DrawWorld->SetGraphicAlgorithmer(m_GraphicAlgorithmer);
}

World::~World()
{
    delete this->m_DrawWorld;
}

void
World::show()
{
    m_WorldWindow->show();
}

void
World::hide()
{
    m_WorldWindow->hide();
}
