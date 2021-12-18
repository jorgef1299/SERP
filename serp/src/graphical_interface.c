#include <gtk/gtk.h>
#include <stdlib.h>

// Set the style provider for the widgets
static void apply_css_provider(GtkWidget *widget, GtkCssProvider *cssstyleProvider)
{
    gtk_style_context_add_provider(gtk_widget_get_style_context(widget), GTK_STYLE_PROVIDER(cssstyleProvider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    // For container widgets, apply to every child widget on the container
    if (GTK_IS_CONTAINER(widget))
    {
        gtk_container_forall(GTK_CONTAINER(widget), (GtkCallback)apply_css_provider, cssstyleProvider);
    }
}

int main(int argc, char *argv[])
{
    GtkBuilder *builder;
    GtkWidget *window;
    GtkLabel *label_motor_esquerda;
    GtkLabel *label_motor_direita;
    GtkCssProvider *css = gtk_css_provider_new();

    // Get path of the base project using a environment variable
    char *project_path = getenv("SERP_PROJECT_PATH");
    char css_file_name[37] = "include/graphical_interface_style.css";
    char glade_file_name[33] = "include/graphical_interface.glade";
    char css_file_path[200];
    char glade_file_path[200];
    sprintf(css_file_path, "%s%s", project_path, css_file_name);
    sprintf(glade_file_path, "%s%s", project_path, glade_file_name);

    // Load CSS file
    gtk_css_provider_load_from_path(css, css_file_path, NULL);

    gtk_init(&argc, &argv);

    builder = gtk_builder_new();

    // Load Glade file
    gtk_builder_add_from_file(builder, glade_file_path, NULL);

    // Load widgets from glade file
    window = GTK_WIDGET(gtk_builder_get_object(builder, "window"));
    label_motor_esquerda = GTK_LABEL(gtk_builder_get_object(builder, "label_motor_esquerda"));
    label_motor_direita = GTK_LABEL(gtk_builder_get_object(builder, "label_motor_direita"));

    // Apply the provided css in the window widget
    apply_css_provider(window, css);

    // GTK connect signals
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_label_set_text(label_motor_esquerda, "1.12");
    gtk_label_set_text(label_motor_direita, "-0.57");

    g_object_unref(css);
    g_object_unref(G_OBJECT(builder));
    gtk_widget_show(window);
    gtk_main();

    return 0;
}
