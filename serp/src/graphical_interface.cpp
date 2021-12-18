#include "graphical_interface.h"

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

std::string stampToString(const ros::Time& stamp, const std::string format="%Y-%m-%d %H:%M:%S")
{
    const int output_size = 100;
    char output[output_size];
    std::time_t raw_time = static_cast<time_t>(stamp.sec);
    struct tm* timeinfo = localtime(&raw_time);
    std::strftime(output, output_size, format.c_str(), timeinfo);
    return std::string(output);
}

void insert_text_to_log(GtkTextView* text_view, GtkTextBuffer* text_buffer, GtkTextIter* text_iter, const char* text, std::string mode) {
    gtk_text_buffer_get_end_iter(text_buffer, text_iter);
    std::string str_text(text);
    std::string str_actual_time;
    std::string str_text_to_insert;
    std::string color;
    str_actual_time = stampToString(ros::Time::now());
    if(mode == "normal") color = "black";
    else if(mode == "error") color = "red";
    str_text_to_insert = "<span color=\"" + color + "\">> [" + str_actual_time + "] - " + str_text + "</span>\n";
    gtk_text_buffer_insert_markup(text_buffer, text_iter, str_text_to_insert.c_str(), -1);
    gtk_text_view_scroll_to_mark(text_view, gtk_text_buffer_get_insert(text_buffer), 0,true,1,1);
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
    char css_file_name[38] = "include/graphical_interface_style.css";
    char glade_file_name[34] = "include/graphical_interface.glade";
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
    log_mensagens = GTK_TEXT_VIEW(gtk_builder_get_object(builder, "log_text_view"));
    log_buffer = GTK_TEXT_BUFFER(gtk_builder_get_object(builder, "log_buffer"));

    // Create text buffer iterator
    log_text_iter = new GtkTextIter();

    gtk_text_view_set_buffer(log_mensagens, log_buffer);

    // Apply the provided css in the window widget
    apply_css_provider(window, css);

    // GTK connect signals
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    gtk_builder_connect_signals(builder, NULL);

    // Init ROS node
    ros::init(argc, argv, "serp_interface_node");
    ros::NodeHandle n_public;

    // Create ROS Service Clients
    client_velocity_setpoint = n_public.serviceClient<serp::VelocitySetPoint>("velocity_setpoint");

    pub_twist = n_public.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    g_object_unref(css);
    g_object_unref(G_OBJECT(builder));
    gtk_widget_show(window);
    gtk_main();

    return 0;
}

extern "C"
{
    void on_button_go_manual_control_clicked(GtkButton *clicked_button) {
        ROS_INFO("Botão GO clicado!"); //TODO: DELETE
        // TODO: Check if the velocity values are valid, if not -> clear the values

        // Send service request //TODO: Send the inserted values instead of the following ones
        serp::VelocitySetPoint srv;
        srv.request.state = true; // activate setpoint mode
        srv.request.vel_motor_left = 42;
        srv.request.vel_motor_right = 23;
        if(client_velocity_setpoint.call(srv) && srv.response.success) {
            ROS_INFO("Successfully defined velocity setpoint to: Motor_Left=%d%%\tMotor_Right=%d%%", srv.request.vel_motor_left, srv.request.vel_motor_right);
        }
        insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Mensagem de teste", "normal");
    }

    void on_button_stop_manual_control_clicked(GtkButton *clicked_button) {
        // Send service request
        serp::VelocitySetPoint srv;
        srv.request.state = false;
        if(client_velocity_setpoint.call(srv) && srv.response.success) {
            ROS_INFO("Successfully stopped velocity setpoint!");
        }
        else {
            ROS_ERROR("Couldn't stop velocity setpoint...");
        }
        insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro...", "error");
    }
}