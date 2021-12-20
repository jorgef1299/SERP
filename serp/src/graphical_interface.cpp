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
    std::stringstream ss;
    ss << std::setw(9) << std::setfill('0') << stamp.nsec;
    const size_t fractional_second_digits = 3;
    return std::string(output) + "." + ss.str().substr(0, fractional_second_digits);
}

void insert_text_to_log(GtkTextView* text_view, GtkTextBuffer* text_buffer, GtkTextIter* text_iter, const char* text, bool erro=false) {
    gtk_text_buffer_get_end_iter(text_buffer, text_iter);
    std::string str_text(text);
    std::string str_actual_time;
    std::string str_text_to_insert;
    std::string color;
    str_actual_time = stampToString(ros::Time::now());
    if(!erro) color = "black";
    else color = "red";
    str_text_to_insert = "<span color=\"" + color + "\">> [" + str_actual_time + "] - " + str_text + "</span>\n";
    gtk_text_buffer_insert_markup(text_buffer, text_iter, str_text_to_insert.c_str(), -1);
    gtk_text_view_scroll_to_mark(text_view, gtk_text_buffer_get_insert(text_buffer), 0,true,1,1);
}

void enable_interface_button(GtkWidget *button, bool is_green=true) {
    context = gtk_widget_get_style_context(button);
    if(is_green) {
        gtk_style_context_add_class(context, "button_green_active");
    }
    else {
        gtk_style_context_add_class(context, "button_red_active");
    }
}

void disable_interface_button(GtkWidget *button, bool is_green=true) {
    context = gtk_widget_get_style_context(button);
    if(is_green) {
        gtk_style_context_remove_class(context, "button_green_active");
    }
    else {
        gtk_style_context_remove_class(context, "button_red_active");
    }
}

void* cb_timer10s(gpointer data) {
    int8_t last_battery_level = 0;
    while(ros::ok()) {
        usleep(12000000); // 12s
        // Call service to get battery level
        std_srvs::Trigger srv;
        if(client_battery_level.call(srv) && srv.response.success) {
            robot.battery_level = (int8_t)std::stoi(srv.response.message);
            if(robot.battery_level < 0 || robot.battery_level > 100) {
                ROS_ERROR("Erro: Nível de bateria inválido!");
                ros::shutdown();
                std::exit(-1);
            }
            if(robot.battery_level != last_battery_level) {
                // Update battery level in the interface
                std::string color;
                if(robot.battery_level < 20) {
                    color = "red";
                }
                else color = "black";
                std::string str_label = "<span color=\"" + color + "\">" + srv.response.message + "%</span>";
                gtk_label_set_markup(label_battery, str_label.c_str());
            }
        }
        else {
            gtk_label_set_markup(label_battery, "---");
        }
    }
    g_thread_exit(NULL);
}

void initializeGtkInterface() {
    robot.state = Stopped;
    robot.battery_level = -1;
    gtk_label_set_label(label_battery, "---");
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
    button_manual_go = GTK_WIDGET(gtk_builder_get_object(builder, "button_manual_go"));
    button_manual_stop = GTK_WIDGET(gtk_builder_get_object(builder, "button_manual_stop"));
    range_motor_left = GTK_RANGE(gtk_builder_get_object(builder, "range_motor_left"));
    range_motor_right = GTK_RANGE(gtk_builder_get_object(builder, "range_motor_right"));
    label_battery = GTK_LABEL(gtk_builder_get_object(builder, "battery_level"));

    // Create text buffer iterator
    log_text_iter = new GtkTextIter();

    gtk_text_view_set_buffer(log_mensagens, log_buffer);

    // Apply the provided css in the window widget
    apply_css_provider(window, css);

    // GTK connect signals
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
    gtk_builder_connect_signals(builder, NULL);

    // Initialize variables
    initializeGtkInterface();

    // Init ROS node
    ros::init(argc, argv, "serp_interface_node");
    ros::NodeHandle n_public;

    // Create ROS Service Clients
    client_velocity_setpoint = n_public.serviceClient<serp::VelocitySetPoint>("velocity_setpoint");
    client_battery_level = n_public.serviceClient<std_srvs::Trigger>("srv_battery_level");

    pub_twist = n_public.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    GThread *thread_timer;
    thread_timer = g_thread_new("Timer", cb_timer10s, NULL);

    g_object_unref(css);
    g_object_unref(G_OBJECT(builder));
    gtk_widget_show(window);
    gtk_main();

    return 0;
}

void gtk_main_quit() {
    ros::shutdown();
    std::exit(0);
}

extern "C"
{
    void on_button_go_manual_control_clicked(GtkButton *clicked_button) {
        if(robot.state == Stopped) {
            // Send service request
            serp::VelocitySetPoint srv;
            srv.request.state = true;
            srv.request.vel_motor_left = robot.motor_left_requested_velocity;
            srv.request.vel_motor_right = robot.motor_right_requested_velocity;
            if(client_velocity_setpoint.call(srv) && srv.response.success) {
                // Change buttons color
                enable_interface_button(button_manual_go, true);
                disable_interface_button(button_manual_stop, false);
                robot.state = ManualControl;
                // Add text to the log box
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Controlo manual ativado!");
            }
            else {
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível ativar o modo de controlo manual...", true);
            }

        }
    }

    void on_button_stop_manual_control_clicked(GtkButton *clicked_button) {
        if(robot.state == ManualControl) {
            // Send service request
            serp::VelocitySetPoint srv;
            srv.request.state = false;
            srv.request.vel_motor_left = robot.motor_left_velocity;
            srv.request.vel_motor_right = robot.motor_right_velocity;
            if(client_velocity_setpoint.call(srv) && srv.response.success) {
                // Change button color to appear disabled
                disable_interface_button(button_manual_go, true);
                enable_interface_button(button_manual_stop, false);
                robot.state = Stopped;
                // Reset Ranges
                gtk_range_set_value(range_motor_left, 0);
                gtk_range_set_value(range_motor_right, 0);
                // Add text to the log box
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Controlo manual desativado!");
            }
            else {
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível desativar o modo de controlo manual...", true);
            }
        }
    }

    void on_range_left_motor_changed(GtkRange *range) {
        gdouble value;
        value = gtk_range_get_value(range);
        if (robot.state == ManualControl) {
            // Send service request
            serp::VelocitySetPoint srv;
            srv.request.state = true; // activate setpoint mode
            srv.request.vel_motor_left = (int8_t) value;
            srv.request.vel_motor_right = robot.motor_right_requested_velocity; // Only update motor left velocity
            if (!(client_velocity_setpoint.call(srv) && srv.response.success)) {
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível atualizar manualmente a velocidade do robô.", true);
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Controlo manual desativado, por segurança!", true);
                robot.state = Stopped;
                enable_interface_button(button_manual_stop, false);
                disable_interface_button(button_manual_go, true);
            }
            else {
                robot.motor_left_requested_velocity = srv.request.vel_motor_left;
            }
        }
        else {
            robot.motor_left_requested_velocity = (int8_t) value;
        }
    }

    void on_range_right_motor_changed(GtkRange *range) {
        gdouble value;
        value = gtk_range_get_value(range);
        if (robot.state == ManualControl) {
            // Send service request
            serp::VelocitySetPoint srv;
            srv.request.state = true; // activate setpoint mode
            srv.request.vel_motor_left = robot.motor_left_requested_velocity; // Only update motor right velocity
            srv.request.vel_motor_right = (int8_t) value;
            if (!(client_velocity_setpoint.call(srv) && srv.response.success)) {
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Não foi possível atualizar manualmente a velocidade do robô.", true);
                insert_text_to_log(log_mensagens, log_buffer, log_text_iter, "Erro: Controlo manual desativado, por segurança!", true);
                robot.state = Stopped;
                enable_interface_button(button_manual_stop, false);
                disable_interface_button(button_manual_go, true);
            }
            else {
                robot.motor_right_requested_velocity = srv.request.vel_motor_right;
            }
        }
        else {
            robot.motor_right_requested_velocity = (int8_t) value;
        }
    }
}